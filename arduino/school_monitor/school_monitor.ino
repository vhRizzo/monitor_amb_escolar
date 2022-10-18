/*
 * Environmental monitoring using various sensors and freeRTOS tasks.
 * 
 * (c)2022 Victor Moura
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *    
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 */

#include <Arduino.h>
#include <driver/adc.h>
#include <esp_adc_cal.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_err.h>
#include <esp_log.h>
#include <esp_task_wdt.h>
#include <Wire.h>
#include <string.h>
#include "driver/gpio.h"
#include <driver/i2s.h>
#include "sos-iir-filter.h"
#include "driver/uart.h"
#include "esp_log.h"
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include "WiFi.h"
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include <HTTPClient.h>
#include <HTTPUpdate.h>



// ------------------------------ TIPOS ------------------------------
typedef struct {
  float temperatura;
  float umidade;
  uint32_t pressao;
} TBME280;

typedef struct {
  float DSM_poeiraPM10;
  float DSM_poeiraPM25;
} TDSM501a;

typedef struct {
  double ruido;
} TINMP441;

typedef struct {
  float coord[2]; //[0] Lat , [1] Lng
} TGPS;

typedef struct __attribute__((__packed__)) {
  float  temperatura;
  float  umidade;
  uint32_t pressao;
  float poeiraPM10;
  float poeiraPM25;
  double ruido;
  float coord[2];
} TDados_mqtt;
// ------------------------------ TIPOS ------------------------------



// ------------------------------ SEMÁFOROS E FILAS ------------------------------
// Temporizações (filas e semáforos) */
#define TEMPO_PARA_LER_FILAS      ( TickType_t )100
#define TEMPO_PARA_INSERIR_FILAS  ( TickType_t )100
#define TEMPO_PARA_OBTER_SEMAFORO ( TickType_t )1000

// Semáforo
SemaphoreHandle_t xSerial_semaphore;

// Filas
QueueHandle_t xQueue_I2S_Evento = NULL; //armazenar interrupção/evento do driver I2S (apenas para destravar a task do I2S)
QueueHandle_t xQueue_I2S_DB = NULL;     //armazenar informacao codificada (para poder alimentar a task de formatar dados)
QueueHandle_t xQueue_BME280 = NULL;     //fila armazenar e comunicar dados coletados com task wifi
QueueHandle_t xQueue_DSM501a = NULL;    //fila armazenar e comunicar dados coletados com task wifi
QueueHandle_t xQueue_GPS = NULL;        //fila armazenar e comunicar dados coletados com task wifi
QueueHandle_t xQueue_Dados_mqtt = NULL;
// ------------------------------ FILAS E SEMAFOROS ------------------------------



// ------------------------------ TAREFAS ------------------------------
void TaskCalcularDSM501a( void * pvParameters );
void Task_Leitura_BME280( void *pvParameters );
void Task_Leitura_GPS( void *pvParameters );
void Task_Ler_INMP441_DMA( void *pvParameters );
void Task_Coletar_dados( void *pvParameters );
void Task_Enviar_wifi( void *pvParameters );
// ------------------------------ TAREFAS ------------------------------



// ------------------------------ DSM501a ------------------------------
#define DSM501a_pin_vermelho GPIO_NUM_34
#define DSM501a_pin_amarelo  GPIO_NUM_35
#define GPIO_INPUT_PIN_SEL  ((1ULL<<DSM501a_pin_vermelho) | (1ULL<<DSM501a_pin_amarelo))
#define ESP_INTR_FLAG_DEFAULT 0

volatile unsigned int estadoV2 = HIGH;
volatile unsigned int estadoV1 = HIGH;

volatile unsigned int pulsosV2 = 0;
volatile unsigned int pulsosV1 = 0;

volatile unsigned long marca_fallingV2 = 0;

volatile unsigned long marca_fallingV1 = 0;

void IRAM_ATTR change_falling_risingV2(void* arg) { //1 micrometro
  uint32_t gpio_estado = gpio_get_level((gpio_num_t)(uint32_t) arg);
  if (gpio_estado == HIGH && estadoV2 == LOW) {
    estadoV2 = HIGH;
    pulsosV2 += micros() - marca_fallingV2;
  }
  else if (gpio_estado == LOW && estadoV2 == HIGH) {
    marca_fallingV2 = micros();
    estadoV2 = LOW;
  }
}

void IRAM_ATTR change_falling_risingV1(void* arg) { //2.5 micrometro
  uint32_t gpio_estado = gpio_get_level((gpio_num_t)(uint32_t)arg);
  if (gpio_estado == HIGH && estadoV1 == LOW) {
    estadoV1 = HIGH;
    pulsosV1 += micros() - marca_fallingV1;
  }
  else if (gpio_estado == LOW && estadoV1 == HIGH) {
    marca_fallingV1 = micros();
    estadoV1 = LOW;
  }
}

void TaskCalcularDSM501a( void * pvParameters ) {
  float ratioV2 = 0;
  float ratioV1 = 0;
  float concentrationV2 = 0;
  float concentrationV1 = 0;

  TDSM501a tmp_dsm501a;

  gpio_config_t io_conf = {};
  io_conf.intr_type = GPIO_INTR_ANYEDGE;
  io_conf.pin_bit_mask = GPIO_INPUT_PIN_SEL;
  io_conf.mode = GPIO_MODE_INPUT;
  io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
  io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
  gpio_config(&io_conf);

  gpio_install_isr_service(ESP_INTR_FLAG_DEFAULT);

  gpio_isr_handler_add(DSM501a_pin_vermelho, change_falling_risingV2, (void*) DSM501a_pin_vermelho); //1 micrometro

  gpio_isr_handler_add(DSM501a_pin_amarelo, change_falling_risingV1, (void*) DSM501a_pin_amarelo); //2.5 micrometro

  unsigned int tempo_analise = 30000; //ms

  while (true) {
    vTaskDelay(pdMS_TO_TICKS(tempo_analise)); //AGUARDAR 30 SEGUNDOS PARA COLETAR AS INFORMACOES ACUMULADAS DA INTERRUPCAO E CALCULAR A CONCENTRAÇÃO ABAIXO...

    unsigned long pulsos_acumuladosV2 = 0;
    unsigned long pulsos_acumuladosV1 = 0;

    gpio_isr_handler_remove(DSM501a_pin_vermelho);
    gpio_isr_handler_remove(DSM501a_pin_amarelo);

    pulsos_acumuladosV2 = pulsosV2;
    pulsosV2 = 0;
    estadoV2 = HIGH;

    pulsos_acumuladosV1 = pulsosV1;
    pulsosV1 = 0;
    estadoV1 = HIGH;

    gpio_isr_handler_add(DSM501a_pin_vermelho, change_falling_risingV2, (void*) DSM501a_pin_vermelho); //1 micrometro
    gpio_isr_handler_add(DSM501a_pin_amarelo, change_falling_risingV1, (void*) DSM501a_pin_amarelo); //2.5 micrometro

    ratioV2 = (float)pulsos_acumuladosV2 / ( (float)tempo_analise * 10.0 ); //otimizado, pois é o mesmo que (lowpulseoccupancyV2 * 10 ) / ((endtime - starttime) * 100); // o *10 para estar em ms
    ratioV1 = (float)pulsos_acumuladosV1 / ( (float)tempo_analise * 10.0 );

    // concentrationV2 = 1.1 * pow(ratioV2, 3) - 3.8 * pow(ratioV2, 2) + 520 * ratioV2 + 0.62;             // curva do código encontrado na internet
    // concentrationV1 = 1.1 * pow(ratioV1, 3) - 3.8 * pow(ratioV1, 2) + 520 * ratioV1 + 0.62;
    
    concentrationV2 = 615.55 * ratioV2; // equação encontrada a partir da reta AVR no datasheet do DSM501a.
    concentrationV1 = 615.55 * ratioV1;

    tmp_dsm501a.DSM_poeiraPM10 = concentrationV2;
    tmp_dsm501a.DSM_poeiraPM25 = concentrationV1;

    xQueueOverwrite(xQueue_DSM501a, (void *)&tmp_dsm501a);
  }
}
// ------------------------------ DSM501a ------------------------------



// ------------------------------ INMP441 ------------------------------
#define LEQ_PERIOD        1           // second(s)
#define WEIGHTING         A_weighting

// NOTE: Some microphones require at least DC-Blocker filter
#define MIC_EQUALIZER     INMP441     // See below for defined IIR filters or set to 'None' to disable
#define MIC_OFFSET_DB     3.0103      // Default offset (sine-wave RMS vs. dBFS). Modify this value for linear calibration

// Customize these values from microphone datasheet
#define MIC_SENSITIVITY   -26         // dBFS value expected at MIC_REF_DB (Sensitivity value from datasheet)
#define MIC_REF_DB        94.0        // Value at which point sensitivity is specified in datasheet (dB)
#define MIC_OVERLOAD_DB   116.0       // dB - Acoustic overload point
#define MIC_NOISE_DB      33          // dB - Noise floor
#define MIC_BITS          24          // valid number of bits in I2S data
#define MIC_CONVERT(s)    (s >> (SAMPLE_BITS - MIC_BITS))
#define MIC_TIMING_SHIFT  0           // Set to one to fix MSB timing for some microphones, i.e. SPH0645LM4H-x

// Calculate reference amplitude value at compile time
constexpr double MIC_REF_AMPL = pow(10, double(MIC_SENSITIVITY) / 20) * ((1 << (MIC_BITS - 1)) - 1);

#define I2S_WS            5
#define I2S_SCK           4
#define I2S_SD            15

#define I2S_PORT          I2S_NUM_0

SOS_IIR_Filter DC_BLOCKER = {
  gain: 1.0,
sos: {{ -1.0, 0.0, +0.9992, 0}}
};

SOS_IIR_Filter INMP441 = {
  gain: 1.00197834654696,
sos: { // Second-Order Sections {b1, b2, -a1, -a2}
    { -1.986920458344451, +0.986963226946616, +1.995178510504166, -0.995184322194091}
  }
};

SOS_IIR_Filter A_weighting = {
  gain: 0.169994948147430,
sos: { // Second-Order Sections {b1, b2, -a1, -a2}
    { -2.00026996133106, +1.00027056142719, -1.060868438509278, -0.163987445885926},
    { +4.35912384203144, +3.09120265783884, +1.208419926363593, -0.273166998428332},
    { -0.70930303489759, -0.29071868393580, +1.982242159753048, -0.982298594928989}
  }
};


#define SAMPLE_RATE       48000 // Hz, fixed to design of IIR filters
#define SAMPLE_BITS       32    // bits
#define SAMPLE_T          int32_t
#define SAMPLES_SHORT     (SAMPLE_RATE / 8) // ~125ms
#define DMA_BANK_SIZE     (SAMPLES_SHORT / 16)
#define DMA_BANKS         32

// Data we push to 'samples_queue'
struct sum_queue_t {
  // Sum of squares of mic samples, after Equalizer filter
  float sum_sqr_SPL;
  // Sum of squares of weighted mic samples
  float sum_sqr_weighted;
  // Debug only, FreeRTOS ticks we spent processing the I2S data
  uint32_t proc_ticks;
};
QueueHandle_t samples_queue;

// Static buffer for block of samples
float samples[SAMPLES_SHORT] __attribute__((aligned(4)));


void mic_i2s_init() {
  const i2s_config_t i2s_config = {
mode: i2s_mode_t(I2S_MODE_MASTER | I2S_MODE_RX),
sample_rate: SAMPLE_RATE,
bits_per_sample: i2s_bits_per_sample_t(SAMPLE_BITS),
channel_format: I2S_CHANNEL_FMT_ONLY_LEFT,
communication_format: i2s_comm_format_t(I2S_COMM_FORMAT_STAND_I2S | I2S_COMM_FORMAT_STAND_I2S),
intr_alloc_flags: ESP_INTR_FLAG_LEVEL1,
dma_buf_count: DMA_BANKS,
dma_buf_len: DMA_BANK_SIZE,
use_apll: true,
tx_desc_auto_clear: false,
    fixed_mclk: 0
  };
  // I2S pin mapping
  const i2s_pin_config_t pin_config = {
bck_io_num:   I2S_SCK,
ws_io_num:    I2S_WS,
data_out_num: -1, // not used
data_in_num:  I2S_SD
  };

  i2s_driver_install(I2S_PORT, &i2s_config, 0, NULL);

#if (MIC_TIMING_SHIFT > 0)
  // Undocumented (?!) manipulation of I2S peripheral registers
  // to fix MSB timing issues with some I2S microphones
  REG_SET_BIT(I2S_TIMING_REG(I2S_PORT), BIT(9));
  REG_SET_BIT(I2S_CONF_REG(I2S_PORT), I2S_RX_MSB_SHIFT);
#endif

  i2s_set_pin(I2S_PORT, &pin_config);
}


#define I2S_TASK_PRI   4
#define I2S_TASK_STACK 2048
//
void mic_i2s_reader_task(void* parameter) {
  mic_i2s_init();

  // Discard first block, microphone may have startup time (i.e. INMP441 up to 83ms)
  size_t bytes_read = 0;
  i2s_read(I2S_PORT, &samples, SAMPLES_SHORT * sizeof(int32_t), &bytes_read, portMAX_DELAY);

  while (true) {
    // Block and wait for microphone values from I2S
    //
    // Data is moved from DMA buffers to our 'samples' buffer by the driver ISR
    // and when there is requested ammount of data, task is unblocked
    //
    // Note: i2s_read does not care it is writing in float[] buffer, it will write
    //       integer values to the given address, as received from the hardware peripheral.
    i2s_read(I2S_PORT, &samples, SAMPLES_SHORT * sizeof(SAMPLE_T), &bytes_read, portMAX_DELAY);

    TickType_t start_tick = xTaskGetTickCount();

    // Convert (including shifting) integer microphone values to floats,
    // using the same buffer (assumed sample size is same as size of float),
    // to save a bit of memory
    SAMPLE_T* int_samples = (SAMPLE_T*)&samples;
    for (int i = 0; i < SAMPLES_SHORT; i++) samples[i] = MIC_CONVERT(int_samples[i]);

    sum_queue_t q;
    // Apply equalization and calculate Z-weighted sum of squares,
    // writes filtered samples back to the same buffer.
    q.sum_sqr_SPL = MIC_EQUALIZER.filter(samples, samples, SAMPLES_SHORT);

    // Apply weighting and calucate weigthed sum of squares
    q.sum_sqr_weighted = WEIGHTING.filter(samples, samples, SAMPLES_SHORT);

    // Debug only. Ticks we spent filtering and summing block of I2S data
    q.proc_ticks = xTaskGetTickCount() - start_tick;

    // Send the sums to FreeRTOS queue where main task will pick them up
    // and further calcualte decibel values (division, logarithms, etc...)
    xQueueSend(samples_queue, &q, portMAX_DELAY);
  }
}

void consome_mic_i2s_reader_task(void* parameter) {
  sum_queue_t q;
  uint32_t Leq_samples = 0;
  double Leq_sum_sqr = 0;
  TINMP441 Leq_dB = {0};

  // Read sum of samaples, calculated by 'i2s_reader_task'
  while (xQueueReceive(samples_queue, &q, portMAX_DELAY)) {

    // Calculate dB values relative to MIC_REF_AMPL and adjust for microphone reference
    double short_RMS = sqrt(double(q.sum_sqr_SPL) / SAMPLES_SHORT);
    double short_SPL_dB = MIC_OFFSET_DB + MIC_REF_DB + 20 * log10(short_RMS / MIC_REF_AMPL);

    // In case of acoustic overload or below noise floor measurement, report infinty Leq value
    if (short_SPL_dB > MIC_OVERLOAD_DB) {
      Leq_sum_sqr = INFINITY;
    } else if (isnan(short_SPL_dB) || (short_SPL_dB < MIC_NOISE_DB)) {
      Leq_sum_sqr = -INFINITY;
    }

    // Accumulate Leq sum
    Leq_sum_sqr += q.sum_sqr_weighted;
    Leq_samples += SAMPLES_SHORT;

    // When we gather enough samples, calculate new Leq value
    if (Leq_samples >= SAMPLE_RATE * LEQ_PERIOD) {
      double Leq_RMS = sqrt(Leq_sum_sqr / Leq_samples);
      Leq_dB.ruido = MIC_OFFSET_DB + MIC_REF_DB + 20 * log10(Leq_RMS / MIC_REF_AMPL);
      Leq_sum_sqr = 0;
      Leq_samples = 0;

      // Serial output, customize (or remove) as needed
      //Serial.printf("%.1f\n", Leq_dB.ruido);
      xQueueOverwrite(xQueue_I2S_DB, (void *)&Leq_dB);
    }
  }
}
// ------------------------------ INMP441 ------------------------------



// ------------------------------ NEO ------------------------------
#define TX_pin_GPS 32 //pin
#define RX_pin_GPS 33 //pin

#define RD_BUF_SIZE 1024
#define PATTERN_CHR_NUM    (1)

//CONSULTAR https://github.com/espressif/esp-idf/tree/bcbef9a8db54d2deef83402f6e4403ccf298803a/examples/peripherals/uart/nmea0183_parser
void Task_Leitura_GPS(void *pvParameters) {
  const uart_port_t uart_num = UART_NUM_2;
  uart_config_t uart_config = {
    .baud_rate = 9600,
    .data_bits = UART_DATA_8_BITS,
    .parity = UART_PARITY_DISABLE,
    .stop_bits = UART_STOP_BITS_1,
    .flow_ctrl = UART_HW_FLOWCTRL_DISABLE, //UART_HW_FLOWCTRL_CTS_RTS,
    .rx_flow_ctrl_thresh = 122,
  };
  // Configure UART parameters
  uart_param_config(uart_num, &uart_config);
  // Set UART pins(TX, RX, RTS, CTS)
  uart_set_pin(uart_num, TX_pin_GPS, RX_pin_GPS, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
  // Setup UART buffered IO with event queue
  const int uart_buffer_size = (RD_BUF_SIZE * 2);
  QueueHandle_t uart_queue;
  // Install UART driver using an event queue here
  uart_driver_install(uart_num, uart_buffer_size, 0 /*sem um tx buffer*/, 20, &uart_queue, 0);


  //adaptado de:https://github.com/espressif/esp-idf/blob/master/examples/peripherals/uart/nmea0183_parser/main/nmea_parser.c
  //habilita a detecção de pattern, no caso '\n' ao final e cada item de padrão de GPS
  uart_enable_pattern_det_baud_intr(uart_num, '\n', PATTERN_CHR_NUM, 9, 0, 0);
  /* Set pattern queue size */
  uart_pattern_queue_reset(uart_num, 20);
  uart_flush(uart_num);
  //fim adaptado

  TGPS gps_dados;

  uart_event_t event;
  size_t buffered_size;
  uint8_t* dtmp = (uint8_t*) malloc(RD_BUF_SIZE);
  while (1) {
    //Waiting for UART event.
    if (xQueueReceive(uart_queue, (void * )&event, (portTickType)portMAX_DELAY)) {
      bzero(dtmp, RD_BUF_SIZE);
      //if ( xSemaphoreTake(xSerial_semaphore, TEMPO_PARA_OBTER_SEMAFORO) == pdTRUE ) {
      //      Serial.println(event.size);
      //      xSemaphoreGive(xSerial_semaphore);
      //}
      switch (event.type) {
        //Event of UART receving data
        /*We'd better handler data event fast, there would be much more data events than
          other types of events. If we take too much time on data event, the queue might
          be full.*/
        case UART_DATA:
          //ESP_LOGI("uart_events", "[UART DATA]: %d", event.size);
          uart_read_bytes(UART_NUM_2, dtmp, event.size, portMAX_DELAY);
          if ( xSemaphoreTake(xSerial_semaphore, TEMPO_PARA_OBTER_SEMAFORO) == pdTRUE ) {
            Serial.print("UART_DATA: ");
            Serial.println((const char*) dtmp);
            xSemaphoreGive(xSerial_semaphore);
          }
          //ESP_LOGI("uart_events", "[DATA EVT]:");
          //uart_write_bytes(UART_NUM_2, (const char*) dtmp, event.size);
          break;
        //Event of HW FIFO overflow detected
        case UART_FIFO_OVF:
          if ( xSemaphoreTake(xSerial_semaphore, TEMPO_PARA_OBTER_SEMAFORO) == pdTRUE ) {
            Serial.println("UART hw fifo de overflow");
            xSemaphoreGive(xSerial_semaphore);
          }
          ESP_LOGI("uart_events", "hw fifo overflow");
          // If fifo overflow happened, you should consider adding flow control for your application.
          // The ISR has already reset the rx FIFO,
          // As an example, we directly flush the rx buffer here in order to read more data.
          uart_flush_input(uart_num);
          xQueueReset(uart_queue);
          break;
        //Event of UART ring buffer full
        case UART_BUFFER_FULL:
          if ( xSemaphoreTake(xSerial_semaphore, TEMPO_PARA_OBTER_SEMAFORO) == pdTRUE ) {
            Serial.println("UART ring buffer full");
            xSemaphoreGive(xSerial_semaphore);
          }
          ESP_LOGI("uart_events", "ring buffer full");
          // If buffer full happened, you should consider encreasing your buffer size
          // As an example, we directly flush the rx buffer here in order to read more data.
          uart_flush_input(uart_num);
          xQueueReset(uart_queue);
          break;
        //Event of UART RX break detected
        case UART_BREAK:
          ESP_LOGI("uart_events", "uart rx break");
          break;
        //Event of UART parity check error
        case UART_PARITY_ERR:
          ESP_LOGI("uart_events", "uart parity error");
          break;
        //Event of UART frame error
        case UART_FRAME_ERR:
          ESP_LOGI("uart_events", "uart frame error");
          break;
        //UART_PATTERN_DET
        case UART_PATTERN_DET:
          int pos = uart_pattern_pop_pos(uart_num);
          //ESP_LOGI("uart_events", "[UART PATTERN DETECTED] pos: %d, buffered size: %d", pos, buffered_size);
          if (pos == -1) {
            // There used to be a UART_PATTERN_DET event, but the pattern position queue is full so that it can not
            // record the position. We should set a larger queue size.
            // As an example, we directly flush the rx buffer here.
            uart_flush_input(uart_num);
          } else {
            uart_read_bytes(uart_num, dtmp, pos, 100 / portTICK_PERIOD_MS); //faz a leitura do buffer até pos, antes do '\n'

            uint8_t pat[PATTERN_CHR_NUM + 1];
            memset(pat, 0, sizeof(pat));
            uart_read_bytes(uart_num, pat, PATTERN_CHR_NUM, 100 / portTICK_PERIOD_MS); //retira o padrão '\n' do buffer, preparando o buffer para ser lido de novo...

            //abaixo token com base no ',' e o if em seguida só analisa em laço while() se 1o token GPGLL que tem latitude e longintude... o resto ignora e segue...
            char *ptr_dtmp_tok = strtok ((char*)dtmp, ","); //https://www.best-microcontroller-projects.com/arduino-strtok.html
            if (ptr_dtmp_tok != NULL) {
              if (strcmp(ptr_dtmp_tok, "$GPGLL") == 0) {
                uint8_t contador = 0; //já usou o "$GPGLL", agora pegar latitude (contador 0) e sequencias...
                ptr_dtmp_tok = strtok (NULL, ","); //tenta próximo token. Se valido, será Latitude, e na próxima interação while() outros do padrão GPGLL...
                while (ptr_dtmp_tok != NULL) {  //pega próximo token valido e analisa //https://www.best-microcontroller-projects.com/arduino-strtok.html
                  contador++;
                  if (contador == 1 || contador == 3) { //latitude || longitude
                    float ll = strtof(ptr_dtmp_tok, NULL);
                    int deg = ((int)ll) / 100;
                    float min = ll - (deg * 100);
                    ll = deg + min / 60.0f;
                    gps_dados.coord[!(contador % 3)] = ll; //contador%3==1 (latitude) , contador%3==0 (longitude),
                    //ao aplicar !, inverte 0 para 1 e vice versa para acomodar no
                    //indice correto do vetor
                  }
                  if (contador == 2 || contador == 4) { // Latitude north(1)/south(-1) information
                    // Longitude east(1)/west(-1) information
                    if (ptr_dtmp_tok[0] == 'S' || ptr_dtmp_tok[0] == 's')
                      gps_dados.coord[0] *= -1;

                    if (ptr_dtmp_tok[0] == 'W' || ptr_dtmp_tok[0] == 'w')
                      gps_dados.coord[1] *= -1;
                  }
                  ptr_dtmp_tok = strtok (NULL, ","); //pega o próximo token válido (se tiver ,, gerará NULL, então alguma imprecisão do GPS... sairá do laço)
                }
                //xQueueSend(xQueue_GPS, (void *)&gps_dados, TEMPO_PARA_INSERIR_FILAS );
                xQueueOverwrite(xQueue_GPS, (void *)&gps_dados);
              }
            }
          }
          break;
      }
    }
  }
}
// ------------------------------ NEO ------------------------------



// ------------------------------ BME280 ------------------------------
void Task_Leitura_BME280(void *pvParameters) {

  Adafruit_BME280 bme;
  TBME280 bme_dados;

  if (! bme.begin(0x76, &Wire)) { //Endereco I2C, qual I2C hardware inicializado no setup()
    if ( xSemaphoreTake(xSerial_semaphore, TEMPO_PARA_OBTER_SEMAFORO) == pdTRUE ) {
      Serial.println("Sensor BME280 não encontrado...");
      xSemaphoreGive(xSerial_semaphore);
    }
    while (1) {
      vTaskDelay( 5000 / portTICK_PERIOD_MS ); //para nao travar as outras tasks..
    }
  }

  // weather monitoring
  //Serial.println("-- Weather Station Scenario --");
  //Serial.println("forced mode, 1x temperature / 1x humidity / 1x pressure oversampling,");
  //Serial.println("filter off");
  bme.setSampling(Adafruit_BME280::MODE_FORCED,
                  Adafruit_BME280::SAMPLING_X1, // temperature
                  Adafruit_BME280::SAMPLING_X1, // pressure
                  Adafruit_BME280::SAMPLING_X1, // humidity
                  Adafruit_BME280::FILTER_OFF   );
  // suggested rate is 1/60Hz (1m)
  int delayTime = 50000; // adaptado para 50s por conta do wdt em 60s


  /* Habilita o monitoramento do Task WDT nesta tarefa */
  esp_task_wdt_add(NULL);

  while (1) {
    //if (xQueueReceive(xQueue_BME280, (void *)&bme_dados, TEMPO_PARA_LER_FILAS) == pdTRUE){
    bme.takeForcedMeasurement();
    bme_dados.temperatura = bme.readTemperature();
    bme_dados.umidade = bme.readHumidity();
    bme_dados.pressao = bme.readPressure();
    xQueueSend(xQueue_BME280, (void *)&bme_dados, TEMPO_PARA_INSERIR_FILAS );
    //}

    esp_task_wdt_reset();
    vTaskDelay( delayTime / portTICK_PERIOD_MS );
  }
}
// ------------------------------ BME280 ------------------------------



// ------------------------------ PRINCIPAL ------------------------------
void Task_Coletar_dados(void *pvParameters) {
  TDados_mqtt dados_mqtt = {0, 0, 0, 0, 0, 0, {0, 0}};
  TBME280 bme_dados;
  TGPS gps_dados;
  TINMP441 inmp441_dados;
  TDSM501a dsm501a_dados;

  /* Habilita o monitoramento do Task WDT nesta tarefa */
  esp_task_wdt_add(NULL);

  while (1) {
    if (xQueueReceive(xQueue_BME280, (void *)&bme_dados, TEMPO_PARA_LER_FILAS) == pdTRUE) {
      dados_mqtt.temperatura =  bme_dados.temperatura;
      dados_mqtt.umidade =  bme_dados.umidade;
      dados_mqtt.pressao =  bme_dados.pressao;
    }

    if (xQueueReceive(xQueue_I2S_DB, (void *)&inmp441_dados, TEMPO_PARA_LER_FILAS) == pdTRUE) {
      dados_mqtt.ruido =  inmp441_dados.ruido;
    }

    if (xQueueReceive(xQueue_DSM501a, (void *)&dsm501a_dados, TEMPO_PARA_LER_FILAS) == pdTRUE) {
      dados_mqtt.poeiraPM10 =  dsm501a_dados.DSM_poeiraPM10;
      dados_mqtt.poeiraPM25 =  dsm501a_dados.DSM_poeiraPM25;
    }

    if (xQueueReceive(xQueue_GPS, (void *)&gps_dados, TEMPO_PARA_LER_FILAS) == pdTRUE) {
      dados_mqtt.coord[0] =  gps_dados.coord[0];
      dados_mqtt.coord[1] =  gps_dados.coord[1];
    }

    if ( xSemaphoreTake(xSerial_semaphore, TEMPO_PARA_OBTER_SEMAFORO) == pdTRUE ) {
      Serial.print("BME280(");
      Serial.print(dados_mqtt.temperatura, 1);
      Serial.print(",");
      Serial.print(dados_mqtt.umidade, 1);
      Serial.print(",");
      Serial.print(dados_mqtt.pressao);
      Serial.print(") ");
      Serial.print("INMP441(");
      Serial.print(dados_mqtt.ruido);
      Serial.print(") ");
      Serial.print("DSM501a(");
      Serial.print(dados_mqtt.poeiraPM10);
      Serial.print(",");
      Serial.print(dados_mqtt.poeiraPM25);
      Serial.print(") ");
      Serial.print("GPS(");
      Serial.print(dados_mqtt.coord[0], 6);
      Serial.print(",");
      Serial.print(dados_mqtt.coord[1], 6);
      Serial.println(") ");
      //Serial.println(saida);
      xSemaphoreGive(xSerial_semaphore);
    }

    xQueueOverwrite(xQueue_Dados_mqtt, (void *)&dados_mqtt); //pois a task de coleta desses dados e envio por mqtt fará a cada 30s

    esp_task_wdt_reset();
    vTaskDelay( 1000 / portTICK_PERIOD_MS );
  }
}
// ------------------------------ PRINCIPAL ------------------------------



// ------------------------------ WIFI/MQTT/OTA ------------------------------
#define FIRMWARE_VERSION 2
#define VC_FILE_URL "http://iot.coenc.ap.utfpr.edu.br/atualizar/version_control_file.txt"

void ota_check_update() {
  Serial.println("Verificando se existem atualizações disponíveis...");
  
  //mqtt_client.disconnect(); // Se usar mqtt deve-se desconectar do broker para não gerar erros no processo de atualização

  WiFiClient wifi_client;
  HTTPClient http_client; // Novo client http
  
  http_client.begin(wifi_client, VC_FILE_URL);
  int httpCode = http_client.GET();       // Código de retorno (sucesso ou erro)
  
  String version_control_file = http_client.getString();
  version_control_file.trim();
  http_client.end();
  
  if (httpCode != HTTP_CODE_OK) {  // Se não foi possível acessar o arquivo de controle de versão mostra um erro
    Serial.print("Falha ao verificar por atualizações -> ");
    Serial.print(String(httpCode));
    Serial.print(": ");
    Serial.println(http_client.errorToString(httpCode));
    return;
  }

  StaticJsonDocument<200> version_control_doc;  // Inicia um documento json para deserializar o arquivo de controle de versão
  if (deserializeJson(version_control_doc, version_control_file)) {  // Se não foi possível realizar a deserialização, mostra um erro
    Serial.println("Arquivo de controle de versão inválido");
    return;
  }

  if(version_control_doc["version"] <= FIRMWARE_VERSION) { // Se a versão indicada no arquivo for menor ou igual a versão atual do sketch (constante FIRMWARE_VERSION) não atualiza
    Serial.println("Não foram encontradas atualizações");
    return;
  } else {
    Serial.println("Nova versão disponível");
    Serial.println("Atualizando...");
    ota_update(version_control_doc["sketch_url"]);
  }
}


void update_started() {
  Serial.println("CALLBACK: Processo de atualização iniciado");
}

void update_finished() {
  Serial.println("CALLBACK: Processo de atualização finalizado");
}

void update_progress(int cur, int total) {
  Serial.printf("CALLBACK: Baixando sketch - %d de %d bytes...\n", cur, total);
}

void update_error(int err) {
  Serial.printf("CALLBACK: Erro no processo de atualização - %d\n", err);
}

/*
 * A função ota_update() é responsável puxar um arquivo de versionamento contendo a versão mais recente e o link do sketch, se o sketch estiver desatualizado realizada a atualização com o link contido no arquivo
 */
void ota_update(String sketch_url) {
  WiFiClient wifi_client;

  httpUpdate.onStart(update_started);
  httpUpdate.onEnd(update_finished);
  httpUpdate.onProgress(update_progress);
  httpUpdate.onError(update_error);

  t_httpUpdate_return update_status = httpUpdate.update(wifi_client, sketch_url); // Atualiza o sketch de acordo com o link contido no arquivo de versionamento

  switch (update_status){ // Verifica o estado da atualização
    case HTTP_UPDATE_FAILED:  // Falhou na atualização
      Serial.printf("HTTP_UPDATE_FAILED Error (%d): %s\n", httpUpdate.getLastError(), httpUpdate.getLastErrorString().c_str());
      break;

    case HTTP_UPDATE_NO_UPDATES:  // Não foram feitas atualizações
      Serial.println("HTTP_UPDATE_NO_UPDATES");
      break;
    
    case HTTP_UPDATE_OK:  // Atualizou com sucesso
      Serial.println("HTTP_UPDATE_OK");
      break;
  }
}


void callback(char* topic, byte* message, unsigned int length) {
  if ( xSemaphoreTake(xSerial_semaphore, TEMPO_PARA_OBTER_SEMAFORO) == pdTRUE ) {
    Serial.print("Message arrived on topic: ");
    Serial.print(topic);
    Serial.print(". Message: ");
    Serial.write(message, length);
    Serial.println();
    if (!strncmp((char *)message, "atualizar", length))
      ota_check_update();
      
    xSemaphoreGive(xSerial_semaphore);
  }
}

void Task_Enviar_wifi( void *pvParameters ) {
  const char* ssid = "Ribeiro_2.4G";
  const char* password = "99955015";
  const char* mqtt_server = "iot.coenc.ap.utfpr.edu.br";
  unsigned long ultimo_publish = 0;

  TDados_mqtt dados_mqtt = {0, 0, 0, 0, 0, 0, {0, 0}};

  WiFiClient espClient;
  PubSubClient client(espClient);

  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    vTaskDelay( 1000 / portTICK_PERIOD_MS );
    if ( xSemaphoreTake(xSerial_semaphore, TEMPO_PARA_OBTER_SEMAFORO) == pdTRUE ) {
      Serial.print(".");
      xSemaphoreGive(xSerial_semaphore);
    }
  }

  if ( xSemaphoreTake(xSerial_semaphore, TEMPO_PARA_OBTER_SEMAFORO) == pdTRUE ) {
    Serial.println("");
    Serial.println("WiFi conectado");
    Serial.println("IP: ");
    Serial.println(WiFi.localIP());
    xSemaphoreGive(xSerial_semaphore);
  }

  client.setServer(mqtt_server, 1883);
  client.setCallback(callback);

  while (1) {
    if (!client.connected()) {
      if ( xSemaphoreTake(xSerial_semaphore, TEMPO_PARA_OBTER_SEMAFORO) == pdTRUE ) {
        Serial.print("Attempting MQTT connection...");
        xSemaphoreGive(xSerial_semaphore);
      }
      // Attempt to connect
      if (client.connect("monitor_escola", "user1", "gabriel")) {
        if ( xSemaphoreTake(xSerial_semaphore, TEMPO_PARA_OBTER_SEMAFORO) == pdTRUE ) {
          Serial.println("connected");
          xSemaphoreGive(xSerial_semaphore);
        }
        // Subscribe
        client.subscribe("school-monitor/sensor/bme280-temperature/state");
      } else {
        if ( xSemaphoreTake(xSerial_semaphore, TEMPO_PARA_OBTER_SEMAFORO) == pdTRUE ) {
          Serial.print("failed, rc=");
          Serial.print(client.state());
          Serial.println(" try again in 5 seconds");
          xSemaphoreGive(xSerial_semaphore);
        }
      }
    }

    else if (millis() - ultimo_publish > 30000) {
      ultimo_publish = millis();
      if (xQueueReceive(xQueue_Dados_mqtt, (void *)&dados_mqtt, TEMPO_PARA_LER_FILAS) == pdTRUE) {
        char tmp[21] = "";
        dtostrf(dados_mqtt.temperatura, 2, 1, tmp);
        client.publish("testejr/bme/temperatura", tmp);
        dtostrf(dados_mqtt.umidade, 2, 0, tmp);
        client.publish("testejr/bme/umidade", tmp);
        dtostrf(dados_mqtt.pressao, 2, 0, tmp);
        client.publish("testejr/bme/pressao", tmp);
        dtostrf(dados_mqtt.ruido, 3, 1, tmp);
        client.publish("testejr/inmp441/decibeis", tmp);
        dtostrf(dados_mqtt.poeiraPM10, 2, 2, tmp);
        client.publish("testejr/dsm/pm10", tmp);
        dtostrf(dados_mqtt.poeiraPM25, 2, 2, tmp);
        client.publish("testejr/dsm/pm25", tmp);
        dtostrf(dados_mqtt.coord[0], 2, 6, tmp);
        client.publish("testejr/gps/lat", tmp);
        dtostrf(dados_mqtt.coord[1], 2, 6, tmp);
        client.publish("testejr/gps/lng", tmp);
      }
    }
    client.loop();
    vTaskDelay( 5000 / portTICK_PERIOD_MS );
    // esp_task_wdt_reset();
  }
}
// ------------------------------ WIFI/MQTT/OTA ------------------------------



void setup() {
  btStop(); //desabilita BlueTooth - caso não for usar para ter economia de energia...

  Serial.begin(115200);
  Serial.println(F("Starting"));

  if (!Wire.begin(21, 22)) {
    Serial.println("Falha ao inicializar I2C...");
    delay(1000);
    ESP.restart();
  }

  xSerial_semaphore = xSemaphoreCreateMutex();

  if (xSerial_semaphore == NULL) {
    Serial.println("Falha ao criar semáforos.");
    delay(1000);
    ESP.restart();
  }

  xQueue_I2S_DB = xQueueCreate( 1, sizeof( TINMP441 ) );
  xQueue_BME280 = xQueueCreate ( 1, sizeof( TBME280 ) );
  xQueue_DSM501a = xQueueCreate ( 1, sizeof( TDSM501a ) ); //obter PM10 PM25 coletado
  xQueue_GPS = xQueueCreate ( 1, sizeof( TGPS ) );
  xQueue_Dados_mqtt = xQueueCreate ( 1, sizeof( TDados_mqtt) );
  samples_queue = xQueueCreate(8, sizeof(sum_queue_t));

  if ( (xQueue_Dados_mqtt == NULL) || (xQueue_I2S_DB == NULL) || (xQueue_DSM501a == NULL) || (xQueue_BME280 == NULL) || (xQueue_GPS == NULL) /*|| (xQueue_bateria == NULL) */) {
    Serial.println("Falha ao criar filas.");
    delay(1000);
    ESP.restart();
  }

  /* Inicia o Task WDT com 60 segundos */
  esp_task_wdt_init(60, true);

  xTaskCreatePinnedToCore(mic_i2s_reader_task, "Mic I2S Reader", I2S_TASK_STACK, NULL, I2S_TASK_PRI, NULL, 0);
  xTaskCreatePinnedToCore(consome_mic_i2s_reader_task, "Mic I2S Consumidor", I2S_TASK_STACK, NULL, I2S_TASK_PRI, NULL, 0);

  xTaskCreatePinnedToCore(TaskCalcularDSM501a,  "TaskCalcularDSM501a", 4096, NULL , 6,  NULL, 0);
  xTaskCreatePinnedToCore(Task_Leitura_BME280, "bme280", 4096, NULL, 5, NULL, 0);

  xTaskCreatePinnedToCore(Task_Leitura_GPS, "gps", 4096, NULL, 3, NULL, 1);
  xTaskCreatePinnedToCore(Task_Coletar_dados, "task_Coletar_dados", 4096, NULL, 2, NULL, 1);
  xTaskCreatePinnedToCore(Task_Enviar_wifi, "task_Enviar_wifi", 4096, NULL, 1, NULL, 1);
}

void loop() {
}
