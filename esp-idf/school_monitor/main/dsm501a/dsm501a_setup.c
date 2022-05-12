#include "global_data.h"

#ifdef DSM501A_SENSOR

#include "dsm501a_setup.h"

QueueHandle_t dsm_queue;

volatile unsigned int estado_anterior_v2 = 1;
volatile unsigned int pulsos_v2 = 0;
volatile unsigned long marca_falling_v2 = 0;

volatile unsigned int estado_anterior_v1 = 1;
volatile unsigned int pulsos_v1 = 0;
volatile unsigned long marca_falling_v1 = 0;

void IRAM_ATTR change_v2_isr ( void *arg )
{
    uint32_t gpio_estado = gpio_get_level((gpio_num_t)(uint32_t) arg);
    if(gpio_estado == estado_anterior_v2) return;
    else if (gpio_estado == 0) {
        marca_falling_v2 = esp_timer_get_time();
        estado_anterior_v2 = 0;
    } else {
        estado_anterior_v2 = 1;
        pulsos_v2 += esp_timer_get_time() - marca_falling_v2;
    }
}

void IRAM_ATTR change_v1_isr ( void *arg )
{
    uint32_t gpio_estado = gpio_get_level((gpio_num_t)(uint32_t) arg);
    if(gpio_estado == estado_anterior_v1) return;
    else if (gpio_estado == 0) {
        marca_falling_v1 = esp_timer_get_time();
        estado_anterior_v1 = 0;
    } else {
        estado_anterior_v1 = 1;
        pulsos_v1 += esp_timer_get_time() - marca_falling_v1;
    }
}

void dsm501a_task( void *pvParameters )
{
    float ratio_v2 = 0;
    float concentration_v2 = 0;
    float ratio_v1 = 0;
    float concentration_v1 = 0;
    dsm501a_t dsm;

    gpio_config_t io_conf = {
        .pin_bit_mask = GPIO_INPUT_PIN_SEL,
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_ANYEDGE,
    };
    gpio_config(&io_conf);
    gpio_install_isr_service(ESP_INTR_FLAG_DEFAULT);

    gpio_isr_handler_add(DSM501a_RED_PIN, change_v2_isr, (void*) DSM501a_RED_PIN);
    gpio_isr_handler_add(DSM501a_YELLOW_PIN, change_v1_isr, (void*) DSM501a_YELLOW_PIN);

    while(true)
    {
        vTaskDelay( (TEMPO_ANALISE*1000)/portTICK_PERIOD_MS );

        gpio_isr_handler_remove(DSM501a_RED_PIN);
        gpio_isr_handler_remove(DSM501a_YELLOW_PIN);

        unsigned long pulsos_acumulados_v2 = pulsos_v2;
        pulsos_v2 = 0;
        estado_anterior_v2 = 1;

        unsigned long pulsos_acumulados_v1 = pulsos_v1;
        pulsos_v1 = 0;
        estado_anterior_v1 = 1;

        ratio_v2 = (float)( (pulsos_acumulados_v2) / (float)(TEMPO_ANALISE * 10000) );
        ratio_v1 = (float)( (pulsos_acumulados_v1) / (float)(TEMPO_ANALISE * 10000) );

        concentration_v2 = 615.55 * ratio_v2;
        concentration_v1 = 615.55 * ratio_v1;

        dsm.poeira_pm_10 = concentration_v2;
        dsm.poeira_pm_25 = concentration_v1;
        xQueueOverwrite(dsm_queue, &dsm);

        gpio_isr_handler_add(DSM501a_RED_PIN, change_v2_isr, (void*) DSM501a_RED_PIN);
        gpio_isr_handler_add(DSM501a_YELLOW_PIN, change_v1_isr, (void*) DSM501a_YELLOW_PIN);
    }
}
#endif
