#include "global_data.h"

#ifdef INMP441_SENSOR
#include "inmp441_setup.h"

inmp441_t Leq_dB = {0};
QueueHandle_t samples_queue;
QueueHandle_t inmp_queue;
float samples[SAMPLES_SHORT] __attribute__((aligned(4))); // Static buffer for block of samples

struct SOS_Coefficients {
  float b1;
  float b2;
  float a1;
  float a2;
};

struct SOS_Delay_State {
    float w0 = 0;
    float w1 = 0;
};

extern "C" {
  int sos_filter_f32(float *input, float *output, int len, const SOS_Coefficients &coeffs, SOS_Delay_State &w);
} 
__asm__ (
  /* ESP32 implementation of IIR Second-Order Section filter 
   * Assumes a0 and b0 coefficients are one (1.0)
   *
   * float* a2 = input;
   * float* a3 = output;
   * int    a4 = len;
   * float* a5 = coeffs;
   * float* a6 = w; 
   * float  a7 = gain; */
  ".text                    \n"
  ".align  4                \n"
  ".global sos_filter_f32   \n"
  ".type   sos_filter_f32,@function\n"
  "sos_filter_f32:          \n"
  "  entry   a1, 16         \n"
  "  lsi     f0, a5, 0      \n" // float f0 = coeffs.b1;
  "  lsi     f1, a5, 4      \n" // float f1 = coeffs.b2;
  "  lsi     f2, a5, 8      \n" // float f2 = coeffs.a1;
  "  lsi     f3, a5, 12     \n" // float f3 = coeffs.a2;
  "  lsi     f4, a6, 0      \n" // float f4 = w[0];
  "  lsi     f5, a6, 4      \n" // float f5 = w[1];
  "  loopnez a4, 1f         \n" // for (; len>0; len--) { 
  "    lsip    f6, a2, 4    \n" //   float f6 = *input++;
  "    madd.s  f6, f2, f4   \n" //   f6 += f2 * f4; // coeffs.a1 * w0
  "    madd.s  f6, f3, f5   \n" //   f6 += f3 * f5; // coeffs.a2 * w1
  "    mov.s   f7, f6       \n" //   f7 = f6; // b0 assumed 1.0
  "    madd.s  f7, f0, f4   \n" //   f7 += f0 * f4; // coeffs.b1 * w0
  "    madd.s  f7, f1, f5   \n" //   f7 += f1 * f5; // coeffs.b2 * w1 -> result
  "    ssip    f7, a3, 4    \n" //   *output++ = f7;
  "    mov.s   f5, f4       \n" //   f5 = f4; // w1 = w0
  "    mov.s   f4, f6       \n" //   f4 = f6; // w0 = f6
  "  1:                     \n" // }
  "  ssi     f4, a6, 0      \n" // w[0] = f4;
  "  ssi     f5, a6, 4      \n" // w[1] = f5;
  "  movi.n   a2, 0         \n" // return 0;
  "  retw.n                 \n"
);

extern "C" {
  float sos_filter_sum_sqr_f32(float *input, float *output, int len, const SOS_Coefficients &coeffs, SOS_Delay_State &w, float gain);
}
__asm__ (
  /* ESP32 implementation of IIR Second-Order section filter with applied gain.
   * Assumes a0 and b0 coefficients are one (1.0)
   * Returns sum of squares of filtered samples
   *
   * float* a2 = input;
   * float* a3 = output;
   * int    a4 = len;
   * float* a5 = coeffs;
   * float* a6 = w;
   * float  a7 = gain; */
  ".text                    \n"
  ".align  4                \n"
  ".global sos_filter_sum_sqr_f32 \n"
  ".type   sos_filter_sum_sqr_f32,@function \n"
  "sos_filter_sum_sqr_f32:  \n"
  "  entry   a1, 16         \n" 
  "  lsi     f0, a5, 0      \n"  // float f0 = coeffs.b1;
  "  lsi     f1, a5, 4      \n"  // float f1 = coeffs.b2;
  "  lsi     f2, a5, 8      \n"  // float f2 = coeffs.a1;
  "  lsi     f3, a5, 12     \n"  // float f3 = coeffs.a2;
  "  lsi     f4, a6, 0      \n"  // float f4 = w[0];
  "  lsi     f5, a6, 4      \n"  // float f5 = w[1];
  "  wfr     f6, a7         \n"  // float f6 = gain;
  "  const.s f10, 0         \n"  // float sum_sqr = 0;
  "  loopnez a4, 1f         \n"  // for (; len>0; len--) {
  "    lsip    f7, a2, 4    \n"  //   float f7 = *input++;
  "    madd.s  f7, f2, f4   \n"  //   f7 += f2 * f4; // coeffs.a1 * w0
  "    madd.s  f7, f3, f5   \n"  //   f7 += f3 * f5; // coeffs.a2 * w1;
  "    mov.s   f8, f7       \n"  //   f8 = f7; // b0 assumed 1.0
  "    madd.s  f8, f0, f4   \n"  //   f8 += f0 * f4; // coeffs.b1 * w0;
  "    madd.s  f8, f1, f5   \n"  //   f8 += f1 * f5; // coeffs.b2 * w1; 
  "    mul.s   f9, f8, f6   \n"  //   f9 = f8 * f6;  // f8 * gain -> result
  "    ssip    f9, a3, 4    \n"  //   *output++ = f9;
  "    mov.s   f5, f4       \n"  //   f5 = f4; // w1 = w0
  "    mov.s   f4, f7       \n"  //   f4 = f7; // w0 = f7;
  "    madd.s  f10, f9, f9  \n"  //   f10 += f9 * f9; // sum_sqr += f9 * f9;
  "  1:                     \n"  // }
  "  ssi     f4, a6, 0      \n"  // w[0] = f4;
  "  ssi     f5, a6, 4      \n"  // w[1] = f5;
  "  rfr     a2, f10        \n"  // return sum_sqr; 
  "  retw.n                 \n"  // 
);

/**
 * Envelops above asm functions into C++ class
 */
struct SOS_IIR_Filter {

    const int num_sos;
    const float gain;
    SOS_Coefficients* sos = NULL;
    SOS_Delay_State* w = NULL;

    /* Dynamic constructor */
    SOS_IIR_Filter(size_t num_sos, const float gain, const SOS_Coefficients _sos[] = NULL): num_sos(num_sos), gain(gain) {
        if (num_sos > 0) {
        sos = new SOS_Coefficients[num_sos];
        if ((sos != NULL) && (_sos != NULL)) memcpy(sos, _sos, num_sos * sizeof(SOS_Coefficients));
        w = new SOS_Delay_State[num_sos]();
        }
    };

    /* Template constructor for const filter declaration */
    template <size_t Array_Size>
    SOS_IIR_Filter(const float gain, const SOS_Coefficients (&sos)[Array_Size]): SOS_IIR_Filter(Array_Size, gain, sos) {};

    /** 
     * Apply defined IIR Filter to input array of floats, write filtered values to output, 
     * and return sum of squares of all filtered values 
     */
    inline float filter(float* input, float* output, size_t len) {
        if ((num_sos < 1) || (sos == NULL) || (w == NULL)) return 0;
        float* source = input; 
        /* Apply all but last Second-Order-Section */
        for(int i=0; i<(num_sos-1); i++) {                
        sos_filter_f32(source, output, len, sos[i], w[i]);      
        source = output;
        }      
        /* Apply last SOS with gain and return the sum of squares of all samples */
        return sos_filter_sum_sqr_f32(source, output, len, sos[num_sos-1], w[num_sos-1], gain);
    }

    ~SOS_IIR_Filter() {
        if (w != NULL) delete[] w;
        if (sos != NULL) delete[] sos;
    }

};

/* Calculate reference amplitude value at compile time */
constexpr double MIC_REF_AMPL = pow(10, double(MIC_SENSITIVITY)/20) * ((1<<(MIC_BITS-1))-1);

/* IIR Filters */
/* DC-Blocker filter - removes DC component from I2S data
 * See: https://www.dsprelated.com/freebooks/filters/DC_Blocker.html
 * a1 = -0.9992 should heavily attenuate frequencies below 10Hz */
SOS_IIR_Filter DC_BLOCKER = {
    .gain = 1.0,
    .sos = {{-1.0, 0.0, +0.9992, 0}}
};

/* Equalizer IIR filters to flatten microphone frequency response
 * See respective .m file for filter design. Fs = 48Khz.
 *
 * Filters are represented as Second-Order Sections cascade with assumption
 * that b0 and a0 are equal to 1.0 and 'gain' is applied at the last step 
 * B and A coefficients were transformed with GNU Octave: 
 * [sos, gain] = tf2sos(B, A)
 * See: https://www.dsprelated.com/freebooks/filters/Series_Second_Order_Sections.html
 * NOTE: SOS matrix 'a1' and 'a2' coefficients are negatives of tf2sos output */

/* TDK/InvenSense INMP441
 * Datasheet: https://www.invensense.com/wp-content/uploads/2015/02/INMP441.pdf
 * B ~= [1.00198, -1.99085, 0.98892]
 * A ~= [1.0, -1.99518, 0.99518] */
SOS_IIR_Filter INMP441 = {
    .gain = 1.00197834654696, 
    .sos = { // Second-Order Sections {b1, b2, -a1, -a2}
        {-1.986920458344451, +0.986963226946616, +1.995178510504166, -0.995184322194091}
    }
};

/* Weighting filters */
/* A-weighting IIR Filter, Fs = 48KHz 
 * (By Dr. Matt L., Source: https://dsp.stackexchange.com/a/36122)
 * B = [0.169994948147430, 0.280415310498794, -1.120574766348363, 0.131562559965936, 0.974153561246036, -0.282740857326553, -0.152810756202003]
 * A = [1.0, -2.12979364760736134, 0.42996125885751674, 1.62132698199721426, -0.96669962900852902, 0.00121015844426781, 0.04400300696788968] */
SOS_IIR_Filter A_weighting = {
    .gain = 0.169994948147430, 
    .sos = { // Second-Order Sections {b1, b2, -a1, -a2}
        {-2.00026996133106, +1.00027056142719, -1.060868438509278, -0.163987445885926},
        {+4.35912384203144, +3.09120265783884, +1.208419926363593, -0.273166998428332},
        {-0.70930303489759, -0.29071868393580, +1.982242159753048, -0.982298594928989}
    }
};

void mic_i2s_init() {
    /* Configuracao dos parametros I2S */
    const i2s_config_t i2s_config = {
        .mode = i2s_mode_t(I2S_MODE_MASTER | I2S_MODE_RX),
        .sample_rate = SAMPLE_RATE,
        .bits_per_sample = i2s_bits_per_sample_t(SAMPLE_BITS),
        .channel_format = I2S_CHANNEL_FMT_ONLY_LEFT,
        .communication_format = i2s_comm_format_t(I2S_COMM_FORMAT_STAND_I2S | I2S_COMM_FORMAT_STAND_I2S),
        .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,
        .dma_buf_count = DMA_BANKS,
        .dma_buf_len = DMA_BANK_SIZE,
        .use_apll = true,
        .tx_desc_auto_clear = false,
        .fixed_mclk = 0,
        .mclk_multiple = I2S_MCLK_MULTIPLE_DEFAULT,
        .bits_per_chan = I2S_BITS_PER_CHAN_DEFAULT
    };
    /* Configuracao dos pinos I2S */
    const i2s_pin_config_t pin_config = {
        .mck_io_num =   I2S_PIN_NO_CHANGE,
        .bck_io_num =   I2S_SCK,
        .ws_io_num =    I2S_WS,
        .data_out_num = I2S_PIN_NO_CHANGE,
        .data_in_num =  I2S_SD
    };

    i2s_driver_install(I2S_PORT, &i2s_config, 0, NULL);

    #if (MIC_TIMING_SHIFT > 0) 
        /* Undocumented (?!) manipulation of I2S peripheral registers
         * to fix MSB timing issues with some I2S microphones */
        REG_SET_BIT(I2S_TIMING_REG(I2S_PORT), BIT(9));   
        REG_SET_BIT(I2S_CONF_REG(I2S_PORT), I2S_RX_MSB_SHIFT);  
    #endif
    
    i2s_set_pin(I2S_PORT, &pin_config);

    samples_queue = xQueueCreate(8, sizeof(sum_queue_t));
}

void mic_i2s_reader_task(void *pvParameters) {
    mic_i2s_init();

    /* Discard first block, microphone may have startup time (i.e. INMP441 up to 83ms) */
    size_t bytes_read = 0;
    i2s_read(I2S_PORT, &samples, SAMPLES_SHORT * sizeof(int32_t), &bytes_read, portMAX_DELAY);

    while (true) {
        /* Block and wait for microphone values from I2S
         *
         * Data is moved from DMA buffers to our 'samples' buffer by the driver ISR
         * and when there is requested ammount of data, task is unblocked
         *
         * Note: i2s_read does not care it is writing in float[] buffer, it will write
         *       integer values to the given address, as received from the hardware peripheral. */
        i2s_read(I2S_PORT, &samples, SAMPLES_SHORT * sizeof(SAMPLE_T), &bytes_read, portMAX_DELAY);
        
        /* Convert (including shifting) integer microphone values to floats, 
         * using the same buffer (assumed sample size is same as size of float), 
         * to save a bit of memory */
        SAMPLE_T* int_samples = (SAMPLE_T*)&samples;
        for(int i=0; i<SAMPLES_SHORT; i++) samples[i] = MIC_CONVERT(int_samples[i]);

        sum_queue_t q;
        /* Apply equalization and calculate Z-weighted sum of squares, 
         * writes filtered samples back to the same buffer. */
        q.sum_sqr_SPL = MIC_EQUALIZER.filter(samples, samples, SAMPLES_SHORT);

        /* Apply weighting and calucate weigthed sum of squares */
        q.sum_sqr_weighted = WEIGHTING.filter(samples, samples, SAMPLES_SHORT);

        /* Send the sums to FreeRTOS queue where main task will pick them up
         * and further calcualte decibel values (division, logarithms, etc...) */
        xQueueSend(samples_queue, &q, portMAX_DELAY);
    }
}

void mic_i2s_filter_task(void *pvParameters) {
    sum_queue_t q;
    uint32_t Leq_samples = 0;
    double Leq_sum_sqr = 0;

    /* Faz a leitura das amostras enviadas pela task 'mic_i2s_reader_task'. */
    while (xQueueReceive(samples_queue, &q, portMAX_DELAY)) {

        /* Calculate dB values relative to MIC_REF_AMPL and adjust for microphone reference */
        double short_RMS = sqrt(double(q.sum_sqr_SPL) / SAMPLES_SHORT);
        double short_SPL_dB = MIC_OFFSET_DB + MIC_REF_DB + 20 * log10(short_RMS / MIC_REF_AMPL);

        /* In case of acoustic overload or below noise floor measurement, report infinty Leq value */
        if (short_SPL_dB > MIC_OVERLOAD_DB) {
            Leq_sum_sqr = INFINITY;
        } else if (isnan(short_SPL_dB) || (short_SPL_dB < MIC_NOISE_DB)) {
            Leq_sum_sqr = -INFINITY;
        }

        /* Accumulate Leq sum */
        Leq_sum_sqr += q.sum_sqr_weighted;
        Leq_samples += SAMPLES_SHORT;

        /* When we gather enough samples, calculate new Leq value */
        if (Leq_samples >= SAMPLE_RATE * TEMPO_ANALISE) {
            double Leq_RMS = sqrt(Leq_sum_sqr / Leq_samples);
            Leq_dB.ruido = MIC_OFFSET_DB + MIC_REF_DB + 20 * log10(Leq_RMS / MIC_REF_AMPL);
            Leq_sum_sqr = 0;
            Leq_samples = 0;
            
            xQueueOverwrite(inmp_queue, &Leq_dB);
        }
    }
}
#endif
