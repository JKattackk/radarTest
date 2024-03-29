//includes
#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/adc.h"
#include "hardware/dma.h"
#include "pico/multicore.h"
#include "hardware/pio.h"
#include "resistor_dac.pio.h"
#include "math.h"
//FFT
#include "kiss_fftr.h"

// Channel 0 is GPIO26
#define CAPTURE_CHANNEL 27
#define CAPTURE_CHANNEL2 28
//capturing 8192 samples from two channels, 4096 per channel
#define CAPTURE_DEPTH 8192
//#define LED_PIN 25
//#define ANT_DIST 10*10^-3
//#define FMCW_SLOPE 
//#define c 3*10^8
#define SAMP_RATE 40*10^3

#define CLOCK 15
#define TX 16
#define CE 14

//globals
uint8_t capture_buf[CAPTURE_DEPTH];
float freqs[CAPTURE_DEPTH];
uint dma_chan;
//functions
void setup();
void sample();


int main(void) {
    stdio_init_all();
    sleep_ms(1000);
    printf("starting setup\n");
    setup();
    

    kiss_fft_scalar fft_in1[CAPTURE_DEPTH/2];
    kiss_fft_scalar fft_in2[CAPTURE_DEPTH/2];
    kiss_fft_cpx fft_out1[CAPTURE_DEPTH/2];
    kiss_fft_cpx fft_out2[CAPTURE_DEPTH/2];
    kiss_fftr_cfg FFTcfg = kiss_fftr_alloc(CAPTURE_DEPTH/2,false,0,0);

    float power;
    int max_idx;
    float max_power;
    printf("beginning loop\n");
    while(1){
        printf("beginning sampling 1\n");
        sample();
        //since sampling is round robin but results are stored in the same buffer, 
        //every second result in the array belongs to the same sampling set
        for (int i=0;i<CAPTURE_DEPTH;i++) {
            if(i%2){fft_in1[(int)(i/2)]=(float)capture_buf[i];}
            else{fft_in2[(int)(i/2)]=(float)capture_buf[i];}
        }
        printf("beginning FFT 1\n");
        // compute fast fourier transform
        kiss_fftr(FFTcfg , fft_in1, fft_out1);
        kiss_fftr(FFTcfg , fft_in2, fft_out2);
        printf("finished FFT 1\n");
        // basically just determine which frequency bin has the most power
        // to determine the dominant frequency in the signal
        max_power = 0;
        max_idx = 0;
        power  = 0;
        for (int i = 1; i < CAPTURE_DEPTH/2; i++) {
            power = fft_out1[i].r*fft_out1[i].r+fft_out1[i].i*fft_out1[i].i;
            if (power>max_power) {
                max_power=power;
                max_idx = i;
            } 
        }
        //phase and frequency calcs
        float max_freq = freqs[max_idx];
        float phase = atan(fft_out1[max_idx].i/fft_out1[max_idx].r);
        printf("Greatest Frequency Component C1: %0.1f Hz\n",max_freq);
        printf("phase C1: %0.1f rad\n", phase);
        //repeating for channel 2
        //combining these loops may make things more efficient
        max_power = 0;
        for (int i = 1; i < CAPTURE_DEPTH/2; i++) {
            float power = fft_out2[i].r*fft_out2[i].r+fft_out2[i].i*fft_out2[i].i;
            if (power>max_power) {
                max_power=power;
                max_idx = i;
            } 
        }
        float max_freq2 = freqs[max_idx];
        float phase2 = atan(fft_out2[max_idx].i/fft_out2[max_idx].r);
        printf("Greatest Frequency Component C2: %0.1f Hz\n",max_freq2);
        printf("phase C2: %0.1f rad\n", phase2);
        //begin FMCW ramp
        gpio_put(TX, 1);
        sleep_ms(1000);
        gpio_put(TX, 0);
        //may need longer delay here if FMCW ramp is still nonlinear
        printf("beginning sampling2\n");
        sample();
        //we only need one of the two channels for this calculation.
        for (int i=0;i<CAPTURE_DEPTH;i++) {
            if(i%2){fft_in1[(int)(i/2)]=(float)capture_buf[i];}
        }
        //calculate FFT
        kiss_fftr(FFTcfg , fft_in1, fft_out1);
        //determine dominant freq
        for (int i = 1; i < CAPTURE_DEPTH/2; i++) {
            float power = fft_out1[i].r*fft_out1[i].r+fft_out1[i].i*fft_out1[i].i;
            if (power>max_power) {
                max_power=power;
                max_idx = i;
            } 
        }
        max_freq = freqs[max_idx];
        printf("Greatest Frequency Component C1: %0.1f Hz\n",max_freq);
        //frequency change (from ramp) would be this minus the doppler shift


    }
}
  

void sample()
{
    sleep_ms(10);

    //configuring DMA channel
    dma_channel_config cfg = dma_channel_get_default_config(dma_chan);
    // Reading from constant address, writing to incrementing byte addresses
    channel_config_set_transfer_data_size(&cfg, DMA_SIZE_8);
    channel_config_set_read_increment(&cfg, false);
    channel_config_set_write_increment(&cfg, true);

    // Pace transfers based on availability of ADC samples
    channel_config_set_dreq(&cfg, DREQ_ADC);

    dma_channel_configure(dma_chan, &cfg,
        capture_buf,    // dst
        &adc_hw->fifo,  // src
        CAPTURE_DEPTH,  // transfer count
        true            // start immediately
    );
    //gpio_put(TIMING_TEST, 1);
    printf("Starting capture\n");
    adc_run(true);

    //wait for DMA to collect enough samples
    //stop the ADC and flush the fifo
    dma_channel_wait_for_finish_blocking(dma_chan);
    //gpio_put(TIMING_TEST, 0);
    printf("Capture finished\n");
    adc_run(false);
    adc_fifo_drain();
    printf("Exiting sample func\n");
}
void setup()
{
    gpio_init(TX);
    gpio_set_dir(TX, GPIO_OUT);
    // claiming the DMA channel here, configuring it in the sample function
    //this prevents us from running out of DMA channels which causes the program to stop
    dma_chan = dma_claim_unused_channel(true);

    //ADC init
    adc_gpio_init(26 + CAPTURE_CHANNEL);
    adc_gpio_init(26 + CAPTURE_CHANNEL2);

    adc_init();
    //sets the ADC to cycle between reading A0 and A1 (00000011 = 3)
    adc_set_round_robin(3);
    adc_fifo_setup(
        true,    // Write each completed conversion to the sample FIFO
        true,    // Enable DMA data request (DREQ)
        1,       // DREQ (and IRQ) asserted when at least 1 sample present
        false,   // We won't see the ERR bit because of 8 bit reads; disable.
        true     // Shift each sample to 8 bits when pushing to FIFO
    );

   //sets sampling speed
   //speed is 48MHz/1200 (CPUfreq)/(clkdiv) = 80,000
   //since we are sampling two channels round robin, each channel gets sampled at 40khz
    adc_set_clkdiv(600);

    //calculating FFT frequency bins
    //while the ADC is sampling at 80khz, the effective per channel
    //sampling frequency is 40000
    float f_max = 40000;
    float f_res = f_max / CAPTURE_DEPTH/2;
    for (int i = 0; i < CAPTURE_DEPTH; i++) {freqs[i] = f_res*i;}
}



    





