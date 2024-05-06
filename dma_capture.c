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

//for programming of ADF4159
uint32_t R0 =  0x801CE000;
uint32_t R1 =  1;
uint32_t R2 =  0x800A;
uint32_t R3 =  0x843;
uint32_t R4_1 =  0x843;
uint32_t R4_2 =  0x843;
uint32_t R5_1 =  0x200015;
uint32_t R5_2 =  0x800005;
uint32_t R6_1 =  0x7FFFFE;
uint32_t R6_2 =  0x800006;
uint32_t R7 =  0x7;

#define CLOCK 15
#define DATA 13
#define LE 12
#define CE 14
#define TX 16
#define TEST 10

uint32_t BITMASK = (0x80000000);

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

//globals
uint8_t capture_buf[CAPTURE_DEPTH];
float freqs[CAPTURE_DEPTH/2];
float power[CAPTURE_DEPTH/2];

uint dma_chan;

float mean;
float phase1;
float phase2;
float max_freqCW;
int max_idx;
float max_power;
float phaseDif;


//functions
void setup();
void sample();
float standardDev();
void loadData();


int main(void) {
    stdio_init_all();
    //wait to allow time for board power to be connected
    //ideally replace this with waiting for a message over usb before beginning setup
    sleep_ms(100);

    //printf("starting setup\n");
    setup();
    

    kiss_fft_scalar fft_in1[CAPTURE_DEPTH/2];
    kiss_fft_scalar fft_in2[CAPTURE_DEPTH/2];
    kiss_fft_cpx fft_out1[CAPTURE_DEPTH/2];
    kiss_fft_cpx fft_out2[CAPTURE_DEPTH/2];
    kiss_fftr_cfg FFTcfg = kiss_fftr_alloc((int)(CAPTURE_DEPTH/2),false,0,0);


    
    //printf("beginning loop\n");
    while(1){
        //printf("beginning sampling 1\n");
        gpio_put(TX, 1);
        sleep_us(1);
        gpio_put(TX, 0);
        sample();
        //since sampling is round robin but results are stored in the same buffer, 
        //every second result in the array belongs to the same sampling set
        for (int i=0;i<(CAPTURE_DEPTH-1);) {
            fft_in1[(int)(i/2)]=(float)capture_buf[i];
            fft_in2[(int)(i/2)]=(float)capture_buf[i+1];
            i = i+2;
        }
        //printf("beginning FFT 1\n");
        // compute fast fourier transform
        kiss_fftr(FFTcfg , fft_in1, fft_out1);
        kiss_fftr(FFTcfg , fft_in2, fft_out2);
        //printf("finished FFT 1\n");

        // Calculating the power in each frequncy bin and determining the dominant frequency
        max_power = 0;
        for (int i = 1; i < (2048); i++) {
            power[i] = fft_out1[i].r*fft_out1[i].r+fft_out1[i].i*fft_out1[i].i;
            if (power[i]>max_power) {
                max_power=power[i];
                max_idx = i;
            } 
        }

        //frequency calcs
        max_freqCW = freqs[max_idx];
        phase1 = atan(fft_out1[max_idx].i/fft_out1[max_idx].r);
        //printf("Greatest Frequency Component C1: %.4f kHz\n",max_freqCW/1000);
        
        

        max_power = 0;
        max_idx = 0;
        for (int i = 1; i < (2048); i++) {
            power[i] = fft_out2[i].r*fft_out2[i].r+fft_out2[i].i*fft_out2[i].i;
            if (power[i]>max_power) {
                max_power=power[i];
                max_idx = i;
            } 
        }
        max_freqCW = freqs[max_idx];
        phase2 = atan(fft_out2[max_idx].i/fft_out2[max_idx].r);
        //printf("Greatest Frequency Component C2: %0.4f kHz\n",max_freqCW/1000);

        //to show difference inherent phase difference caused by sampling style
        if(gpio_get(TEST))
        {
            phaseDif = 57.2957795*(phase2-phase1)  - (max_freqCW*360)/80000;
        }else
        {
            phaseDif = 57.2957795*(phase2-phase1);
        }

        if (phaseDif < 0)
        {
            phaseDif = phaseDif+180;
        }
        //phase shift is induced due to time between samples for each channel.
        //sampling both channels at the same time would be ideal but we'll account for it here instead.

        //printf("phase difference: %0.1f deg\n\n", phaseDif);
        //printf("standard deviation is %.2f \n", standardDev(power));

        //using SD based threshhold for signal power to determine if distinct signal is present

        if((max_power-mean) > (35*standardDev(power)))
        {
            printf("%.2f,%.2f,1\n",max_freqCW,phaseDif);
        }else
        {
            printf("%.2f,%.2f,0\n",max_freqCW,phaseDif);
        }
    }
}
  

void sample()
{
    adc_select_input(CAPTURE_CHANNEL);
    //sleep_ms(10);

    //configuring DMA channel
    dma_channel_config cfg = dma_channel_get_default_config(dma_chan);
    // Reading from constant address, writing to incrementing byte addresses
    channel_config_set_transfer_data_size(&cfg, DMA_SIZE_8);
    channel_config_set_read_increment(&cfg, false);
    channel_config_set_write_increment(&cfg, true);
    channel_config_set_dreq(&cfg, DREQ_ADC);

    dma_channel_configure(dma_chan, &cfg,
        capture_buf,    // dst
        &adc_hw->fifo,  // src
        CAPTURE_DEPTH,  // transfer count
        true            // start immediately
    );
    //gpio_put(TIMING_TEST, 1);
    //printf("Starting capture\n");
    adc_run(true);

    //wait for DMA to collect enough samples
    dma_channel_wait_for_finish_blocking(dma_chan);
    //gpio_put(TIMING_TEST, 0);
    //printf("Capture finished\n");
    //stop the ADC and flush the fifo
    adc_run(false);
    adc_fifo_drain();
    //printf("Exiting sample func\n");
}
void setup()
{
    // claiming the DMA channel here, configuring it in the sample function
    //this prevents us from running out of DMA channels which causes the program to stop
    dma_chan = dma_claim_unused_channel(true);

    //ADC init
    adc_gpio_init(CAPTURE_CHANNEL);
    adc_gpio_init(CAPTURE_CHANNEL2);

    adc_init();
    adc_set_round_robin(6);
    //sets the ADC to cycle between reading A1 and A2 (00000110 = 6)
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
    float f_res = f_max / (CAPTURE_DEPTH/2);
    for (int i = 0; i < (CAPTURE_DEPTH/2); i++) {freqs[i] = f_res*i;}

    gpio_init(CLOCK);
    gpio_init(DATA);
    gpio_init(LE);
    gpio_init(TX);
    gpio_init(CE);
    gpio_init(TEST);

    gpio_set_dir(TX, GPIO_OUT);
    gpio_set_dir(CLOCK, GPIO_OUT);
    gpio_set_dir(DATA, GPIO_OUT);
    gpio_set_dir(LE, GPIO_OUT);
    gpio_set_dir(CE, GPIO_OUT);
    //gpio_set_dir(TEST, GPIO_IN);
    gpio_pull_up(TEST);

    gpio_put(CE, 1);
    gpio_put(LE, 0);
    sleep_ms(10);
    //load ADF4159 registers
    loadData(R7);
    gpio_put(LE, 1);
    sleep_us(1);
    gpio_put(LE, 0);
    sleep_us(100);

    loadData(R6_1);
    gpio_put(LE, 1);
    sleep_us(1);
    gpio_put(LE, 0);
    sleep_us(100);

    loadData(R6_2);
    gpio_put(LE, 1);
    sleep_us(1);
    gpio_put(LE, 0);
    sleep_us(100);

    loadData(R5_1);
    gpio_put(LE, 1);
    sleep_us(1);
    gpio_put(LE, 0);
    sleep_us(100);

    loadData(R5_2);
    gpio_put(LE, 1);
    sleep_us(1);
    gpio_put(LE, 0);
    sleep_us(100);

    loadData(R4_1);
    gpio_put(LE, 1);
    sleep_us(1);
    gpio_put(LE, 0);
    sleep_us(100);

    loadData(R4_2);
    gpio_put(LE, 1);
    sleep_us(1);
    gpio_put(LE, 0);
    sleep_us(100);

    loadData(R3);
    gpio_put(LE, 1);
    sleep_us(1);
    gpio_put(LE, 0);
    sleep_us(100);

    loadData(R2);
    gpio_put(LE, 1);
    sleep_us(1);
    gpio_put(LE, 0);
    sleep_us(100);

    loadData(R1);
    gpio_put(LE, 1);
    sleep_us(1);
    gpio_put(LE, 0);
    sleep_us(100);

    loadData(R0);
    gpio_put(LE, 1);
    sleep_us(1);
    gpio_put(LE, 0);
    sleep_us(100);
    //gpio_put(CE, 1);
}

float standardDev(float data[]) {
    float sum = 0;
    mean = 0;
    float SD = 0;
    int i;
    for (i = 0; i < 2048; i++) {
        sum += data[i];
    }
    mean = sum / 2048;
    for (i = 0; i < 2048; i++) {
        SD += pow(data[i] - mean, 2);
    }
    return sqrt(SD/2048);
}


void loadData(uint32_t R)
{
    //outputs MSB on data, clocks it in, and then shifts MSB out left
    for(int i = 0; i < 32; i++)
    {
        //load data
        gpio_put(DATA, (BITMASK & R));
        sleep_us(1);
        //clock in data
        gpio_put(CLOCK, 1);
        sleep_us(1);
        gpio_put(CLOCK, 0);
        sleep_us(1);
        R = R << 1;
    }
    gpio_put(DATA, 0);
}
    





