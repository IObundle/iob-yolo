//import custom libraries
#include "system.h"
#include "iob-uart.h"
#include "iob-eth.h"
#include "iob_timer.h"
#include "firmware.h"
#include "iob-cache.h"
#include "versat.hpp"

//import c libraries
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

//define tiling of each layer
#define LAYER_1_TILING_W 32 //must be divisor of 416
#define LAYER_1_TILING_H 32
#define LAYER_3_TILING_W 52 //must be divisor of 208
#define LAYER_3_TILING_H 2
#define LAYER_5_TILING_W 26 //must be divisor of 104
#define LAYER_5_TILING_H 2
#define LAYER_7_TILING_W 4  //must be divisor of 52
#define LAYER_7_TILING_H 4
#define LAYER_9_TILING_W 2  //must be divisor of 26
#define LAYER_9_TILING_H 2
#define LAYER_11_TILING_W 1  //must be divisor of 13
#define LAYER_11_TILING_H 1

//define peripheral base addresses
#define UART (UART_BASE<<(DATA_W-N_SLAVES_W))
#define ETHERNET (ETHERNET_BASE<<(ADDR_W-N_SLAVES_W))
#define TIMER (TIMER_BASE<<(ADDR_W-N_SLAVES_W))
#define DDR_MEM (CACHE_BASE<<(ADDR_W-N_SLAVES_W))
#define VERSAT (VERSAT_BASE<<(ADDR_W-N_SLAVES_W))

//Constants for image resize
#define w_scale ((float)(IMG_W-1)/(NEW_W-1))
#define h_scale ((float)(IMG_H-1)/(NEW_H-1))

//define general constants
#define ETH_NBYTES (1024-18) //minimum ethernet payload excluding FCS
#define INPUT_FILE_SIZE IMAGE_INPUT //8 bits per pixel
#define NUM_INPUT_FRAMES (INPUT_FILE_SIZE/ETH_NBYTES)
#define WEIGHTS_FILE_SIZE (TOTAL_WEIGHTS*2) //16 bits per weight
#define NUM_WEIGHT_FRAMES (WEIGHTS_FILE_SIZE/ETH_NBYTES)
#define OUTPUT_FILE_SIZE (DATA_LAYER_12*2) //16 bits per output
#define NUM_OUTPUT_FRAMES (OUTPUT_FILE_SIZE/ETH_NBYTES)
#define ix_size (NEW_W*4)
#define iy_size (NEW_H)
#define dx_size (NEW_W*2)
#define dy_size (NEW_H*2)

//define DDR mapping
#define ix_BASE_ADDRESS (DDR_MEM + (int)pow(2,MAINRAM_ADDR_W)) //after main mem
#define iy_BASE_ADDRESS (ix_BASE_ADDRESS + ix_size*2) //16 bits
#define dx_BASE_ADDRESS (iy_BASE_ADDRESS + iy_size*2) //16 bits
#define dy_BASE_ADDRESS (dx_BASE_ADDRESS + dx_size*2) //16 bits
#define WEIGHTS_BASE_ADDRESS (dy_BASE_ADDRESS + dy_size*2) //16 bits
#define DATA_BASE_ADDRESS (WEIGHTS_BASE_ADDRESS + WEIGHTS_FILE_SIZE) //16 bits

//ETHERNET variables
int rcv_timeout = 5000;
char data_to_send[ETH_NBYTES];
char data_rcv[ETH_NBYTES+18];
unsigned int bytes_to_receive, bytes_to_send, count_bytes;

//TIMER variables
unsigned int start, end, total_time;

//data base address pointers
int16_t * fp_data, * fp_image, * fp_weights;
int16_t * ix, * iy, * dx, * dy;

//weights and data updatable pointers
unsigned int weight_pos = 0, data_pos = NETWORK_INPUT_AUX;

//define base address of data pointers
void define_memory_regions() {

  //image
  fp_image = (int16_t *) DATA_BASE_ADDRESS;

  //data
  fp_data = (int16_t *) (DATA_BASE_ADDRESS + 2*IMAGE_INPUT);

  //resize
  ix = (int16_t *) ix_BASE_ADDRESS;
  iy = (int16_t *) iy_BASE_ADDRESS;
  dx = (int16_t *) dx_BASE_ADDRESS;
  dy = (int16_t *) dy_BASE_ADDRESS;

  //weights
  fp_weights = (int16_t *) WEIGHTS_BASE_ADDRESS;
}

//reset certain DDR positions to zero due to padding
void reset_DDR() {
  
  //local variables
  unsigned int i, pos;

  //measure initial time
  uart_printf("\nSetting DDR positions to zero\n");
  start = timer_get_count_us(TIMER);

  //resized image
  for(i = 0; i < IMAGE_INPUT; i++) fp_image[i] = 0;
  pos = NETWORK_INPUT_AUX;

  //input network
  for(i = 0; i < NETWORK_INPUT; i++) fp_data[pos + i] = 0;
  pos += NETWORK_INPUT;

  //layer 2
  for(i = 0; i < DATA_LAYER_2; i++) fp_data[pos + i] = 0;
  pos += DATA_LAYER_2;

  //layer 4
  for(i = 0; i < DATA_LAYER_4; i++) fp_data[pos + i] = 0;
  pos += DATA_LAYER_4;

  //layer 6
  for(i = 0; i < DATA_LAYER_6; i++) fp_data[pos + i] = 0;
  pos += DATA_LAYER_6;

  //layer 8
  for(i = 0; i < DATA_LAYER_8; i++) fp_data[pos + i] = 0;
  pos += DATA_LAYER_8;

  //layer 9
  for(i = 0; i < DATA_LAYER_9; i++) fp_data[pos + i] = 0;
  pos += DATA_LAYER_9;

  //layer 10
  for(i = 0; i < DATA_LAYER_10; i++) fp_data[pos + i] = 0;
  pos += DATA_LAYER_10;

  //layer 11
  for(i = 0; i < DATA_LAYER_11; i++) fp_data[pos + i] = 0x8000; //min value
  pos += DATA_LAYER_11;

  //layer 12
  for(i = 0; i < DATA_LAYER_12; i++) fp_data[pos + i] = 0; //min value

  //measure final time
  end = timer_get_count_us(TIMER);
  uart_printf("DDR reset to zero done in %d ms\n", (end-start)/1000);
}

void rcv_frame(unsigned int NUM_DATA_FRAMES, unsigned int DATA_SIZE, char * data_p) {

  //Local variables
  int i, j;
  count_bytes = 0;

  //Loop to receive intermediate data frames
  for(j = 0; j < NUM_DATA_FRAMES+1; j++) {

     //wait to receive frame
     while(eth_rcv_frame(data_rcv, ETH_NBYTES+18, rcv_timeout) !=0);

     // start timer
     if(j == 0) start = timer_get_count_us(TIMER);   

     //check if it is last packet (has less data that full payload size)
     if(j == NUM_DATA_FRAMES) bytes_to_receive = DATA_SIZE - count_bytes;
     else bytes_to_receive = ETH_NBYTES;

     //save in DDR
     for(i = 0; i < bytes_to_receive; i++) {
       if(DATA_SIZE == INPUT_FILE_SIZE) data_p[j*ETH_NBYTES*2 + i*2] = data_rcv[14+i];
       else data_p[j*ETH_NBYTES + i] = data_rcv[14+i];
       data_to_send[i] = data_rcv[14+i];
     }

     //send data back as ack
     eth_send_frame(data_to_send, ETH_NBYTES);

     //update byte counter
     count_bytes += ETH_NBYTES;
  }
}

//receive input image
void receive_data() {

  //Receive input image
  uart_printf("\nReady to receive input image and weights...\n");
  char * fp_image_char = (char *) DATA_BASE_ADDRESS;
  rcv_frame(NUM_INPUT_FRAMES, INPUT_FILE_SIZE, fp_image_char);
  end = timer_get_count_us(TIMER);
  uart_printf("Image received in %d ms\n", (end-start)/1000);

  //Receive weights
  char * fp_weights_char = (char *) WEIGHTS_BASE_ADDRESS;
  rcv_frame(NUM_WEIGHT_FRAMES, WEIGHTS_FILE_SIZE, fp_weights_char);
  end = timer_get_count_us(TIMER);
  uart_printf("weights transferred in %d ms\n", (end-start)/1000);
}

//fill resized image region with grey (0.5 = 0x0080 in Q8.8)
void fill_grey() {
  int i, j, k;
  for(i = 0; i < IMG_C; i++)
    for(j = 0; j < YOLO_INPUT; j++)
      for(k = 0; k < YOLO_INPUT; k++)
	fp_data[i*(YOLO_INPUT+2)*(YOLO_INPUT+2) + (j+1)*(YOLO_INPUT+2) + (k+1) + NETWORK_INPUT_AUX] = 0x0080;
}

//initializes ix, iy, dx and dy variables
void prepare_resize() {

  //local variables
  int i, val_i;
  float val, val_d;

  //loop to initialize ix and dx
  for(i = 0; i < NEW_W; i++) {
    val = i*w_scale;
    val_i = (int) val;
    val_d = val - val_i;
    //for iy  
    ix[2*i] = val_i;
    ix[2*i+1] = val_i + 1;
    //for iy+1
    ix[2*i+2*NEW_W] = val_i + IMG_W;
    ix[2*i+1+2*NEW_W] = val_i + 1 + IMG_W;
    //dx
    dx[2*i] = (int16_t)((1-val_d)*((int16_t)1<<14)); //Q2.14
    dx[2*i+1] = (int16_t)(val_d*((int16_t)1<<14)); //Q2.14
  }

  //loop to initialize iy and dy
  for(i = 0; i < NEW_H; i++) {
    val = i*h_scale;
    iy[i] = (int) val;
    val_d = val - iy[i];
    //dy
    dy[2*i] = (int16_t)((1-val_d)*((int16_t)1<<14)); //Q2.14
    dy[2*i+1] = (int16_t)(val_d*((int16_t)1<<14)); //Q2.14
  }
}

//resize input image to 416x312x3
void resize_image() {

  //local variables
  int i, j, k;
  unsigned int data_time, config_time, run_time = 0;
#ifdef SIM
  int num_err = 0;
#endif

  ///////////////////////////////////////////////////////////////////////////////
  //     1st step -> IMG_WxIMG_HxIMG_C to (NEW_W*2)xNEW_H*IMG_C
  ///////////////////////////////////////////////////////////////////////////////
  
  //load ix and dx to versat mem0 and mem2
  start = timer_get_count_us(TIMER);
  for(i = 0; i < ix_size; i++) stage[0].memA[0].write(i, ix[i]);
  for(i = 0; i < dx_size; i++) stage[0].memA[2].write(i, dx[i]);
  end = timer_get_count_us(TIMER);
  data_time = end - start;

  //configure mem0 to read: ix, ix+1, ix+2*NEW_W, ix+2*NEW_W+1 sequence
  start = timer_get_count_us(TIMER);
  stage[0].memA[0].setDuty(2);
  stage[0].memA[0].setPer(2);
  stage[0].memA[0].setIncr(1);
  stage[0].memA[0].setIter(2);
  stage[0].memA[0].setShift(2*NEW_W-2);
  stage[0].memA[0].setPer2(NEW_W);
  stage[0].memA[0].setIncr2(2);
  stage[0].memA[0].setIter2(1);

  //configure mem1 to read input pixels (addressed by mem0)
  stage[0].memA[1].setDelay(MEMP_LAT);
  stage[0].memA[1].setDuty(4*NEW_W);
  stage[0].memA[1].setPer(4*NEW_W);
  stage[0].memA[1].setIter(1);
  stage[0].memA[1].setSel(sMEMA[0]);
  stage[0].memA[1].setExt(1);

  //configure mem2 to read 1-dx, dx sequence twice
  stage[0].memA[2].setDuty(2);
  stage[0].memA[2].setDelay(MEMP_LAT);
  stage[0].memA[2].setPer(2);
  stage[0].memA[2].setIncr(1);
  stage[0].memA[2].setIter(2);
  stage[0].memA[2].setShift(-2);
  stage[0].memA[2].setPer2(NEW_W);
  stage[0].memA[2].setIncr2(2);
  stage[0].memA[2].setIter2(1);

  //pixel * 1-dx/dx = res0
  stage[0].yolo[0].setSelA(sMEMA[1]);
  stage[0].yolo[0].setSelB(sMEMA[2]);
  stage[0].yolo[0].setIter(2*NEW_W);
  stage[0].yolo[0].setPer(2);
  stage[0].yolo[0].setDelay(2*MEMP_LAT);
  stage[0].yolo[0].setShift(7); //Q10.22 to Q1.15

  //store res0 in mem3
  stage[0].memA[3].setDelay(2*MEMP_LAT+YOLO_LAT+(2-1));
  stage[0].memA[3].setDuty(1);
  stage[0].memA[3].setPer(2);
  stage[0].memA[3].setIncr(1);
  stage[0].memA[3].setIter(2*NEW_W);
  stage[0].memA[3].setSel(sYOLO[0]);
  stage[0].memA[3].setInWr(1);
  end = timer_get_count_us(TIMER);
  config_time = end - start;

  //loops for performing resizing 1st step
  #ifdef SIM
  for(k = 0; k < 1; k++) {
    for(j = 0; j < 1; j++) {
#else
  for(k = 0; k < IMG_C; k++) {
    for(j = 0; j < NEW_H; j++) {
#endif

      //Store 2 lines of pixels in mem1
      //Extra pixel is necessary to be multiplied by zero
      start = timer_get_count_us(TIMER);
      for(i = 0; i < IMG_W*2+1; i++)
        stage[0].memA[1].write(i, fp_image[k*IMG_W*IMG_H + iy[j]*IMG_W + i]);
      end = timer_get_count_us(TIMER);
      data_time += (end - start);

      //Wait until done
      start = timer_get_count_us(TIMER);
      run();
      while(done() == 0);
      end = timer_get_count_us(TIMER);
      run_time += (end - start);

      //store result in DDR
      start = timer_get_count_us(TIMER);
      for(i = 0; i < NEW_W*2; i++)
      #ifdef SIM
        if(fp_data[k*2*NEW_W*NEW_H + j*2*NEW_W + i] != stage[0].memA[3].read(i)) num_err++;
      #else
        fp_data[k*2*NEW_W*NEW_H + j*2*NEW_W + i] = stage[0].memA[3].read(i);
      #endif
      end = timer_get_count_us(TIMER);
      data_time += (end - start);
    }
  }
#ifdef SIM
  uart_printf("Resize 1st step done with %d errors\n", num_err);
  num_err = 0;
#endif

  ///////////////////////////////////////////////////////////////////////////////
  //     2nd step -> (NEW_W*2)xNEW_H*IMG_C to NEW_W*NEW_H*IMG_C
  ///////////////////////////////////////////////////////////////////////////////
  
  //clear configuration of all stages
  globalClearConf();

  //load dy to versat mem1
  start = timer_get_count_us(TIMER);
  for(i = 0; i < dy_size; i++) stage[0].memA[1].write(i, dy[i]);
  end = timer_get_count_us(TIMER);
  data_time += end - start;

  //configure mem0 to read res0
  start = timer_get_count_us(TIMER);
  stage[0].memA[0].setDuty(2);
  stage[0].memA[0].setPer(2);
  stage[0].memA[0].setIncr(1);
  stage[0].memA[0].setIter(NEW_W);

  //configure mem1 to read 1-dy, dy
  stage[0].memA[1].setDuty(2);
  stage[0].memA[1].setPer(2);
  stage[0].memA[1].setIncr(1);
  stage[0].memA[1].setIter(NEW_W);
  stage[0].memA[1].setShift(-2);

  //pixel * 1-dy/dy = res1
  stage[0].yolo[0].setSelA(sMEMA[0]);
  stage[0].yolo[0].setSelB(sMEMA[1]);
  stage[0].yolo[0].setIter(NEW_W);
  stage[0].yolo[0].setPer(2);
  stage[0].yolo[0].setDelay(MEMP_LAT);
  stage[0].yolo[0].setShift(21); //Q3.29 to Q8.8

  //store res1 in mem3
  stage[0].memA[3].setDelay(MEMP_LAT+YOLO_LAT+(2-1));
  stage[0].memA[3].setDuty(1);
  stage[0].memA[3].setPer(2);
  stage[0].memA[3].setIncr(1);
  stage[0].memA[3].setIter(NEW_W);
  stage[0].memA[3].setSel(sYOLO[0]);
  stage[0].memA[3].setInWr(1);
  end = timer_get_count_us(TIMER);
  config_time += end - start;

  //loops for performing resizing 2nd step
  #ifdef SIM
  for(k = 0; k < 1; k++) {
    for(j = 0; j < 1; j++) {
#else
  for(k = 0; k < IMG_C; k++) {
    for(j = 0; j < NEW_H; j++) {
#endif

      //Store res1 in mem0
      start = timer_get_count_us(TIMER);
      for(i = 0; i < 2*NEW_W; i++)
        stage[0].memA[0].write(i, fp_data[k*2*NEW_W*NEW_H + j*2*NEW_W + i]);
      end = timer_get_count_us(TIMER);
      data_time += (end - start);

      //run
      start = timer_get_count_us(TIMER);
      run();

      //configure mem1 start
      #ifndef SIM
      if(j == NEW_H-1)
        stage[0].memA[0].setStart(0);
      else
        stage[0].memA[1].setStart(2*(j+1));
    #endif

      //Wait until done
      while(done() == 0);
      end = timer_get_count_us(TIMER);
      run_time += (end - start);

      //store result in DDR
      start = timer_get_count_us(TIMER);
      for(i = 0; i < NEW_W; i++)
      #ifdef SIM
        if(fp_data[k*(YOLO_INPUT+2)*(YOLO_INPUT+2) + (j+1)*(YOLO_INPUT+2) + (i+1) + EXTRA_W + ((YOLO_INPUT+2)*EXTRA_H) + NETWORK_INPUT_AUX] != stage[0].memA[3].read(i)) num_err++;
      #else
        fp_data[k*(YOLO_INPUT+2)*(YOLO_INPUT+2) + (j+1)*(YOLO_INPUT+2) + (i+1) + EXTRA_W + ((YOLO_INPUT+2)*EXTRA_H) + NETWORK_INPUT_AUX] = stage[0].memA[3].read(i);
      #endif
      end = timer_get_count_us(TIMER);
      data_time += (end - start);
    }
  }
#ifdef SIM
  uart_printf("Resize 2nd step done with %d errors\n", num_err);
#endif
  uart_printf("FU config time = %d us\n", config_time);
  uart_printf("FU run time = %d us\n", run_time);
  uart_printf("FU load/store data time = %d us\n", data_time);
  uart_printf("Total time = %d ms\n", (config_time + run_time + data_time)/1000);
  total_time = (config_time + run_time + data_time)/1000;
}

//perform convolutional layer
void conv_layer(int w, int c, int num_ker, int ker_size, int til_w, int til_h, int maxpool, int outpadd, int stride) {

  //clear configuration of all stages
  globalClearConf();

  //update pointers
  int16_t * input = (int16_t *) fp_data + data_pos;
  data_pos += (w+2)*(w+2)*c;
  int16_t * output = (int16_t *) fp_data + data_pos;
  int16_t * weights = (int16_t *) fp_weights + weight_pos;
  weight_pos += num_ker*ker_size*ker_size*c;
  int16_t * bias = (int16_t *) fp_weights + weight_pos;
  weight_pos += num_ker;

  //local variables
  int i, j, k, l, m;
#ifdef SIM
  int num_err = 0;
#endif
  unsigned int data_time = 0, config_time, run_time = 0;

  //configure mem0 to read ker_size*ker_size*c blocks from tiled FM
  start = timer_get_count_us(TIMER);
  stage[0].memA[0].setDuty(ker_size*c);
  stage[0].memA[0].setPer(ker_size*c);
  stage[0].memA[0].setIncr(1);
  stage[0].memA[0].setIter(ker_size);
  stage[0].memA[0].setShift((til_w+2)*c-ker_size*c);
  stage[0].memA[0].setIncr2(c);
  if(maxpool) {
    stage[0].memA[0].setPer2(2);
    stage[0].memA[0].setIter2(2);
    stage[0].memA[0].setShift2((til_w+2)*c-2*c);
    stage[0].memA[0].setPer3(til_w/2);
    stage[0].memA[0].setIncr3(2*c);
    stage[0].memA[0].setIter3(til_h/2);
    stage[0].memA[0].setShift3((til_w+2)*c*2-til_w*c);
  } else {
    stage[0].memA[0].setPer2(til_w);
    stage[0].memA[0].setIter2(til_h);
    stage[0].memA[0].setShift2((til_w+2)*c-til_w*c);
  }

  //configure mem1 to read weights
  stage[0].memA[1].setDuty(ker_size*ker_size*c);
  stage[0].memA[1].setPer(ker_size*ker_size*c);
  stage[0].memA[1].setIncr(1);
  stage[0].memA[1].setIter(til_w*til_h);
  stage[0].memA[1].setShift(-ker_size*ker_size*c);

  //configure mem2 to read bias
  stage[0].memA[2].setIter(til_w*til_h);
  stage[0].memA[2].setPer(ker_size*ker_size*c);

  //configure yolo0 to perform convolutions
  stage[0].yolo[0].setSelA(sMEMA[0]);
  stage[0].yolo[0].setSelB(sMEMA[1]);
  stage[0].yolo[0].setSelC(sMEMA[2]);
  stage[0].yolo[0].setIter(til_w*til_h);
  stage[0].yolo[0].setPer(ker_size*ker_size*c);
  stage[0].yolo[0].setDelay(MEMP_LAT);
  stage[0].yolo[0].setBias(1);
  stage[0].yolo[0].setShift(10);
  stage[0].yolo[0].setLeaky(1);
  stage[0].yolo[0].setMaxpool(maxpool);

  //configure mem3 to write convolution results
  stage[0].memA[3].setDuty(1);
  stage[0].memA[3].setIncr(1);
  if(maxpool) {
    stage[0].memA[3].setPer(ker_size*ker_size*c*4); //CAREFUL WITH PERIOD_W!!!
    stage[0].memA[3].setIter(til_w*til_h/4);
    stage[0].memA[3].setDelay(MEMP_LAT+YOLO_LAT+ker_size*ker_size*c*4-1);
  } else {
    stage[0].memA[3].setPer(ker_size*ker_size*c);
    stage[0].memA[3].setIter(til_w*til_h);
    stage[0].memA[3].setDelay(MEMP_LAT+YOLO_LAT+ker_size*ker_size*c-1);
  }
  stage[0].memA[3].setSel(sYOLO[0]);
  stage[0].memA[3].setInWr(1);
  end = timer_get_count_us(TIMER);
  config_time = end - start;

  //loops for performing convolution
#ifdef SIM
  for(l = 0; l < 1; l++) {
    for(m = 0; m < 1; m++) {
#else
  for(l = 0; l < w/til_h; l++) {
    for(m = 0; m < w/til_w; m++) {
#endif

      //Send input FM tile (z-x-y format)
      start = timer_get_count_us(TIMER);
      for(i = 0; i < c; i++)
        for(j = 0; j < til_h+2; j++)
          for(k = 0; k < til_w+2; k++)
            stage[0].memA[0].write((j*(til_w+2)+k)*c + i, input[i*(w+2)*(w+2) + (l*til_h+j)*(w+2) + (k+m*til_w)]);
      end = timer_get_count_us(TIMER);
      data_time += (end - start);

    #ifdef SIM
      for(k = 0; k < 1; k++) {
    #else
      for(k = 0; k < num_ker; k++) {
    #endif

        //load weights (z-x-y format)
        start = timer_get_count_us(TIMER);
        for(j = 0; j < c; j++)
          for(i = 0; i < ker_size*ker_size; i++)
            stage[0].memA[1].write(i*c + j, weights[k*c*ker_size*ker_size + j*ker_size*ker_size + i]);
        stage[0].memA[2].write(0, bias[k]);
        end = timer_get_count_us(TIMER);
        data_time += end - start;

        //run
        start = timer_get_count_us(TIMER);
        run();

        //Wait until done
        while(done() == 0);
        end = timer_get_count_us(TIMER);
        run_time += (end - start);

        //store result in DDR
        start = timer_get_count_us(TIMER);
	for(i = 0; i < til_h/(1+maxpool); i++)
          for(j = 0; j < til_w/(1+maxpool); j++)
          #ifdef SIM
            if(output[k*(w/(1+maxpool)+2*outpadd+stride)*(w/(1+maxpool)+2*outpadd+stride) + ((til_h/(1+maxpool))*l+i+outpadd)*(w/(1+maxpool)+2*outpadd+stride) + ((til_w/(1+maxpool))*m+j+outpadd)] != stage[0].memA[3].read(i*(til_w/(1+maxpool))+j)) num_err++;
          #else
            output[k*(w/(1+maxpool)+2*outpadd+stride)*(w/(1+maxpool)+2*outpadd+stride) + ((til_h/(1+maxpool))*l+i+outpadd)*(w/(1+maxpool)+2*outpadd+stride) + ((til_w/(1+maxpool))*m+j+outpadd)] = stage[0].memA[3].read(i*(til_w/(1+maxpool))+j);
          #endif

        //end measuring data load/store time
        end = timer_get_count_us(TIMER);
        data_time += (end - start);
      }
    }
  }
#ifdef SIM
  uart_printf("Layer done with %d errors\n", num_err);
#endif
  uart_printf("FU config time = %d us\n", config_time);
  uart_printf("FU run time = %d us\n", run_time);
  uart_printf("FU load/store data time = %d us\n", data_time);
  uart_printf("Total time = %d ms\n", (config_time + run_time + data_time)/1000);
  total_time += (config_time + run_time + data_time)/1000;
}

//perform maxpool layer
//when inpadd = 1, it does downsampling
void maxpool_layer(int w, int c, int inpadd, int stride) {

  //clear configuration of all stages
  globalClearConf();

  //update pointers
  int16_t * input = (int16_t *) fp_data + data_pos;
  data_pos += (w+2*inpadd+stride)*(w+2*inpadd+stride)*c;
  int16_t * output = (int16_t *) fp_data + data_pos;

  //local variables
  int i, j, k, l;
#ifdef SIM
  int num_err = 0;
#endif
  unsigned int data_time = 0, config_time, run_time = 0;

  //configure mem0 to read input FMs
  start = timer_get_count_us(TIMER);
  stage[0].memA[0].setDuty(2);
  stage[0].memA[0].setPer(2);
  stage[0].memA[0].setIncr(1);
  stage[0].memA[0].setIter(2);
  stage[0].memA[0].setShift(w+stride-2);
  stage[0].memA[0].setPer2(w/(1+inpadd));
  stage[0].memA[0].setIncr2(1+inpadd);
  stage[0].memA[0].setIter2(w/(1+inpadd));
  stage[0].memA[0].setShift2(w+stride-w*stride);

  //configure yolo0 to perform maxpool
  stage[0].yolo[0].setSelA(sMEMA[0]);
  stage[0].yolo[0].setIter(w*(1+stride));
  stage[0].yolo[0].setPer(w*(1+stride));
  stage[0].yolo[0].setDelay(MEMP_LAT);
  stage[0].yolo[0].setMaxpool(1);
  stage[0].yolo[0].setBypass(1);

  //configure mem3 to write maxpool results
  stage[0].memA[3].setDuty(1);
  stage[0].memA[3].setIncr(1);
  stage[0].memA[3].setPer(4);
  stage[0].memA[3].setIter(w*w/(1+3*inpadd));
  stage[0].memA[3].setDelay(MEMP_LAT+YOLO_BYPASS_LAT+4-1);
  stage[0].memA[3].setSel(sYOLO[0]);
  stage[0].memA[3].setInWr(1);
  end = timer_get_count_us(TIMER);
  config_time = end - start;

#ifdef SIM
  for(k = 0; k < 1; k++) {
#else
  for(k = 0; k < c; k++) {
#endif

    //Send input FM tile (x-y-z format)
    start = timer_get_count_us(TIMER);
    for(j = 0; j < w+stride; j++)
      for(i = 0; i < w+stride; i++)
        stage[0].memA[0].write(j*(w+stride) + i, input[k*(w+2*inpadd+stride)*(w+2*inpadd+stride) + (j+inpadd)*(w+2*inpadd+stride) + i+inpadd]);
    end = timer_get_count_us(TIMER);
    data_time += (end - start);

    //run
    start = timer_get_count_us(TIMER);
    run();

    //Wait until done
    while(done() == 0);
    end = timer_get_count_us(TIMER);
    run_time += (end - start);

    //store result in DDR
    start = timer_get_count_us(TIMER);
    for(j = 0; j < w/(1+inpadd); j++)
      for(i = 0; i < w/(1+inpadd); i++)
      #ifdef SIM
        if(output[k*(w/(1+inpadd)+2)*(w/(1+inpadd)+2) + (j+1)*(w/(1+inpadd)+2) + i+1] != stage[0].memA[3].read(j*(w/(1+inpadd)) + i)) num_err++;
      #else
        output[k*(w/(1+inpadd)+2)*(w/(1+inpadd)+2) + (j+1)*(w/(1+inpadd)+2) + i+1] = stage[0].memA[3].read(j*(w/(1+inpadd)) + i);
      #endif
    end = timer_get_count_us(TIMER);
    data_time += (end - start);
  }
#ifdef SIM
  uart_printf("Layer done with %d errors\n", num_err);
#endif
  uart_printf("FU config time = %d us\n", config_time);
  uart_printf("FU run time = %d us\n", run_time);
  uart_printf("FU load/store data time = %d us\n", data_time);
  uart_printf("Total time = %d ms\n", (config_time + run_time + data_time)/1000);
  total_time += (config_time + run_time + data_time)/1000;
}

//send detection results back
void send_data() {

  //Loop to send output of yolo layer
  int i, j;
  count_bytes = 0;
  char * fp_data_char = (char *) fp_image + (IMAGE_INPUT + NETWORK_INPUT_AUX + NETWORK_INPUT + DATA_LAYER_2 + DATA_LAYER_4 + DATA_LAYER_6 + DATA_LAYER_8 + DATA_LAYER_9 + DATA_LAYER_10 + DATA_LAYER_11)*2;
  for(j = 0; j < NUM_OUTPUT_FRAMES+1; j++) {

     // start timer
     if(j == 0) start = timer_get_count_us(TIMER);

     //check if it is last packet (has less data that full payload size)
     if(j == NUM_OUTPUT_FRAMES) bytes_to_send = OUTPUT_FILE_SIZE - count_bytes;
     else bytes_to_send = ETH_NBYTES;

     //prepare variable to be sent
     for(i = 0; i < bytes_to_send; i++) data_to_send[i] = fp_data_char[j*ETH_NBYTES + i];

     //send frame
     eth_send_frame(data_to_send, ETH_NBYTES);

     //wait to receive frame as ack
     if(j != NUM_OUTPUT_FRAMES) while(eth_rcv_frame(data_rcv, ETH_NBYTES+18, rcv_timeout) !=0);

     //update byte counter
     count_bytes += ETH_NBYTES;
  }

  //measure transference time
  end = timer_get_count_us(TIMER);
  uart_printf("\noutput layer transferred in %d ms\n\n", (end-start)/1000);
}

int main(int argc, char **argv) {

  //init UART
  uart_init(UART,UART_CLK_FREQ/UART_BAUD_RATE);

  //send init message
  uart_printf("\nYOLO HW FULL\n\n");
  uart_txwait();

#ifndef SIM
  //init ETHERNET
  eth_init(ETHERNET);
  eth_set_rx_payload_size(ETH_NBYTES);
#endif

  //init VERSAT
  versat_init(VERSAT);

  //define memory regions
  define_memory_regions();

  //load data and reset DDR to zero
#ifndef SIM
  reset_DDR();
  receive_data();
  fill_grey();

  //initialize ix, iy, dx and dy arrays
  prepare_resize();

  //resize input image
  uart_printf("\nResizing input image...\n");
  resize_image();

  //layer1,2 (conv + maxpool)
  uart_printf("\nRunning layers 1 and 2...\n");
  conv_layer(YOLO_INPUT, IMG_C, LAYER_1_NUM_KER, LAYER_1_KER_SIZE, LAYER_1_TILING_W, LAYER_1_TILING_H, LAYER_1_MAXPOOL, LAYER_1_OUTPADD, LAYER_1_STRIDE);

  //layer3,4 (conv + maxpool)
  uart_printf("\nRunning layers 3 and 4...\n");
  conv_layer(LAYER_3_W, LAYER_1_NUM_KER, LAYER_3_NUM_KER, LAYER_3_KER_SIZE, LAYER_3_TILING_W, LAYER_3_TILING_H, LAYER_3_MAXPOOL, LAYER_3_OUTPADD, LAYER_3_STRIDE);

  //layer5,6 (conv + maxpool)
  uart_printf("\nRunning layers 5 and 6...\n");
  conv_layer(LAYER_5_W, LAYER_3_NUM_KER, LAYER_5_NUM_KER, LAYER_5_KER_SIZE, LAYER_5_TILING_W, LAYER_5_TILING_H, LAYER_5_MAXPOOL, LAYER_5_OUTPADD, LAYER_5_STRIDE);

  //layer7,8 (conv + maxpool)
  uart_printf("\nRunning layers 7 and 8...\n");
  conv_layer(LAYER_7_W, LAYER_5_NUM_KER, LAYER_7_NUM_KER, LAYER_7_KER_SIZE, LAYER_7_TILING_W, LAYER_7_TILING_H, LAYER_7_MAXPOOL, LAYER_7_OUTPADD, LAYER_7_STRIDE);

  //layer9 (conv)
  uart_printf("\nRunning layer 9...\n");
  conv_layer(LAYER_9_W, LAYER_7_NUM_KER, LAYER_9_NUM_KER, LAYER_9_KER_SIZE, LAYER_9_TILING_W, LAYER_9_TILING_H, LAYER_9_MAXPOOL, LAYER_9_OUTPADD, LAYER_9_STRIDE);

  //layer10 (maxpool)
  uart_printf("\nRunning layer 10...\n");
  maxpool_layer(LAYER_9_W, LAYER_9_NUM_KER, LAYER_10_INPADD, LAYER_10_STRIDE);

  //layer11 (conv)
  uart_printf("\nRunning layer 11...\n");
  conv_layer(LAYER_11_W, LAYER_9_NUM_KER, LAYER_11_NUM_KER, LAYER_11_KER_SIZE, LAYER_11_TILING_W, LAYER_11_TILING_H, LAYER_11_MAXPOOL, LAYER_11_OUTPADD, LAYER_11_STRIDE);
#else
  weight_pos += WEIGHTS_LAYER_1 + WEIGHTS_LAYER_3 + WEIGHTS_LAYER_5 + WEIGHTS_LAYER_7 + WEIGHTS_LAYER_9 + WEIGHTS_LAYER_11;
  data_pos += NETWORK_INPUT + DATA_LAYER_2 + DATA_LAYER_4 + DATA_LAYER_6 + DATA_LAYER_8 + DATA_LAYER_9 + DATA_LAYER_10;
#endif

  //layer12 (maxpool)
  uart_printf("\nRunning layer 12...\n");
  maxpool_layer(LAYER_11_W, LAYER_11_NUM_KER, LAYER_12_INPADD, LAYER_12_STRIDE);

  //return data
  uart_printf("\nTotal execution time = %d seconds\n", total_time/1000);
#ifndef SIM
  send_data();
#endif
  uart_putc(4);
  return 0;
}
