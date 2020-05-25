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

//define number of versats used in specific function
#define RESIZE_NSTAGES 2 //must be even and divisor of 312 (e.g. 2, 4, 6, 8, 12, 24, 26, 52)

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
#define OUTPUT_FILE_SIZE (DATA_LAYER_1*2) //16 bits per output
#define NUM_OUTPUT_FRAMES (OUTPUT_FILE_SIZE/ETH_NBYTES)
#define ix_size (NEW_W*4)
#define iy_size (NEW_H)
#define dx_size (NEW_W*2)
#define dy_size (NEW_H*4)

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
unsigned int start, end;

//data base address pointers
int16_t * fp_data, * fp_image, * fp_weights;
int16_t * ix, * iy, * dx, * dy;

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
  unsigned int i;

  //measure initial time
  uart_printf("\nSetting DDR positions to zero\n");
  start = timer_get_count_us(TIMER);

  //resized image
  for(i = 0; i < IMAGE_INPUT; i++) fp_image[i] = 0;

  //input network
  for(i = 0; i < NETWORK_INPUT; i++) fp_data[i] = 0;

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
	fp_data[i*(YOLO_INPUT+2)*(YOLO_INPUT+2) + (j+1)*(YOLO_INPUT+2) + (k+1)] = 0x0080;
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
    dy[4*i] = 0;
    dy[4*i+1] = (int16_t)((1-val_d)*((int16_t)1<<14)); //Q2.14
    dy[4*i+2] = 0;
    dy[4*i+3] = (int16_t)(val_d*((int16_t)1<<14)); //Q2.14
  }
}

void resize_image() {

  //local variables
  uint16_t r, c, k;
  int32_t mul;
  int16_t val_w, next_val_w, val_h;
#ifdef SIM
  int num_err = 0;
#endif

  //perform resize
#ifdef SIM
  for(k = 0; k < 1; k++) {
    for(r = 0; r < 1; r++) {
#else
  for(k = 0; k < IMG_C; k++) {
    for(r = 0; r < NEW_H; r++) {
#endif
      for(c = 0; c < NEW_W; c++) {     

        //Width reduction
	mul = (int32_t)((int32_t)dx[2*c]*(int32_t)(fp_image[k*IMG_W*IMG_H + iy[r]*IMG_W + ix[2*c]])); //Q2.14 * Q8.8 = Q10.22
        mul += (int32_t)((int32_t)dx[2*c+1]*(int32_t)(fp_image[k*IMG_W*IMG_H + iy[r]*IMG_W + ix[2*c+1]])); //Q10.22
        val_w = (int16_t) (mul >> 7); //Q10.22 to Q1.15
        mul = (int32_t)((int32_t)dx[2*c]*(int32_t)(fp_image[k*IMG_W*IMG_H + iy[r]*IMG_W + ix[2*c+2*NEW_W]])); //Q2.14 * Q8.8 = Q10.22
        mul += (int32_t)((int32_t)dx[2*c+1]*(int32_t)(fp_image[k*IMG_W*IMG_H + iy[r]*IMG_W + ix[2*c+1+2*NEW_W]])); //Q10.22
        next_val_w = (int16_t) (mul >> 7); //Q10.22 to Q1.15

        //Height reduction
        mul = (int32_t)((int32_t)dy[4*r+1]*(int32_t)val_w); //Q2.14 * Q1.15 = Q3.29
        mul += (int32_t)((int32_t)dy[4*r+3]*(int32_t)next_val_w); //Q3.29
        val_h = (int16_t)(mul >> 21); //Q3.29 to Q8.8

        //Save new value
      #ifdef SIM
	if(fp_data[k*(NTW_IN_W+2)*(NTW_IN_H+2) + (r+1)*(NTW_IN_W+2) + (c+1) + EXTRA_W + ((NTW_IN_W+2)*EXTRA_H)] != val_h) num_err++;
      #else	
        fp_data[k*(YOLO_INPUT+2)*(YOLO_INPUT+2) + (r+1)*(YOLO_INPUT+2) + (c+1) + EXTRA_W + ((YOLO_INPUT+2)*EXTRA_H)] = val_h;
      #endif
      }
    }
  }
#ifdef SIM
  uart_printf("Resized done with %d errors\n", num_err);
#endif
}

//resize input image
/*void resize_image() {

  //local variables
  int i, j, k, l;
  unsigned int data_time, config_time, run_time = 0;
#ifdef SIM
  int num_err = 0;
#endif

  //load ix and dx to versat
  start = timer_get_count_us(TIMER);
  for(j = 0; j < RESIZE_NSTAGES; j += 2) {
    for(i = 0; i < ix_size; i++) stage[j].memA[0].write(i, ix[i]);
    for(i = 0; i < dx_size; i++) stage[j].memA[1].write(i, dx[i]);
  }
  end = timer_get_count_us(TIMER);
  data_time = end - start;

  //loop to configure versat stages
  start = timer_get_count_us(TIMER);
  for(i = 0; i < RESIZE_NSTAGES; i+=2) {

    ///////////////////////////////////////////////////////////////////////////////
    //                            STAGE i
    ///////////////////////////////////////////////////////////////////////////////

    //configure mem0 to read: ix, ix+1, ix+2*NEW_W, ix+2*NEW_W+1 sequence
    //start, iter, incr, delay, per, duty, sel, shift, in_wr
    stage[i].memA[0].setConf(0, 2, 1, 0, 2, 2, 0, 2*NEW_W-2, 0);
    //iter2, per2, shift2, incr2
    stage[i].memA[0].setConf(1, NEW_W, 0, 2);
    stage[i].memA[0].writeConf();

    //configure mem1 to read 1-dx, dx sequence twice
    stage[i].memA[1].setConf(0, 2, 1, MEMP_LAT, 2, 2, 0, -2, 0);
    //iter2, per2, shift2, incr2
    stage[i].memA[1].setConf(1, NEW_W, 0, 2);
    stage[i].memA[1].writeConf();

    //configure mem2 to read input pixels (addressed by mem0)
    //start, iter, incr, delay, per, duty, sel, shift, in_wr, rvrs, ext
    stage[i].memA[2].setConf(0, 1, 0, MEMP_LAT, 4*NEW_W, 4*NEW_W, sMEMA[0], 0, 0, 0, 1);
    stage[i].memA[2].writeConf();

    //pixel * 1-dx/dx = res0
    //sela, selb, iter, per, delay, shift
    stage[i].muladdlite[0].setConf(sMEMA[1], sMEMA[2], 2*NEW_W, 2, 2*MEMP_LAT, 7); //Q10.22 to Q1.15
    stage[i].muladdlite[0].writeConf();

    //final results are stores in last layer, not first
    if(i == 0) {

      //res0 * 0/1-dy/0/dy = res1
      stage[RESIZE_NSTAGES].muladdlite[1].setConf(sMEMA_p[1], sMULADDLITE_p[1], NEW_W, 4, 2*MEMP_LAT+MULADDLITE_LAT, 21); //Q3.29 to Q8.8
      stage[RESIZE_NSTAGES].muladdlite[1].writeConf();

      //store res1 in mem3
      stage[RESIZE_NSTAGES].memA[3].setConf(0, NEW_W, 1, 2*MEMP_LAT+2*MULADDLITE_LAT+(4-1), 4, 1, sMULADDLITE[1], 0, 1);
      stage[RESIZE_NSTAGES].memA[3].writeConf();

    } else {

      //res0 * 0/1-dy/0/dy = res1
      stage[i].muladdlite[1].setConf(sMEMA_p[1], sMULADDLITE_p[1], NEW_W, 4, 2*MEMP_LAT+MULADDLITE_LAT, 21); //Q3.29 to Q8.8
      stage[i].muladdlite[1].writeConf();

      //store res1 in mem3
      stage[i].memA[3].setConf(0, NEW_W, 1, 2*MEMP_LAT+2*MULADDLITE_LAT+(4-1), 4, 1, sMULADDLITE[1], 0, 1);
      stage[i].memA[3].writeConf();
    }

    ///////////////////////////////////////////////////////////////////////////////
    //                            STAGE i+1
    ///////////////////////////////////////////////////////////////////////////////

    //configure mem0 (stage 1) to read 0, 1-dy, 0, dy sequence
    stage[i+1].memA[0].setConf(0, NEW_W, 1, MEMP_LAT+MULADDLITE_LAT, 4, 4, 0, -4, 0);
    stage[i+1].memA[0].writeConf();

    //res0 * 0/1-dy/0/dy = res1
    stage[i+1].muladdlite[0].setConf(sMEMA[0], sMULADDLITE_p[0], NEW_W, 4, 2*MEMP_LAT+MULADDLITE_LAT, 21); //Q3.29 to Q8.8
    stage[i+1].muladdlite[0].writeConf();

    //store res1 in mem3
    stage[i+1].memA[3].setConf(0, NEW_W, 1, 2*MEMP_LAT+2*MULADDLITE_LAT+(4-1), 4, 1, sMULADDLITE[0], 0, 1);
    stage[i+1].memA[3].writeConf();

    //configure mem2 to read input pixels (addressed by mem0 of previous stage)
    stage[i+1].memA[2].setConf(0, 1, 0, MEMP_LAT, 4*NEW_W, 4*NEW_W, sMEMA_p[0], 0, 0, 0, 1);
    stage[i+1].memA[2].writeConf();

    //pixel * 1-dx/dx = res0
    stage[i+1].muladdlite[1].setConf(sMEMA_p[1], sMEMA[2], 2*NEW_W, 2, 2*MEMP_LAT, 7); //Q10.22 to Q1.15
    stage[i+1].muladdlite[1].writeConf();

    //configure mem1 to read 0, 1-dy, 0, dy sequence
    stage[i+1].memA[1].setConf(0, NEW_W, 1, MEMP_LAT+MULADDLITE_LAT, 4, 4, 0, -4, 0);
    stage[i+1].memA[1].writeConf();
  }

  //end measuring config time
  end = timer_get_count_us(TIMER);
  config_time = end - start;

  //loops for performing resizing
#ifdef SIM
  for(k = 0; k < 1; k++) {
    for(j = 0; j < 1; j++) {
#else
  for(k = 0; k < IMG_C; k++) {
    for(j = 0; j < NEW_H; j += RESIZE_NSTAGES) {
#endif

      //Store 2 lines of pixels in mem2
      //Extra pixel is necessary to be multiplied by zero
      start = timer_get_count_us(TIMER);
      for(l = 0; l < RESIZE_NSTAGES; l++)
        for(i = 0; i < IMG_W*2+1; i++)
          stage[l].memA[2].write(i, fp_image[k*IMG_W*IMG_H + iy[j+l]*IMG_W + i]);

      //store dy in mem0 and mem1
      for(l = 0; l < RESIZE_NSTAGES; l+=2) {
        for(i = 0; i < 4; i++) {
          stage[l+1].memA[0].write(i, dy[4*(j+l)+i]);
          stage[l+1].memA[1].write(i, dy[4*(j+l+1)+i]);
        }
      }
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
      for(i = 0; i < NEW_W; i++)
	for(l = 0; l < RESIZE_NSTAGES; l++)
        #ifdef SIM
          if(fp_data[k*(YOLO_INPUT+2)*(YOLO_INPUT+2) + (j+l+1)*(YOLO_INPUT+2) + (i+1) + EXTRA_W + ((YOLO_INPUT+2)*EXTRA_H)] != stage[l+1].memA[3].read(i)) num_err++;
        #else
          fp_data[k*(YOLO_INPUT+2)*(YOLO_INPUT+2) + (j+l+1)*(YOLO_INPUT+2) + (i+1) + EXTRA_W + ((YOLO_INPUT+2)*EXTRA_H)] = stage[l+1].memA[3].read(i);
        #endif

      //end measuring data load/store time
      end = timer_get_count_us(TIMER);
      data_time += (end - start);
    }
  }
#ifdef SIM
  uart_printf("Resizing done with %d errors\n", num_err);
#endif
  uart_printf("FU config time = %d us\n", config_time);
  uart_printf("FU run time = %d us\n", run_time);
  uart_printf("FU load/store data time = %d us\n", data_time);
}*/

//layer 1 (conv)
/*void layer1() {

  //update pointers
  int16_t * output = (int16_t*) fp_data + NETWORK_INPUT;
  int16_t * scales = (int16_t *) fp_weights + LAYER_1_NUM_KER*LAYER_1_KER_SIZE*LAYER_1_KER_SIZE*IMG_C;
  int16_t * bias = (int16_t *) scales + LAYER_1_NUM_KER;

  //local variables
  int i, j, k, l, m, n;
  int32_t acc;
  int16_t batch_norm;

  //run layer
#ifdef SIM
  int num_err = 0;
  for(i = 0; i < 1; i++) {
    for(j = 0; j < 1; j++) {
#else
  for(i = 0; i < LAYER_1_NUM_KER; i++) {
    for(j = 0; j < YOLO_INPUT; j++) {
#endif

      for(k = 0; k < YOLO_INPUT; k++) {
        //convolution
        acc = 0;
        for(l = 0; l < IMG_C; l++)
          for(m = 0; m < LAYER_1_KER_SIZE; m++)
            for(n = 0; n < LAYER_1_KER_SIZE; n++)
              acc += (int32_t)((int32_t)fp_data[j*(YOLO_INPUT+2) + k + l*(YOLO_INPUT+2)*(YOLO_INPUT+2) + m*(YOLO_INPUT+2) + n]*(int32_t)fp_weights[i*IMG_C*LAYER_1_KER_SIZE*LAYER_1_KER_SIZE + l*LAYER_1_KER_SIZE*LAYER_1_KER_SIZE + m*LAYER_1_KER_SIZE + n]); //Q8.8*Q4.12=Q12.20

        //batch normalize -> Q12.20 to Q10.6 * Q8.8 = Q18.14 to Q8.8
        batch_norm = (int16_t)(((int32_t)(acc>>14) * (int32_t) scales[i]) >> 6) + bias[i];

        //activation function -> Q8.8*Q1.15 = Q9.23 to Q8.8
        if(batch_norm < 0) batch_norm = (int16_t)(((int32_t)batch_norm * (int32_t)3276) >> 15);
      #ifdef SIM
        if(output[i*YOLO_INPUT*YOLO_INPUT + j*YOLO_INPUT + k] != batch_norm) num_err++;
      #else
        output[i*YOLO_INPUT*YOLO_INPUT + j*YOLO_INPUT + k] = batch_norm;
      #endif
      }
    }
  }
#ifdef SIM
  uart_printf("Layer 1 done with %d errors\n", num_err);
#endif
}*/

//layer 1 (conv)
void layer1() {

  //clear configuration of all stages
  globalClearConf();

  //update pointers
  int16_t * output = (int16_t*) fp_data + NETWORK_INPUT;
  int16_t * scales_bias = (int16_t *) fp_weights + LAYER_1_NUM_KER*LAYER_1_KER_SIZE*LAYER_1_KER_SIZE*IMG_C;

  //local variables
  int i, j, k, l, m;
#ifdef SIM
  int num_err = 0;
#endif
  unsigned int data_time, config_time, run_time = 0;
  int16_t batch_norm;

  //load weights
  start = timer_get_count_us(TIMER);
  for(i = 0; i < LAYER_1_NUM_KER; i++) {
    for(j = 0; j < LAYER_1_KER_SIZE*LAYER_1_KER_SIZE; j++) {
      stage[0].memA[1].write(i*LAYER_1_KER_SIZE*LAYER_1_KER_SIZE+j, fp_weights[3*i*LAYER_1_KER_SIZE*LAYER_1_KER_SIZE + j]);
      stage[0].memA[3].write(i*LAYER_1_KER_SIZE*LAYER_1_KER_SIZE+j, fp_weights[(3*i+1)*LAYER_1_KER_SIZE*LAYER_1_KER_SIZE + j]);
      stage[1].memA[1].write(i*LAYER_1_KER_SIZE*LAYER_1_KER_SIZE+j, fp_weights[(3*i+2)*LAYER_1_KER_SIZE*LAYER_1_KER_SIZE + j]);
    }
  }
  end = timer_get_count_us(TIMER);
  data_time = end - start;

  //loop to configure versat stages
  start = timer_get_count_us(TIMER);

  ///////////////////////////////////////////////////////////////////////////////
  //                            STAGE 0
  ///////////////////////////////////////////////////////////////////////////////
  
  //configure mem0 to read 3x3 blocks from 54x34 channel0 FM
  //start, iter, incr, delay, per, duty, sel, shift, in_wr
  //iter2, per2, shift2, incr2
  stage[0].memA[0].setConf(0, 3, 1, 0, 3, 3, 0, 54-3, 0);
  stage[0].memA[0].setConf(32, 52, 2, 1);
  stage[0].memA[0].writeConf();

  //configure mem1 to read channel0 weights
  stage[0].memA[1].setConf(0, 32*52, 1, 0, 9, 9, 0, -9, 0);
  stage[0].memA[1].writeConf();

  //configure muladdlite0 to perform channel0 convolutions
  stage[0].muladdlite[0].setConf(sMEMA[0], sMEMA[1], 32*52, 9, MEMP_LAT, 0);
  stage[0].muladdlite[0].setAccOUT(1);
  stage[0].muladdlite[0].writeConf();

  //configure mem2 to read 3x3 blocks from 54x34 channel1 FM
  stage[0].memA[2].setConf(0, 3, 1, MULADDLITE_LAT+9, 3, 3, 0, 54-3, 0);
  stage[0].memA[2].setConf(32, 52, 2, 1);
  stage[0].memA[2].writeConf();

  //configure mem3 to read channel1 weights
  stage[0].memA[3].setConf(0, 32*52, 1, MULADDLITE_LAT+9, 9, 9, 0, -9, 0);
  stage[0].memA[3].writeConf();

  //configure muladdlite1 to perform channel1 convolutions
  stage[0].muladdlite[1].setConf(sMEMA[2], sMEMA[3], 32*52, 9, MEMP_LAT+MULADDLITE_LAT+9, 0);
  stage[0].muladdlite[1].setConf(sMULADDLITE[0], 1, 1, 0);
  stage[0].muladdlite[1].writeConf();

  ///////////////////////////////////////////////////////////////////////////////
  //                            STAGE 1
  ///////////////////////////////////////////////////////////////////////////////

  //configure mem0 to read 3x3 blocks from 54x34 channel2 FM
  stage[1].memA[0].setConf(0, 3, 1, 2*(MULADDLITE_LAT+9), 3, 3, 0, 54-3, 0);
  stage[1].memA[0].setConf(32, 52, 2, 1);
  stage[1].memA[0].writeConf();

  //configure mem1 to read channel2 weights
  stage[1].memA[1].setConf(0, 32*52, 1, 2*(MULADDLITE_LAT+9), 9, 9, 0, -9, 0);
  stage[1].memA[1].writeConf();

  //configure muladdlite0 to perform channel2 convolutions
  stage[1].muladdlite[0].setConf(sMEMA[0], sMEMA[1], 32*52, 9, MEMP_LAT+2*(MULADDLITE_LAT+9), 14);
  stage[1].muladdlite[0].setConf(sMULADDLITE_p[1], 1, 0, 0);
  stage[1].muladdlite[0].writeConf();

  //configure mem3 to write convolution results
  stage[1].memA[3].setConf(0, 32*52, 1, MEMP_LAT+3*(MULADDLITE_LAT+9)-1, 9, 1, sMULADDLITE[0], 0, 1);
  stage[1].memA[3].writeConf();

  //end measuring config time
  end = timer_get_count_us(TIMER);
  config_time = end - start;

  //loops for performing convolution
#ifdef SIM
  for(l = 0; l < 1; l++) {
    for(m = 0; m < 1; m++) {
#else
  for(l = 0; l < 13; l++) {
    for(m = 0; m < 8; m++) {
#endif

      //Send input FM 54x34 tile
      start = timer_get_count_us(TIMER);
      for(i = 0; i < 34; i++) {
        for(j = 0; j < 54; j++) {
          stage[0].memA[0].write(i*54+j, fp_data[(32*l+i)*418 + 52*m + j]);
          stage[0].memA[2].write(i*54+j, fp_data[(32*l+i)*418 + 52*m + j + 418*418]);
          stage[1].memA[0].write(i*54+j, fp_data[(32*l+i)*418 + 52*m + j + 418*418*2]);
        }
      }
      end = timer_get_count_us(TIMER);
      data_time += (end - start);

    #ifdef SIM
      for(k = 0; k < 1; k++) {
    #else
      for(k = 0; k < LAYER_1_NUM_KER; k++) {
    #endif

        //Configure weight mems start
        stage[0].memA[1].setStart(k*LAYER_1_KER_SIZE*LAYER_1_KER_SIZE);
        stage[0].memA[3].setStart(k*LAYER_1_KER_SIZE*LAYER_1_KER_SIZE);
        stage[1].memA[1].setStart(k*LAYER_1_KER_SIZE*LAYER_1_KER_SIZE);

        //run
        start = timer_get_count_us(TIMER);
        run();

        //Wait until done
        while(done() == 0);
        end = timer_get_count_us(TIMER);
        run_time += (end - start);

        //store result in DDR
        start = timer_get_count_us(TIMER);
        for(i = 0; i < 32; i++) {
          for(j = 0; j < 52; j++) {

            //batch normalize -> Q12.20 to Q10.6 * Q8.8 = Q18.14 to Q8.8
            batch_norm = (int16_t)(((int32_t)(stage[1].memA[3].read(i*52+j)) * (int32_t) scales_bias[k]) >> 6) + scales_bias[16+k];

            //activation function -> Q8.8*Q1.15 = Q9.23 to Q8.8
            if(batch_norm < 0) batch_norm = (int16_t)(((int32_t)batch_norm * (int32_t)3276) >> 15);

            //Store in DDR
          #ifdef SIM
            if(output[k*YOLO_INPUT*YOLO_INPUT + (32*l+i)*YOLO_INPUT + 52*m + j] != batch_norm) num_err++;
	  #else
            output[k*YOLO_INPUT*YOLO_INPUT + (32*l+i)*YOLO_INPUT + 52*m + j] = batch_norm;
	  #endif
          }
        }

        //end measuring data load/store time
        end = timer_get_count_us(TIMER);
        data_time += (end - start);
      }
    }
  }
#ifdef SIM
  uart_printf("Layer 1 done with %d errors\n", num_err);
#endif
  uart_printf("FU config time = %d us\n", config_time);
  uart_printf("FU run time = %d us\n", run_time);
  uart_printf("FU load/store data time = %d us\n", data_time);
}

//send detection results back
void send_data() {

  //Loop to send output of yolo layer
  int i, j;
  count_bytes = 0;
  char * fp_data_char = (char *) fp_image + (IMAGE_INPUT + NETWORK_INPUT)*2;
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
#endif

  //layer1 (conv)
  uart_printf("\nRunning layer 1...\n");
  layer1();

  //return data
#ifndef SIM
  send_data();
#endif
  uart_putc(4);
  return 0;
}
