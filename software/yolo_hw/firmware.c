//import custom libraries
#include "system.h" //must be defined before versat API!!!
#include "periphs.h"
#include "iob-uart.h"
#include "iob-eth.h"
#include "iob_timer.h"
#include "new_versat.hpp"
#include "firmware.h"

//import c libraries
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#ifdef SIM
  int k_delta;
#endif

//define peripheral base addresses
#ifndef PCSIM
  //USE_DDR==1 to run yolo anyways
  #if (RUN_DDR==0) // running firmware from SRAM 
    #define DDR_MEM (EXTRA_BASE)
  #else //running firmware from DDR
    #define DDR_MEM ((1<<(FIRM_ADDR_W)))
  #endif
#else
  uint8_t ddr_mem_vect[1<<(DDR_ADDR_W-2)]; //DDR addressable space, up to 2**30
  #define DDR_MEM (&ddr_mem_vect[0])
#endif //ifndef PCSIM

//TILE sizes
#define LAYER_1_TILE_W 32  //34*4*3 = 408
#define LAYER_3_TILE_W 16  //18*4*16 = 1152
#define LAYER_5_TILE_W 8   //10*4*32 = 1280
#define LAYER_7_TILE_W 4   //6*4*64 = 1536
#define LAYER_9_TILE_W 2   //4*3*128 = 1536

//Weight len
#define LAYER_1_W_LEN (LAYER_1_KER_SIZE*LAYER_1_KER_SIZE*LAYER_1_C + LAYER_1_W_OFF - 1)
#define LAYER_3_W_LEN (LAYER_3_KER_SIZE*LAYER_3_KER_SIZE*LAYER_1_NUM_KER - 1)
#define LAYER_5_W_LEN ((LAYER_5_KER_SIZE*LAYER_5_KER_SIZE*LAYER_3_NUM_KER)/2 - 1)
#define LAYER_7_W_LEN ((LAYER_7_KER_SIZE*LAYER_7_KER_SIZE*LAYER_5_NUM_KER)/3 - 1)
#define LAYER_9_W_LEN ((LAYER_9_KER_SIZE*LAYER_9_KER_SIZE*LAYER_7_NUM_KER)/6 - 1)

//define ethernet constants
#define ETH_NBYTES (1024-18) //minimum ethernet payload excluding FCS
#define INPUT_FILE_SIZE ((TOTAL_WEIGHTS + DATA_LAYER_1)*2) //16 bits
#define NUM_INPUT_FRAMES (INPUT_FILE_SIZE/ETH_NBYTES)
#define OUTPUT_FILE_SIZE (DATA_LAYER_9*2) //16 bits
#define NUM_OUTPUT_FRAMES (OUTPUT_FILE_SIZE/ETH_NBYTES)

//define DDR mapping
#define WEIGHTS_BASE_ADDRESS (DDR_MEM + (1 << (FIRM_ADDR_W))) //after main mem
#define DATA_BASE_ADDRESS (WEIGHTS_BASE_ADDRESS + 2*TOTAL_WEIGHTS) //16 bits

//ETHERNET variables
int rcv_timeout = 5000;
char data_to_send[ETH_NBYTES];
char data_rcv[ETH_NBYTES+18];
unsigned int bytes_to_receive, bytes_to_send, count_bytes;

//TIMER variables
unsigned int start, end;

//weights and data updatable initial positions
unsigned int w_pos = 0, p_pos = 0;

//reset certain DDR positions to zero due to padding
void reset_DDR() {

  //local variables
  unsigned int i, pos = DATA_LAYER_1;
  int16_t * fp_data = (int16_t *) DATA_BASE_ADDRESS;

  //measure initial time
  uart_printf("\nSetting DDR positions to zero\n");
  start = timer_time_us(TIMER_BASE);

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

  //measure final time
  end = timer_time_us(TIMER_BASE);
  uart_printf("DDR reset to zero done in %d ms\n", (end-start)/1000);
}

//receive weigths and resized padded image
void rcv_data() {

  //Local variables
  int i, j;
  count_bytes = 0;
  char * data_p = (char *) WEIGHTS_BASE_ADDRESS;
  uart_printf("\nReady to receive input image and weights...\n");

  //Loop to receive intermediate data frames
  for(j = 0; j < NUM_INPUT_FRAMES+1; j++) {

     //wait to receive frame
     while(eth_rcv_frame(data_rcv, ETH_NBYTES+18, rcv_timeout) !=0);

     //start timer
     if(j == 0) start = timer_time_us(TIMER_BASE);

     //check if it is last packet (has less data that full payload size)
     if(j == NUM_INPUT_FRAMES) bytes_to_receive = INPUT_FILE_SIZE - count_bytes;
     else bytes_to_receive = ETH_NBYTES;

     //save in DDR
     for(i = 0; i < bytes_to_receive; i++) {
       data_p[j*ETH_NBYTES + i] = data_rcv[14+i];
       data_to_send[i] = data_p[j*ETH_NBYTES + i];
     }

     //send data back as ack
     eth_send_frame(data_to_send, ETH_NBYTES);

     //update byte counter
     count_bytes += ETH_NBYTES;
  }
  end = timer_time_us(TIMER_BASE);
  uart_printf("Image and weights received in %d ms\n", (end-start)/1000);
}

//1st layer convolution
void conv(int w, int c, int num_ker, int ker_size, int til_w, int w_off, int p_off, int w_len, int mp) {

  //local variables
  int j, k, l;
#ifdef SIM
  k_delta = w/((1+mp)*nSTAGES);
#endif
  unsigned int b_in = WEIGHTS_BASE_ADDRESS + 2*w_pos;
  unsigned int w_in = b_in + 2*num_ker;
  unsigned int p_in = DATA_BASE_ADDRESS + 2*p_pos;
  unsigned int p_out;

  //update initial positions
  w_pos += num_ker*(1 + ker_size*ker_size*c + w_off);
  p_pos += (w+2)*((w+2)*c+p_off);
  p_out = DATA_BASE_ADDRESS + 2*(p_pos + (w/(1+mp)+2+1)*num_ker); //pass first padding line and column

  /////////////////////////////////////////////////////////////////////////
  //                          CONFIGURATIONS
  /////////////////////////////////////////////////////////////////////////

  // configure xyolo_read vreads to read bias and kernel from DDR
  versat.yread.setLen(w_len);
  versat.yread.setOffset(2*(ker_size*ker_size*c + w_off));
  versat.yread.setExtPer((ker_size*ker_size*c + w_off)/16);
  versat.yread.setExtIncr(16);
  versat.yread.setExtIter(1);

  // configure xyolo_write vread to read tile from input fm
  versat.ywrite.read.setLen((c*(til_w+2)+p_off)/16-1);
  versat.ywrite.read.setOffset(2*((1+mp)*((w+2)*c+p_off)));
  versat.ywrite.read.setExtPer((c*(til_w+2)+p_off)/16);
  versat.ywrite.read.setExtIncr(16);
  versat.ywrite.read.setExtShift(((w+2)*c) - (c*(til_w+2))); //+2 due to padding

  // configure xyolo_read vreads to write 1 + 3x3x3 kernel to flow_outs
  versat.yread.setIntPer(ker_size*ker_size*c);
  versat.yread.setIntIncr(1);
  versat.yread.setIntIter((1+mp)*til_w);
  versat.yread.setIntShift(-ker_size*ker_size*c);

  // configure xyolo_write vread to write FM tile to xyolo
  versat.ywrite.read.setIntPer(ker_size*c);
  versat.ywrite.read.setIntIncr(1);
  versat.ywrite.read.setIntIter(ker_size);
  versat.ywrite.read.setIntShift((til_w+2)*c+p_off - ker_size*c); //+2 due to padding
  versat.ywrite.read.setIntIncr2(c);
  if(mp) {
    versat.ywrite.read.setIntPer2(2);
    versat.ywrite.read.setIntIter2(2);
    versat.ywrite.read.setIntShift2((til_w+2)*c+p_off - 2*c); //+2 due to padding
    versat.ywrite.read.setIntPer3(til_w/2); // /2 due to maxpool
    versat.ywrite.read.setIntIncr3(2*c);
    versat.ywrite.read.setIntIter3(1);
  } else if(til_w != 1) {
    versat.ywrite.read.setIntPer2(til_w);
    versat.ywrite.read.setIntIter2(1);
  }

  // configure xyolo to perform convolution + maxpool
  versat.ywrite.yolo.setIter((1+mp)*til_w);
  versat.ywrite.yolo.setPer(ker_size*ker_size*c);
  versat.ywrite.yolo.setShift(10);
  versat.ywrite.yolo.setBias(1);
  versat.ywrite.yolo.setLeaky(1);
  versat.ywrite.yolo.setMaxpool(mp);

  // configure xwrite to write convolution results
  versat.ywrite.write.setIntDuty(1);
  versat.ywrite.write.setIntDelay(XYOLO_READ_LAT + XYOLO_WRITE_LAT - 2);
  versat.ywrite.write.setIntPer((1+3*mp)*ker_size*ker_size*c);
  versat.ywrite.write.setIntIncr(num_ker/nYOLOvect);
  versat.ywrite.write.setIntIter(til_w/(1+mp));

  // configure xyolo_write vwrite to write result back to DDR
  versat.ywrite.write.setLen(((til_w/(1+mp))*(num_ker/nYOLOvect))-1);
  versat.ywrite.write.setOffset(2*((w/(1+mp)+2)*num_ker));
  versat.ywrite.write.setExtPer(1);

#ifdef SIM
  for(k = 0; k < k_delta; k++) {
    //uart_printf("%d\n", k);
#else
  for(k = 0; k < w/((1+mp)*nSTAGES); k++) {
#endif
    for(j = 0; j < w/til_w; j++) {
    //for(j = 0; j < 2; j++) {

	#ifdef SIM
	  uart_printf("%d\n", j);
	#endif

      // configure xyolo_write vread to read tile from input fm
      versat.ywrite.read.setExtIter(ker_size+mp);
      versat.ywrite.read.setExtAddr(p_in + 2*(k*(1+mp)*((w+2)*c+p_off)*nSTAGES + j*til_w*c));

      // simulate for first yolo layer
      for(l = 0; l < num_ker/nYOLOvect; l++) {

        // read filter
        versat.yread.setExtAddr(w_in + 2*l*nYOLOvect*(ker_size*ker_size*c + w_off));
        versat.yread.setBiasExtAddr(b_in + 2*l*nYOLOvect);

        // configure xyolo_write vwrite start
        versat.ywrite.write.setIntStart(l);

        // configure xyolo_write vwrite to write result back to DDR
        if(l == num_ker/nYOLOvect-1) {
          versat.ywrite.write.setExtAddr(p_out + 2*(k*(w/(1+mp)+2)*num_ker*nSTAGES + j*(til_w/(1+mp)*num_ker)));
  	  versat.ywrite.write.setExtIter(((til_w/(1+mp))*(num_ker/nYOLOvect)));
        } else versat.ywrite.write.setExtIter(0);

        // wait until done
        while(versat.done()==0);

        // run configuration
        versat.run();

        // stop xyolo_write vread reading from DDR
        versat.ywrite.read.setExtIter(0);
      }
    }
  }

  // clear configs
  versat.clear();
}

//send results back
void send_data() {

  //Loop to send data
  int i, j;
  count_bytes = 0;
  char * fp_data_char = (char *) DATA_BASE_ADDRESS + 2*(DATA_LAYER_1 + DATA_LAYER_2 + DATA_LAYER_4 + DATA_LAYER_6 + DATA_LAYER_8);
  for(j = 0; j < NUM_OUTPUT_FRAMES+1; j++) {

    //start timer
    if(j == 0) start = timer_time_us(TIMER_BASE);

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
  end = timer_time_us(TIMER_BASE);
  uart_printf("\noutput layer transferred in %d ms\n\n", (end-start)/1000);
}

int main(int argc, char **argv) {

  //init UART
  uart_init(UART_BASE,FREQ/BAUD);

  //send init message
  uart_printf("\nNEW VERSAT TEST\n\n");

  //init VERSAT
  versat_init(VERSAT_BASE);

#ifndef SIM

  //init ETHERNET
  eth_init(ETHERNET_BASE);
  eth_set_rx_payload_size(ETH_NBYTES);

  //reset DDR due to zero-padding
  reset_DDR();

  //receive data via ethernet
  rcv_data();

  //layers 1 and 2
  uart_printf("\nRunning layers 1 and 2...\n");
  start = timer_time_us(TIMER_BASE);
  conv(LAYER_1_W, LAYER_1_C, LAYER_1_NUM_KER, LAYER_1_KER_SIZE, LAYER_1_TILE_W, LAYER_1_W_OFF, LAYER_1_P_OFF, LAYER_1_W_LEN, LAYER_1_MAXPOOL);
  end = timer_time_us(TIMER_BASE);
  uart_printf("Convolution + maxpool done in %d us\n\n", end-start);

  //layers 3 and 4
  uart_printf("\nRunning layers 3 and 4...\n");
  start = timer_time_us(TIMER_BASE);
  conv(LAYER_3_W, LAYER_1_NUM_KER, LAYER_3_NUM_KER, LAYER_3_KER_SIZE, LAYER_3_TILE_W, 0, 0, LAYER_3_W_LEN, LAYER_3_MAXPOOL);
  end = timer_time_us(TIMER_BASE);
  uart_printf("Convolution + maxpool done in %d us\n\n", end-start);

  //layers 5 and 6
  uart_printf("\nRunning layers 5 and 6...\n");
  start = timer_time_us(TIMER_BASE);
  conv(LAYER_5_W, LAYER_3_NUM_KER, LAYER_5_NUM_KER, LAYER_5_KER_SIZE, LAYER_5_TILE_W, 0, 0, LAYER_5_W_LEN, LAYER_5_MAXPOOL);
  end = timer_time_us(TIMER_BASE);
  uart_printf("Convolution + maxpool done in %d us\n\n", end-start);

  //layers 7 and 8
  uart_printf("\nRunning layers 7 and 8...\n");
  start = timer_time_us(TIMER_BASE);
  conv(LAYER_7_W, LAYER_5_NUM_KER, LAYER_7_NUM_KER, LAYER_7_KER_SIZE, LAYER_7_TILE_W, 0, 0, LAYER_7_W_LEN, LAYER_7_MAXPOOL);
  end = timer_time_us(TIMER_BASE);
  uart_printf("Convolution + maxpool done in %d us\n\n", end-start);
#else
  w_pos += WEIGHTS_LAYER_1 + WEIGHTS_LAYER_3 + WEIGHTS_LAYER_5 + WEIGHTS_LAYER_7;
  p_pos += DATA_LAYER_1 + DATA_LAYER_2 + DATA_LAYER_4 + DATA_LAYER_6;
#endif

  //layer 9
  uart_printf("\nRunning layer 9...\n");
  start = timer_time_us(TIMER_BASE);
  conv(LAYER_9_W, LAYER_7_NUM_KER, LAYER_9_NUM_KER, LAYER_9_KER_SIZE, LAYER_9_TILE_W, 0, 0, LAYER_9_W_LEN, LAYER_9_MAXPOOL);
  // end versat
  versat_end();
  end = timer_time_us(TIMER_BASE);
  uart_printf("Convolution done in %d us\n\n", end-start);

  //verify results
#ifdef SIM
  int16_t * fp_data = (int16_t *) DATA_BASE_ADDRESS;
  fp_data += DATA_LAYER_1 + DATA_LAYER_2 + DATA_LAYER_4 + DATA_LAYER_6 + DATA_LAYER_8;
  int i, j, k;
  uart_printf("Verifying...\n\n");
  uart_printf("1st line = %x\n", DATA_BASE_ADDRESS + 2*(DATA_LAYER_1 + DATA_LAYER_2 + DATA_LAYER_4 + DATA_LAYER_6 + DATA_LAYER_8 + (LAYER_9_W+2)*LAYER_9_NUM_KER));
  uart_printf("2nd line = %x\n", DATA_BASE_ADDRESS + 2*(DATA_LAYER_1 + DATA_LAYER_2 + DATA_LAYER_4 + DATA_LAYER_6 + DATA_LAYER_8 + 2*(LAYER_9_W+2)*LAYER_9_NUM_KER));
  for(i = 0; i < k_delta*nSTAGES+1; i++) {
    uart_printf("%d\n", i);
    for(j = 0; j < (LAYER_9_W+2); j++)
      for(k = 0; k < LAYER_9_NUM_KER; k++)
        if(fp_data[i*(LAYER_9_W+2)*LAYER_9_NUM_KER + j*LAYER_9_NUM_KER + k] != fp_data[DATA_LAYER_9 + i*(LAYER_9_W+2)*LAYER_9_NUM_KER + j*LAYER_9_NUM_KER + k])
          uart_printf("(%x) res = %x, act = %x\n", DATA_BASE_ADDRESS + 2*(DATA_LAYER_1 + DATA_LAYER_2 + DATA_LAYER_4 + DATA_LAYER_6 + DATA_LAYER_8 + i*(LAYER_9_W+2)*LAYER_9_NUM_KER + j*LAYER_9_NUM_KER + k), fp_data[i*(LAYER_9_W+2)*LAYER_9_NUM_KER + j*LAYER_9_NUM_KER + k] & 0xFFFF, fp_data[DATA_LAYER_9 + i*(LAYER_9_W+2)*LAYER_9_NUM_KER + j*LAYER_9_NUM_KER + k] & 0xFFFF);  }
#else
  send_data();
#endif

  //finish
  uart_putc(ETX);
  return 0;
}