//import custom libraries
#include "system.h" //must be defined before versat API!!!
#include "periphs.h"
#include "iob-uart.h"
#include "iob-eth.h"
#include "iob_timer.h"

#include "new_versat.hpp"

//import c libraries
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

//define peripheral base addresses
//#define DDR_MEM (CACHE_BASE<<(ADDR_W-N_SLAVES_W))
// set pointer to DDR base
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

//global constants
#define NTW_IN_C 3
#define NTW_IN_W 416
#define NTW_IN_KER_SIZE 3
#define NTW_IN_NUM_KER 16
#define WEIGHT_SIZE (NTW_IN_NUM_KER*(1 + NTW_IN_KER_SIZE*NTW_IN_KER_SIZE*NTW_IN_C + 5)) //+5 to be 32 byte aligned
#define DATA_LAYER_1 ((NTW_IN_W+2)*((NTW_IN_W+2)*NTW_IN_C+10)) //+10 to be 32 byte aligned
#define DATA_LAYER_3 ((NTW_IN_W/2+2)*(NTW_IN_W/2+2)*NTW_IN_NUM_KER)
#define TILE_W 32

//define ethernet constants
#define ETH_NBYTES (1024-18) //minimum ethernet payload excluding FCS
#define INPUT_FILE_SIZE ((WEIGHT_SIZE + DATA_LAYER_1)*2) //16 bits
#define NUM_INPUT_FRAMES (INPUT_FILE_SIZE/ETH_NBYTES)
#define OUTPUT_FILE_SIZE (DATA_LAYER_3*2) //16 bits
#define NUM_OUTPUT_FRAMES (OUTPUT_FILE_SIZE/ETH_NBYTES)

//define DDR mapping
#define WEIGHTS_BASE_ADDRESS (DDR_MEM + (1 << (FIRM_ADDR_W))) //after main mem
#define LAYER_1_BASE_ADDRESS (WEIGHTS_BASE_ADDRESS + 2*WEIGHT_SIZE) //16 bits
#define LAYER_3_BASE_ADDRESS (LAYER_1_BASE_ADDRESS + 2*DATA_LAYER_1) //aligned layer 3 input

//ETHERNET variables
int rcv_timeout = 5000;
char data_to_send[ETH_NBYTES];
char data_rcv[ETH_NBYTES+18];
unsigned int bytes_to_receive, bytes_to_send, count_bytes;

//TIMER variables
unsigned int start, end;

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
void conv() {

  //local variables
  int j, k, l;
  unsigned int bias_ba = WEIGHTS_BASE_ADDRESS; //bias base address
  unsigned int weights_ba = WEIGHTS_BASE_ADDRESS + 2*NTW_IN_NUM_KER; //weight base address
#ifdef SIM
  int k_delta = 1; //NTW_IN_W/(2*nSTAGES);
#endif

  /////////////////////////////////////////////////////////////////////////
  //                          CONFIGURATIONS
  /////////////////////////////////////////////////////////////////////////

  // configure xyolo_read vreads to read bias and kernel from DDR
  versat.yread.setLen(NTW_IN_KER_SIZE*NTW_IN_KER_SIZE*NTW_IN_C + 5 - 1); //+5 so each filter is 32 byte aligned
  versat.yread.setOffset(2*(NTW_IN_KER_SIZE*NTW_IN_KER_SIZE*NTW_IN_C + 5)); //+4 so each filter is 32 byte aligned
  versat.yread.setExtPer((NTW_IN_KER_SIZE*NTW_IN_KER_SIZE*NTW_IN_C + 5)/16); //+4 so each filter is 32 byte aligned
  versat.yread.setExtIncr(16);
  versat.yread.setPingPong(1);

  // configure xyolo_write vread to read tile from input fm
  versat.ywrite.read.setLen((NTW_IN_C*(TILE_W+2)+10)/16-1); //+10 so each line is 32 byte aligned
  versat.ywrite.read.setOffset(2*(2*((NTW_IN_W+2)*NTW_IN_C+10))); //+10 so each line is 32 byte aligned
  versat.ywrite.read.setExtPer((NTW_IN_C*(TILE_W+2)+10)/16); //+10 so each line is 32 byte aligned
  versat.ywrite.read.setExtIncr(16);
  versat.ywrite.read.setExtIter(NTW_IN_KER_SIZE+1); //+1 due to maxpool // 4
  versat.ywrite.read.setExtShift(((NTW_IN_W+2)*NTW_IN_C) - (NTW_IN_C*(TILE_W+2))); //+2 due to padding

  // configure xyolo_read vreads to write 1 + 3x3x3 kernel to flow_outs
  versat.yread.setIntPer(NTW_IN_KER_SIZE*NTW_IN_KER_SIZE*NTW_IN_C); // 27
  versat.yread.setIntIncr(1);
  versat.yread.setIntIter(2*TILE_W); //x2 due to maxpool // 8
  versat.yread.setIntShift(-NTW_IN_KER_SIZE*NTW_IN_KER_SIZE*NTW_IN_C); // -27

  // configure xyolo_write vread to write FM tile to xyolo
  versat.ywrite.read.setIntPer(NTW_IN_KER_SIZE*NTW_IN_C); // 9
  versat.ywrite.read.setIntIncr(1);
  versat.ywrite.read.setIntIter(NTW_IN_KER_SIZE); // 3
  versat.ywrite.read.setIntShift((TILE_W+2)*NTW_IN_C+10 - NTW_IN_KER_SIZE*NTW_IN_C); //+2 due to padding // 9
  versat.ywrite.read.setIntPer2(2);
  versat.ywrite.read.setIntIncr2(NTW_IN_C); // 3
  versat.ywrite.read.setIntIter2(2);
  versat.ywrite.read.setIntShift2((TILE_W+2)*NTW_IN_C+10 - 2*NTW_IN_C); //+2 due to padding // 12
  versat.ywrite.read.setIntPer3(TILE_W/2); // /2 due to maxpool //2
  versat.ywrite.read.setIntIncr3(2*NTW_IN_C); // 6
  versat.ywrite.read.setIntIter3(1);

  // configure xyolo to perform convolution + maxpool
  versat.ywrite.yolo.setIter(2*TILE_W); //x2 due to maxpool // 8
  versat.ywrite.yolo.setPer(NTW_IN_KER_SIZE*NTW_IN_KER_SIZE*NTW_IN_C); // 27
  versat.ywrite.yolo.setShift(10);
  versat.ywrite.yolo.setBias(1);
  versat.ywrite.yolo.setLeaky(1);
  versat.ywrite.yolo.setMaxpool(1);

  // configure xwrite to write convolution results
  versat.ywrite.write.setIntDuty(1);
  versat.ywrite.write.setIntDelay(XYOLO_READ_LAT + XYOLO_WRITE_LAT - 2); // 1+5-2 = 4
  versat.ywrite.write.setIntPer(4*NTW_IN_KER_SIZE*NTW_IN_KER_SIZE*NTW_IN_C); // 108
  versat.ywrite.write.setIntIncr(1);
  versat.ywrite.write.setIntIter(TILE_W/2); // 2

  // configure xyolo_write vwrite to write result back to DDR
  versat.ywrite.write.setLen(16-1); //send 16 values per stage
  versat.ywrite.write.setOffset(2*((NTW_IN_W/2+2)*NTW_IN_NUM_KER));
  versat.ywrite.write.setExtPer(1);
  versat.ywrite.write.setExtIncr(nYOLOvect);
  versat.ywrite.write.setExtIter(TILE_W/2); // 2
  versat.ywrite.write.setExtShift(NTW_IN_NUM_KER-nYOLOvect); // 16-16 = 0

  // simulate for first yolo layer
  for(l = 0; l < NTW_IN_NUM_KER/nYOLOvect; l++) {

    // read filter
    versat.yread.setExtIter(1);
    versat.yread.setExtAddr(weights_ba + 2*l*nYOLOvect*(NTW_IN_KER_SIZE*NTW_IN_KER_SIZE*NTW_IN_C + 5));
    versat.yread.setBiasExtAddr(bias_ba + 2*l*nYOLOvect);

  #ifdef SIM
    for(k = 0; k < k_delta; k++) {
       //uart_printf("%d\n", k);
  #else
    for(k = 0; k < NTW_IN_W/(2*nSTAGES); k++) { //2 due to maxpool
  #endif
      for(j = 0; j < NTW_IN_W/TILE_W; j++) {
      //for(j = 0; j < 3; j++) {

        // configure xyolo_write vread to read tile from input fm
        versat.ywrite.read.setExtAddr(LAYER_1_BASE_ADDRESS + 2*(k*2*((NTW_IN_W+2)*NTW_IN_C+10)*nSTAGES + j*TILE_W*NTW_IN_C));

        // configure xyolo_write vwrite to write result back to DDR
        versat.ywrite.write.setExtAddr(LAYER_3_BASE_ADDRESS + 2*((NTW_IN_W/2+2+1)*NTW_IN_NUM_KER + k*(NTW_IN_W/2+2)*NTW_IN_NUM_KER*nSTAGES + j*(TILE_W/2)*NTW_IN_NUM_KER + l*nYOLOvect));

        // wait until done
        while(versat.done()==0);

        // run configuration
        versat.run();

       // stop xyolo_read reading from DDR
       versat.yread.setExtIter(0);
      }
    }
  }

  // clear configs
  versat.clear();

  // wait until done
  while(versat.done()==0);

  // end versat
  versat_end();

#ifdef SIM
  int16_t * fp_data = (int16_t *) LAYER_3_BASE_ADDRESS;
  int i;
  uart_printf("Verifying...\n\n");
  uart_printf("INITIAL ADDRESS of 2nd line = %x\n", LAYER_3_BASE_ADDRESS + 2*((NTW_IN_W/2+2)*NTW_IN_NUM_KER));
  uart_printf("INITIAL ADDRESS of 3rd line = %x\n", LAYER_3_BASE_ADDRESS + 2*2*((NTW_IN_W/2+2)*NTW_IN_NUM_KER));
  for(i = 0; i < k_delta*nSTAGES+1; i++) {
    uart_printf("%d\n", i);
    for(j = 0; j < NTW_IN_W/2+2; j++)
      for(k = 0; k < NTW_IN_NUM_KER; k++)
        if(fp_data[i*(NTW_IN_W/2+2)*NTW_IN_NUM_KER + j*NTW_IN_NUM_KER + k] != fp_data[DATA_LAYER_3 + i*(NTW_IN_W/2+2)*NTW_IN_NUM_KER + j*NTW_IN_NUM_KER + k])
          uart_printf("(%x) res = %x, act = %x\n", LAYER_3_BASE_ADDRESS + 2*(i*(NTW_IN_W/2+2)*NTW_IN_NUM_KER + j*NTW_IN_NUM_KER + k), fp_data[i*(NTW_IN_W/2+2)*NTW_IN_NUM_KER + j*NTW_IN_NUM_KER + k] & 0xFFFF, fp_data[DATA_LAYER_3 + i*(NTW_IN_W/2+2)*NTW_IN_NUM_KER + j*NTW_IN_NUM_KER + k] & 0xFFFF);  }
#endif
}

//send results back
void send_data() {

  //Loop to send data
  int i, j;
  count_bytes = 0;
  char * fp_data_char = (char *) LAYER_3_BASE_ADDRESS;
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

  //local variables
  unsigned int i;

#ifndef SIM

  //init ETHERNET
  eth_init(ETHERNET_BASE);
  eth_set_rx_payload_size(ETH_NBYTES);

  //set positions to zero due to output padding
  int16_t * fp_image = (int16_t *) LAYER_3_BASE_ADDRESS;
  for(i = 0; i < DATA_LAYER_3; i++) fp_image[i] = 0;

  //receive data via ethernet
  rcv_data();

#else
  //test config params of xyolo_read
  versat.yread.setExtAddr(1);
  versat.yread.setOffset(2);
  versat.yread.setIntAddr(3);
  versat.yread.setExtIter(4);
  versat.yread.setExtPer(5);
  versat.yread.setExtShift(6);
  versat.yread.setExtIncr(7);
  versat.yread.setIntIter(8);
  versat.yread.setIntPer(9);
  versat.yread.setIntStart(10);
  versat.yread.setIntShift(11);
  versat.yread.setIntIncr(12);

  // test config params of xyolo_write
  // vwrite
  versat.ywrite.write.setExtAddr(1);
  versat.ywrite.write.setOffset(2);
  versat.ywrite.write.setIntAddr(3);
  versat.ywrite.write.setExtIter(4);
  versat.ywrite.write.setExtPer(5);
  versat.ywrite.write.setExtShift(6);
  versat.ywrite.write.setExtIncr(7);
  versat.ywrite.write.setIntStart(8);
  versat.ywrite.write.setIntDuty(9);
  versat.ywrite.write.setIntDelay(10);
  versat.ywrite.write.setIntIter(11);
  versat.ywrite.write.setIntPer(12);
  versat.ywrite.write.setIntShift(13);
  versat.ywrite.write.setIntIncr(14);
  // vread
  versat.ywrite.read.setExtAddr(15);
  versat.ywrite.read.setOffset(16);
  versat.ywrite.read.setIntAddr(17);
  versat.ywrite.read.setExtIter(18);
  versat.ywrite.read.setExtPer(19);
  versat.ywrite.read.setExtShift(20);
  versat.ywrite.read.setExtIncr(21);
  versat.ywrite.read.setIntStart(22);
  versat.ywrite.read.setIntIter(23);
  versat.ywrite.read.setIntPer(24);
  versat.ywrite.read.setIntShift(25);
  versat.ywrite.read.setIntIncr(26);
  versat.ywrite.read.setIntIter2(27);
  versat.ywrite.read.setIntPer2(28);
  versat.ywrite.read.setIntShift2(29);
  versat.ywrite.read.setIntIncr2(30);
  // xyolo
  versat.ywrite.yolo.setIter(31);
  versat.ywrite.yolo.setPer(32);
  versat.ywrite.yolo.setShift(10);
  versat.ywrite.yolo.setBias(1);
  versat.ywrite.yolo.setLeaky(1);
  versat.ywrite.yolo.setMaxpool(1);
  versat.ywrite.yolo.setBypass(1);

  //clear configurations
  versat.clear();
#endif

  //DEBUG
  int16_t* w_p  = (int16_t*) (WEIGHTS_BASE_ADDRESS);
  uart_printf("\n\n%x\n\n", w_p[0]&0xFFFF);

  //first layer conv
  uart_printf("Running convolution...\n");
  start = timer_time_us(TIMER_BASE);
  conv();
  end = timer_time_us(TIMER_BASE);
  uart_printf("\nConvolution done in %d us\n\n", (end-start));

  //send results back
#ifndef SIM
  send_data();
#endif

  //finish
  uart_putc(ETX);
  return 0;
}
