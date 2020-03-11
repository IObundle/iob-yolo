#include "system.h"
#include "iob-uart.h"
#include "iob-eth.h"

#include <stdint.h>
#include <stdio.h>
#include <string.h>

#define UART (UART_BASE<<(DATA_W-N_SLAVES_W))
#define SOFT_RESET (SOFT_RESET_BASE<<(ADDR_W-N_SLAVES_W))
#define ETHERNET (ETHERNET_BASE<<(ADDR_W-N_SLAVES_W))

#define ETH_NBYTES (256-18) //minimum ethernet payload excluding FCS

#define DDR_MEM (CACHE_BASE<<(ADDR_W-N_SLAVES_W))
#define WEIGHTS (0x00004000) //weights 0x0000 4000 to 0x0280 3FFF
#define DATA (0x02804000) //data 0x0280 4000 to 0x0500 3FFF

//note that the DDR is only addresses with 0x8000 0000 check this

int main() {

  //init UART
  uart_init(UART,UART_CLK_FREQ/UART_BAUD_RATE);

  //send init message
  uart_printf("\nDDR TEST\n");
  uart_printf("\nbefore txwait\n");
  uart_txwait();

  uart_printf("After txwait\n");

  //Write loops
  //Weights vector
  int i = 0;

  volatile int *weights = (volatile int*) (DDR_MEM + WEIGHTS);
  int num_weights = (DATA - WEIGHTS)/sizeof(int);
  int w_step = num_weights/100000;

  uart_printf("Writing weights to DDR...");

  for(i=0;i<num_weights;i+=w_step){
    weights[i] = i;
    //if(i%(50000)==0){
    //uart_printf("ww: %d\n", i);
      //}
  }

  uart_printf("done\n");

  //Data vector
  volatile int *data = (volatile int*) (DDR_MEM + DATA);
  int num_data = (DATA - WEIGHTS)/sizeof(int);
  int d_step = num_data/100000;

  uart_printf("Writing data to DDR...");

  for(i=0;i<num_data;i+=d_step){
    data[i] = i;
    //    if(i%(50000)==0){
    //  uart_printf("wd: %d\n", i);
      //x}
  }

  uart_printf("done\n");

  uart_printf("num_weights = %d\tnum_data = %d\n", num_weights, num_data);

  //Read loops
  int w = -1, d = -1;
  int err=0;
  int acc=0;

  uart_printf("\nReading and verifying weights...");

  for(i=0; i<num_weights;i+=w_step){
    w = weights[i];
    acc++;
    //    if(i%50000 == 0){
    //uart_printf("rw: %d\n", i);
      //}
    if(w != i){
      uart_printf("Wrong weight %d: %d at pos %p\n", i, w, weights[i]);
      err++;
    }
  }
  
  uart_printf("done\n");

  uart_printf("\nReading and verifying weights...");
  
  for(i=0; i<num_data;i+=d_step){
    d = data[i];
    acc++;
    //    if(i%50000 == 0){
    //uart_printf("rd: %d\n", i);
      //}
    if(d != i){
      uart_printf("Wrong data %d: %d at pos %p\n", i, d, data[i]);
      err++;
    }
  }

  uart_printf("done\n");

  uart_printf("DDR Test terminated with %d errors out of %d read operations\n", err, acc);

  //end program
  uart_putc(4);
  return 0;
}
