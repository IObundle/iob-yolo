#include "system.h"
#include "iob-uart.h"
#include "iob_timer.h"

#include <stdint.h>
#include <stdio.h>
#include <string.h>

#define UART (UART_BASE<<(DATA_W-N_SLAVES_W))
#define DDR_MEM (CACHE_BASE<<(ADDR_W-N_SLAVES_W))
#define WEIGHTS (DDR_MEM + 0x00004000) //weights 0x0000 4000 to 0x0280 3FFF
#define DATA (DDR_MEM + 0x02804000) //data 0x0280 4000 to 0x0500 3FFF
#define TIMER (TIMER_BASE<<(ADDR_W-N_SLAVES_W))

//note that the DDR is only addresses with 0x8000 0000 check this

int main() {

  //init UART
  uart_init(UART, UART_CLK_FREQ/UART_BAUD_RATE);

  //send init message
  uart_printf("\nDDR TEST\n");
  uart_txwait();

  //Write loops

  //Weights vector
  int i = 0;
  volatile int *weights = (volatile int*) (WEIGHTS);
  int num_weights = (DATA - WEIGHTS)/sizeof(int);
  int w_step = num_weights/100000;
  uart_printf("\nWriting weights to DDR...\n");
  int start = timer_get_count(TIMER);
  for(i = 0; i < num_weights; i += w_step) weights[i] = i;
  int end = timer_get_count(TIMER);
  uart_printf("done in %d us\n", ((end-start)*1000000)/UART_CLK_FREQ);

  //Data vector
  volatile int *data = (volatile int*) (DATA);
  int num_data = (DATA - WEIGHTS)/sizeof(int);
  int d_step = num_data/100000;
  uart_printf("\nWriting data to DDR...\n");
  start = timer_get_count(TIMER);
  for( i = 0; i < num_data; i += d_step) data[i] = i;
  end = timer_get_count(TIMER);
  uart_printf("done in %d us\n", ((end-start)*1000000)/UART_CLK_FREQ);
  uart_printf("num_weights = %d\tnum_data = %d\n", num_weights, num_data);

  //Read loops
  int w = -1, d = -1;
  int err=0;
  int acc=0;

  //Weights vector
  uart_printf("\nReading and verifying weights...\n");
  start = timer_get_count(TIMER);
  for(i = 0; i < num_weights; i += w_step) {
    w = weights[i];
    acc++;
    if(w != i) {
      uart_printf("Wrong weight %d: %d at pos %p\n", i, w, weights[i]);
      err++;
    }
  }
  end = timer_get_count(TIMER);
  uart_printf("done in %d us\n", ((end-start)*1000000)/UART_CLK_FREQ);

  //Data vector
  uart_printf("\nReading and verifying data...\n");
  start = timer_get_count(TIMER);
  for(i = 0; i < num_data; i += d_step){
    d = data[i];
    acc++;
    if(d != i) {
      uart_printf("Wrong data %d: %d at pos %p\n", i, d, data[i]);
      err++;
    }
  }
  end = timer_get_count(TIMER);
  uart_printf("done in %d us\n", ((end-start)*1000000)/UART_CLK_FREQ);
  uart_printf("\nDDR Test terminated with %d errors out of %d read operations\n", err, acc);

  //end program
  uart_putc(4);
  return 0;
}
