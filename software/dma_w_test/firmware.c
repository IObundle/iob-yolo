//import custom libraries
#include "system.h" //must be defined before versat API!!!
#include "periphs.h"
#include "iob-uart.h"
#include "iob_timer.h"

//import c libraries
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

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

int main(int argc, char **argv) {

  //init UART
  uart_init(UART_BASE,FREQ/BAUD);

  //send init message
  uart_printf("\nDMA TEST\n\n");

  //set positions to zero due to output padding
  int16_t * val = (int16_t *) (DDR_MEM + 0x1000);
  int i;
  for(i = 0; i < 16; i++) uart_printf("(%x) %x\n", &val[16*i], val[16*i]);

  //finish
  uart_putc(ETX);
  return 0;
}
