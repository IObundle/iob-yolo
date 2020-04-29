//import custom libraries
#include "system.h"
#include "iob-uart.h"
#include "iob_timer.h"
#include "versat.hpp"

//import c libraries
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

//define peripheral base addresses
#define UART (UART_BASE<<(DATA_W-N_SLAVES_W))
#define TIMER (TIMER_BASE<<(ADDR_W-N_SLAVES_W))
#define VERSAT (VERSAT_BASE<<(ADDR_W-N_SLAVES_W))

int main(int argc, char **argv) {

  //local variables
  int i, j, k, l, m;
  int pixels[25*nSTAGE], weights[9*nSTAGE], bias, res;
  unsigned int start, end;

  //init UART
  uart_init(UART,UART_CLK_FREQ/UART_BAUD_RATE);

  //send init message
  uart_printf("\nVERSAT TEST \n\n");
  uart_txwait();

  //init VERSAT
  start = timer_get_count_us(TIMER);
  versat_init(VERSAT);
  end = timer_get_count_us(TIMER);
  uart_printf("Deep versat initialized in %d us\n", (end-start));

  //write data in versat mems
  start = timer_get_count_us(TIMER);
  for(j = 0; j < nSTAGE; j++) {

    //write 5x5 feature map in mem0
    for(i = 0; i < 25; i++) {
      pixels[25*j+i] = rand()%50-25;
      versat[j].memA[0].write(i, pixels[25*j+i]);
    }

    //write 3x3 kernel and bias in mem1
    for(i = 0; i < 9; i++) {
      weights[9*j+i] = rand()%10-5;
      versat[j].memA[1].write(i, weights[9*j+i]);
    }

    //write bias after weights of VERSAT 0
    if(j == 0) {
      bias = rand()%20-10;
      versat[j].memA[1].write(9, bias);
    }
  }
  end = timer_get_count_us(TIMER);
  uart_printf("\nData stored in versat mems in %d us\n", (end-start));

  //expected result of 3D convolution
  uart_printf("\nExpected result of 3D convolution\n");
  for(i = 0; i < 3; i++) {
    for(j = 0; j < 3; j++) {
      res = bias;
      for(k = 0; k < nSTAGE; k++) {
        for(l = 0; l < 3; l++) {
          for(m = 0; m < 3; m++) {
            res += pixels[i*5+j+k*25+l*5+m] * weights[9*k+l*3+m];
          }
        }
      }
      uart_printf("%d\t", res);
    }
    uart_printf("\n");
  }

  //loop to configure versat stages
  int delay = 0, in_1_alulite = sMEMA[1];
  start = timer_get_count_us(TIMER);
  for(i = 0; i < nSTAGE; i++) {

    //configure mem0A to read 3x3 block from feature map
    //start, iter, incr, delay, per, duty, sel, shift, in_wr
    versat[i].memA[0].setConf(0, 3, 1, delay, 3, 3, 0, 5-3, 0);
    versat[i].memA[0].writeConf();
  
    //configure mem1A to read kernel
    versat[i].memA[1].setConf(0, 10, 1, delay, 1, 1, 0, 0, 0);
    versat[i].memA[1].writeConf();

    //configure muladd0
    versat[i].muladd[0].setConf(sMEMA[0], sMEMA[1], MULADD_MUL_LOW_MACC, 1, 9, MEMP_LAT + delay);
    versat[i].muladd[0].writeConf();

    //configure ALULite0 to add bias to muladd result
    versat[i].alulite[0].setConf(in_1_alulite, sMULADD[0], ALULITE_ADD);
    versat[i].alulite[0].writeConf();

    //update variables
    if(i==0) in_1_alulite = sALULITE_p[0];
    if(i!=nSTAGE-1) delay += 2;
  }

  //config mem2A to store ALULite output
  versat[nSTAGE-1].memA[2].setConf(0, 1, 1, MEMP_LAT + 8 + MULADD_LAT + ALULITE_LAT + delay, 1, 1, sALULITE[0], 0, 1);
  versat[nSTAGE-1].memA[2].writeConf();
  end = timer_get_count_us(TIMER);
  uart_printf("\nConfigurations (except start) made in %d us\n", (end-start));
  
  //perform convolution
  start = timer_get_count_us(TIMER);
  for(i = 0; i < 3; i++) {
    for(j = 0; j < 3; j++) {

      //configure start values of memories
      for(k = 0; k < nSTAGE; k++) versat[k].memA[0].setStart(i*5+j);
      versat[nSTAGE-1].memA[2].setStart(i*3+j);

      //run configurations
      run();

      //wait until done is done
      while(done() == 0);
    }
  }  
  end = timer_get_count_us(TIMER);
  uart_printf("\n3D convolution done in %d us\n", (end-start));

  //display results
  uart_printf("\nActual convolution result\n");
  for(i = 0; i < 3; i++) {
    for(j = 0; j < 3; j++) uart_printf("%d\t", versat[nSTAGE-1].memA[2].read(i*3+j));
    uart_printf("\n");
  }

  //clear conf_reg of VERSAT 0
  versat[0].clearConf();

#ifdef CONF_MEM_USE
  //store conf_reg of VERSAT 1 in conf_mem (addr 0)
  versat[1].confMemWrite(0);
#endif

  //global conf clear
  globalClearConf();

#ifdef CONF_MEM_USE
  //store conf_mem (addr 0) in conf_reg of VERSAT2
  versat[1].confMemRead(0);
#endif

  //return data
  uart_printf("\n");
  uart_putc(4);
  return 0;
}
