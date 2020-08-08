//import custom libraries
#include "system.h"
#include "periphs.h"

#include "iob-uart.h"
#include "iob_timer.h"
#include "versat.hpp"

//import c libraries
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

int main(int argc, char **argv) {

  //local variables
  int i, j, k, l, m;
  int16_t pixels[25*nSTAGE], weights[9*nSTAGE], bias, res;
  unsigned int start, end;

  //init UART
  uart_init(UART_BASE,FREQ/BAUD);

  //send init message
  uart_printf("\nVERSAT TEST \n\n");

  //init VERSAT
  start = timer_time_us(TIMER_BASE);
  versat_init(VERSAT_BASE);
  end = timer_time_us(TIMER_BASE);
  uart_printf("Deep versat initialized in %d us\n", (end-start));

  //write data in versat mems
  start = timer_time_us(TIMER_BASE);
  for(j = 0; j < nSTAGE; j++) {

    //write 5x5 feature map in mem0
    for(i = 0; i < 25; i++) {
      pixels[25*j+i] = rand()%50-25;
      stage[j].memA[0].write(i, pixels[25*j+i]);
    }

    //write 3x3 kernel in mem1
    for(i = 0; i < 9; i++) {
      weights[9*j+i] = rand()%10-5;
      stage[j].memA[1].write(i, weights[9*j+i]);
    }

    //write bias in mem2 of versat0
    if(j == 0) {
      bias = rand()%20-10;
      stage[j].memA[2].write(0, bias);
    }
  }
  end = timer_time_us(TIMER_BASE);
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

  /////////////////////////////////////////////////////////////////////////////////
  // 3D CONVOLUTION WITH 2-LOOP ADDRGEN
  /////////////////////////////////////////////////////////////////////////////////

  uart_printf("\n3D CONVOLUTION WITH 2-LOOP ADDRGEN\n");

  //configure mem2A to read bias
  int delay = 0, in_1_alulite = sMEMA[2];
  start = timer_time_us(TIMER_BASE);
  stage[0].memA[2].setIter(1);
  stage[0].memA[2].setPer(9);
  stage[0].memA[2].setDuty(9);

  //loop to configure versat stages
  for(i = 0; i < nSTAGE; i++) {

    //configure mem0A to read 3x3 block from feature map
    stage[i].memA[0].setIter(3);
    stage[i].memA[0].setIncr(1);
    stage[i].memA[0].setDelay(delay);
    stage[i].memA[0].setPer(3);
    stage[i].memA[0].setDuty(3);
    stage[i].memA[0].setShift(5-3);
  
    //configure mem1A to read kernel
    stage[i].memA[1].setIter(1);
    stage[i].memA[1].setIncr(1);
    stage[i].memA[1].setDelay(delay);
    stage[i].memA[1].setPer(9);
    stage[i].memA[1].setDuty(9);

    //configure muladd0
    stage[i].muladd[0].setSelA(sMEMA[0]);
    stage[i].muladd[0].setSelB(sMEMA[1]);
    stage[i].muladd[0].setFNS(MULADD_MACC);
    stage[i].muladd[0].setIter(1);
    stage[i].muladd[0].setPer(9);
    stage[i].muladd[0].setDelay(MEMP_LAT + delay);

    //configure ALULite0 to add bias to muladd result
    stage[i].alulite[0].setOpA(in_1_alulite);
    stage[i].alulite[0].setOpB(sMULADD[0]);
    stage[i].alulite[0].setFNS(ALULITE_ADD);

    //update variables
    if(i==0) in_1_alulite = sALULITE_p[0];
    if(i!=nSTAGE-1) delay += 2;
  }

  //config mem2A to store ALULite output
  stage[nSTAGE-1].memA[2].setIter(1);
  stage[nSTAGE-1].memA[2].setIncr(1);
  stage[nSTAGE-1].memA[2].setDelay(MEMP_LAT + 8 + MULADD_LAT + ALULITE_LAT + delay);
  stage[nSTAGE-1].memA[2].setPer(1);
  stage[nSTAGE-1].memA[2].setDuty(1);
  stage[nSTAGE-1].memA[2].setSel(sALULITE[0]);
  stage[nSTAGE-1].memA[2].setInWr(1);
  end = timer_time_us(TIMER_BASE);
  uart_printf("\nConfigurations (except start) made in %d us\n", (end-start));
  
  //perform convolution
  start = timer_time_us(TIMER_BASE);
  for(i = 0; i < 3; i++) {
    for(j = 0; j < 3; j++) {

      //configure start values of memories
      for(k = 0; k < nSTAGE; k++) stage[k].memA[0].setStart(i*5+j);
      stage[nSTAGE-1].memA[2].setStart(i*3+j);

      //run configurations
      run();

      //wait until done is done
      while(done() == 0);
    }
  }  
  end = timer_time_us(TIMER_BASE);
  uart_printf("\n3D convolution done in %d us\n", (end-start));

  //display results
  uart_printf("\nActual convolution result\n");
  for(i = 0; i < 3; i++) {
    for(j = 0; j < 3; j++) uart_printf("%d\t", stage[nSTAGE-1].memA[2].read(i*3+j));
    uart_printf("\n");
  }

  /////////////////////////////////////////////////////////////////////////////////
  // 3D CONVOLUTION WITH 4-LOOP ADDRGEN
  /////////////////////////////////////////////////////////////////////////////////

  uart_printf("\n3D CONVOLUTION WITH 4-LOOP ADDRGEN\n");

  //loop to configure versat stages
  delay = 0, in_1_alulite = sMEMA[2];
  start = timer_time_us(TIMER_BASE);

  //configure mem2A of versat0 to read bias
  stage[0].memA[2].setIter(9);

  for(i = 0; i < nSTAGE; i++) {

    //configure mem0A to read all 3x3 blocks from feature map
    stage[i].memA[0].setIter2(3); 
    stage[i].memA[0].setPer2(3); 
    stage[i].memA[0].setShift2(5-3); 
    stage[i].memA[0].setIncr2(1); 
    stage[i].memA[0].setStart(0);
  
    //configure mem1A to read kernel
    stage[i].memA[1].setIter(9);
    stage[i].memA[1].setIncr(1);
    stage[i].memA[1].setDelay(delay);
    stage[i].memA[1].setPer(9);
    stage[i].memA[1].setDuty(9);
    stage[i].memA[1].setShift(-9);

    //configure muladd0
    stage[i].muladd[0].setIter(9);

    //configure ALULite0 to add bias to muladd result
    stage[i].alulite[0].setOpA(in_1_alulite);

    //update variables
    if(i==0) in_1_alulite = sALULITE_p[0];
    if(i!=nSTAGE-1) delay += 2;
  }

  //config mem2A to store ALULite output
  //start, iter, incr, delay, per, duty, sel, shift, in_wr
  stage[nSTAGE-1].memA[2].setStart(10);
  stage[nSTAGE-1].memA[2].setIter(9);
  stage[nSTAGE-1].memA[2].setIncr(1);
  stage[nSTAGE-1].memA[2].setDelay(MEMP_LAT + MULADD_LAT + ALULITE_LAT + delay - 1);
  stage[nSTAGE-1].memA[2].setPer(9);
  stage[nSTAGE-1].memA[2].setDuty(1);
  stage[nSTAGE-1].memA[2].setSel(sALULITE[0]);
  stage[nSTAGE-1].memA[2].setInWr(1);
  end = timer_time_us(TIMER_BASE);
  uart_printf("\nConfigurations (except start) made in %d us\n", (end-start));
  
  //perform convolution
  start = timer_time_us(TIMER_BASE);
  run();
  while(done() == 0);
  end = timer_time_us(TIMER_BASE);
  uart_printf("\n3D convolution done in %d us\n", (end-start));

  //display results
  uart_printf("\nActual convolution result\n");
  for(i = 0; i < 3; i++) {
    for(j = 0; j < 3; j++) uart_printf("%d\t", (int16_t) stage[nSTAGE-1].memA[2].read(i*3+j+10));
    uart_printf("\n");
  }

  //clear conf_reg of VERSAT 0
  stage[0].clearConf();

#ifdef CONF_MEM_USE
  //store conf_reg of VERSAT 1 in conf_mem (addr 0)
  stage[1].confMemWrite(0);
#endif

  //global conf clear
  globalClearConf();

#ifdef CONF_MEM_USE
  //store conf_mem (addr 0) in conf_reg of VERSAT2
  stage[1].confMemRead(0);
#endif

  //return data
  uart_printf("\n");
  uart_putc(ETX);
  return 0;
}
