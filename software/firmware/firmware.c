#include "stdlib.h"
#include <stdio.h>
#include "system.h"
#include "periphs.h"
#include "iob-uart.h"
#include "printf.h"
#include "iob_timer.h"

void run();

int main()
{
  //init uart
  uart_init(UART_BASE,FREQ/BAUD);   

  //init timer
  timer_init(TIMER_BASE);

  run();

  uart_finish();
}
