#include "stdlib.h"
#include <stdio.h>
#include "system.h"
#include "periphs.h"
#include "iob-uart.h"
#include "printf.h"

int main()
{
  //init uart
  uart_init(UART_BASE,FREQ/BAUD);   

  run_test();

  uart_finish();
}
