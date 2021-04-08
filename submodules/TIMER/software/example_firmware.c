#include "system.h"
#include "periphs.h"
#include "iob-uart.h"
#include "iob_timer.h"

int main()
{
  unsigned long long elapsed;
  unsigned int elapsedu;

  //init timer and uart
  timer_init(TIMER_BASE);
  uart_init(UART_BASE, FREQ/BAUD);

  uart_printf("\nHello world!\n");

  //read current timer count, compute elapsed time
  elapsed  = timer_get_count();
  elapsedu = timer_time_us();

  uart_printf("\nExecution time: %d clock cycles\n", (unsigned int) elapsed);
  uart_printf("\nExecution time: %dus @%dMHz\n\n", elapsedu, FREQ/1000000);

  uart_finish();
}
