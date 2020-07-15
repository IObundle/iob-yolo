#include "system.h"
#include "periphs.h"

#include "iob-uart.h"
#include "iob_timer.h"

//Set DDR pointer
#ifdef PCSIM
static int ddr[10000];
#else
  #if (USE_DDR==1)
    #if (RUN_DDR==0)
      int *ddr = (int*) EXTRA_BASE;
    #else
      int *ddr = (int*) (1<<(FIRM_ADDR_W+1));
    #endif
  #endif
#endif //ifdef PCSIM


int main()
{ 
  //init uart 
  uart_init(UART_BASE,FREQ/BAUD);   
  
  //timer variables
  unsigned int start, end;

  timer_reset(TIMER_BASE);
  start = timer_time_us(TIMER_BASE);

  uart_printf("\n\n\nHello world!\n\n\n");

#if (USE_DDR==1)
  int i=0;
  int N=1024;
  int count=0;
  //test DDR:
  uart_printf("Performing DDR test:\n");

  uart_printf("\tWriting to DDR...");
  
  for(i=0;i<N;i++){
    ddr[i] = N-i;
  }

  uart_printf("done!\n");
  
  uart_printf("\tReading from DDR...\n");
  int j;

  for(i=0;i<N;i++){
    if(ddr[i] != (N-i)){
      uart_printf("ddr[%d]=%d, expected %d\n", i, ddr[i], N-i);
      count++;
    }
  }

  uart_printf("Test complete with %d errors out of %d\n", count, N);

  
#endif
  
  end = timer_time_us(TIMER_BASE);

  uart_printf("Elapsed time: %dus\n", (end-start));

}
