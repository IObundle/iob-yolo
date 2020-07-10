#include "system.h"
#include "periphs.h"

#include "iob-uart.h"

//Set DDR pointer
#if (USE_DDR==1)
  #if (RUN_DDR==0)
    int *ddr = (int*) EXTRA_BASE;
  #else
    int *ddr = (int*) (1<<(FIRM_ADDR_W+1));
  #endif
#endif
int main()
{ 
  //init uart 
  uart_init(UART_BASE,FREQ/BAUD);   
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
  
  for(i=0;i<N;i++){
    if(ddr[i] != (N-i)){
      uart_printf("ddr[%d]=%d, expected %d\n", i, ddr[i], N-i);
      count++;
    }
  }

  uart_printf("Test complete with %d errors out of %d\n", count, N);

  
#endif
  
}
