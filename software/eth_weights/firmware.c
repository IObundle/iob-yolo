#include "system.h"
#include "iob-uart.h"
#include "iob-eth.h"
#include "iob_timer.h"

#include <stdint.h>
#include <stdio.h>
#include <string.h>

#define UART (UART_BASE<<(DATA_W-N_SLAVES_W))
#define SOFT_RESET (SOFT_RESET_BASE<<(ADDR_W-N_SLAVES_W))
#define ETHERNET (ETHERNET_BASE<<(ADDR_W-N_SLAVES_W))
#define TIMER (TIMER_BASE<<(ADDR_W-N_SLAVES_W))

#define ETH_NBYTES (256-18) //minimum ethernet payload excluding FCS
#define DATA_FILE_SIZE (418*418*3*2) //16 bits per input
#define WEIGHTS_FILE_SIZE (17704732) //16 bits per input
#define NUM_DATA_FRAMES (DATA_FILE_SIZE/ETH_NBYTES)
#define NUM_WEIGHT_FRAMES (WEIGHTS_FILE_SIZE/ETH_NBYTES)

int main() {

  //init UART
  uart_init(UART,UART_CLK_FREQ/UART_BAUD_RATE);

  //send init message
  uart_printf("\nETHERNET TEST\n");
  uart_txwait();

  //init ETHERNET
  eth_init(ETHERNET);
  eth_set_rx_payload_size(ETH_NBYTES);

  //ETHERNET variables
  int rcv_timeout = 5000;
  char data_to_send[ETH_NBYTES];
  char data_rcv[ETH_NBYTES+18];

  //Local variables
  int i, j;
  int16_t input_data[ETH_NBYTES/2];
  char * input_data_p = (char *) &input_data;
  uart_printf("\n");

  //transmission synchronization

  uart_printf("CHANGED FIRMWARE\n");

  //wait for PC message
  while(eth_rcv_frame(data_rcv, ETH_NBYTES+18, rcv_timeout)!= 0);

  uart_printf("PC Initial message\n");

  //check data received
  if ((data_rcv[14] != 'P') || (data_rcv[15] != 'C') || (data_rcv[16] != '\n')){
    uart_printf("Failed synchronization: %s", &(data_rcv[14]));
    uart_putc(4);
    return 0;
  }
  else{
    uart_printf("Synchronized with PC\n");
  }


  data_to_send[0] = 'O';
  data_to_send[1] = 'K';
  data_to_send[2] = '\n';

  eth_send_frame(data_to_send, ETH_NBYTES);

  //measure initial time for data transmission
  int start = timer_get_count(TIMER);

  int wait = 0;
  //Loop to receive and send back data frames
  for(j = 0; j < NUM_DATA_FRAMES+1; j++) {

     //wait to receive frame
    /* if (j==0){ */
      while(eth_rcv_frame(data_rcv, ETH_NBYTES+18, rcv_timeout) !=0){
      //uart_printf("[%d] RCV wait loop\n", j);
      }
    /* } */
    /* else{ */
    /*   wait = 0; */
    /*   while(eth_rcv_frame(data_rcv, ETH_NBYTES+18, rcv_timeout) !=0){ */
    /* 	wait++; */
    /* 	if (wait == 500){ */
    /* 	  break; */
    /* 	} */
    /*   } */
    /* } */
     //save in local mem
     for(i = 0; i < ETH_NBYTES; i++) input_data_p[i] = data_rcv[14+i];

     //prepare to send back
     for(i = 0; i < ETH_NBYTES; i++) data_to_send[i] = input_data_p[i];

     /* //init ETHERNET */
     /* eth_init(ETHERNET); */
     /* eth_set_rx_payload_size(ETH_NBYTES); */

     //send frame back
     /* eth_send_frame(data_to_send, ETH_NBYTES); */
     //uart_printf("[%d]\n", j);

     /* //reset core */
     /* MEMSET(ETHERNET, ETH_SOFTRST, 1); */
     /* MEMSET(ETHERNET, ETH_SOFTRST, 0); */
     uart_printf("iter %d, MAC_dest: ", j);
     for(i = 0; i < 6; i++) uart_printf("%x", data_rcv[i]);
     /* uart_printf(", MAC_src: "); */
     /* for(i = 0; i < 6; i++) uart_printf("%x", data_rcv[6+i]); */
     /* uart_printf(", ETH_TYPE: "); */
     /* for(i = 0; i < 2; i++) uart_printf("%x", data_rcv[12+i]); */
     uart_printf("\n");

  }

  //measure final time for data transmission
  int end = timer_get_count(TIMER);
  uart_printf("Data transmission done in %d us\n", ((end-start)*1000000)/UART_CLK_FREQ);

  /* //measure initial time for weight transmission */
  /* timer_reset(TIMER); */
  /* start = timer_get_count(TIMER); */

  /* //Loop to receive and send back weight frames */
  /* for(j = 0; j < NUM_WEIGHT_FRAMES+1; j++) { */

  /*    //wait to receive frame */
  /*    while(eth_rcv_frame(data_rcv, ETH_NBYTES+18, rcv_timeout) !=0); */

  /*    //save in local mem */
  /*    for(i = 0; i < ETH_NBYTES; i++) input_data_p[i] = data_rcv[14+i]; */

  /*    //prepare to send back */
  /*    for(i = 0; i < ETH_NBYTES; i++) data_to_send[i] = input_data_p[i]; */

  /*    //send frame back */
  /*    eth_send_frame(data_to_send, ETH_NBYTES); */
  /* } */

  //measure final time for weight transmission
  end = timer_get_count(TIMER);
  uart_printf("Weight transmission done in %d us\n", ((end-start)*1000000)/UART_CLK_FREQ);

  //end program
  uart_putc(4);
  return 0;
}
