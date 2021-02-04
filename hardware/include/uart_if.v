// UART interface signals
`OUTPUT(uart_txd,  1), //Transmit data channel
`INPUT(uart_rxd,   1), //Receive data channel
`OUTPUT(uart_rts,  1), //Request to Send flow control signal
`INPUT(uart_cts,   1), //Clear to Send flow control signal
