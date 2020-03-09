#!/bin/bash
source /opt/ic_tools/init/init-xcelium1903-hf013
xmvlog $CFLAGS $PROG_SIZE $VSRC
xmelab $EFLAGS worklib."$TEST"_tb:module
xmsim  $SFLAGS worklib."$TEST"_tb:module
