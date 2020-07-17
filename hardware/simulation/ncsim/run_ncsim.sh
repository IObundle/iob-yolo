#!/bin/bash
source /opt/ic_tools/init/init-xcelium1903-hf013
xmvlog $CFLAGS $VSRC
xmelab $EFLAGS worklib."$1"_tb:module
xmsim  $SFLAGS worklib."$1"_tb:module
