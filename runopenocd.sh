#!/bin/bash

~/git/openocd/src/openocd -s ~/git/openocd/tcl -f interface/stlink.cfg -f target/stm32f3x.cfg
