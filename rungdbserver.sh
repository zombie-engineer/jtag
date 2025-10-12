#!/bin/bash

python3 ./gdbserver.py -v --serial /dev/ttyJTAG --osdesc os.json --log-rsp
