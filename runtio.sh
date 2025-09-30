#!/bin/bash

# -m OCRNL stands for send '\r\n' for newline, where
# CR is 'carriage return' (or '\r') and NL is a 'new line' (or '\n')
tio /dev/ttyJTAG -m OCRNL
