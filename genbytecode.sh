#!/bin/bash

INSTR=$1
CROSSPREFIX=/home/user_user/gcc-arm-10.3-2021.07-x86_64-aarch64-none-elf/bin/aarch64-none-elf

AS=$CROSSPREFIX-as
OBJDUMP=$CROSSPREFIX-objdump
TMP_S_FILE=/tmp/bytecode.S
TMP_O_FILE=/tmp/bytecode.o
rm $TMP_S_FILE 2>/dev/null
echo $1 > $TMP_S_FILE
x=$($AS $TMP_S_FILE -o $TMP_O_FILE && $OBJDUMP -d $TMP_O_FILE | tail -1 | cut -f2)
echo 0x$x
