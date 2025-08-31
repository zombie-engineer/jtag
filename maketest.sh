#!/bin/bash
gcc -g test.c -ICore/Src -ICore/Inc Core/Src/cmd.c -o test_parse \
&& ./test_parse
