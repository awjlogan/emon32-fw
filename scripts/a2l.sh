#!/bin/bash
# Convert an address to the source line

arm-none-eabi-addr2line --exe build/emon32.elf "$1"

