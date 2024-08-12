#!/bin/sh -eux
arm-none-eabi-objcopy -O binary release/midiguitar.elf midiguitar.bin
dfu-util -s 0x08000000 -a 0 -D midiguitar.bin
