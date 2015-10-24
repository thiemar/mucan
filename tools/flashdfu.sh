#!/bin/bash

python startdfu.py $1
sleep 1 && \
dfu-util --device 0483:df11 -a 0 --dfuse-address 0x08000000 --download ../firmware/app.bin && \
sleep 1 && \
dfu-util --device 0483:df11 -a 1 --dfuse-address 0x1FFFF800:leave --download ../firmware/ob.bin > /dev/null 2>&1 && \
echo "DFU completed"
