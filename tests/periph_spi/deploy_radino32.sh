BOARD=radino32 make
arm-none-eabi-objcopy -I ihex -O binary bin/radino32/tests_periph_spi.hex bin/radino32/tests_periph_spi.bin
dfu-util -a 0 -s 0x08000000:leave -D bin/radino32/tests_periph_spi.bin