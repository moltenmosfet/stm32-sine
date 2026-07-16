OPENCM3 = ../../libopencm3
PREFIX  = arm-none-eabi-

CFLAGS  = -Os -g -Wall -Wextra -mcpu=cortex-m3 -mthumb -DSTM32F1 \
          -I$(OPENCM3)/include -ffunction-sections -fdata-sections
LDFLAGS = -T f103c8.ld -L$(OPENCM3)/lib -L$(OPENCM3)/lib/stm32/f1 \
          -nostartfiles -Wl,--gc-sections --specs=nano.specs --specs=nosys.specs

all: gates.bin

gates.elf: main.c f103c8.ld
	$(PREFIX)gcc $(CFLAGS) main.c $(LDFLAGS) -lopencm3_stm32f1 -o $@
	$(PREFIX)size $@

gates.bin: gates.elf
	$(PREFIX)objcopy -O binary $< $@

flash: gates.bin
	st-flash --reset write gates.bin 0x8000000

flash-ocd: gates.elf
	openocd -f interface/stlink.cfg -f target/stm32f1x.cfg \
	        -c "program gates.elf verify reset exit"

clean:
	rm -f gates.elf gates.bin
