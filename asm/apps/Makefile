ASM_FLAGS=-I../lib

all: tiny_basic.bin z80_uart_monitor.bin

clean:
	rm -f ldr
	rm -f *.lst *.bin *.hex *.sym *.cim

DATE := $(shell date --rfc-3339=seconds)
GIT_VERSION := $(shell git describe --long --dirty; git show -s --format='%ci')

%.bin: %.asm
	#cat $< | sed -e "s/@@DATE@@/$(DATE)/g" -e "s/@@GIT_VERSION@@/$(GIT_VERSION)/g" | z80asm - -o $@ --list=$(basename $@).lst --label=$(basename $@).sym $(ASM_FLAGS)
	#z80asm - -o $@ --list=$(basename $@).lst --label=$(basename $@).sym $(ASM_FLAGS)
	echo "BUILDING $(basename $@).cim from $<"
	zmac --od . --oo cim -o $(basename $@).cim -L -l $< >$(basename $@).lst
	mv $(basename $@).cim $(basename $@).bin


world: clean all

