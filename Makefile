#
# Author, Copyright: Oleg Borodin <onborodin@gmail.com> 2018
#
.SECONDARY:

RTOS_LOC = RTOS

CFLAGS+= -I. -Os -mthumb -mcpu=cortex-m3 -march=armv7-m -msoft-float -DSTM32F1 -std=c99
CFLAGS+= -fno-common -ffunction-sections -fdata-sections
CFLAGS+= -g -gdwarf-2
CFLAGS+= -I${RTOS_LOC}

LDFLAGS+= ${CFLAGS}
LDFLAGS+= --static
LDFLAGS+= -nostartfiles
LDFLAGS+= -T master.ld

LDFLAGS+= -Wl,-Map=master.map
LDFLAGS+= -Wl,--cref -Wl,--gc-sections
LDFLAGS+= -Wl,--start-group -lc -lm -lgcc -Wl,--end-group
LDFLAGS+= -lopencm3_stm32f1
LDFLAGS+= -L${RTOS_LOC} -lrtos



RTOS_OBJS+= $(RTOS_LOC)/croutine.o
RTOS_OBJS+= $(RTOS_LOC)/event_groups.o
RTOS_OBJS+= $(RTOS_LOC)/heap_3.o
RTOS_OBJS+= $(RTOS_LOC)/list.o
RTOS_OBJS+= $(RTOS_LOC)/port.o
RTOS_OBJS+= $(RTOS_LOC)/queue.o
RTOS_OBJS+= $(RTOS_LOC)/stream_buffer.o
RTOS_OBJS+= $(RTOS_LOC)/tasks.o
RTOS_OBJS+= $(RTOS_LOC)/timers.o

MASTER_OBJS+= master.o
MASTER_OBJS+= syscall.o
MASTER_OBJS+= opencm3.o
MASTER_OBJS+= st7735.o
MASTER_OBJS+= console.o
MASTER_OBJS+= static.o
MASTER_OBJS+= i2creg.o
MASTER_OBJS+= mpu6050.o


all: rtos master.bin

rtos:  $(RTOS_LOC)/librtos.a

${RTOS_OBJS}: FreeRTOSConfig.h


$(RTOS_LOC)/librtos.a: $(RTOS_OBJS)
	cd $(@D) && arm-eabi-ar rcv $(@F) $(^F)

master.elf: $(MASTER_OBJS) 
	arm-eabi-gcc $(^F) $(LDFLAGS) -o $@ 
	arm-eabi-size --format=berkeley $@

%.o: %.c
	arm-eabi-gcc $(CFLAGS) -c -o $@ $<

%.o: %.S
	arm-eabi-as $(ASFLAGS) -o $@ $<

%.bin: %.elf
	arm-eabi-objcopy -O binary $< $@

%.elf: %.o
	arm-eabi-gcc $(^F) $(LDFLAGS) -o $@ 
	arm-eabi-size --format=berkeley $@

clean:
	rm -f *.i *.o *.elf *.bin *.map *~ *.hex *.d *.s
	cd $(RTOS_LOC) && rm -f *.o *.d *~ lib*.a


upload: all master.upl

%.upl: %.bin
	@openocd \
	    -c 'puts "--- START --------------------"' \
	    -f 'interface/stlink-v2.cfg' \
	    -f 'target/stm32f1x.cfg'  \
	    -c 'puts "--- INIT --------------------"' \
	    -c "init" \
	    -c "reset halt" \
	    -c 'puts "--- WRITE --------------------"' \
	    -c "flash write_image erase $< 0x08000000"\
	    -c 'puts "--- VERIFY --------------------"' \
	    -c "verify_image $<" \
	    -c 'puts "--- RESET --------------------"' \
	    -c "reset" \
	    -c 'puts "--- DONE --------------------"' \
	    -c "shutdown"

debug:
	@openocd \
	    -c 'puts "--- START --------------------"' \
	    -f 'interface/stlink-v2.cfg' \
	    -f 'target/stm32f1x.cfg'  \
	    -c 'puts "--- INIT --------------------"' \
	    -c "init" \
	    -c "halt" \
	    -c "poll"
#EOF
