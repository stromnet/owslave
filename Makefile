# makefile, written by guido socher
MCU=atmega88a
#MCU=atmega8		# ds2423.c won't compile
#MCU=attiny13		# ds2423.c and uart.c won't compile (no uart)
#MCU=attiny84		# uart.c won't compile (no uart)
#MCU=atmega168
#MCU=atmega32		# ds2423.c won't compile

MAKE=gmake

#MCU_PROG=m168
#MCU_PROG=m32
#MCU_PROG=t84
MCU_PROG=m8
#PROG=usbtiny
#PROG=ponyser -P /dev/ttyS1
PROG=dapa -E noreset

# List of fuses: Mega8
# High: RSTDSBL, WDTON, SPIEN, CKOPT	 EESAVE, BOOTSZ1, BOOTSZ0, BOOTRST
# Low:  BODLEVEL, BODEN, SUT1, SUT0		 CKSEL3..0
# If spelled out below, it means we want it UNprogrammed and thus 1

# External serial programing, 16MHz external crystal, default bootsize
# FuseH: SPIEN, CKOPT, BOOTSZ1, BOOTSZ0
FUSEH=0xC9
# fast rising power
# Low: SUT0
FUSEL=0xEF

ARCH=avr
CC=$(ARCH)-gcc
OBJCOPY=$(ARCH)-objcopy
OBJDUMP=$(ARCH)-objdump
OBJSIZE=$(ARCH)-size

#-------------------
help: 
	@echo "Usage: make TYPE | TYPE_burn"
	@echo "Known Types: ds2408 ds2423"

#-------------------

# device codes
ds2408_CODE=29
ds2423_CODE=1D
ds2406_CODE=12

#CFLAGS+=-DUSE_WATCHDOG
#CFLAGS+=-DSKIP_WATCHDOG_INITIAL_RESET
#CFLAGS+=-DSKIP_MCUSR_READOUT

# for ds2423
#CFLAGS+=-DANALOG -DNCOUNTERS=6

# For ds2406
#CFLAGS+=-DWITH_PWM

all: $(DEVNAME).hex $(DEVNAME).lss $(DEVNAME).bin

ds2408 ds2423 ds2406:
	 @$(MAKE) $@_dev

%_burn: %_dev
	@$(MAKE) DEVNAME=$(subst _burn,,$@) DEVCODE=$($(subst _burn,,$@)_CODE) burn

%_dev:
	@$(MAKE) DEVNAME=$(subst _dev,,$@) all

# optimize for size!
ifeq ($(ARCH),avr)
  CFLAGS+=-g -mmcu=$(MCU) -Wall -Wstrict-prototypes -Os
  #-mcall-prologues
  UART=avr_uart.o
else
  CFLAGS+=-g -mcpu=cortex-m0 -mthumb -Wall -Wstrict-prototypes -Os
  UART=cortexm0_uart.o
endif
#  -I/usr/local/avr/include -B/usr/local/avr/lib
#-------------------

%.o : %.c Makefile $(wildcard *.h)
	$(CC) $(CFLAGS) -c $<
$(DEVNAME).out : onewire.o $(DEVNAME).o $(UART)
	$(CC) $(CFLAGS) -o $@ -Wl,-Map,$(DEVNAME).map,--cref $^
$(DEVNAME).hex : $(DEVNAME).out 
	$(OBJCOPY) -R .eeprom -O ihex $< $@
	@echo ""
	@echo -n "Flash size: "
	@$(OBJSIZE) $< | grep  -vE 'text.+data'|cut -f1,2|tr -d ' '|tr '\t' '+'|bc 
	@echo ""
$(DEVNAME).lss : $(DEVNAME).out 
	$(OBJDUMP) -h -S $< > $@
$(DEVNAME).bin : $(DEVNAME).out 
	$(OBJCOPY) -O binary $< $@

# DS2408
#DEVCODE=29
# DS2423
#DEVCODE=1D

$(DEVNAME).eeprom:
	python gen_eeprom.py $(DEVCODE) > $@
#------------------
burn: $(DEVNAME).hex $(DEVNAME).eeprom
	avrdude -c $(PROG) -p $(MCU_PROG) -U flash:w:$(DEVNAME).hex:i -U eeprom:w:$(DEVNAME).eeprom:i 
	#avrdude -V -c $(PROG) -p $(MCU_PROG) -U $(PRG).bin
#-------------------
clean:
	rm -f *.o *.map *.out *.hex* *.bin *.lss
#-------------------

