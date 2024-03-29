#*******************************************************************
#* Makefile for MSII
#*
#* COPYRIGHT: Philip L Johnson 2004
#* This header must appear on all derivatives of this file.
#*
#*******************************************************************
# Extensively reworked for MS2/Extra + MS3 by Ken Culver and James Murray  2006
# $Id: Makefile,v 1.77.2.2 2012/12/28 13:30:17 jsmcortina Exp $

# * Copyright 2007, 2008, 2009, 2010, 2011, 2012 James Murray and Kenneth Culver
# *
# * This file is a part of Megasquirt-3.
# *
# *
#    Origin: Philip L Johnson 2004
#    Major: James Murray / Kenneth Culver
#    Majority: Philip L Johnson / James Murray / Kenneth Culver
# *
# * You should have received a copy of the code LICENSE along with this source, please
# * ask on the www.msextra.com forum if you did not.
# *

ifdef SYSTEMDRIVE
BINDIR = c:/usr/bin
else
BINDIR:=$(shell which m68hc11-elf-gcc | sed -e's,m68hc11-elf-gcc,,g')
endif

PROGPREF=$(BINDIR)/m68hc11-elf-
#PROGPREF=/usr/src/jsm/build/usr/bin/m68hc11-elf-

CC      = $(PROGPREF)gcc
OBJCOPY = $(PROGPREF)objcopy
OBJDUMP = $(PROGPREF)objdump
NM      = $(PROGPREF)nm
GREP    = grep
SED     = sed
ECHO    = /bin/echo
CAT     = cat

CFLAGS  = -g -Wall -Werror -O -fomit-frame-pointer -m9s12x -mshort -msoft-reg-count=5 -mauto-incdec -fsigned-char -Wa,-mm9s12x
#XFLAGS  = -g -Wall -Werror -O -fomit-frame-pointer -m9s12xg -mshort -msoft-reg-count=5 -mauto-incdec -fsigned-char -Wa,-mxgate
XFLAGS  = -g -Wall -Werror -O -fomit-frame-pointer -m9s12xg -mshort -msoft-reg-count=5 -mauto-incdec -fsigned-char -Wa,-mxgate,--xgate-ramoffset

LDFLAGS = -Wl,-defsym,vectors_addr=0xff10,-m,m68hc12elfb,-T,m68hc12elfb.x -Wl,--no-warn-mismatch

CSRCS=ms3_can.c ms3_can_isr.c ms3_ego.c ms3_ign.c ms3_ign_wheel.c\
      ms3_main.c ms3_misc.c \
      premain.c ms3_init.c sigs.c ms3_ign_in.c ms3_inj.c ms3_idle.c\
      ms3_user.c sd_card.c compress.c ms3_pid.c serial.c

ASRCS=ms3_conferr.s \
      ms3_burnfactor.s isr_sci.s isr_rtc.s ms3_asm.s \
      isr_timerovf.s isr_ign.s xg_conf.s isr_spi.s isr_pit.s compress-asm.s \
      crc32.s interp.s

XSRCS=xgate.s

OBJS=$(CSRCS:.c=.o)
AOBJS= $(ASRCS:.s=.o)
XOBJS= $(XSRCS:.s=.o)

OBJCOPYARGS=--output-target=srec \
	--only-section=.rodata   --change-section-lma   .rodata=0x7fe800 \
	--only-section=.cnfdata  --change-section-lma  .cnfdata=0x100000 \
	--only-section=.cnfdata2 --change-section-lma .cnfdata2=0x103000 \
	--only-section=.lookup   --change-section-lma   .lookup=0x104000 \
	--only-section=.textf2   --change-section-lma   .textf2=0x7c8000 \
	--only-section=.textf3   --change-section-lma   .textf3=0x7cc000 \
	--only-section=.textf4   --change-section-lma   .textf4=0x7d0000 \
	--only-section=.textf5   --change-section-lma   .textf5=0x7d4000 \
	--only-section=.textf6   --change-section-lma   .textf6=0x7d8000 \
	--only-section=.textf7   --change-section-lma   .textf7=0x7dc000 \
	--only-section=.textf8   --change-section-lma   .textf8=0x7e0000 \
	--only-section=.textf9   --change-section-lma   .textf9=0x7e4000 \
	--only-section=.textfa   --change-section-lma   .textfa=0x7e8000 \
	--only-section=.textfb   --change-section-lma   .textfb=0x7ec000 \
	--only-section=.textfc   --change-section-lma   .textfc=0x7f0000 \
	--only-section=.text3    --change-section-lma    .text3=0x7f4000 \
	--only-section=.textfe   --change-section-lma   .textfe=0x7f8000 \
	--only-section=.text     --change-section-lma     .text=0x7fc000 \
	--only-section=.vectors  --change-section-lma  .vectors=0x7fef10

#default
all:	ms3

ms3:	setms3 ms3.s19
ms3pro:	setms3pro ms3pro.s19

dump: ms3.dmp

setms3:
	$(ECHO) "// setup for MS3" > opt.h
	grep -v "^#[12345]" core.ini | sed 's/^#0//' > ms3.ini

setms3pro:
	$(ECHO) "// setup for MS3PRO" > opt.h
	$(ECHO) "#define MS3PRO" >> opt.h
	grep -v "^#[02345]" core.ini | sed 's/^#1//' > ms3pro.ini

ms3_main_vars.h:
	$(ECHO) "/* DO NOT EDIT - this file is automatically generated during build */" > ms3_main_vars.h
	cat ms3_vars.h | sed 's/extern //' | grep -v "^#" >> ms3_main_vars.h

ms3h_defines.inc:
	$(ECHO) "; DO NOT EDIT - this file is automatically generated during build" > ms3h_defines.inc
	grep "^#define" ms3_vars.h | awk '{print ".equ",$$2,",",$$3};' >> ms3h_defines.inc

ms3_defines.h:
	$(ECHO) "/* DO NOT EDIT - this file is automatically generated during build */" > ms3_defines.h
	grep "^#define" ms3_vars.h >> ms3_defines.h
	
release: all
	rm -rf release
	mkdir release
	cp megasquirt-ii.ini.ms3 ms3.s19 copyini.bat ms2dl.exe utils/CHOICE.COM utils/download-MS2-firmware.bat README.txt release
	# Make sure you updated sigs.c

$(AOBJS): %.o: %.s ms3h_defines.inc
	$(CC) $(CFLAGS) -c $<

$(XOBJS): %.o: %.s ms3h_defines.inc
	$(CC) $(XFLAGS) -c $<

$(OBJS): %.o: %.c cltfactor.inc egofactor.inc matfactor.inc hcs12xedef.h ms3.h hcs12xedef.inc maffactor.inc ms3_structs.inc ms3h.inc ms3_main_vars.h ms3_defines.h
	$(CC) $(CFLAGS) -c $<

ms3.elf: $(XOBJS) $(OBJS) $(AOBJS)
	$(CC) $(CFLAGS) $(LDFLAGS) -o ms3.elf $(XOBJS) $(OBJS) $(AOBJS)

#note vectors at 0x7f_ef10 when using monitor
ms3.s19: ms3.elf
	$(OBJCOPY) $(OBJCOPYARGS) ms3.elf ms3.s19
	$(GREP) Format sigs.c | $(SED) -e 's/.*MS3/QMS3/' | $(SED) -e 's/\".*//' >> ms3.s19
	$(CAT) s19tail.inc >> ms3.s19

#note vectors at 0x7f_ef10 when using monitor
ms3pro.s19: ms3.elf
	$(OBJCOPY) $(OBJCOPYARGS) ms3.elf ms3pro.s19
	$(GREP) Format sigs.c | $(SED) -e 's/.*MS3/QMS3/' | $(SED) -e 's/\".*//' >> ms3pro.s19
	$(CAT) s19tail.inc >> ms3pro.s19

# was -Ssdh the 's' dumps the full contents of all sections
ms3.dmp: ms3.elf
	$(NM) ms3.elf | sort | uniq > ms3.map
	$(OBJDUMP) -Sdh -mm9s12x ms3.elf > ms3.dmp
	$(OBJDUMP) -d -mm9s12xg --section=.textfa --start-address=0x`grep xgate_isr_start ms3.map | awk '{print $$1}'` --stop-address=0x`grep xgate_isr_end ms3.map | awk '{print $$1}'`  ms3.elf >> ms3.dmp &

.PHONY: clean
clean:
	$(RM)  ms3.elf ms3*.s19 ms3.dmp ms3.map *~ ms3_main_vars.h ms3_defines.h ms3h_defines.inc ms3*.ini
	$(RM) $(OBJS) $(AOBJS) $(XOBJS)
	$(RM) -r release
	$(RM) opt.h
