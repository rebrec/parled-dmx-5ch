# Makefile for PIC microcontrollers using sdcc and pk2cmd

# Specific microcontroller type:
DEVICE=16f737
FAMILY=pic14
PRJ=uart
# C compiler, assembler, device programmer:
CC=sdcc --use-non-free -m$(FAMILY)
PK2=pk2cmd -Ppic$(DEVICE)

All: $(PRJ).hex

write:
	$(PK2) -M -F$(PRJ).hex -R 

on:
	$(PK2) -T

off:
	$(PK2) -W

erase:
	$(PK2) -E

clean:
	rm -f *.o *.cod *.hex *.lst *.err

$(PRJ).hex: $(PRJ).c
	$(CC) -p$(DEVICE) $(PRJ).c

