EXEC = main
CC = gcc
COMMON = -I.
CFLAGS = -Wall -Wno-unused-result -Wno-unknown-pragmas -Wfatal-errors -fPIC -O3
EXECOBJ = main.o
FILE = main.c

$(EXEC):
	$(CC) $(COMMON) $(CFLAGS) -c $(FILE)
	$(CC) $(EXECOBJ) -lm -o $@

clean:
	rm -rf $(EXEC) $(EXECOBJ)
	rm -rf images/*.bin
