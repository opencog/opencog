

all: wni


wni: wni.o
	$(CC) -o $@ $^ -lwordnet


.c.o:
	$(CC) -g -O2 -c $^
