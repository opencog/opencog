INCLUDE = include

CFLAGS = -g -O0 -lm -lcxcore -lcv -lhighgui -lcvaux

all: destin

destin:
	gcc -I$(INCLUDE) $(CFLAGS) -o bin/destin src/node.c src/destin.c src/destin_mnist.c src/util.c

clean:
	rm bin/* 
