include ../Makefile.inc
CPPFLAGS = -I../utilities -I../multivector

all: lobpcg.o

lobpcg.o: lobpcg.c lobpcg.h ../utilities/fortran_matrix.h

clean: FORCE
	rm -f lobpcg.o

FORCE:
