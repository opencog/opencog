#
# Makefile for the making the program to make the types.
# (CMake currently can't do this) 

all: type-definer
	cat type.script |./type-definer
	rm -f type.yacc.h

type-definer: type.yacc.o type.lex.o
	cc -o $@ $^ -lfl

.l.c:
	flex -o $@ $<

.y.c:
	yacc --debug -d -o $@ $<

.c.o:
	cc -O2 -c $< 

clean:
	rm -f *.o

distclean: clean
	rm -f lex.yy.c y.tab.c y.tab.h
	rm -f type.lex.c type.yacc.c type.yacc.h
	rm -f type-definer

