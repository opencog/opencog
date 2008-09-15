#!/bin/sh

bison -d -v type.yacc.y
mv type.yacc.tab.c type.yacc.c
mv type.yacc.tab.h type.yacc.h
flex -otype.lex.c type.lex.l
gcc type.lex.c type.yacc.c -o typeparser -lfl
./typeparser < type.script
#mv type_*.h core
rm type.yacc.h type.yacc.c type.lex.c type.yacc.output typeparser
