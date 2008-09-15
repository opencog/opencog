%{

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "type.yacc.h"

%}

%%

[A-Z_]+ { 
  yylval.str=(char *) malloc (strlen (yytext)+1);
  strcpy (yylval.str,yytext);
  return TOK_NAME;  
} 

[A-Za-z]+ {
  yylval.str=(char *) malloc (strlen (yytext)+1);
  strcpy (yylval.str,yytext);
  return TOK_STRING;  
}

"//"[^\n]* /* comments are ignored */

[ \t]+ /* spaces are ignored */

"\"" return TOK_QUOTE;
"," return TOK_COMMA; 
"<-" return TOK_SUPER;
"\n" return TOK_ENTER;

. {
  char str[200];
  sprintf (str,"Syntax: <%s>\n",yytext);
  yyerror (str); 
}

%%

