%{

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define MAXSIZE 200
#define MAXNAMES 10

#define MAXTYPES 600

char namepool[MAXNAMES][MAXSIZE];
char argspool[MAXNAMES][MAXSIZE];
int total_names;
int total_args;

typedef struct {
  char name[MAXSIZE],alias[MAXNAMES][MAXSIZE];
  char super[MAXNAMES][MAXSIZE];
  char id[MAXSIZE];
  int aliassize, supersize, level;
  int parent;
} nmtype;

static nmtype types[MAXTYPES];
static int maxtypes=0;

static void check_alias (void);
char *reparse (char *);

%}

%union {
  char *str;
}

%token <str> TOK_NAME
%token <str> TOK_STRING
%token TOK_ENTER
%token TOK_COMMA
%token TOK_SUPER
%token TOK_QUOTE
%type <str> opstring

%%

source: item | source item;

item: 
  TOK_ENTER 
  | declaration TOK_ENTER;

declaration:
  name opstring {
    int i;

    strcpy (types[maxtypes].name,namepool[0]);
    check_alias();
    if ($2==NULL)
       strcpy(types[maxtypes].id,reparse(namepool[0]));
    else
       strcpy(types[maxtypes].id,$2);
    types[maxtypes].supersize=0;
    maxtypes++;
  }
  | name TOK_SUPER args opstring {
    int i;
    strcpy (types[maxtypes].name,namepool[0]);
    check_alias();
    if ($4==NULL)
       strcpy(types[maxtypes].id,reparse(namepool[0]));
    else
       strcpy(types[maxtypes].id,$4);
    types[maxtypes].supersize=total_args;
    for (i=0; i<total_args; i++)
      strcpy (types[maxtypes].super[i],argspool[i]);
    maxtypes++;
  }
  ;

opstring: 
  /* nothing */ {
    $$=NULL;
  }
  | TOK_QUOTE TOK_STRING TOK_QUOTE {
    $$=(char *) malloc (200);
    strcpy ($$,$2);
  }
  ;

name:
  TOK_NAME {
    total_names=1;
    strcpy (namepool[0],yylval.str);
  }
  | name TOK_COMMA TOK_NAME {
    strcpy (namepool[total_names++],yylval.str);
  }
  ;

args:
  TOK_NAME {
    total_args=1;
    strcpy (argspool[0],yylval.str);
  }
  | args TOK_COMMA TOK_NAME {
    strcpy (argspool[total_args++],yylval.str);
  }
  ;

%%

int yyerror (char *error) {
  printf ("Error: %s\n",error);
  exit (1);
}

static void check_alias(void) {
  int i;
    if (total_names>1) {
      for (i=1; i<total_names; i++)
        strcpy (types[maxtypes].alias[i-1],namepool[i]);
      types[maxtypes].aliassize=total_names-1;
    }
    else types[maxtypes].aliassize=0; 
}

static void gen_defines(void) {
  FILE *f;
  int i,j;

  f=fopen ("type_codes.h","w");
  fprintf (f,"/* Computer-generated type_codes.h */\n\n");
  for (i=0; i<maxtypes; i++) {
   fprintf (f,"#define %s %d\n",types[i].name,i);
   for (j=0; j<types[i].aliassize; j++)
     fprintf (f,"#define %s %d\n",types[i].alias[j],i);
  }
  fprintf (f,"#define NUMBER_OF_CLASSES %d\n",i);
  fprintf (f,"#define NOTYPE %d\n",i+1);
  fclose (f);
}

char *reparse (char *str) {
  char *name;
  int upper=1;
  int i,j;

  name=(char *) malloc (200);
  for (i=0,j=0; i<strlen(str); i++) {
    if (str[i]=='_') {
      upper=1;
      continue;
    }
    name[j++]=upper?str[i]:str[i]-'A'+'a';
    upper=0;
  }  
  name[j]=0;
  return name;
}

static void gen_names(void) {
  FILE *f;
  int i;

  f=fopen ("type_names.h","w");
  fprintf (f,"/* Computer-generated type_names.h */\n\n");

  for (i=0; i<maxtypes; i++) 
   fprintf (f,"#define %s_CLASS \"%s\"\n",types[i].name,types[i].id);
  fclose (f);
}

static void gen_classint(void) {
  FILE *f;
  int i;

  f=fopen ("type_classint.h","w");
  fprintf (f,"/* Computer-generated type_classint.h */\n\n");

  for (i=0; i<maxtypes; i++) 
   fprintf (f,"class_to_int[(*getClassName()) [%s]] = %s;\n",types[i].name,types[i].name);
  fclose (f);
}

static void gen_intclass(void) {
  FILE *f;
  int i;

  f=fopen ("type_intclass.h","w");
  fprintf (f,"/* Computer-generated type_intclass.h */\n\n");

  for (i=0; i<maxtypes; i++) 
   fprintf (f,"int_to_class[%s] = %s_CLASS;\n",types[i].name,types[i].name);
  fclose (f);
}

int reverse (char *s) {
  int i;
  char str[200];

  for (i=0; i<maxtypes; i++)
    if (!strcmp (s,types[i].name))
      return i;
  
  sprintf (str,"Type not found: %s",s);
  yyerror (str);
  return 0;
}

void fill_level (int i) {
  int j,maxlevel=0;
  if (types[i].level>=0)
    return;

  if (types[i].supersize==0) {
    types[i].level=0;
    return;
  }

  for (j=0; j<types[i].supersize; j++) {
    fill_level (reverse(types[i].super[j]));
    if (types[reverse(types[i].super[j])].level>maxlevel)
      maxlevel=types[reverse(types[i].super[j])].level;
  }
  types[i].level=1+maxlevel;
  types[i].parent = reverse(types[i].super[0]);
}

static void gen_inheritance(void) {
  FILE *f;
  int i,maxlevel=0,j,k;

  f=fopen ("type_inheritance.h","w");
  fprintf (f,"/* Computer-generated type_inheritance.h */\n\n");

  for (i=0; i<maxtypes; i++)
    types[i].level=-1;

  for (i=0; i<maxtypes; i++) {
    fill_level (i);
    if (types[i].level>maxlevel)
      maxlevel=types[i].level;
  }

  for (j=0; j<=maxlevel; j++)
    for (i=0; i<maxtypes; i++)
      if (types[i].level==j && types[i].supersize>0)
        for (k=0; k<types[i].supersize; k++)
          fprintf (f,"init_inheritance(map, %s, %s);\n",types[i].super[k],types[i].name);

  fclose (f);
}

/**
 * Generate scheme syntactic sugar for atom creation
 */
static void gen_scm(void)
{
  FILE *f;
  int i,j;

  f=fopen ("type_constructors.scm","w");
  fprintf (f,"scm\n\n");
  fprintf (f,"; DO NOT EDIT THIS FILE!  This file was automatically\n");
  fprintf (f,"; generated from atom definitions in types.script\n;\n");
  fprintf (f,"; This file contains basic scheme wrappers for atom creation.\n;\n");
  fprintf (f,"; Issue the following at terminal to load:\n");
  fprintf (f,"; cat type_constructors.scm |telnet localhost 17001\n;\n");
  for (i=1; i<maxtypes; i++) 
  {
    int parent = types[i].parent;
    if (0 == parent) parent = i;
    while (parent !=1 && parent != 2)
    {
      parent = types[parent].parent;
    }
    fprintf (f, "(define (%s . x)\n",types[i].id);
    if (parent == 1)
      fprintf (f, "\t(apply cog-new-node (append (list '%s) x)))\n",types[i].id);
    else 
      fprintf (f, "\t(apply cog-new-link (append (list '%s) x)))\n",types[i].id);
  }
  fprintf (f,".\n");
  fprintf (f,"exit\n");
  fclose (f);
}

int main (int argc, char **argv) {
  yyparse();

  gen_defines();
  gen_names();
  gen_classint();
  gen_intclass();
  gen_inheritance();
  gen_scm();
  return 0;
}
