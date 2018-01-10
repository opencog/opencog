%{
/**************************************************************************
   Lex specification for post-process knowledge file

   6/96 ALB
   Possible areas of improvement:
    . allow sets to be referred to anywhere in file, not just
      after they are defined. (but then you need to be careful
      about infinite loops involving recursively defined sets)

**************************************************************************/

#include "pp_lexer.h"
#include "pp-structures.h"
#include "string-set.h"
#include "utilities.h"

#define YY_NO_INPUT
#define YY_NO_UNPUT

/* forward references for non-exported functions */
static void setup(PPLexTable *lt);
static bool set_label(PPLexTable *lt, const char *label);
static bool add_string_to_label(PPLexTable *lt, const char *str);
static bool add_set_of_strings_to_label(PPLexTable *lt,const char *label_of_set);
static int  get_index_of_label(PPLexTable *lt, const char *label);
static PPLexTable *clt=NULL; /* ptr to lex table we're currently filling in */
%}

%s INCLUDE

LABEL          [^ \t\"\n:]+:
NONLABELSET    @[^ \t\n\":]+
NONLABEL       [^ \t\"\n:]+
NONLABELQUOTED \"[^\t\"\n:]+\"
WHITE          [ \t\n]*
COMMENT        ^[\ \t]*;.*$

%%

{COMMENT}                 ;
{WHITE}                   ;

<INITIAL>{LABEL}          { set_label(clt, yytext); }
<INITIAL>{NONLABELSET}    { add_set_of_strings_to_label(clt, &(yytext[1])); }
<INITIAL>{NONLABEL}       { add_string_to_label(clt, yytext); }
<INITIAL>{NONLABELQUOTED} { add_string_to_label(clt, yytext); }

.  { prt_error("pp_lexer: unable to parse knowledge file (line %i).\n", yylineno); }


%%

/************************ exported functions ******************************/

void pp_lexer_close(PPLexTable *lt)
{
  int i;
  pp_label_node *node,*next;
  for (i=0; i<PP_LEXER_MAX_LABELS; i++)
    {
      /* free the linked list */
      node = lt->nodes_of_label[i];
      while (node)
        {
          next = node->next;
          xfree(node, sizeof(pp_label_node));
          node=next;
        }
    }
  string_set_delete(lt->string_set);
  if (lt->tokens) free(lt->tokens);
  xfree(lt, sizeof(PPLexTable));
}

PPLexTable *pp_lexer_open(FILE *f)
{
  PPLexTable *lt;
  bool yylex_ok = false;

  assert(f, "pp_lexer_open: passed a NULL file pointer");

  yyin = f;            /* redirect lex to look at the specified file */
  lt = (PPLexTable*) xalloc (sizeof(PPLexTable));
  setup(lt);
  clt = lt;    /* set lt to be the current table, which yylex will fill in */
  if (YY_NULL == yylex()) yylex_ok = true;
  clt = NULL;
  lt->idx_of_active_label=-1;

  if (!yylex_ok)
  {
    pp_lexer_close(lt);
    return NULL;
  }
  return lt;
}

int pp_lexer_set_label(PPLexTable *lt, const char *label)
{
  /* set lexer state to first node of this label */
  lt->idx_of_active_label = get_index_of_label(lt, label);
  if (lt->idx_of_active_label==-1) return 0;    /* label not found */
  lt->current_node_of_active_label=lt->nodes_of_label[lt->idx_of_active_label];
  return 1;
}

int pp_lexer_count_tokens_of_label(PPLexTable *lt)
{
  /* counts all tokens, even the commas */
  int n;
  pp_label_node *p;
  if (lt->idx_of_active_label==-1)
  {
    prt_error("Error: pp_lexer: current label is invalid\n");
    return -1;
  }
  for (n=0, p=lt->nodes_of_label[lt->idx_of_active_label]; p;p=p->next, n++){}
  return n;
}


const char *pp_lexer_get_next_token_of_label(PPLexTable *lt)
{
  /* retrieves next token of set label, or NULL if list exhausted */
  static const char *p;
  if (lt->current_node_of_active_label==NULL) return NULL;
  p = lt->current_node_of_active_label->str;
  lt->current_node_of_active_label=lt->current_node_of_active_label->next;
  return p;
}

int pp_lexer_count_commas_of_label(PPLexTable *lt)
{
  int n;
  pp_label_node *p;
  if (lt->idx_of_active_label==-1)
  {
    prt_error("Error: pp_lexer: current label is invalid\n");
    return -1;
  }
  for (n=0,p=lt->nodes_of_label[lt->idx_of_active_label];p!=NULL;p=p->next)
    if (!strcmp(p->str, ",")) n++;
  return n;
}

const char **pp_lexer_get_next_group_of_tokens_of_label(PPLexTable *lt, size_t *n_tokens)
{
  /* all tokens until next comma, null-terminated */
  int n;
  pp_label_node *p;

  p = lt->current_node_of_active_label;
  for (n=0; p!=NULL && strcmp(p->str,","); n++, p=p->next) {}
  if (n > lt->extents) {
     lt->extents = n;
     lt->tokens = (const char **) realloc (lt->tokens, n * sizeof(const char*));
  }

  p = lt->current_node_of_active_label;
  for (n=0; p!=NULL && strcmp(p->str,","); n++, p=p->next)
       lt->tokens[n] = string_set_add(p->str, lt->string_set);

  /* advance "current node of label" state */
  lt->current_node_of_active_label = p;
  if (p!=NULL) lt->current_node_of_active_label = p->next;

  *n_tokens = n;
  return lt->tokens;
}

int yywrap(void)
{
  /* must return 1 for end of input, 0 otherwise */
  return 1;
}

/********************** non-exported functions ************************/

static void setup(PPLexTable *lt)
{
  int i;
  for (i=0; i<PP_LEXER_MAX_LABELS; i++)
    {
      lt->nodes_of_label[i]     = NULL;
      lt->last_node_of_label[i] = NULL;
      lt->labels[i] = NULL;
    }
  lt->string_set = string_set_create();
  lt->tokens = NULL;
  lt->extents = 0;
}

static bool set_label(PPLexTable *lt, const char *label)
{
  int i;
  char *c;
  char *label_sans_colon;

  /* check for and then slice off the trailing colon */
  label_sans_colon = strdup(label);
  c=&(label_sans_colon[strlen(label_sans_colon)-1]);
  if (*c != ':')
  {
    prt_error("Error: Label %s must end with :\n", label);
    return false;
  }
  *c = 0;

  /* have we seen this label already? If so, abort */
  for (i=0;lt->labels[i]!=NULL && strcmp(lt->labels[i],label_sans_colon);i++) {}
  if (lt->labels[i]!=NULL)
  {
    prt_error("Error: pp_lexer: label %s multiply defined!\n", label_sans_colon);
    return false;
  }

  /* new label. Store it */
  if (i == PP_LEXER_MAX_LABELS-1)
  {
    prt_error("Error: pp_lexer: too many labels. Raise PP_LEXER_MAX_LABELS\n");
    return false;
  }
  lt->labels[i] = string_set_add(label_sans_colon, lt->string_set);
  lt->idx_of_active_label = i;

  free(label_sans_colon);

  return true;
}

static bool check_string(const char *str)
{
  if (strlen(str)>1 && strchr(str, ',')!=NULL)
  {
    prt_error("Error: pp_lexer: string %s contains a comma, which is a no-no.\n", str);
    return false;
  }
  return true;
}


static bool add_string_to_label(PPLexTable *lt, const char *str)
{
  /* add the single string str to the set of strings associated with label */
  pp_label_node *new_node;

  if (lt->idx_of_active_label == -1)
  {
    prt_error("Error: pp_lexer: invalid syntax (line %i)\n", yylineno);
    return false;
  }

  /* make sure string is legal */
  if (!check_string(str)) return false;

  /* create a new node in (as yet to be determined) linked list of strings */
  new_node = (pp_label_node *) xalloc (sizeof(pp_label_node));
  new_node->str = string_set_add(str, lt->string_set);
  new_node->next = NULL;

  /* stick newly-created node at the *end* of the appropriate linked list */
  if (lt->last_node_of_label[lt->idx_of_active_label]==NULL)
    {
      /* first entry on linked list */
      lt->nodes_of_label[lt->idx_of_active_label]     = new_node;
      lt->last_node_of_label[lt->idx_of_active_label] = new_node;
    }
  else
    {
      /* non-first entry on linked list */
      lt->last_node_of_label[lt->idx_of_active_label]->next = new_node;
      lt->last_node_of_label[lt->idx_of_active_label]       = new_node;
    }

  return true;
}

static bool add_set_of_strings_to_label(PPLexTable *lt,const char *label_of_set)
{
  /* add the set of strings, defined earlier by label_of_set, to the set of
     strings associated with the current label */
  pp_label_node *p;
  int idx_of_label_of_set;
  if (lt->idx_of_active_label==-1)
  {
    prt_error("Error: pp_lexer: invalid syntax (line %i)\n", yylineno);
    return false;
  }
  if ((idx_of_label_of_set = get_index_of_label(lt, label_of_set))==-1)
  {
    prt_error("Error: pp_lexer: label %s must be defined before it's referred to (line %i)\n",
              label_of_set, yylineno);
    return false;
  }
  for (p=lt->nodes_of_label[idx_of_label_of_set]; p!=NULL; p=p->next)
    add_string_to_label(lt, p->str);

  return true;
}

static int get_index_of_label(PPLexTable *lt, const char *label)
{
  int i;
  for (i=0; lt->labels[i]!=NULL; i++)
    if (!strcmp(lt->labels[i], label)) return i;
  return -1;
}
