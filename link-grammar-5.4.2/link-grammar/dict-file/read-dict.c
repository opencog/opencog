/*************************************************************************/
/* Copyright (c) 2004                                                    */
/* Daniel Sleator, David Temperley, and John Lafferty                    */
/* Copyright 2013, 2014 Linas Vepstas                                    */
/* All rights reserved                                                   */
/*                                                                       */
/* Use of the link grammar parsing system is subject to the terms of the */
/* license set forth in the LICENSE file included with this software.    */
/* This license allows free redistribution and use in source and binary  */
/* forms, with or without modification, subject to certain conditions.   */
/*                                                                       */
/*************************************************************************/

#include <string.h>

#include "dict-common/dict-affix.h"   // For is_stem()
#include "dict-common/dict-common.h"
#include "dict-common/dict-defines.h" // For SUBSCRIPT_MARK
#include "dict-common/file-utils.h"
#include "dict-common/idiom.h"
#include "dict-common/regex-morph.h"
#include "error.h"
#include "print/print.h"
#include "externs.h"
#include "read-dict.h"
#include "string-set.h"
#include "tokenize/tok-structures.h" // needed for MT_WALL
#include "utilities.h"
#include "word-file.h"

/*
  The dictionary format:

  In what follows:
    Every "%" symbol and everything after it is ignored on every line.
    Every newline or tab is replaced by a space.

  The dictionary file is a sequence of ENTRIES.  Each ENTRY is one or
  more WORDS (a sequence of upper or lower case letters) separated by
  spaces, followed by a ":", followed by an EXPRESSION followed by a
  ";".  An EXPRESSION is an expression where the operators are "&"
  or "and" or "|" or "or", and there are three types of parentheses:
  "()", "{}", and "[]".  The terminal symbols of this grammar are the
  connectors, which are strings of letters or numbers or *s.
  Expressions may be written in prefix or infix form. In prefix-form,
  the expressions are lisp-like, with the operators &, | preceding
  the operands. In infix-form, the operators are in the middle. The
  current dictionaries are in infix form.  If the C preprocessor
  constant INFIX_NOTATION is defined, then the dictionary is assumed
  to be in infix form.

  The connector begins with an optional @, which is followed by an upper
  case sequence of letters. Each subsequent *, lower case letter or
  number is a subscript. At the end is a + or - sign.  The "@" allows
  this connector to attach to one or more other connectors.

  Here is a sample dictionary entry (in infix form):

      gone:         T- & {@EV+};

  (See our paper for more about how to interpret the meaning of the
  dictionary expressions.)

  A previously defined word (such as "gone" above) may be used instead
  of a connector to specify the expression it was defined to be.  Of
  course, in this case, it must uniquely specify a word in the
  dictionary, and have been previously defined.

  If a word is of the form "/foo", then the file current-dir/foo
  is a so-called word file, and is read in as a list of words.
  A word file is just a list of words separated by blanks or newlines.

  A word that contains the character "_" defines an idiomatic use of
  the words separated by the "_".  For example "kind of" is an idiomatic
  expression, so a word "kind_of" is defined in the dictionary.
  Idiomatic expressions of any number of words can be defined in this way.
  When the word "kind" is encountered, all the idiomatic uses of the word
  are considered.

  An expression enclosed in "[..]" is give a cost of 1.  This means
  that if any of the connectors inside the square braces are used,
  a cost of 1 is incurred.  (This cost is the first element of the cost
  vector printed when a sentence is parsed.)  Of course if something is
  inside of 10 levels of "[..]" then using it incurs a cost of 10.
  These costs are called "disjunct costs".  The linkages are printed out
  in order of non-increasing disjunct cost.

  A number following a square bracket over-rides the cost of that bracket.
  Thus, [...].5 has a cost of 0.5 while [...]2.0 has a cost of 2; that
  is it is the same as [[...]].  Any floating point number (including
  exponents!) is allowed.

  The expression "(A+ or ())" means that you can choose either "A+" or
  the empty expression "()", that is, that the connector "A+" is
  optional.  This is more compactly expressed as "{A+}".  In other words,
  curly braces indicate an optional expression.

  The expression "(A+ or [])" is the same as that above, but there is a
  cost of 1 incurred for choosing not to use "A+".  The expression
  "(EXP1 & [EXP2])" is exactly the same as "[EXP1 & EXP2]".  The difference
  between "({[A+]} & B+)" and "([{A+}] & B+)" is that the latter always
  incurs a cost of 1, while the former only gets a cost of 1 if "A+" is
  used.

  The dictionary writer is not allowed to use connectors that begin in
  "ID".  This is reserved for the connectors automatically
  generated for idioms.

  Dictionary words may be followed by a dot (period, "."), and a "subscript"
  identifying the word type. The subscript may be one or more letters or
  numbers, but must begin with a letter. Currently, the dictionary contains
  (mostly?) subscripts consisting of a single letter, and these serve mostly
  to identify the part-of-speech. In general, subscripts can also be used
  to distinguish different word senses.
*/

static bool link_advance(Dictionary dict);

static void dict_error2(Dictionary dict, const char * s, const char *s2)
{
#define ERRBUFLEN 1024
	char tokens[ERRBUFLEN], t[ERRBUFLEN];
	int pos = 1;
	int i;

	/* The link_advance used to print the error message can
	 * throw more errors while printing... */
	if (dict->recursive_error) return;
	dict->recursive_error = true;

	tokens[0] = '\0';
	for (i=0; i<5 && dict->token[0] != '\0' ; i++)
	{
		pos += snprintf(t, ERRBUFLEN, "\"%s\" ", dict->token);
		strncat(tokens, t, ERRBUFLEN-1-pos);
		link_advance(dict);
	}
	tokens[pos] = '\0';

	if (s2)
	{
		prt_error("Error parsing dictionary %s.\n"
		          "%s %s\n\t line %d, tokens = %s",
		          dict->name, s, s2, dict->line_number, tokens);
	}
	else
	{
		prt_error("Error parsing dictionary %s.\n"
		          "%s\n\t line %d, tokens = %s",
		          dict->name, s, dict->line_number, tokens);
	}
	dict->recursive_error = false;
}

static void dict_error(Dictionary dict, const char * s)
{
	dict_error2(dict, s, NULL);
}

static void warning(Dictionary dict, const char * s)
{
	prt_error("Warning: %s\n"
	        "\tline %d, current token = \"%s\"",
	        s, dict->line_number, dict->token);
}

/**
 * This gets the next UTF8 character from the input, eliminating comments.
 * If we're in quote mode, it does not consider the % character for
 * comments.   Note that the returned character is a wide character!
 */
typedef char* utf8char;
static utf8char get_character(Dictionary dict, int quote_mode)
{
	static char uc[7];
	int i = 0;

	while (1)
	{
		char c = *(dict->pin++);

		/* Skip over all comments */
		if ((c == '%') && (!quote_mode))
		{
			while ((c != 0x0) && (c != '\n')) c = *(dict->pin++);
			dict->line_number++;
			continue;
		}

		/* Newline counts as whitespace */
		if (c == '\n')
			dict->line_number++;

		/* If it's a 7-bit ascii, we are done */
		if ((0 == i) && ((c & 0x80) == 0x0))
		{
			uc[0] = c;
			uc[1] = 0x0;
			return uc;
		}

		uc[0] = c;
		i = 1;
		while (i < 6)
		{
			c = *(dict->pin++);
			/* If we're onto the next char, we're done. */
			if (((c & 0x80) == 0x0) || ((c & 0xc0) == 0xc0))
			{
				dict->pin--;
				uc[i] = 0x0;
				return uc;
			}
			uc[i] = c;
			i++;
		}
		dict_error(dict, "UTF8 char is too long");
		return NULL;
	}
	uc[0] = 0x0;
	return uc;
}


/*
 * This set of 10 characters are the ones defining the syntax of the
 * dictionary.
 */
#define SPECIAL "(){};[]&^|:"

/* Commutative (symmetric) AND */
#define SYM_AND '^'

/* Bi-directional connector: + or - */
#define ANY_DIR '$'

/* Wild-card link type */
#define WILD_TYPE '*'

/**
 * Return true if the input character is one of the special
 * characters used to define the syntax of the dictionary.
 */
static bool char_is_special(char c)
{
	return (NULL != strchr(SPECIAL, c));
}

/**
 * This reads the next token from the input into 'token'.
 * Return 1 if a character was read, else return 0 (and print a warning).
 */
static bool link_advance(Dictionary dict)
{
	utf8char c;
	int nr, i;
	int quote_mode;

	dict->is_special = false;

	if (dict->already_got_it != '\0')
	{
		dict->is_special = char_is_special(dict->already_got_it);
		if (dict->already_got_it == EOF) {
			dict->token[0] = '\0';
		} else {
			dict->token[0] = dict->already_got_it; /* specials are one byte */
			dict->token[1] = '\0';
		}
		dict->already_got_it = '\0';
		return true;
	}

	do
	{
		c = get_character(dict, false);
		if (NULL == c) return false;
	}
	while (lg_isspace(c[0]));

	quote_mode = false;

	i = 0;
	for (;;)
	{
		if (i > MAX_TOKEN_LENGTH-3) {
			dict_error(dict, "Token too long");
			return false;
		}
		if (quote_mode) {
			if (c[0] == '"' &&
			    /* Check the next character too, to allow " in words */
			    (*dict->pin == ':' || *dict->pin == ';' ||
			    lg_isspace(*dict->pin))) {

				quote_mode = false;
				dict->token[i] = '\0';
				return true;
			}
			if (lg_isspace(c[0])) {
				dict_error(dict, "White space inside of token");
				return false;
			}

			nr = 0;
			while (c[nr]) {dict->token[i] = c[nr]; i++; nr++; }
		} else {
			if ('\0' == c[1] && char_is_special(c[0]))
			{
				if (i == 0)
				{
					dict->token[0] = c[0];  /* special toks are one char always */
					dict->token[1] = '\0';
					dict->is_special = true;
					return true;
				}
				dict->token[i] = '\0';
				dict->already_got_it = c[0];
				return true;
			}
			if (c[0] == 0x0) {
				if (i == 0) {
					dict->token[0] = '\0';
					return true;
				}
				dict->token[i] = '\0';
				dict->already_got_it = '\0';
				return true;
			}
			if (lg_isspace(c[0])) {
				dict->token[i] = '\0';
				return true;
			}
			if (c[0] == '\"') {
				quote_mode = true;
			} else {
				nr = 0;
				while (c[nr]) {dict->token[i] = c[nr]; i++; nr++; }
			}
		}
		c = get_character(dict, quote_mode);
		if (NULL == c) return false;
	}
	return true;
}

/**
 * Returns true if this token is a special token and it is equal to c
 */
static int is_equal(Dictionary dict, char c)
{
	return (dict->is_special &&
	        c == dict->token[0] &&
	        dict->token[1] == '\0');
}

/**
 * Make sure the string s is a valid connector.
 * Return true if the connector is valid, else return false,
 * and print an appropriate warning message.
 */
static bool check_connector(Dictionary dict, const char * s)
{
	int i;
	i = strlen(s);
	if (i < 1) {
		dict_error(dict, "Expecting a connector.");
		return false;
	}
	i = s[i-1];  /* the last character of the token */
	if ((i != '+') && (i != '-') && (i != ANY_DIR)) {
		dict_error(dict, "A connector must end in a \"+\", \"-\" or \"$\".");
		return false;
	}
	if (*s == '@') s++;
	if (!isupper((int)*s) && ('h' != *s) && ('d' != *s)) {
		dict_error(dict, "The first letter of a connector must be h,d or uppercase.");
		return false;
	}
	if ((*s == 'I') && (*(s+1) == 'D')) {
		dict_error(dict, "Connectors beginning with \"ID\" are forbidden");
		return false;
	}
	while (*(s+1)) {
		if ((!isalnum((int)*s)) && (*s != WILD_TYPE)) {
			dict_error(dict, "All letters of a connector must be ASCII alpha-numeric.");
			return false;
		}
		s++;
	}
	return true;
}

/* ======================================================================== */
/**
 * Dictionary entry comparison and ordering functions.
 *
 * The data structure storing the dictionary is simply a binary tree.
 * The entries in the binary tree are sorted by alphabetical order.
 * There is one catch, however: words may have suffixes (a dot, followed
 * by the suffix), and these suffixes are to be handled appropriately
 * during sorting and comparison.
 *
 * The use of suffixes means that the ordering of the words is not
 * exactly the order given by strcmp.  The order must be such that, for
 * example, "make" < "make.n" < "make-up" -- suffixed words come after
 * the bare words, but before any other other words with non-alphabetic
 * characters (such as the hyphen in "make-up", or possibly UTF8
 * characters). Thus, plain "strcmp" can't be used to determine
 * dictionary order.
 *
 * Thus, a set of specialized string comparison and ordering functions
 * are provided. These "do the right thing" when matching string with
 * and without suffixes.
 */
/**
 * dict_order_strict - order two dictionary words in proper sort order.
 * Return zero if the strings match, else return in a unique order.
 * The order is NOT (locale-dependent) UTF8 sort order; its ordered
 * based on numeric values of single bytes.  This will uniquely order
 * UTF8 strings, just not in a LANG-dependent (locale-dependent) order.
 * But we don't need/want locale-dependent ordering!
 */
/* verbose version, for demonstration only */
/*
int dict_order_strict(char *s, char *t)
{
	int ss, tt;
	while (*s != '\0' && *s == *t) {
		s++;
		t++;
	}
	if (*s == SUBSCRIPT_MARK) {
		ss = 1;
	} else {
		ss = (*s)<<1;
	}
	if (*t == SUBSCRIPT_MARK) {
		tt = 1;
	} else {
		tt = (*t)<<1;
	}
	return (ss - tt);
}
*/

/* terse version */
/* If one word contains a dot, the other one must also! */
static inline int dict_order_strict(const char *s, Dict_node * dn)
{
	const char * t = dn->string;
	while (*s != '\0' && *s == *t) {s++; t++;}
	return ((*s == SUBSCRIPT_MARK)?(1):(*s))  -  ((*t == SUBSCRIPT_MARK)?(1):(*t));
}

/**
 * dict_order_bare() -- order user vs. dictionary string.
 *
 * Similar to above, except that a "bare" search string will match
 * a dictionary entry with a dot.
 *
 * Assuming that s is a pointer to the search string, and that t is
 * a pointer to a dictionary string, this returns 0 if they match,
 * returns >0 if s>t, and <0 if s<t.
 *
 * The matching is done as follows.  Walk down the strings until you
 * come to the end of one of them, or until you find unequal characters.
 * If the dictionary string contains a SUBSCRIPT_MARK, then replace the
 * mark by "\0", and take the difference.
 */

static inline int dict_order_bare(const char *s, const Dict_node * dn)
{
	const char * t = dn->string;
	while (*s != '\0' && *s == *t) {s++; t++;}
	return (*s)  -  ((*t == SUBSCRIPT_MARK)?(0):(*t));
}

/**
 * dict_order_wild() -- order dictionary strings, with wildcard.
 *
 * This routine is used to support command-line parser users who
 * want to search for all dictionary entries of some given word or
 * partial word, containing a wild-card. This is done by using the
 * !!blah* command at the command-line.  Users need this function to
 * debug the dictionary.  This is the ONLY place in the link-parser
 * where wild-card search is needed; ordinary parsing does not use it.
 *
 * !!blah*.sub is also supported.
 *
 * Assuming that s is a pointer to a search string, and that
 * t is a pointer to a dictionary string, this returns 0 if they
 * match, >0 if s>t, and <0 if s<t.
 *
 * The matching is done as follows.  Walk down the strings until
 * you come to the end of one of them, or until you find unequal
 * characters.  A "*" matches anything before the subscript mark.
 * Otherwise, replace SUBSCRIPT_MARK by "\0", and take the difference.
 * his behavior matches that of the function dict_order_bare().
 */
#define D_DOW 6
static inline int dict_order_wild(const char * s, const Dict_node * dn)
{
	const char * t = dn->string;

	lgdebug(+D_DOW, "search-word='%s' dict-word='%s'\n", s, t);
	while((*s != '\0') && (*s != SUBSCRIPT_MARK) && (*s == *t)) {s++; t++;}

	if (*s == WILD_TYPE) return 0;

	lgdebug(D_DOW, "Result: '%s'-'%s'=%d\n",
	 s, t, ((*s == SUBSCRIPT_MARK)?(0):(*s)) - ((*t == SUBSCRIPT_MARK)?(0):(*t)));
	return ((*s == SUBSCRIPT_MARK)?(0):(*s)) - ((*t == SUBSCRIPT_MARK)?(0):(*t));
}
#undef D_DOW

/**
 * dict_match --  return true if strings match, else false.
 * A "bare" string (one without a subscript) will match any corresponding
 * string with a suffix; so, for example, "make" and "make.n" are
 * a match.  If both strings have subscripts, then the subscripts must match.
 *
 * A subscript is the part that follows the SUBSCRIPT_MARK.
 */
static bool dict_match(const char * s, const char * t)
{
	char *ds, *dt;
	ds = strrchr(s, SUBSCRIPT_MARK);
	dt = strrchr(t, SUBSCRIPT_MARK);

#if SUBSCRIPT_MARK == '.'
	/* a dot at the end or a dot followed by a number is NOT
	 * considered a subscript */
	if ((dt != NULL) && ((*(dt+1) == '\0') ||
	    (isdigit((int)*(dt+1))))) dt = NULL;
	if ((ds != NULL) && ((*(ds+1) == '\0') ||
	    (isdigit((int)*(ds+1))))) ds = NULL;
#endif

	/* dt is NULL when there's no prefix ... */
	if (dt == NULL && ds != NULL) {
		if (((int)strlen(t)) > (ds-s)) return false;   /* we need to do this to ensure that */
		return (strncmp(s, t, ds-s) == 0);             /* "i.e." does not match "i.e" */
	} else if (dt != NULL && ds == NULL) {
		if (((int)strlen(s)) > (dt-t)) return false;
		return (strncmp(s, t, dt-t) == 0);
	} else {
		return (strcmp(s, t) == 0);
	}
}

/* ======================================================================== */

static inline Dict_node * dict_node_new(void)
{
	return (Dict_node*) xalloc(sizeof(Dict_node));
}

static inline void free_dict_node(Dict_node *dn)
{
	xfree((char *)dn, sizeof(Dict_node));
}

/**
 * prune_lookup_list -- discard all list entries that don't match string
 * Walk the lookup list (of right links), discarding all nodes that do
 * not match the dictionary string s. The matching is dictionary matching:
 * subscripted entries will match "bare" entries.
 */
static Dict_node * prune_lookup_list(Dict_node *llist, const char * s)
{
	Dict_node *dn, *dnx, *list_new;

	list_new = NULL;
	for (dn = llist; dn != NULL; dn = dnx)
	{
		dnx = dn->right;
		/* now put dn onto the answer list, or free it */
		if (dict_match(dn->string, s))
		{
			dn->right = list_new;
			list_new = dn;
		}
		else
		{
			free_dict_node(dn);
		}
	}

	/* now reverse the list back */
	llist = NULL;
	for (dn = list_new; dn != NULL; dn = dnx)
	{
		dnx = dn->right;
		dn->right = llist;
		llist = dn;
	}
	return llist;
}

/* ======================================================================== */
static bool subscr_match(const char *s, const Dict_node * dn)
{
	const char * s_sub = strrchr(s, SUBSCRIPT_MARK);
	const char * t_sub;

	if (NULL == s_sub) return true;
	t_sub = strrchr(dn->string, SUBSCRIPT_MARK);
	if (NULL == t_sub) return false;
	if ( 0 == strcmp(s_sub, t_sub)) return true;

	return false;
}

/**
 * rdictionary_lookup() -- recursive dictionary lookup
 * Walk binary tree, given by 'dn', looking for the string 's'.
 * For every node in the tree where 's' matches,
 * make a copy of that node, and append it to llist.
 */
static Dict_node *
rdictionary_lookup(Dict_node *llist,
                   const Dict_node * dn,
                   const char * s,
                   bool match_idiom,
                   int (*dict_order)(const char *, const Dict_node *))
{
	/* see comment in dictionary_lookup below */
	int m;
	Dict_node * dn_new;
	if (dn == NULL) return llist;

	m = dict_order(s, dn);

	if (m >= 0)
	{
		llist = rdictionary_lookup(llist, dn->right, s, match_idiom, dict_order);
	}
	if ((m == 0) && (match_idiom || !is_idiom_word(dn->string)) &&
		 (dict_order != dict_order_wild || subscr_match(s, dn)))
	{
		dn_new = dict_node_new();
		*dn_new = *dn;
		dn_new->right = llist;
		llist = dn_new;
	}
	if (m <= 0)
	{
		llist = rdictionary_lookup(llist, dn->left, s, match_idiom, dict_order);
	}
	return llist;
}

/**
 * file_lookup_list() - return list of words in the file-backed dictionary.
 *
 * Returns a pointer to a lookup list of the words in the dictionary.
 * Matches include words that appear in idioms.  To exclude idioms, use
 * abridged_lookup_list() to obtain matches.
 *
 * This list is made up of Dict_nodes, linked by their right pointers.
 * The node, file and string fields are copied from the dictionary.
 *
 * The returned list must be freed with file_free_lookup().
 */
Dict_node * file_lookup_list(const Dictionary dict, const char *s)
{
	Dict_node * llist =
		rdictionary_lookup(NULL, dict->root, s, true, dict_order_bare);
	llist = prune_lookup_list(llist, s);
	return llist;
}

bool file_boolean_lookup(Dictionary dict, const char *s)
{
	Dict_node *llist = file_lookup_list(dict, s);
	bool boool = (llist != NULL);
	file_free_lookup(llist);
	return boool;
}

void file_free_lookup(Dict_node *llist)
{
	Dict_node * n;
	while (llist != NULL)
	{
		n = llist->right;
		free_dict_node(llist);
		llist = n;
	}
}

void free_insert_list(Dict_node *ilist)
{
	Dict_node * n;
	while (ilist != NULL)
	{
		n = ilist->left;
		free_dict_node(ilist);
		ilist = n;
	}
}

/**
 * file_lookup_wild -- allows for wildcard searches (globs)
 * Used to support the !! command in the parser command-line tool.
 */
Dict_node * file_lookup_wild(Dictionary dict, const char *s)
{
	bool lookup_idioms = test_enabled("lookup-idioms");
	char * ds = strrchr(s, SUBSCRIPT_DOT); /* Only the rightmost dot is a
	                                          candidate for SUBSCRIPT_DOT */
	char * ws = strrchr(s, WILD_TYPE);     /* A SUBSCRIPT_DOT can only appear
                                             after a wild-card */
	Dict_node * result;
	char * stmp = strdup(s);

	/* It is not a SUBSCRIPT_DOT if it is at the end or before the wild-card.
	 * E.g: "Dr.", "i.*", "." */
	if ((NULL != ds) && ('\0' != ds[1]) && ((NULL == ws) || (ds > ws)))
		stmp[ds-s] = SUBSCRIPT_MARK;

	result =
	 rdictionary_lookup(NULL, dict->root, stmp, lookup_idioms, dict_order_wild);
	free(stmp);
	return result;
}

/**
 * abridged_lookup_list() - return lookup list of words in the dictionary
 *
 * Returns a pointer to a lookup list of the words in the dictionary.
 * Excludes any idioms that contain the word; use
 * dictionary_lookup_list() to obtain the complete list.
 *
 * This list is made up of Dict_nodes, linked by their right pointers.
 * The node, file and string fields are copied from the dictionary.
 *
 * The returned list must be freed with file_free_lookup().
 */
static Dict_node * abridged_lookup_list(const Dictionary dict, const char *s)
{
	Dict_node *llist;
	llist = rdictionary_lookup(NULL, dict->root, s, false, dict_order_bare);
	llist = prune_lookup_list(llist, s);
	return llist;
}

/* ======================================================================== */
/**
 * Allocate a new Exp node and link it into the exp_list for freeing later.
 */
Exp * Exp_create(Exp_list *eli)
{
	Exp * e;
	e = (Exp *) xalloc(sizeof(Exp));
	e->next = eli->exp_list;
	eli->exp_list = e;
	return e;
}

static inline void exp_free(Exp * e)
{
	xfree((char *)e, sizeof(Exp));
}

/**
 * This creates a node with zero children.  Initializes
 * the cost to zero.
 */
static Exp * make_zeroary_node(Exp_list * eli)
{
	Exp * n;
	n = Exp_create(eli);
	n->type = AND_type;  /* these must be AND types */
	n->cost = 0.0;
	n->u.l = NULL;
	return n;
}

/**
 * This creates a node with one child (namely e).  Initializes
 * the cost to zero.
 */
static Exp * make_unary_node(Exp_list * eli, Exp * e)
{
	Exp * n;
	n = Exp_create(eli);
	n->type = AND_type;  /* these must be AND types */
	n->cost = 0.0;
	n->u.l = (E_list *) xalloc(sizeof(E_list));
	n->u.l->next = NULL;
	n->u.l->e = e;
	return n;
}

/**
 * Create an AND_type expression. The expressions nl, nr will be
 * AND-ed together.
 */
static Exp * make_and_node(Exp_list * eli, Exp* nl, Exp* nr)
{
	E_list *ell, *elr;
	Exp* n;

	n = Exp_create(eli);
	n->type = AND_type;
	n->cost = 0.0;

	n->u.l = ell = (E_list *) xalloc(sizeof(E_list));
	ell->next = elr = (E_list *) xalloc(sizeof(E_list));
	elr->next = NULL;

	ell->e = nl;
	elr->e = nr;
	return n;
}

/**
 * Create an OR_type expression. The expressions nl, nr will be
 * OR-ed together.
 */
static Exp * make_or_node(Exp_list *eli, Exp* nl, Exp* nr)
{
	E_list *ell, *elr;
	Exp* n;

	n = Exp_create(eli);
	n->type = OR_type;
	n->cost = 0.0;

	n->u.l = ell = (E_list *) xalloc(sizeof(E_list));
	ell->next = elr = (E_list *) xalloc(sizeof(E_list));
	elr->next = NULL;

	ell->e = nl;
	elr->e = nr;
	return n;
}

/**
 * This creates an OR node with two children, one the given node,
 * and the other as zeroary node.  This has the effect of creating
 * what used to be called an optional node.
 */
static Exp * make_optional_node(Exp_list *eli, Exp * e)
{
	return make_or_node(eli, make_zeroary_node(eli), e);
}

/**
 * make_dir_connector() -- make a single node for a connector
 * that is a + or a - connector.
 *
 * Assumes the current token is the connector.
 */
static Exp * make_dir_connector(Dictionary dict, int i)
{
	Exp* n = Exp_create(&dict->exp_list);
	n->dir = dict->token[i];
	dict->token[i] = '\0';   /* get rid of the + or - */
	if (dict->token[0] == '@')
	{
		n->u.string = string_set_add(dict->token+1, dict->string_set);
		n->multi = true;
	}
	else
	{
		n->u.string = string_set_add(dict->token, dict->string_set);
		n->multi = false;
	}
	n->type = CONNECTOR_type;
	n->cost = 0.0;
	return n;
}

/* ======================================================================== */
/**
 * make_connector() -- make a node for a connector or dictionary word.
 *
 * Assumes the current token is a connector or dictionary word.
 */
static Exp * make_connector(Dictionary dict)
{
	Exp * n;
	Dict_node *dn, *dn_head;
	int i;

	i = strlen(dict->token) - 1;  /* this must be +, - or * if a connector */
	if ((dict->token[i] != '+') &&
	    (dict->token[i] != '-') &&
	    (dict->token[i] != ANY_DIR))
	{
		/* If we are here, token is a word */
		patch_subscript(dict->token);
		dn_head = abridged_lookup_list(dict, dict->token);
		dn = dn_head;
		while ((dn != NULL) && (strcmp(dn->string, dict->token) != 0))
		{
			dn = dn->right;
		}
		if (dn == NULL)
		{
			file_free_lookup(dn_head);
			dict_error(dict, "\nPerhaps missing + or - in a connector.\n"
			                 "Or perhaps you forgot the subscript on a word.\n"
			                 "Or perhaps a word is used before it is defined.\n");
			return NULL;
		}
		n = make_unary_node(&dict->exp_list, dn->exp);
		file_free_lookup(dn_head);
	}
	else
	{
		/* If we are here, token is a connector */
		if (!check_connector(dict, dict->token))
		{
			return NULL;
		}
		if ((dict->token[i] == '+') || (dict->token[i] == '-'))
		{
			/* A simple, unidirectional connector. Just make that. */
			n = make_dir_connector(dict, i);
		}
		else if (dict->token[i] == ANY_DIR)
		{
			Exp *plu, *min;
			/* If we are here, then it's a bi-directional connector.
			 * Make both a + and a - version, and or them together.  */
			dict->token[i] = '+';
			plu = make_dir_connector(dict, i);
			dict->token[i] = '-';
			min = make_dir_connector(dict, i);

			n = make_or_node(&dict->exp_list, plu, min);
		}
		else
		{
			dict_error(dict, "Unknown connector direction type.");
			return NULL;
		}
	}

	if (!link_advance(dict))
	{
		exp_free(n);
		return NULL;
	}
	return n;
}

/* ======================================================================== */
/* Empty-word handling. */

/** Insert ZZZ+ connectors.
 *  This function was mainly used to support using empty-words, a concept
 *  that has been eliminated. However, it is still used to support linking of
 *  quotes that don't get the QUc/QUd links.
 */
void add_empty_word(Dictionary const dict, X_node *x)
{
	Exp *zn, *an;
	E_list *elist, *flist;
	Exp_list eli = { NULL };

	/* The left-wall already has ZZZ-. The right-wall will not arrive here. */
	if (MT_WALL == x->word->morpheme_type) return;

	/* Replace plain-word-exp by {ZZZ+} & (plain-word-exp) in each X_node.  */
	for(; NULL != x; x = x->next)
	{
		/* Ignore stems for now, decreases a little the overhead for
		 * stem-suffix languages. */
		if (is_stem(x->string)) continue; /* Avoid an unneeded overhead. */
		//lgdebug(+0, "Processing '%s'\n", x->string);

		/* zn points at {ZZZ+} */
		zn = Exp_create(&eli);
		zn->dir = '+';
		zn->u.string = string_set_add(EMPTY_CONNECTOR, dict->string_set);
		zn->multi = false;
		zn->type = CONNECTOR_type;
		zn->cost = 0.0;
		zn = make_optional_node(&eli, zn);

		/* flist is plain-word-exp */
		flist = (E_list *) xalloc(sizeof(E_list));
		flist->next = NULL;
		flist->e = x->exp;

		/* elist is {ZZZ+} , (plain-word-exp) */
		elist = (E_list *) xalloc(sizeof(E_list));
		elist->next = flist;
		elist->e = zn;

		/* an will be {ZZZ+} & (plain-word-exp) */
		an = Exp_create(&eli);
		an->type = AND_type;
		an->cost = 0.0;
		an->u.l = elist;

		x->exp = an;
	}
}

/* ======================================================================== */

/**
 * Return true if the string is a (floating point) number.
 * Float points can be proceeded by a single plus or minus sign.
 */
static bool is_number(const char * str)
{
	if ('+' == str[0] || '-' == str[0]) str++;
	if (strspn(str, "0123456789.") == strlen(str))
		return true;

	return false;
}

/* ======================================================================== */

/* INFIX_NOTATION is always defined; we simply never use the format below. */
/* #if ! defined INFIX_NOTATION */
#if 0

static Exp * expression(Dictionary dict);
/**
 * We're looking at the first of the stuff after an "and" or "or".
 * Build a Exp node for this expression.  Set the cost and optional
 * fields to the default values.  Set the type field according to type
 */
static Exp * operator_exp(Dictionary dict, int type)
{
	Exp * n;
	E_list first;
	E_list * elist;
	n = Exp_create(dict);
	n->type = type;
	n->cost = 0.0;
	elist = &first;
	while((!is_equal(dict, ')')) && (!is_equal(dict, ']')) && (!is_equal(dict, '}'))) {
		elist->next = (E_list *) xalloc(sizeof(E_list));
		elist = elist->next;
		elist->next = NULL;
		elist->e = expression(dict);
		if (elist->e == NULL) {
			return NULL;
		}
	}
	if (elist == &first) {
		dict_error(dict, "An \"or\" or \"and\" of nothing");
		return NULL;
	}
	n->u.l = first.next;
	return n;
}

/**
 * Looks for the stuff that is allowed to be inside of parentheses
 * either & or | followed by a list, or a terminal symbol.
 */
static Exp * in_parens(Dictionary dict)
{
	Exp * e;

	if (is_equal(dict, '&') || (strcmp(token, "and")==0)) {
		if (!link_advance(dict)) {
			return NULL;
		}
		return operator_exp(dict, AND_type);
	} else if (is_equal(dict, '|') || (strcmp(dict->token, "or")==0)) {
		if (!link_advance(dict)) {
			return NULL;
		}
		return operator_exp(dict, OR_type);
	} else {
		return expression(dict);
	}
}

/**
 * Build (and return the root of) the tree for the expression beginning
 * with the current token.  At the end, the token is the first one not
 * part of this expression.
 */
static Exp * expression(Dictionary dict)
{
	Exp * n;
	if (is_equal(dict, '(')) {
		if (!link_advance(dict)) {
			return NULL;
		}
		n = in_parens(dict);
		if (!is_equal(dict, ')')) {
			dict_error(dict, "Expecting a \")\".");
			return NULL;
		}
		if (!link_advance(dict)) {
			return NULL;
		}
	} else if (is_equal(dict, '{')) {
		if (!link_advance(dict)) {
			return NULL;
		}
		n = in_parens(dict);
		if (!is_equal(dict, '}')) {
			dict_error(dict, "Expecting a \"}\".");
			return NULL;
		}
		if (!link_advance(dict)) {
			return NULL;
		}
		n = make_optional_node(dict, n);
	} else if (is_equal(dict, '[')) {
		if (!link_advance(dict)) {
			return NULL;
		}
		n = in_parens(dict);
		if (!is_equal(dict, ']')) {
			dict_error(dict, "Expecting a \"]\".");
			return NULL;
		}
		if (!link_advance(dict)) {
			return NULL;
		}
		n->cost += 1.0;
	} else if (!dict->is_special) {
		n = make_connector(dict);
		if (n == NULL) {
			return NULL;
		}
	} else if (is_equal(dict, ')') || is_equal(dict, ']')) {
		/* allows "()" or "[]" */
		n = make_zeroary_node(dict);
	} else {
			dict_error(dict, "Connector, \"(\", \"[\", or \"{\" expected.");
		return NULL;
	}
	return n;
}

/* ======================================================================== */
#else /* This is for infix notation */

static Exp * restricted_expression(Dictionary dict, int and_ok, int or_ok);

/**
 * Build (and return the root of) the tree for the expression beginning
 * with the current token.  At the end, the token is the first one not
 * part of this expression.
 */
static Exp * make_expression(Dictionary dict)
{
	return restricted_expression(dict, true, true);
}

static Exp * restricted_expression(Dictionary dict, int and_ok, int or_ok)
{
	Exp *nl = NULL;

	if (is_equal(dict, '('))
	{
		if (!link_advance(dict)) {
			return NULL;
		}
		nl = make_expression(dict);
		if (nl == NULL) {
			return NULL;
		}
		if (!is_equal(dict, ')')) {
			dict_error(dict, "Expecting a \")\".");
			return NULL;
		}
		if (!link_advance(dict)) {
			return NULL;
		}
	}
	else if (is_equal(dict, '{'))
	{
		if (!link_advance(dict)) {
			return NULL;
		}
		nl = make_expression(dict);
		if (nl == NULL) {
			return NULL;
		}
		if (!is_equal(dict, '}')) {
			dict_error(dict, "Expecting a \"}\".");
			return NULL;
		}
		if (!link_advance(dict)) {
			return NULL;
		}
		nl = make_optional_node(&dict->exp_list, nl);
	}
	else if (is_equal(dict, '['))
	{
		if (!link_advance(dict)) {
			return NULL;
		}
		nl = make_expression(dict);
		if (nl == NULL) {
			return NULL;
		}
		if (!is_equal(dict, ']')) {
			dict_error(dict, "Expecting a \"]\".");
			return NULL;
		}
		if (!link_advance(dict)) {
			return NULL;
		}

		/* A square bracket can have a number after it.
		 * If it is present, then that number is interpreted
		 * as the cost of the bracket. Else, the cost of a
		 * square bracket is 1.0.
		 */
		if (is_number(dict->token))
		{
			nl->cost += atof(dict->token);
			if (!link_advance(dict)) {
				return NULL;
			}
		}
		else
		{
			nl->cost += 1.0;
		}
	}
	else if (!dict->is_special)
	{
		nl = make_connector(dict);
		if (nl == NULL) {
			return NULL;
		}
	}
	else if (is_equal(dict, ')') || is_equal(dict, ']'))
	{
		/* allows "()" or "[]" */
		nl = make_zeroary_node(&dict->exp_list);
	}
	else
	{
		dict_error(dict, "Connector, \"(\", \"[\", or \"{\" expected.");
		return NULL;
	}

	/* Non-commuting AND */
	if (is_equal(dict, '&') || (strcmp(dict->token, "and") == 0))
	{
		Exp *nr;

		if (!and_ok) {
			warning(dict, "\"and\" and \"or\" at the same level in an expression");
		}
		if (!link_advance(dict)) {
			return NULL;
		}
		nr = restricted_expression(dict, true, false);
		if (nr == NULL) {
			return NULL;
		}
		return make_and_node(&dict->exp_list, nl, nr);
	}
	/* Commuting OR */
	else if (is_equal(dict, '|') || (strcmp(dict->token, "or") == 0))
	{
		Exp *nr;

		if (!or_ok) {
			warning(dict, "\"and\" and \"or\" at the same level in an expression");
		}
		if (!link_advance(dict)) {
			return NULL;
		}
		nr = restricted_expression(dict, false, true);
		if (nr == NULL) {
			return NULL;
		}
		return make_or_node(&dict->exp_list, nl, nr);
	}
	/* Commuting AND */
	else if (is_equal(dict, SYM_AND) || (strcmp(dict->token, "sym") == 0))
	{
		Exp *nr, *na, *nb, *or;

		if (!and_ok) {
			warning(dict, "\"and\" and \"or\" at the same level in an expression");
		}
		if (!link_advance(dict)) {
			return NULL;
		}
		nr = restricted_expression(dict, true, false);
		if (nr == NULL) {
			return NULL;
		}

		/* Expand A ^ B into the expr ((A & B) or (B & A)).
		 * Must be wrapped in unary node so that it can be
		 * mixed with ordinary ands at the same level. */
		na = make_and_node(&dict->exp_list, nl, nr);
		nb = make_and_node(&dict->exp_list, nr, nl);
		or = make_or_node(&dict->exp_list, na, nb);
		return make_unary_node(&dict->exp_list, or);
	}

	return nl;
}

#endif

/* ======================================================================== */
/* Implementation of the DSW algo for rebalancing a binary tree.
 * The point is -- after building the dictionary tree, we rebalance it
 * once at the end. This is a **LOT LOT** quicker than maintaining an
 * AVL tree along the way (less than quarter-of-a-second vs. about
 * a minute or more!) FWIW, the DSW tree is even more balanced than
 * the AVL tree is (it's less deep, more full).
 *
 * The DSW algo, with C++ code, is described in
 *
 * Timothy J. Rolfe, "One-Time Binary Search Tree Balancing:
 * The Day/Stout/Warren (DSW) Algorithm", inroads, Vol. 34, No. 4
 * (December 2002), pp. 85-88
 * http://penguin.ewu.edu/~trolfe/DSWpaper/
 */

static Dict_node *rotate_right(Dict_node *root)
{
	Dict_node *pivot = root->left;
	root->left = pivot->right;
	pivot->right = root;
	return pivot;
}

static Dict_node * dsw_tree_to_vine (Dict_node *root)
{
	Dict_node *vine_tail, *vine_head, *rest;
	Dict_node vh;

	vine_head = &vh;
	vine_head->left = NULL;
	vine_head->right = root;
	vine_tail = vine_head;
	rest = root;

	while (NULL != rest)
	{
		/* If no left, we are done, do the right */
		if (NULL == rest->left)
		{
			vine_tail = rest;
			rest = rest->right;
		}
		/* eliminate the left subtree */
		else
		{
			rest = rotate_right(rest);
			vine_tail->right = rest;
		}
	}

	return vh.right;
}

static void dsw_compression (Dict_node *root, unsigned int count)
{
	unsigned int j;
	for (j = 0; j < count; j++)
	{
		/* Compound left rotation */
		Dict_node * pivot = root->right;
		root->right = pivot->right;
		root = pivot->right;
		pivot->right = root->left;
		root->left = pivot;
	}
}

/* Return size of the full portion of the tree
 * Gets the next pow(2,k)-1
 */
static inline unsigned int full_tree_size (unsigned int size)
{
	unsigned int pk = 1;
	while (pk < size) pk = 2*pk + 1;
	return pk/2;
}

static Dict_node * dsw_vine_to_tree (Dict_node *root, int size)
{
	Dict_node vine_head;
	unsigned int full_count = full_tree_size(size +1);

	vine_head.left = NULL;
	vine_head.right = root;

	dsw_compression(&vine_head, size - full_count);
	for (size = full_count ; size > 1 ; size /= 2)
	{
		dsw_compression(&vine_head, size / 2);
	}
	return vine_head.right;
}

/* ======================================================================== */
/**
 * Insert the new node into the dictionary below node n.
 * Give error message if the new element's string is already there.
 * Assumes that the "n" field of new is already set, and the left
 * and right fields of it are NULL.
 *
 * The resulting tree is highly unbalanced. It needs to be rebalanced
 * before being used.  The DSW algo below is ideal for that.
 */
Dict_node * insert_dict(Dictionary dict, Dict_node * n, Dict_node * newnode)
{
	int comp;

	if (NULL == n) return newnode;

	comp = dict_order_strict(newnode->string, n);
	if (comp < 0)
	{
		if (NULL == n->left)
		{
			n->left = newnode;
			return n;
		}
		n->left = insert_dict(dict, n->left, newnode);
		return n;
		/* return rebalance(n); Uncomment to get an AVL tree */
	}
	else if (comp > 0)
	{
		if (NULL == n->right)
		{
			n->right = newnode;
			return n;
		}
		n->right = insert_dict(dict, n->right, newnode);
		return n;
		/* return rebalance(n); Uncomment to get an AVL tree */
	}
	else
	{
		char t[256];
		snprintf(t, 256, "The word \"%s\" has been multiply defined\n", newnode->string);
		dict_error(dict, t);
		return NULL;
	}
}

/**
 * insert_list() -
 * p points to a list of dict_nodes connected by their left pointers.
 * l is the length of this list (the last ptr may not be NULL).
 * It inserts the list into the dictionary.
 * It does the middle one first, then the left half, then the right.
 *
 * Note: I think this "insert middle, then left, then right" algo has
 * its origins as a lame attempt to hack around the fact that the
 * resulting binary tree is rather badly unbalanced. This has been
 * fixed by using the DSW rebalancing algo. Now, that would seem
 * to render this crazy bisected-insertion algo obsolete, but ..
 * oddly enough, it seems to make the DSW balancing go really fast!
 * Faster than a simple insertion. Go figure. I think this has
 * something to do with the fact that the dictionaries are in
 * alphabetical order! This subdivision helps randomize a bit.
 */
void insert_list(Dictionary dict, Dict_node * p, int l)
{
	Dict_node * dn, *dn_head, *dn_second_half;
	int k, i; /* length of first half */

	if (l == 0) return;

	k = (l-1)/2;
	dn = p;
	for (i = 0; i < k; i++)
	{
		dn = dn->left;
	}

	/* dn now points to the middle element */
	dn_second_half = dn->left;
	dn->left = dn->right = NULL;

	if (contains_underbar(dn->string))
	{
		insert_idiom(dict, dn);
	}
	else if (is_idiom_word(dn->string))
	{
		prt_error("Warning: Word \"%s\" found near line %d of %s.\n"
		        "\tWords ending \".Ix\" (x a number) are reserved for idioms.\n"
		        "\tThis word will be ignored.",
		        dn->string, dict->line_number, dict->name);
		free_dict_node(dn);
	}
	else if ((dn_head = abridged_lookup_list(dict, dn->string)) != NULL)
	{
		char *u;
		Dict_node *dnx;

		u = strchr(dn->string, SUBSCRIPT_MARK);
		if (u) *u = SUBSCRIPT_DOT;
		prt_error("Warning: The word \"%s\" "
		          "found near line %d of %s matches the following words:",
	             dn->string, dict->line_number, dict->name);
		for (dnx = dn_head; dnx != NULL; dnx = dnx->right) {
			prt_error("\a\t%s", dnx->string);
		}
		prt_error("\a\n\tThis word will be ignored.\n");
		file_free_lookup(dn_head);
		free_dict_node(dn);
	}
	else
	{
		dict->root = insert_dict(dict, dict->root, dn);
		dict->num_entries++;
	}

	insert_list(dict, p, k);
	insert_list(dict, dn_second_half, l-k-1);
}

/**
 * read_entry() -- read one dictionary entry
 * Starting with the current token, parse one dictionary entry.
 * A single dictionary entry must have one and only one colon in it,
 * and is terminated by a semi-colon.
 * Add these words to the dictionary.
 */
static bool read_entry(Dictionary dict)
{
	Exp *n;
	int i;

	Dict_node *dnx, *dn = NULL;

	while (!is_equal(dict, ':'))
	{
		if (dict->is_special)
		{
			dict_error(dict, "I expected a word but didn\'t get it.");
			goto syntax_error;
		}

		/* If it's a word-file name */
		/* However, be careful to reject "/.v" which is the division symbol
		 * used in equations (.v means verb-like) */
		if ((dict->token[0] == '/') && (dict->token[1] != '.'))
		{
			dn = read_word_file(dict, dn, dict->token);
			if (dn == NULL)
			{
				prt_error("Error opening word file %s\n", dict->token);
				return false;
			}
		}
		else if ((dict->token[0] == '#') && (0 == strcmp(dict->token, "#include")))
		{
			bool rc;
			char* instr;
			char* dict_name;
			const char * save_name;
			bool save_is_special;
			const char * save_input;
			const char * save_pin;
			char save_already_got_it;
			int save_line_number;
			size_t skip_slash;

			if (!link_advance(dict)) goto syntax_error;

			skip_slash          = ('/' == dict->token[0]) ? 1 : 0;
			dict_name           = strdup(dict->token);
			save_name           = dict->name;
			save_is_special     = dict->is_special;
			save_input          = dict->input;
			save_pin            = dict->pin;
			save_already_got_it = dict->already_got_it;
			save_line_number    = dict->line_number;

			/* OK, token contains the filename to read ... */
			instr = get_file_contents(dict_name + skip_slash);
			if (NULL == instr)
			{
				prt_error("Error: Could not open subdictionary \"%s\"\n", dict_name);
				goto syntax_error;
			}
			dict->input = instr;
			dict->pin = dict->input;

			/* The line number and dict name are used for error reporting */
			dict->line_number = 0;
			dict->name = dict_name;

			/* Now read the thing in. */
			rc = read_dictionary(dict);

			dict->name           = save_name;
			dict->is_special     = save_is_special;
			dict->input          = save_input;
			dict->pin            = save_pin;
			dict->already_got_it = save_already_got_it;
			dict->line_number    = save_line_number;

			free(instr);
			free(dict_name);
			if (!rc) goto syntax_error;

			/* when we return, point to the next entry */
			if (!link_advance(dict)) goto syntax_error;

			/* If a semicolon follows the include, that's OK... ignore it. */
			if (';' == dict->token[0])
			{
				if (!link_advance(dict)) goto syntax_error;
			}

			return true;
		}
		else
		{
			Dict_node * dn_new = dict_node_new();
			dn_new->left = dn;
			dn_new->right = NULL;
			dn_new->exp = NULL;
			dn = dn_new;
			dn->file = NULL;

			/* Note: The following patches a dot in regexes appearing in
			 * the affix file... It is corrected later. */
			patch_subscript(dict->token);
			dn->string = string_set_add(dict->token, dict->string_set);
		}

		/* Advance to next entry, unless error */
		if (!link_advance(dict)) goto syntax_error;
	}

	/* pass the : */
	if (!link_advance(dict))
	{
		goto syntax_error;
	}

	n = make_expression(dict);
	if (n == NULL)
	{
		goto syntax_error;
	}

	if (!is_equal(dict, ';'))
	{
		dict_error(dict, "Expecting \";\" at the end of an entry.");
		goto syntax_error;
	}

	/* pass the ; */
	if (!link_advance(dict))
	{
		goto syntax_error;
	}

	/* At this point, dn points to a list of Dict_nodes connected by
	 * their left pointers. These are to be inserted into the dictionary */
	i = 0;
	for (dnx = dn; dnx != NULL; dnx = dnx->left)
	{
		dnx->exp = n;
		i++;
	}
	dict->insert_entry(dict, dn, i);
	return true;

syntax_error:
	free_insert_list(dn);
	return false;
}


static void rprint_dictionary_data(Dict_node * n)
{
	if (n == NULL) return;
	rprint_dictionary_data(n->left);
	printf("%s: ", n->string);
	print_expression(n->exp);
	printf("-6-\n");
	rprint_dictionary_data(n->right);
}

/**
 * Dump the entire contents of the dictionary
 * XXX This is not currently called by anything, but is a "good thing
 * to keep around".
 */
void print_dictionary_data(Dictionary dict)
{
	rprint_dictionary_data(dict->root);
}

bool read_dictionary(Dictionary dict)
{
	if (!link_advance(dict))
	{
		return false;
	}
	/* The last character of a dictionary is NUL.
	 * Note: At the end of reading a dictionary, dict->pin points to one
	 * character after the input. Referring its [-1] element is safe even if
	 * the dict file size is 0. */
	while ('\0' != dict->pin[-1])
	{
		if (!read_entry(dict))
		{
			return false;
		}
	}
	dict->root = dsw_tree_to_vine(dict->root);
	dict->root = dsw_vine_to_tree(dict->root, dict->num_entries);
	return true;
}

/* ======================================================================= */
