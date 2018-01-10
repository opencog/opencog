/*************************************************************************/
/* Copyright (c) 2004                                                    */
/* Daniel Sleator, David Temperley, and John Lafferty                    */
/* All rights reserved                                                   */
/*                                                                       */
/* Use of the link grammar parsing system is subject to the terms of the */
/* license set forth in the LICENSE file included with this software.    */
/* This license allows free redistribution and use in source and binary  */
/* forms, with or without modification, subject to certain conditions.   */
/*                                                                       */
/*************************************************************************/

/***********************************************************************
 pp_knowledge.c
 7/97
 Contains rules and associated information for post processing. This
 information is supplied in a human-readable file and is parsed by
 pp_lexer.h
***********************************************************************/

#include "externs.h"
#include "pp_knowledge.h"
#include "pp_lexer.h"
#include "pp_linkset.h"
#include "string-set.h"
#include "dict-common/file-utils.h" // for dictopen
#include "utilities.h" // for xalloc

#define D_PPK 10                       /* verbosity level for this file */
#define PP_MAX_UNIQUE_LINK_NAMES 1024  /* just needs to be approximate */

/****************** non-exported functions ***************************/

static bool check_domain_is_legal(pp_knowledge *k, const char *p)
{
  if (0x0 != p[1])
  {
    prt_error("Error: File %s: Domain (%s) must be a single character\n",
              k->path, p);
    return false;
  }
  return true;
}

static void initialize_set_of_links_starting_bounded_domain(pp_knowledge *k)
{
  int i,j,d,domain_of_rule;
  k->set_of_links_starting_bounded_domain =
    pp_linkset_open(PP_MAX_UNIQUE_LINK_NAMES);
  for (i=0; k->bounded_rules[i].msg!=0; i++)
    {
      domain_of_rule = k->bounded_rules[i].domain;
      for (j=0; (d=(k->starting_link_lookup_table[j].domain))!=-1; j++)
        if (d==domain_of_rule)
          pp_linkset_add(k->set_of_links_starting_bounded_domain,
                      k->starting_link_lookup_table[j].starting_link);
    }
}

/**
 * Read table of [link, domain type].
 * This tells us what domain type each link belongs to.
 * This lookup table *must* be defined in the knowledge file.
 */
static bool read_starting_link_table(pp_knowledge *k)
{
  const char *p;
  const char label[] = "STARTING_LINK_TYPE_TABLE";
  size_t i, even;
  int n_tokens;

  if (!pp_lexer_set_label(k->lt, label))
  {
    prt_error("Error: File %s: Couldn't find starting link table %s\n",
              k->path, label);
    return false;
  }

  n_tokens = pp_lexer_count_tokens_of_label(k->lt);
  if (-1 == n_tokens) return false;
  even = n_tokens % 2;
  if(0 != even)
  {
    prt_error("Error: Link table must have format [<link> <domain name>]+\n");
    return false;
  }

  k->nStartingLinks = n_tokens/2;
  k->starting_link_lookup_table = (StartingLinkAndDomain*)
    xalloc((1+k->nStartingLinks)*sizeof(StartingLinkAndDomain));
  for (i=0; i<k->nStartingLinks; i++)
  {
      /* read the starting link itself */
      k->starting_link_lookup_table[i].starting_link =
         string_set_add(pp_lexer_get_next_token_of_label(k->lt),k->string_set);

      /* read the domain type of the link */
      p = pp_lexer_get_next_token_of_label(k->lt);
      if (!check_domain_is_legal(k, p)) return false;
      k->starting_link_lookup_table[i].domain = (int) p[0];
  }

  /* end sentinel */
  k->starting_link_lookup_table[k->nStartingLinks].domain = -1;
  return true;
}

static pp_linkset LINK_SET_ERROR; /* sentinel value for error */

static pp_linkset *read_link_set(pp_knowledge *k,
                                 const char *label, String_set *ss)
{
  /* read link set, marked by label in knowledge file, into a set of links
     whose handle is returned. Return NULL if link set not defined in file,
     in which case the set is taken to be empty. */
  int n_strings,i;
  pp_linkset *ls;
  if (!pp_lexer_set_label(k->lt, label))
  {
    if (verbosity_level(+D_PPK))
      prt_error("Warning: File %s: Link set %s not defined: assuming empty\n",
             k->path, label);
    n_strings = 0;
  }
  else
  {
    n_strings = pp_lexer_count_tokens_of_label(k->lt);
    if (-1 == n_strings) return &LINK_SET_ERROR;
  }
  ls = pp_linkset_open(n_strings);
  for (i=0; i<n_strings; i++)
    pp_linkset_add(ls,
                   string_set_add(pp_lexer_get_next_token_of_label(k->lt),ss));
  return ls;
}

static bool read_link_sets(pp_knowledge *k)
{
  String_set *ss = k->string_set;   /* shorthand */
  k->domain_starter_links     =read_link_set(k,"DOMAIN_STARTER_LINKS",ss);
  if (&LINK_SET_ERROR == k->domain_starter_links) return false;

  k->urfl_domain_starter_links=read_link_set(k,"URFL_DOMAIN_STARTER_LINKS",ss);
  if (&LINK_SET_ERROR == k->urfl_domain_starter_links) return false;

  k->domain_contains_links    =read_link_set(k,"DOMAIN_CONTAINS_LINKS",ss);
  if (&LINK_SET_ERROR == k->domain_contains_links) return false;

  k->ignore_these_links       =read_link_set(k,"IGNORE_THESE_LINKS",ss);
  if (&LINK_SET_ERROR == k->ignore_these_links) return false;

  k->restricted_links         =read_link_set(k,"RESTRICTED_LINKS",ss);
  if (&LINK_SET_ERROR == k->domain_starter_links) return false;

  k->must_form_a_cycle_links  =read_link_set(k,"MUST_FORM_A_CYCLE_LINKS",ss);
  if (&LINK_SET_ERROR == k->must_form_a_cycle_links) return false;

  k->urfl_only_domain_starter_links=
      read_link_set(k,"URFL_ONLY_DOMAIN_STARTER_LINKS",ss);
  if (&LINK_SET_ERROR == k->urfl_only_domain_starter_links) return false;

  k->left_domain_starter_links=read_link_set(k,"LEFT_DOMAIN_STARTER_LINKS",ss);
  if (&LINK_SET_ERROR == k->left_domain_starter_links) return false;

  return true;
}

static void free_link_sets(pp_knowledge *k)
{
  pp_linkset_close(k->domain_starter_links);
  pp_linkset_close(k->urfl_domain_starter_links);
  pp_linkset_close(k->domain_contains_links);
  pp_linkset_close(k->ignore_these_links);
  pp_linkset_close(k->restricted_links);
  pp_linkset_close(k->must_form_a_cycle_links);
  pp_linkset_close(k->urfl_only_domain_starter_links);
  pp_linkset_close(k->left_domain_starter_links);
}

static bool read_form_a_cycle_rules(pp_knowledge *k, const char *label)
{
  size_t n_commas, n_tokens;
  size_t r, i;
  pp_linkset *lsHandle;
  const char **tokens;
  if (!pp_lexer_set_label(k->lt, label)) {
      k->n_form_a_cycle_rules = 0;
      if (verbosity_level(+D_PPK))
          prt_error("Warning: File %s: Not using any 'form a cycle' rules\n",
                    k->path);
  }
  else {
    n_commas = pp_lexer_count_commas_of_label(k->lt);
    k->n_form_a_cycle_rules = (n_commas + 1)/2;
  }
  k->form_a_cycle_rules=
    (pp_rule*) xalloc ((1+k->n_form_a_cycle_rules)*sizeof(pp_rule));
  for (r=0; r<k->n_form_a_cycle_rules; r++)
    {
      /* read link set */
      tokens = pp_lexer_get_next_group_of_tokens_of_label(k->lt, &n_tokens);
      if (n_tokens <= 0)
      {
        prt_error("Error: File %s: Syntax error\n", k->path);
        return false;
      }
      lsHandle = pp_linkset_open(n_tokens);
      for (i=0; i<n_tokens; i++)
          pp_linkset_add(lsHandle,string_set_add(tokens[i], k->string_set));
      k->form_a_cycle_rules[r].link_set = lsHandle;

      /* read error message */
      tokens = pp_lexer_get_next_group_of_tokens_of_label(k->lt, &n_tokens);
      if (n_tokens > 1)
      {
         prt_error("Error: File %s: Invalid syntax (rule %zu of %s)\n",
                   k->path, r+1,label);
         return false;
      }
      k->form_a_cycle_rules[r].msg = string_set_add(tokens[0], k->string_set);
      k->form_a_cycle_rules[r].use_count = 0;
    }

  /* sentinel entry */
  k->form_a_cycle_rules[k->n_form_a_cycle_rules].msg = 0;
  k->form_a_cycle_rules[k->n_form_a_cycle_rules].use_count = 0;

  return true;
}

static bool read_bounded_rules(pp_knowledge *k, const char *label)
{
  const char **tokens;
  size_t n_commas, n_tokens;
  size_t r;
  if (!pp_lexer_set_label(k->lt, label)) {
      k->n_bounded_rules = 0;
      if (verbosity_level(+D_PPK))
        prt_error("Warning: File %s: Not using any 'bounded' rules\n", k->path);
  }
  else {
    n_commas = pp_lexer_count_commas_of_label(k->lt);
    k->n_bounded_rules = (n_commas + 1)/2;
  }
  k->bounded_rules = (pp_rule*) xalloc ((1+k->n_bounded_rules)*sizeof(pp_rule));
  for (r=0; r<k->n_bounded_rules; r++)
    {
      /* read domain */
      tokens = pp_lexer_get_next_group_of_tokens_of_label(k->lt, &n_tokens);
      if (n_tokens!=1)
      {
        prt_error("Error: File %s: Invalid syntax: rule %zu of %s\n",
                  k->path, r+1,label);
        return false;
      }
      k->bounded_rules[r].domain = (int) tokens[0][0];

      /* read error message */
      tokens = pp_lexer_get_next_group_of_tokens_of_label(k->lt, &n_tokens);
      if (n_tokens!=1)
      {
        prt_error("Error: File %s: Invalid syntax: rule %zu of %s\n",
                  k->path, r+1,label);
        return false;
      }
      k->bounded_rules[r].msg = string_set_add(tokens[0], k->string_set);
      k->bounded_rules[r].use_count = 0;
    }

  /* sentinel entry */
  k->bounded_rules[k->n_bounded_rules].msg = 0;
  k->bounded_rules[k->n_bounded_rules].use_count = 0;

  return true;
}

static bool read_contains_rules(pp_knowledge *k, const char *label,
                                pp_rule **rules, size_t *nRules)
{
  /* Reading the 'contains_one_rules' and reading the
     'contains_none_rules' into their respective arrays */
  size_t n_tokens, i, r;
  int n_commas;
  const char *p;
  const char **tokens;
  if (!pp_lexer_set_label(k->lt, label)) {
      *nRules = 0;
      if (verbosity_level(+D_PPK))
        prt_error("Warning: File %s: Not using any %s rules\n", k->path, label);
  }
  else {
    n_commas = pp_lexer_count_commas_of_label(k->lt);
    if (-1 == n_commas) return false;
    *nRules = (n_commas + 1)/3;
  }
  *rules = (pp_rule*) xalloc ((1+*nRules)*sizeof(pp_rule));
  for (r=0; r<*nRules; r++)
    {
      /* first read link */
      tokens = pp_lexer_get_next_group_of_tokens_of_label(k->lt, &n_tokens);
      if (n_tokens > 1)
      {
        prt_error("Error: File %s: Invalid syntax in %s (rule %zu)\n",
                  k->path, label, r+1);
        return false;
      }

      (*rules)[r].selector = string_set_add(tokens[0], k->string_set);

      /* read link set */
      tokens = pp_lexer_get_next_group_of_tokens_of_label(k->lt, &n_tokens);
      (*rules)[r].link_set = pp_linkset_open(n_tokens);
      (*rules)[r].link_set_size = n_tokens;
      (*rules)[r].link_array = (const char **) xalloc((1+n_tokens)*sizeof(const char*));
      for (i=0; i<n_tokens; i++)
      {
        p = string_set_add(tokens[i], k->string_set);
        pp_linkset_add((*rules)[r].link_set, p);
        (*rules)[r].link_array[i] = p;
      }
      (*rules)[r].link_array[i]=0; /* NULL-terminator */

      /* read error message */
      tokens = pp_lexer_get_next_group_of_tokens_of_label(k->lt, &n_tokens);
      if (n_tokens > 1)
      {
        prt_error("Error: File %s: Invalid syntax in %s (rule %zu)\n",
                  k->path, label, r+1);
        return false;
      }

      (*rules)[r].msg = string_set_add(tokens[0], k->string_set);
      (*rules)[r].use_count = 0;
    }

  /* sentinel entry */
  (*rules)[*nRules].msg = 0;
  (*rules)[*nRules].use_count = 0;

  return true;
}


static bool read_rules(pp_knowledge *k)
{
  if (!read_form_a_cycle_rules(k, "FORM_A_CYCLE_RULES")) return false;
  if (!read_bounded_rules(k,  "BOUNDED_RULES")) return false;
  if (!read_contains_rules(k, "CONTAINS_ONE_RULES" ,
      &(k->contains_one_rules), &(k->n_contains_one_rules)))
    return false;
  if (!read_contains_rules(k, "CONTAINS_NONE_RULES",
      &(k->contains_none_rules), &(k->n_contains_none_rules)))
    return false;
  return true;
}

static void free_rules(pp_knowledge *k)
{
  size_t r;
  size_t rs = sizeof(pp_rule);
  pp_rule *rule;
  if (NULL != k->contains_one_rules)
  {
    for (r=0; k->contains_one_rules[r].msg!=0; r++)
    {
      rule = &(k->contains_one_rules[r]);    /* shorthand */
      xfree((void*) rule->link_array, (1+rule->link_set_size)*sizeof(char*));
      pp_linkset_close(rule->link_set);
    }
    for (r=0; k->contains_none_rules[r].msg!=0; r++)
    {
      rule = &(k->contains_none_rules[r]);   /* shorthand */
      xfree((void *)rule->link_array, (1+rule->link_set_size)*sizeof(char*));
      pp_linkset_close(rule->link_set);
    }
  }

  for (r = 0; r < k->n_form_a_cycle_rules; r++)
    pp_linkset_close(k->form_a_cycle_rules[r].link_set);
  xfree((void*)k->bounded_rules,           rs*(1+k->n_bounded_rules));
  xfree((void*)k->form_a_cycle_rules,      rs*(1+k->n_form_a_cycle_rules));
  xfree((void*)k->contains_one_rules,      rs*(1+k->n_contains_one_rules));
  xfree((void*)k->contains_none_rules,     rs*(1+k->n_contains_none_rules));
}

/********************* exported functions ***************************/

pp_knowledge *pp_knowledge_open(const char *path)
{
  /* read knowledge from disk into pp_knowledge */
  FILE *f = dictopen(path, "r");
  if (NULL == f)
  {
    prt_error("Error: Couldn't find post-process knowledge file %s\n", path);
    return NULL;
  }
  pp_knowledge *k = (pp_knowledge *) xalloc (sizeof(pp_knowledge));
  *k = (pp_knowledge){0};
  k->lt = pp_lexer_open(f);
  fclose(f);
  if (NULL == k->lt) goto failure;
  k->string_set = string_set_create();
  k->path = string_set_add(path, k->string_set);
  if (!read_starting_link_table(k)) goto failure;

  if (!read_link_sets(k)) goto failure;
  if (!read_rules(k)) goto failure;
  initialize_set_of_links_starting_bounded_domain(k);

  /* If the knowledge file was empty, do nothing at all. */
  if (0 == k->nStartingLinks)
  {
    pp_knowledge_close(k);
    return NULL;
  }

  return k;

failure:
  prt_error("Error: Unable to open knowledge file %s.\n", path);
  pp_knowledge_close(k);
  return NULL;
}

void pp_knowledge_close(pp_knowledge *k)
{
  if (!k) return;
  /* clear the memory taken up by k */
  xfree((void*)k->starting_link_lookup_table,
        ((1+k->nStartingLinks)*sizeof(StartingLinkAndDomain)));
  free_link_sets(k);
  free_rules(k);
  pp_linkset_close(k->set_of_links_starting_bounded_domain);
  string_set_delete(k->string_set);
  if (NULL != k->lt) pp_lexer_close(k->lt);
  xfree((void*)k, sizeof(pp_knowledge));
}

