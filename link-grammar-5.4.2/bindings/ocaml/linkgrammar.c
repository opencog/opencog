/****************************************************************/
/* OCaml Linkgrammar module                                     */
/*                                                              */
/* Interfacing OCaml API to LinkGrammar C API                   */
/*                                                              */
/* Author: Ramu Ramamurthy ramu_ramamurthy at yahoo dot com     */
/* (C) 2006                                                     */
/*                                                              */
/* This is released under the BSD license                       */
/****************************************************************/
/*
#include <stdio.h>
/********************************************/
/* Caml Includes                            */
/********************************************/
#include <caml/mlvalues.h>
#include <caml/memory.h>
#include <caml/alloc.h>
#include <caml/custom.h>
/********************************************/
/* Link Grammar Includes                    */
/********************************************/
#include <link-grammar/link-includes.h>


#define Po_val(v) (*((Parse_Options *)Data_custom_val(v)))
#define Dict_val(v) (*((Dictionary *)Data_custom_val(v)))
#define Sent_val(v) (*((Sentence *)Data_custom_val(v)))
#define Linkage_val(v) (*((Linkage *)Data_custom_val(v)))

#define CAML_GC_ALLOC_USED (1)
#define CAML_GC_ALLOC_MAX (10)

static struct custom_operations custom_default_ops = {
  "linkgrammar",
  custom_finalize_default,
  custom_compare_default,
  custom_hash_default,
  custom_serialize_default,
  custom_deserialize_default
};


/* following are the caml interface funcs */

/* start of operations on parse options */
value po_create(value nothing) {
  CAMLparam1(nothing);
  CAMLlocal1(block);
  block = alloc_custom(&custom_default_ops, sizeof(Parse_Options), 
		       CAML_GC_ALLOC_USED, CAML_GC_ALLOC_MAX);
  Po_val(block) = parse_options_create();
  CAMLreturn(block);
}

value free_po(value l) {
  Parse_Options po = Po_val(l);
  /*printf("**************free po\n");fflush(stdout);*/
  parse_options_delete(po);
  return Val_unit;
}

value po_get_verbosity(value po) {
  CAMLparam1(po);
  Parse_Options opts = Po_val(po);
  int val = parse_options_get_verbosity(opts);
  return Val_long(val);
}

value po_set_verbosity(value po, value lim) {
  CAMLparam2(po,lim);
  Parse_Options opts = Po_val(po);
  parse_options_set_verbosity(opts, Long_val(lim));
  CAMLreturn(Val_unit);
}


value po_get_linkage_limit(value po) {
  CAMLparam1(po);
  Parse_Options opts = Po_val(po);
  int val = parse_options_get_linkage_limit(opts);
  return Val_long(val);
}

value po_set_linkage_limit(value po, value lim) {
  CAMLparam2(po,lim);
  Parse_Options opts = Po_val(po);
  parse_options_set_linkage_limit(opts, Long_val(lim));
  CAMLreturn(Val_unit);
}

value po_get_disjunct_cost(value po) {
  CAMLparam1(po);
  Parse_Options opts = Po_val(po);
  int val = parse_options_get_disjunct_cost(opts);
  return Val_long(val);
}

value po_set_disjunct_cost(value po, value cost) {
  CAMLparam2(po,cost);
  Parse_Options opts = Po_val(po);
  parse_options_set_disjunct_cost(opts, Long_val(cost));
  CAMLreturn(Val_unit);
}

value po_get_min_null_count(value po) {
  CAMLparam1(po);
  Parse_Options opts = Po_val(po);
  int val = parse_options_get_min_null_count(opts);
  CAMLreturn(Val_long(val));
}

value po_set_min_null_count(value po, value count) {
  CAMLparam2(po,count);
  Parse_Options opts = Po_val(po);
  parse_options_set_min_null_count(opts, Long_val(count));
  CAMLreturn(Val_unit);
}

value po_get_max_null_count(value po) {
  CAMLparam1(po);
  Parse_Options opts = Po_val(po);
  int val = parse_options_get_max_null_count(opts);
  CAMLreturn(Val_long(val));
}

value po_set_max_null_count(value po, value count) {
  CAMLparam2(po,count);
  Parse_Options opts = Po_val(po);
  parse_options_set_max_null_count(opts, Long_val(count));
  CAMLreturn(Val_unit);
}

value po_get_null_block(value po) {
  CAMLparam1(po);
  Parse_Options opts = Po_val(po);
  int val = parse_options_get_null_block(opts);
  CAMLreturn(Val_long(val));
}

value po_set_null_block(value po, value count) {
  CAMLparam2(po,count);
  Parse_Options opts = Po_val(po);
  parse_options_set_null_block(opts, Long_val(count));
  CAMLreturn(Val_unit);
}

value po_get_short_length(value po) {
  CAMLparam1(po);
  Parse_Options opts = Po_val(po);
  int val = parse_options_get_short_length(opts);
  CAMLreturn(Val_long(val));
}

value po_set_short_length(value po, value count) {
  CAMLparam2(po,count);
  Parse_Options opts = Po_val(po);
  parse_options_set_short_length(opts, Long_val(count));
  CAMLreturn(Val_unit);
}

value po_get_islands_ok(value po) {
  CAMLparam1(po);
  Parse_Options opts = Po_val(po);
  int val = parse_options_get_islands_ok(opts);
  CAMLreturn(Val_long(val));
}

value po_set_islands_ok(value po, value count) {
  CAMLparam2(po,count);
  Parse_Options opts = Po_val(po);
  parse_options_set_islands_ok(opts, Long_val(count));
  CAMLreturn(Val_unit);
}

value po_get_max_parse_time(value po) {
  CAMLparam1(po);
  Parse_Options opts = Po_val(po);
  int val = parse_options_get_max_parse_time(opts);
  CAMLreturn(Val_long(val));
}

value po_set_max_parse_time(value po, value count) {
  CAMLparam2(po,count);
  Parse_Options opts = Po_val(po);
  parse_options_set_max_parse_time(opts, Long_val(count));
  CAMLreturn(Val_unit);
}

value po_get_timer_expired(value po) {
  CAMLparam1(po);
  Parse_Options opts = Po_val(po);
  int val = parse_options_timer_expired(opts);
  CAMLreturn(Val_long(val));
}

value po_reset_resources(value po) {
  CAMLparam1(po);
  Parse_Options opts = Po_val(po);
  parse_options_reset_resources(opts);
  CAMLreturn(Val_unit);
}


value po_get_allow_null(value po) {
  CAMLparam1(po);
  Parse_Options opts = Po_val(po);
  int val = parse_options_get_allow_null(opts);
  CAMLreturn(Val_long(val));
}

value po_set_allow_null(value po, value count) {
  CAMLparam2(po,count);
  Parse_Options opts = Po_val(po);
  parse_options_set_allow_null(opts, Long_val(count));
  CAMLreturn(Val_unit);
}

value po_get_all_short_connectors(value po) {
  CAMLparam1(po);
  Parse_Options opts = Po_val(po);
  int val = parse_options_get_all_short_connectors(opts);
  CAMLreturn(Val_long(val));
}

value po_set_all_short_connectors(value po, value count) {
  CAMLparam2(po,count);
  Parse_Options opts = Po_val(po);
  parse_options_set_all_short_connectors(opts, Long_val(count));
  CAMLreturn(Val_unit);
}
/* end of operations on parse_options */

/* start of operations on dictionary */
value dict_create(value lang) {
  CAMLparam1(lang);
  CAMLlocal1(block);
  char *lang_str = String_val(lang);

  Dictionary dict = dictionary_create_lang(lang_str);

  if (!dict) {
    printf("Cant open Dictionary!\n"); fflush(stdout);
    raise_not_found();
  }

  block = alloc_custom(&custom_default_ops, sizeof(Dictionary), 
		       CAML_GC_ALLOC_USED, CAML_GC_ALLOC_MAX);
  Dict_val(block) = dict;

  CAMLreturn(block);
}

value free_dict(value l) {
  Dictionary dict = Dict_val(l);
  /*printf("*****************free dict %x\n",(unsigned int)dict);fflush(stdout);*/
  dictionary_delete(dict);
  return Val_unit;
}


/* end of operation on dictionary */


/* start of operations on sentences */
value sent_create(value str, value dic) {
  CAMLparam2(str,dic);
  CAMLlocal1(block);
  char *sent_str = String_val(str);
  Dictionary dict = Dict_val(dic);
  Sentence sent = sentence_create(sent_str, dict);
  if (!sent) {
    /* throw an exception here */
    /* possibly later throw a user defined exception */
    printf("NOTTTT  FOUND\n");fflush(stdout);
    raise_not_found();
  } else {

    block = alloc_custom(&custom_default_ops, sizeof(Sentence), 
			 CAML_GC_ALLOC_USED, CAML_GC_ALLOC_MAX);
    Sent_val(block) = sent;
    CAMLreturn(block);
  }
}

value free_sentence(value l) {
  Sentence sent = Sent_val(l);
  /*printf("***************free sent %x\n",(unsigned int)sent);fflush(stdout);*/
  sentence_delete(sent);
  return Val_unit;
}


value sent_parse(value sentence, value po) {
  CAMLparam2(sentence,po);
  Sentence sent = Sent_val(sentence);
  Parse_Options opts = Po_val(po);
  int ret_val = sentence_parse(sent, opts);
  /* printf("parse done\n");fflush(stdout); */
  CAMLreturn(Val_long(ret_val));
}

value sent_length(value sentence) {
  CAMLparam1(sentence);
  Sentence sent = Sent_val(sentence);
  int val = sentence_length(sent);
  CAMLreturn(Val_long(val));
}

value sent_get_word(value sentence, value ith) {
  CAMLparam2(sentence,ith);
  Sentence sent = Sent_val(sentence);
  char *str = sentence_get_word(sent, Long_val(ith));
  value block = copy_string(str);
  string_delete(str);
  CAMLreturn(block);
}

value sent_null_count(value sentence) {
  CAMLparam1(sentence);
  Sentence sent = Sent_val(sentence);
  int val = sentence_null_count(sent);
  CAMLreturn(Val_long(val));
}

value sent_num_linkages_found(value sentence) {
  CAMLparam1(sentence);
  Sentence sent = Sent_val(sentence);
  int val = sentence_num_linkages_found(sent);
  CAMLreturn(Val_long(val));
}

value sent_num_valid_linkages(value sentence) {
  CAMLparam1(sentence);
  Sentence sent = Sent_val(sentence);
  int val = sentence_num_valid_linkages(sent);
  CAMLreturn(Val_long(val));
}

value sent_num_linkages_post_processed(value sentence) {
  CAMLparam1(sentence);
  Sentence sent = Sent_val(sentence);
  int val = sentence_num_linkages_post_processed(sent);
  CAMLreturn(Val_long(val));
}

value sent_num_violations(value sentence, value ith) {
  CAMLparam2(sentence,ith);
  Sentence sent = Sent_val(sentence);
  int val = sentence_num_violations(sent, Long_val(ith));
  CAMLreturn(Val_long(val));
}

value sent_disjunct_cost(value sentence, value ith) {
  CAMLparam2(sentence,ith);
  Sentence sent = Sent_val(sentence);
  int val = sentence_disjunct_cost(sent, Long_val(ith));
  CAMLreturn(Val_long(val));
}
/* end of operation on sentences */


/* start of operations on linkages */
value link_create(value s, value ith, value po) {
  CAMLparam3(s, ith, po);
  CAMLlocal1(block);
  
  Sentence sent = Sent_val(s);
  Parse_Options opts = Po_val(po);
  Linkage link = linkage_create(Long_val(ith),sent,opts);
  if (!link) {
    /* throw an exception here */
    /* possibly later throw a user defined exception */
    raise_not_found();
  } else {

    block = alloc_custom(&custom_default_ops, sizeof(Linkage), 
			 CAML_GC_ALLOC_USED, CAML_GC_ALLOC_MAX);
    Linkage_val(block) = link;
    CAMLreturn(block);
  }
}

value free_linkage(value l) {
  Linkage link = Linkage_val(l);
  /*printf("*************free linkage %x\n", (unsigned int)link);fflush(stdout);*/
  linkage_delete(link);
  return Val_unit;
}


value link_get_num_words(value l) {
  CAMLparam1(l);
  Linkage link = Linkage_val(l);
  int val = linkage_get_num_words(link);
  CAMLreturn(Val_long(val));
}

value link_get_num_links(value l) {
  CAMLparam1(l);
  Linkage link = Linkage_val(l);
  int val = linkage_get_num_links(link);
  CAMLreturn(Val_long(val));
}

value link_get_link_length(value l, value index) {
  CAMLparam2(l,index);
  Linkage link = Linkage_val(l);
  int val = linkage_get_link_length(link,Long_val(index));
  CAMLreturn(Val_long(val));
}

value link_get_link_lword(value l, value index) {
  CAMLparam2(l,index);
  Linkage link = Linkage_val(l);
  int val = linkage_get_link_lword(link,Long_val(index));
  CAMLreturn(Val_long(val));
}

value link_get_link_rword(value l, value index) {
  CAMLparam2(l,index);
  Linkage link = Linkage_val(l);
  int val = linkage_get_link_rword(link,Long_val(index));
  CAMLreturn(Val_long(val));
}

value link_print_diagram(value l) {
  CAMLparam1(l);
  CAMLlocal1(block);
  Linkage link = Linkage_val(l);
  char *str = linkage_print_diagram(link);
  block = copy_string(str);
  string_delete(str);
  CAMLreturn(block);
}

value link_print_postscript(value l, value mode) {
  CAMLparam2(l,mode);
  CAMLlocal1(block);
  Linkage link = Linkage_val(l);
  char *str = linkage_print_postscript(link, Long_val(mode));
  block = copy_string(str);
  string_delete(str);
  CAMLreturn(block);
}

value link_print_links_and_domains(value l) {
  CAMLparam1(l);
  CAMLlocal1(block);
  Linkage link = Linkage_val(l);
  char *str = linkage_print_links_and_domains(link);
  block = copy_string(str);
  string_delete(str);
  CAMLreturn(block);
}

value link_get_link_label(value l, value index) {
  CAMLparam2(l,index);
  CAMLlocal1(block);
  Linkage link = Linkage_val(l);
  char *str = linkage_get_link_label(link, Long_val(index));
  block = copy_string(str);
  CAMLreturn(block);
}

value link_get_link_llabel(value l, value index) {
  CAMLparam2(l,index);
  CAMLlocal1(block);
  Linkage link = Linkage_val(l);
  char *str = linkage_get_link_llabel(link, Long_val(index));
  block = copy_string(str);
  CAMLreturn(block);
}

value link_get_link_rlabel(value l, value index) {
  CAMLparam2(l,index);
  CAMLlocal1(block);
  Linkage link = Linkage_val(l);
  char *str = linkage_get_link_rlabel(link, Long_val(index));
  block = copy_string(str);
  CAMLreturn(block);
}

value link_get_link_num_domains(value l, value index) {
  CAMLparam2(l,index);
  Linkage link = Linkage_val(l);
  int val = linkage_get_link_num_domains(link,Long_val(index));
  CAMLreturn(Val_long(val));
}

value link_get_violation_name(value l) {
  CAMLparam1(l);
  CAMLlocal1(block);
  Linkage link = Linkage_val(l);
  char *str = linkage_get_violation_name(link);
  block = copy_string(str);
  CAMLreturn(block);
}

value link_get_link_domain_name_i(value l, value index, value i) {
  CAMLparam3(l, index, i);
  CAMLlocal1(block);
  Linkage link = Linkage_val(l);
  char **names = linkage_get_link_domain_names(link, Long_val(index));
  char *str = names[Long_val(i)];
  block = copy_string(str);
  CAMLreturn(block);
}

value link_get_word(value l, value index) {
  CAMLparam2(l,index);
  CAMLlocal1(block);
  Linkage link = Linkage_val(l);
  char *str = linkage_get_word(link, Long_val(index));
  block = copy_string(str);
  CAMLreturn(block);
}

value link_unused_word_cost(value l) {
  CAMLparam1(l);
  Linkage link = Linkage_val(l);
  int val = linkage_unused_word_cost(link);
  CAMLreturn(Val_long(val));
}

value link_disjunct_cost(value l) {
  CAMLparam1(l);
  Linkage link = Linkage_val(l);
  int val = linkage_disjunct_cost(link);
  CAMLreturn(Val_long(val));
}

value link_link_cost(value l) {
  CAMLparam1(l);
  Linkage link = Linkage_val(l);
  int val = linkage_link_cost(link);
  CAMLreturn(Val_long(val));
}
/* end of operations on linkages */
