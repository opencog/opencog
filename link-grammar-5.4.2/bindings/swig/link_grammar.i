/**********************************************************************
*
* SWIG Foreign Function Interface (FFI) definition for the Link Grammar
* shared (dynamic) library.
*
***********************************************************************/
%module clinkgrammar
%{

#include <link-grammar/link-includes.h>

%}

%nodefaultdtor lg_errinfo;

/* Grab non-function definitions so we do not need to repeat them here. */
%rename("$ignore", %$isfunction) "";
#define link_public_api(x) x
#ifndef bool                         /* Prevent syntax errors if no bool. */
#define bool int
#endif /* bool */
%immutable;                          /* Future-proof for const definitions. */
%include ../link-grammar/link-includes.h
%mutable;
%rename("%s") "";                    /* Grab everything for the rest of file. */


const char * linkgrammar_get_version(void);
const char * linkgrammar_get_dict_version(Dictionary dict);
const char * linkgrammar_get_dict_locale(Dictionary dict);

/**********************************************************************
*
* Functions to manipulate Dictionaries
*
***********************************************************************/

Dictionary dictionary_create_lang(const char * lang);
Dictionary dictionary_create_default_lang(void);
const char * dictionary_get_lang(Dictionary dict);

void dictionary_delete(Dictionary dict);
void dictionary_set_data_dir(const char * path);
%newobject dictionary_get_data_dir;
char * dictionary_get_data_dir(void);

/**********************************************************************
*
* Functions to manipulate Parse Options
*
***********************************************************************/

Parse_Options parse_options_create(void);
int parse_options_delete(Parse_Options opts);
void parse_options_set_verbosity(Parse_Options opts, int verbosity);
int  parse_options_get_verbosity(Parse_Options opts);
void parse_options_set_linkage_limit(Parse_Options opts, int linkage_limit);
int  parse_options_get_linkage_limit(Parse_Options opts);
void parse_options_set_disjunct_cost(Parse_Options opts, double disjunct_cost);
double parse_options_get_disjunct_cost(Parse_Options opts);
void parse_options_set_min_null_count(Parse_Options opts, int null_count);
int  parse_options_get_min_null_count(Parse_Options opts);
void parse_options_set_max_null_count(Parse_Options opts, int null_count);
int  parse_options_get_max_null_count(Parse_Options opts);
void parse_options_set_islands_ok(Parse_Options opts, int islands_ok);
int  parse_options_get_islands_ok(Parse_Options opts);
void parse_options_set_short_length(Parse_Options opts, int short_length);
int  parse_options_get_short_length(Parse_Options opts);
void parse_options_set_max_memory(Parse_Options  opts, int mem);
int  parse_options_get_max_memory(Parse_Options opts);
void parse_options_set_max_parse_time(Parse_Options  opts, int secs);
int  parse_options_get_max_parse_time(Parse_Options opts);
void parse_options_set_cost_model_type(Parse_Options opts, Cost_Model_type cm);
Cost_Model_type  parse_options_get_cost_model_type(Parse_Options opts);
int  parse_options_timer_expired(Parse_Options opts);
int  parse_options_memory_exhausted(Parse_Options opts);
int  parse_options_resources_exhausted(Parse_Options opts);
void parse_options_set_display_morphology(Parse_Options opts, int val);
int  parse_options_get_display_morphology(Parse_Options opts);
void parse_options_set_spell_guess(Parse_Options opts, int val);
int  parse_options_get_spell_guess(Parse_Options opts);
void parse_options_set_all_short_connectors(Parse_Options opts, int val);
int  parse_options_get_all_short_connectors(Parse_Options opts);
void parse_options_reset_resources(Parse_Options opts);
void parse_options_set_use_sat_parser(Parse_Options opts, bool val);
int parse_options_get_use_sat_parser(Parse_Options opts);

/**********************************************************************
*
* Functions to manipulate Sentences
*
***********************************************************************/

Sentence sentence_create(const char *input_string, Dictionary dict);
void sentence_delete(Sentence sent);
int  sentence_split(Sentence sent, Parse_Options opts);
int  sentence_parse(Sentence sent, Parse_Options opts);
int  sentence_length(Sentence sent);
int  sentence_null_count(Sentence sent);
int  sentence_num_linkages_found(Sentence sent);
int  sentence_num_valid_linkages(Sentence sent);
int  sentence_num_linkages_post_processed(Sentence sent);
int  sentence_num_violations(Sentence sent, int i);
double sentence_disjunct_cost(Sentence sent, int i);
int  sentence_link_cost(Sentence sent, int i);

/**********************************************************************
*
* Functions that create and manipulate Linkages.
* When a Linkage is requested, the user is given a
* copy of all of the necessary information, and is responsible
* for freeing up the storage when he/she is finished, using
* the routines provided below.
*
***********************************************************************/

/**********************************************************************
*
* These functions allocate strings to be returned, so need to be
* newobject'd to avoid memory leaks
*
***********************************************************************/
%newobject linkage_print_senses;
%newobject linkage_print_links_and_domains;
%newobject linkage_print_diagram;
%newobject linkage_print_postscript;
%newobject linkage_print_constituent_tree;


Linkage linkage_create(int index, Sentence sent, Parse_Options opts);
void linkage_delete(Linkage linkage);

%typemap(newfree) char * {
   linkage_free_diagram($1);
}
char * linkage_print_diagram(Linkage linkage, bool display_walls, size_t screen_width);

%typemap(newfree) char * {
   linkage_free_postscript($1);
}
char * linkage_print_postscript(Linkage linkage, bool display_walls, bool print_ps_header);

%typemap(newfree) char * {
   linkage_free_links_and_domains($1);
}
char * linkage_print_links_and_domains(Linkage linkage);

%typemap(newfree) char * {
   linkage_free_senses($1);
}
char * linkage_print_senses(Linkage linkage);

%typemap(newfree) char * {
   linkage_free_constituent_tree_str($1);
}
char * linkage_print_constituent_tree(Linkage linkage, ConstituentDisplayStyle mode);

%typemap(newfree) char * {
   linkage_free_disjuncts($1);
}
char * linkage_print_disjuncts(const Linkage linkage);

%typemap(newfree) char * {
   linkage_free_pp_msgs($1);
}
char * linkage_print_pp_msgs(Linkage linkage);

// Reset to default.
%typemap(newfree) char * {
   free($1);
}
int linkage_get_num_words(Linkage linkage);
int linkage_get_num_links(Linkage linkage);
int linkage_get_link_lword(Linkage linkage, int index);
int linkage_get_link_rword(Linkage linkage, int index);
int linkage_get_link_length(Linkage linkage, int index);
const char * linkage_get_link_label(Linkage linkage, int index);
const char * linkage_get_link_llabel(Linkage linkage, int index);
const char * linkage_get_link_rlabel(Linkage linkage, int index);
int linkage_get_link_num_domains(Linkage linkage, int index);
const char ** linkage_get_link_domain_names(Linkage linkage, int index);
const char ** linkage_get_words(Linkage linkage);
//const char *  linkage_get_disjunct(Linkage linkage, int w);
const char *  linkage_get_word(Linkage linkage, int w);
int linkage_get_word_byte_start(Linkage linkage, int index);
int linkage_get_word_byte_end(Linkage linkage, int index);
int linkage_get_word_char_start(Linkage linkage, int index);
int linkage_get_word_char_end(Linkage linkage, int index);

int linkage_unused_word_cost(Linkage linkage);
double linkage_disjunct_cost(Linkage linkage);
int linkage_link_cost(Linkage linkage);
double linkage_corpus_cost(Linkage linkage);
const char * linkage_get_violation_name(Linkage linkage);

/* Error-handling facility calls. */
%rename(_lg_error_formatmsg) lg_error_formatmsg;
%newobject lg_error_formatmsg;
char * lg_error_formatmsg(lg_errinfo *lge);
int lg_error_clearall(void);
%rename(_prt_error) prt_error;
/* For security, the first argument should always contain a single "%s"
 * (e.g. "%s\n"), and the second one should always be a C string. */
int prt_error(const char *, const char *);
bool lg_error_flush(void);
/*
 * void *lg_error_set_handler_data(void *);
 * A wrapper to this function is complex and is not implemented here.  However,
 * such a wrapper may not be needed anyway since this function is provided
 * mainly for the low-level implementation the error callback, so bound
 * languages can free the memory of the callback data.
 */


#ifdef SWIGPYTHON
%extend lg_errinfo
{
    %pythoncode
    %{
        def formatmsg(self):
            return _lg_error_formatmsg(self)
        __swig_destroy__ = _clinkgrammar.delete_lg_errinfo
        __del__ = lambda self: None
    %}
}

%{
static lg_error_handler default_error_handler;

static lg_errinfo *dup_lg_errinfo(lg_errinfo *lge)
{
   lg_errinfo *mlge = (lg_errinfo *)malloc(sizeof(lg_errinfo));
   mlge->severity_label = strdup(lge->severity_label);
   mlge->text = strdup(lge->text);
   mlge->severity = lge->severity;

   return mlge;
}

/**
 * This function is installed as the C error callback when an error callback
 * is set by the Python code to a Python function (but not when set to None
 * or to the library default error handler).
 * When invoked by the LG library, it calls the Python function along with
 * its data. Both appear in func_and_data, which is a Python tuple of 2
 * elements - a function and an arbitrary data object.
*/
static void PythonCallBack(lg_errinfo *lge, void *func_and_data)
{
   lg_errinfo *mlgep = dup_lg_errinfo(lge);
   PyObject *pylge = SWIG_NewPointerObj(SWIG_as_voidptr(mlgep),
                                       SWIGTYPE_p_lg_errinfo, SWIG_POINTER_OWN);
   PyObject *func = PyTuple_GetItem((PyObject *)func_and_data, 0);
   PyObject *data = PyTuple_GetItem((PyObject *)func_and_data, 1);

   PyObject *args = Py_BuildValue("OO", pylge, data);
   PyObject *rc = PyEval_CallObject(func, args); /* Py LG error cb. */

   Py_DECREF(pylge);
   Py_DECREF(args);
   if (NULL == rc)
       PyErr_Print();
   Py_XDECREF(rc);
}
%}

/* The second argument of the default callback can be NULL or
   a severity_level integer. Validate that and convert it to C int. */
%typemap(in) int *pedh_data
{
   int arg;
   bool error = false;
   const char errmsg[] = "The default error handler data argument (arg 2) "
                         "must be an integer (0 to lg_None) or None.";

   if (Py_None == $input)
   {
      $1 = NULL;
   }
   else
   {
      if (!PyInt_Check($input))
      {
         SWIG_exception_fail(SWIG_TypeError, errmsg);
         error = true;
      }
      else
      {
          arg = (int)PyInt_AsLong($input);
      }

      if ((arg < 0) || (arg > lg_None))
      {
         SWIG_exception_fail(SWIG_ValueError, errmsg);
         error = true;
      }

      if (error) return NULL;
      $1 = &arg;
   }
}

%inline %{
void _py_error_default_handler(lg_errinfo *lge, int *pedh_data)
{
    default_error_handler(lge, (void *)pedh_data);
}

/**
 * Set a Python function/data as the LG error handler callback.
 * Note that because the LG library cannot directly call a Python function,
 * the actual callback function is a C proxy function PythonCallBack() and
 * the Python function/data is set as the C callback data.
 */
PyObject *_py_error_set_handler(PyObject *func_and_data)
{
   const void *old_func_and_data = lg_error_set_handler_data(NULL);
   PyObject *func = PyTuple_GetItem((PyObject *)func_and_data, 0);
   lg_error_handler old_handler;

   if (Py_None == func)
   {
      old_handler = lg_error_set_handler(NULL, NULL);
   }
   else
   {
      if (!PyCallable_Check(func)) {
          PyErr_SetString(PyExc_TypeError, "Argument 1 must be callable");
          return NULL;
      }
      old_handler = lg_error_set_handler(PythonCallBack, func_and_data);
      Py_INCREF(func_and_data);
   }

   if (NULL == (PyObject *)old_handler)
      Py_RETURN_NONE;

   if (PythonCallBack == old_handler)
   {
      func = PyTuple_GetItem((PyObject *)old_func_and_data, 0);
      Py_INCREF(func);
      Py_XDECREF(old_func_and_data);
      return func;
   }

   /* This must be the first call. Grab the C default error handler. */
   default_error_handler = old_handler;

   /* Signify this is the default error handler by a string object. */
   return Py_BuildValue("s", "");
}

PyObject *_py_error_printall(PyObject *func_and_data)
{
   Py_INCREF(func_and_data);
   int n = lg_error_printall(PythonCallBack, func_and_data);
   Py_DECREF(func_and_data);

   PyObject *py_n = PyInt_FromLong(n);
   return py_n;
}

void delete_lg_errinfo(lg_errinfo *lge) {
  free((void *)lge->severity_label);
  free((void *)lge->text);
  free((void *)lge);
}
%}
#endif /* SWIGPYTHON */
