
#include "link-includes.h"

// Already declared in link-includes.h
// const char * linkgrammar_get_dict_locale(Dictionary dict);
// const char * linkgrammar_get_version(void);
// const char * linkgrammar_get_dict_version(Dictionary dict);

void dictionary_setup_locale(Dictionary dict);
void dictionary_setup_defines(Dictionary dict);
void afclass_init(Dictionary dict);
bool afdict_init(Dictionary dict);
void affix_list_add(Dictionary afdict, Afdict_class *, const char *);
