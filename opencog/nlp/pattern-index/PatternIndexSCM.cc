#include "PatternIndexSCM.h"
#include "PatternIndexAPI.h"
#include <opencog/guile/SchemePrimitive.h>

using namespace opencog;

PatternIndexSCM::PatternIndexSCM() 
{
    static bool opencog_nlp_patternindex_init_done = false;
    if (opencog_nlp_patternindex_init_done) return;
    opencog_nlp_patternindex_init_done = true;
    scm_with_guile(init_in_guile, this);
}

PatternIndexSCM::~PatternIndexSCM() 
{
}

Handle PatternIndexSCM::create_new_index(Handle scmPath)
{
    return PatternIndexAPI::getInstance().createNewIndex_h(scmPath->getName());
}

Handle PatternIndexSCM::query(Handle indexKey, Handle queryLink)
{
    return PatternIndexAPI::getInstance().query(indexKey, queryLink);
}

Handle PatternIndexSCM::minePatterns(Handle indexKey)
{
    return PatternIndexAPI::getInstance().minePatterns(indexKey);
}

void PatternIndexSCM::init()
{
#ifdef HAVE_GUILE
    define_scheme_primitive("scm-api-create-new-index", &PatternIndexSCM::create_new_index, this, "pattern-index");
    define_scheme_primitive("scm-api-query", &PatternIndexSCM::query, this, "pattern-index");
    define_scheme_primitive("scm-api-mine-patterns", &PatternIndexSCM::minePatterns, this, "pattern-index");
#endif
}

void PatternIndexSCM::init_in_module(void* data)
{
    PatternIndexSCM *self = (PatternIndexSCM *) data;
    self->init();
}

void* PatternIndexSCM::init_in_guile(void* self)
{
    // XXX TODO CHange module name
    scm_c_define_module("opencog nlp pattern-index", init_in_module, self);
    scm_c_use_module("opencog nlp pattern-index");
    return NULL;
}

void opencog_patternindex_init(void)
{
    static PatternIndexSCM patternIndexSCM;
}
