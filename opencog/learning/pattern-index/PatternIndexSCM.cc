#include "PatternIndexSCM.h"
#include "PatternIndexAPI.h"
#include <opencog/guile/SchemePrimitive.h>
#include <opencog/util/Config.h>

using namespace opencog;

PatternIndexSCM::PatternIndexSCM() 
{
    static bool opencog_learning_patternindex_init_done = false;
    if (opencog_learning_patternindex_init_done) return;
    opencog_learning_patternindex_init_done = true;
    scm_with_guile(init_in_guile, this);
    config().load("lib/opencog.conf");
}

PatternIndexSCM::~PatternIndexSCM() 
{
}

void PatternIndexSCM::init_in_module(void* data)
{
    PatternIndexSCM *self = (PatternIndexSCM *) data;
    self->init();
}

void* PatternIndexSCM::init_in_guile(void* self)
{
    scm_c_define_module("opencog learning pattern-index", init_in_module, self);
    scm_c_use_module("opencog learning pattern-index");
    return NULL;
}

void PatternIndexSCM::init()
{
#ifdef HAVE_GUILE
    // Bindings for Guile's API
    define_scheme_primitive("scm-api-create-index",
                            &PatternIndexSCM::create_index,
                            this,
                            "pattern-index");
    define_scheme_primitive("scm-api-query",
                            &PatternIndexSCM::query,
                            this,
                            "pattern-index");
    define_scheme_primitive("scm-api-mine-patterns",
                            &PatternIndexSCM::minePatterns,
                            this,
                            "pattern-index");
#endif
}

Handle PatternIndexSCM::create_index(Handle scmPath)
{
    return PatternIndexAPI::getInstance().createIndex(scmPath->get_name());
}

Handle PatternIndexSCM::query(Handle indexKey, Handle queryLink)
{
    return PatternIndexAPI::getInstance().query(indexKey, queryLink);
}

Handle PatternIndexSCM::minePatterns(Handle indexKey)
{
    return PatternIndexAPI::getInstance().minePatterns(indexKey);
}

void opencog_patternindex_init(void)
{
    static PatternIndexSCM patternIndexSCM;
}
