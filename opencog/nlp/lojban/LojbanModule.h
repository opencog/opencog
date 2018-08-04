/*
 * LojbanModule.h
 *
 * Copyright (C) 2014 OpenCog Foundation
 *
 * Author: Roman Treutlein
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU Affero General Public License v3 as
 * published by the Free Software Foundation and including the exceptions
 * at http://opencog.org/wiki/Licenses
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU Affero General Public License
 * along with this program; if not, write to:
 * Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
 */

#ifndef _OPENCOG_LOJBAN_MODULE_H
#define _OPENCOG_LOJBAN_MODULE_H

#include <opencog/cogserver/server/Module.h>
#include <opencog/cogserver/server/Factory.h>
#include <opencog/cogserver/server/CogServer.h>

#include <opencog/atomspace/AtomSpace.h>
#include <opencog/atoms/base/Handle.h>

#include <opencog/guile/SchemeEval.h>

typedef void *HsStablePtr;  /* C representation of a Haskell StablePtr */

extern "C"
{
    opencog::Handle *lojban_parse(opencog::AtomSpace *, HsStablePtr,const char *);

    char* lojban_print(opencog::AtomSpace *, HsStablePtr, opencog::Handle *);

    HsStablePtr lojban_init();

    void lojban_exit(HsStablePtr);
}

namespace opencog
{
namespace nlp
{

/**
 * An OpenCog module for supporting Surface Realization.
 *
 * A module linked to the Surface Realization pattern matching code
 * and scheme bindings.
 */
class LojbanModule : public Module
{
private:

    HsStablePtr _wordlist;

    CogServer & _cs;
    AtomSpace * _as;

    SchemeEval scmeval;

public:

    DECLARE_CMD_REQUEST(LojbanModule, "load-lojban", do_load_lojban,
                        "Load Lojban Data for learning.\n",
                        "Usage: load-lojban path\n", false, true)

    DECLARE_CMD_REQUEST(LojbanModule, "parse-lojban", do_parse_lojban,
                        "Parse a Lojban Sentence to Atomese.\n",
                        "Usage: parse-lojban sentence\n", false, true)

    LojbanModule(CogServer&);
    virtual ~LojbanModule();

    const char * id(void);
    virtual void init(void);

};

}
}

#endif // _OPENCOG_SUREAL_MODULE_H
