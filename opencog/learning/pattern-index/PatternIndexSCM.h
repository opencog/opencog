/*
 * PatternIndexSCM.h
 *
 * Copyright (C) 2017 OpenCog Foundation
 *
 * Author: Andre Senna <https://github.com/andre-senna>
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

#ifndef _OPENCOG_PATTERNINDEXSCM_H
#define _OPENCOG_PATTERNINDEXSCM_H

#include <opencog/atoms/base/Handle.h>

#include <string>

namespace opencog
{

/**
 * This class is a wrapper around PatternIndexAPI and contains all the black
 * magic required to integrate PatternIndexAPI with Guile.
 */
class PatternIndexSCM 
{

public:

    PatternIndexSCM();
    ~PatternIndexSCM();

private:

    // Scheme API - implementation of some public-defined scheme API
    Handle create_index(Handle scmPath);
    Handle query(Handle indexKey, Handle queryLink);
    Handle minePatterns(Handle indexKey);

    // Guile integration magic
    void init();
    static void init_in_module(void* data);
    static void* init_in_guile(void* self);
    void opencog_patternindex_init(void);

};

}

// Guile integration magic
extern "C" {
    void opencog_patternindex_init(void);
};

#endif // _OPENCOG_PATTERNINDEXSCM_H
