/*
 * SuRealModule.h
 *
 * Copyright (C) 2014 OpenCog Foundation
 *
 * Author: William Ma <https://github.com/williampma>
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

#ifndef _OPENCOG_SUREAL_MODULE_H
#define _OPENCOG_SUREAL_MODULE_H


#include <opencog/server/Module.h>

#include "SuRealSCM.h"


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
class SuRealModule : public Module
{
public:
    SuRealModule(CogServer&);
    virtual ~SuRealModule();

    const char * id(void);
    virtual void init(void);

private:
    SuRealSCM* m_scm;
};

}
}

#endif // _OPENCOG_SUREAL_MODULE_H
