/*
 * LGDictExpContainer.h
 *
 * Copyright (C) 2015 OpenCog Foundation
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

#ifndef _OPENCOG_LG_DICT_EXP_H
#define _OPENCOG_LG_DICT_EXP_H

#include <link-grammar/dict-api.h>

#include <opencog/atomspace/AtomSpace.h>


namespace opencog
{

/**
 * Link Grammar expression container.
 *
 * A helper class for doing operations on LG expression.
 */
class LGDictExpContainer
{
public:
    LGDictExpContainer(Exp_type, const Exp* exp);
    LGDictExpContainer(Exp_type, const std::vector<LGDictExpContainer>&);

    HandleSeq to_handle(const Handle& h);

private:
    void basic_flatten();
    void basic_dnf();
    void basic_normal_order();

    Exp_type m_type;

    std::string m_string;
    char m_direction;
    bool m_multi;

    std::vector<LGDictExpContainer> m_subexps;
};

}

#endif // _OPENCOG_LG_DICT_EXP_H
