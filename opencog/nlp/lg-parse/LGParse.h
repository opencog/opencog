/*
 * LGParse.h
 *
 * Copyright (C) 2017 Linas Vepstas
 *
 * Author: Linas Vepstas <linasvepstas@gmail.com>
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

#ifndef _OPENCOG_LG_PARSE_H
#define _OPENCOG_LG_PARSE_H

#include <link-grammar/dict-api.h>
#include <opencog/atomspace/AtomSpace.h>

namespace opencog
{
namespace nlp
{

/**
 * Link Grammar parser.
 *
 * An atomspace wrapper to the LG parser.
 */
class LGParser
{
public:
    LGParser(Dictionary, AtomSpace*);
    ~LGParser();

    void parse(const std::string& sentence);
private:

    Dictionary _dictionary;
    AtomSpace* _as;
};

}}
#endif // _OPENCOG_LG_PARSE_H
