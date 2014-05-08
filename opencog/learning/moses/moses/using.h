/*
 * opencog/learning/moses/moses/using.h
 *
 * Copyright (C) 2002-2008 Novamente LLC
 * All Rights Reserved
 *
 * Written by Moshe Looks
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
#ifndef _MOSES_USING_H
#define _MOSES_USING_H

#include <functional>
#include <iostream>
#include <set>
#include <vector>

#include <boost/bind.hpp>

// uncomment this line for debug information to be given during execution
// #define DEBUG_INFO 

namespace opencog { namespace moses {

    using std::min;
    using std::max;
    using std::accumulate;
    using std::set;
    using std::string;
    using std::unary_function;
    using std::make_pair;
    using std::vector;
    using std::advance;
    using std::cin;
    using std::cout;
    using std::endl;
    using std::istream;
    using std::stringstream;

    using boost::bind;

} // ~namespace moses
} // ~namespace opencog

#endif
