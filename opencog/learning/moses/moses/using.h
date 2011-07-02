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

#include <iostream>
#include <set>
#include <functional>
#include <vector>

#include <boost/logic/tribool.hpp>
#include <boost/bind.hpp>
#include <boost/iterator/counting_iterator.hpp>
#include <boost/iterator/transform_iterator.hpp>
#include <boost/iterator/permutation_iterator.hpp>
#include <boost/iterator/indirect_iterator.hpp>

#include <opencog/comboreduct/combo/vertex.h>
#include <opencog/util/Logger.h>

// uncomment this line for debug information to be given during execution
// #define DEBUG_INFO 

namespace opencog { namespace moses {

    using namespace opencog::combo;

    using std::min;
    using std::max;
    using std::accumulate;
    using std::set;
    using std::unary_function;
    using std::make_pair;
    using std::vector;
    using std::advance;
    using std::cout;
    using std::endl;

    using boost::logic::tribool;
    using boost::logic::indeterminate;
    using boost::bind;
    using boost::make_counting_iterator;
    using boost::make_transform_iterator;
    using boost::make_permutation_iterator;
    using boost::make_indirect_iterator;

    using opencog::logger;

} // ~namespace moses
} // ~namespace opencog

#endif
