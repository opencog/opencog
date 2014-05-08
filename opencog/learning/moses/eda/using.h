/*
 * opencog/learning/moses/eda/using.h
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
#ifndef _EDA_USING_H
#define _EDA_USING_H

#include <boost/bind.hpp>
#include <boost/iterator/counting_iterator.hpp>
#include <boost/iterator/transform_iterator.hpp>
#include <boost/utility/result_of.hpp>
#include <boost/next_prior.hpp>
#include <boost/type_traits.hpp>

#include <vector>
#include <algorithm>
#include <utility>

#include <opencog/util/functional.h>
#include <opencog/util/numeric.h>

/// anything that gets imported into the eda namespace with a using
/// directive should go here
namespace opencog { 
namespace moses {
using boost::bind;
using boost::ref;
using boost::make_counting_iterator;
using boost::make_transform_iterator;
using boost::result_of;
using boost::next;
using boost::prior;

using std::unary_function;
using std::binary_function;

using std::vector;

using std::distance;
using std::copy;
using std::transform;
using std::nth_element;
using std::accumulate;
using std::adjacent_find;

using std::pair;
using std::make_pair;

} // ~namespace moses
} // ~namespace opencog

#endif
