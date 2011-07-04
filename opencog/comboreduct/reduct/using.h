/*
 * opencog/comboreduct/reduct/using.h
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
#ifndef _REDUCT_USING_H
#define _REDUCT_USING_H

#include <boost/variant.hpp>
//#include <boost/bind.hpp>
#include <boost/iterator/counting_iterator.hpp>
#include <boost/iterator/indirect_iterator.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/ptr_container/ptr_vector.hpp>

#include <opencog/comboreduct/combo/vertex.h>

#include <functional>
#include <algorithm>

/// anything that gets imported into the reduct namespace with a using
/// directive should go here
namespace opencog { namespace reduct {
using namespace opencog::combo;
using boost::variant;
using boost::static_visitor;
//  using boost::bind;
using boost::make_counting_iterator;
using boost::make_indirect_iterator;
using boost::apply_visitor;
using boost::shared_ptr;
using boost::ptr_vector;
using std::find_if;
using std::distance;
using std::make_pair;  

} // ~namespace reduct
} // ~namespace opencog

#endif
