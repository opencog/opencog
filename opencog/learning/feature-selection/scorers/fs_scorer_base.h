/** fs_scorer_base.h --- 
 *
 * Copyright (C) 2013 OpenCog Foundation
 *
 * Author: Nil Geisweiller
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


#ifndef _OPENCOG_BASE_SCORER_H
#define _OPENCOG_BASE_SCORER_H

#include <opencog/comboreduct/table/table.h>

namespace opencog {

using namespace combo;

template<typename FeatureSet>
struct fs_scorer_base : public std::unary_function<FeatureSet, double>
{
    // ctor
    fs_scorer_base(const CTable& ctable, double confi)
        : _ctable(ctable), _confi(confi), _usize(_ctable.uncompressed_size()) {}

    // dtor
    virtual ~fs_scorer_base() {};

    virtual double operator()(const FeatureSet& features) const = 0;

protected:

    // Very rough approximation of the confidence in the feature
    // quality measure
    double confidence(unsigned fs_size) const {
        return _usize / (_usize + exp(-_confi*fs_size));
    }
    
    const CTable& _ctable;
    double _confi;              // confidence intensity
    unsigned _usize;             // uncompressed ctable size
};

}

#endif // _OPENCOG_BASE_SCORER_H
