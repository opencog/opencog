/*
 * opencog/learning/dimensionalembedding/CoverTreePoint.h
 *
 * Copyright (C) 2011 by OpenCog Foundation
 * All Rights Reserved
 *
 * Written by David Crane <dncrane@gmail.com>
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

#ifndef _OPENCOG_COVER_TREE_POINT_H
#define _OPENCOG_COVER_TREE_POINT_H

#include <string>
#include <vector>
#include <opencog/util/numeric.h>

#include <opencog/atomspace/AtomSpace.h>
#include <opencog/learning/dimensionalembedding/DimEmbedModule.h>

/**
 * The CoverTreePoint class and its methods are required by the cover tree
 * implementation
 */
namespace opencog {

class CoverTreePoint {
 private:
    Handle _handle;
    std::vector<double> _vector;

 public:    
 CoverTreePoint(const Handle& handle, const std::vector<double>& vector) : _handle(handle), _vector(vector) {}
    
    double distance(const CoverTreePoint& p) const;
    void print(AtomSpace& atomspace);
    
    const std::vector<double>& getVector() const {
        return _vector;
    }
    const Handle& getHandle() const {
        return _handle;
    }

    bool operator==(const CoverTreePoint& p) const {
        return (_handle==p.getHandle() && this->distance(p)==0.0);
    } 
};

 void CoverTreePoint::print(AtomSpace& atomspace) {
     std::ostringstream oss;
     if(!atomspace.is_valid_handle(_handle)) {
         oss << "[NODE'S BEEN DELETED]" << " : (";
     } else {
         oss << _handle->toShortString() << " : (";
     }
     for(std::vector<double>::const_iterator it=_vector.begin();
         it!=_vector.end(); ++it)
         {
             oss << *it << " ";
         }       
     oss << ")" << std::endl;
     printf("%s", oss.str().c_str());
}
 
 double CoverTreePoint::distance(const CoverTreePoint& p) const {
     const std::vector<double>& v=p.getVector();
     OC_ASSERT(_vector.size()==v.size());
     std::vector<double>::const_iterator it1=_vector.begin();
     std::vector<double>::const_iterator it2=v.begin();
     
     double distance=0;
     //Calculate euclidean distance between v1 and v2
     for(; it1!=_vector.end(); it1++, it2++) {
         distance += sq(*it1 - *it2);
     }
     
     distance=sqrt(distance);
     return distance;
 }
 
}//namespace

#endif //_OPENCOG_COVER_TREE_POINT_H
