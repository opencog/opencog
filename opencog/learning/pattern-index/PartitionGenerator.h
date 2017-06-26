/*
 * PartitionGenerator.h
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

#ifndef _OPENCOG_PARTITIONGENERATOR_H
#define _OPENCOG_PARTITIONGENERATOR_H

#include <string>
#include <set>
#include <stdexcept>

namespace opencog
{

/**
 * Implements an iterator to all possible partitions of a set with a given cardinality.
 *
 * For example:
 *
 *
 *   PartitionGenerator part(4);
 *
 *   while (! part.depleted()) {
 *       part.printForDebug("", "\n");
 *       part.generateNext();
 *   }
 *
 * whould give us:
 *
 *   {{0} {1 2 3}}
 *   {{1} {0 2 3}}
 *   {{2} {0 1 3}}
 *   {{3} {0 1 2}}
 *   {{0} {1} {2 3}}
 *   {{0} {2} {1 3}}
 *   {{0} {3} {1 2}}
 *   {{1} {2} {0 3}}
 *   {{1} {3} {0 2}}
 *   {{2} {3} {0 1}}
 *   {{0} {1} {2} {3}}
 *
 * NOTE: trivial partition {{}, {0 1 2 3}} is EXCLUDED by default.
 * 
 * getPartition() may be used to get the partition at a given iteration.
 *
 */
class PartitionGenerator 
{

public:

    typedef std::set<unsigned int> IntegerSet;
    struct LessThanSet {
        bool operator()(const IntegerSet &a, const IntegerSet &b) const {
            if (a.size() < b.size()) return true;
            if (b.size() < a.size()) return false;
            IntegerSet::const_iterator it1 = a.begin();
            IntegerSet::const_iterator it2 = b.begin();
            while (it1 != a.end()) {
                if ((*it1) < (*it2)) return true;
                if ((*it2) < (*it1)) return false;
                it1++;
                it2++;
            }
            return false;
        }
    };
    typedef std::set<IntegerSet, LessThanSet> IntegerSetSet;
    struct LessThanSetSet {
        bool operator()(const IntegerSetSet &a, const IntegerSetSet &b) const {
            if (a.size() < b.size()) return true;
            if (b.size() < a.size()) return false;
            IntegerSetSet::const_iterator it1 = a.begin();
            IntegerSetSet::const_iterator it2 = b.begin();
            while (it1 != a.end()) {
                PartitionGenerator::LessThanSet comp;
                if (comp((*it1), (*it2))) return true;
                if (comp((*it2), (*it1))) return false;
                it1++;
                it2++;
            }
            return false;
        }
    };
    typedef std::set<IntegerSetSet, LessThanSetSet> IntegerSetSetSet;

    /*
     * includeTrivial = true will include the trivial partition of S: {{}, S}
     */
    PartitionGenerator(unsigned int n, bool includeTrivial = false);
    ~PartitionGenerator();
    bool depleted() const;
    void generateNext();
    IntegerSetSet getPartition() const;
    void printForDebug(std::string prefix, std::string suffix) const;

private:

    IntegerSetSetSet partitions;
    IntegerSetSetSet::const_iterator partitionIterator;

    void computePartitions(IntegerSetSetSet &answer, unsigned int n);
};

}

#endif // _OPENCOG_PARTITIONGENERATOR_H







