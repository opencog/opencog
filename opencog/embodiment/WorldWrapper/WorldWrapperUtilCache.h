/*
 * opencog/embodiment/WorldWrapper/WorldWrapperUtilCache.h
 *
 * Copyright (C) 2002-2009 Novamente LLC
 * All Rights Reserved
 * Author(s): Carlos Lopes
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

#ifndef _WORLD_WRAPPER_UTIL_CACHE_H
#define _WORLD_WRAPPER_UTIL_CACHE_H

#include <string>
#include <vector>
#include <iostream>
#include <boost/unordered_map.hpp>
#include <boost/functional/hash.hpp>

#define CACHE_MISS -9999.0f
#define STR_SIZE_FOR_HASH 3

namespace opencog { namespace world {

/**
 * Predicate data that should be cached. These predicates are the key in
 * cache hash map.
 */
struct Predicate {

    std::string name;
    std::vector<std::string> arguments;

    /**
     * Constructors
     */
    Predicate();
    Predicate(const std::string& name, const std::vector<std::string>& arguments);

    /**
     * Equality operator. Predicate's name and arguments should match. The order
     * that the arguments are inserted matter so be carefull.
     */
    bool operator==(const Predicate& p) const;

    /**
     * Hash function for predicates which consider not only the predicate name
     * but also its arguments.
     */
    friend std::size_t hash_value(const Predicate& p) {

        std::size_t seed = 0;

        // hash is created using the last STR_SIZE_FOR_HASH characters from the
        // predicate name ...
        if (p.name.size() <= STR_SIZE_FOR_HASH) {
            boost::hash_combine(seed, p.name);
        } else {
            boost::hash_range(seed, p.name.end() - STR_SIZE_FOR_HASH, p.name.end());
        }

        for (const std::string& arg : p.arguments) {
            // ... and arguments. This is necessary because most of then
            // have a common prefix (is_, id_, etc).
            if (arg.size() <= STR_SIZE_FOR_HASH) {
                boost::hash_combine(seed, arg);
            } else {
                boost::hash_range(seed, arg.end() - STR_SIZE_FOR_HASH, arg.end());
            }
        }
        //printf("Predicate %s with hash %d\n", p.name.c_str(), seed);
        return seed;
    }

}; // struct Predicate

/**
 * Implements a timestamped cache for WWUtil perceptions reducing
 * AtomSpace queries needed. By timestamped cache it is meant that
 * every query or insertion needs a timestamp, and if the given
 * timestamp differs from the current one, all cache is erased.
 */
class WorldWrapperUtilCache
{

public:

    // typedefs
    typedef boost::unordered_map < Predicate, float,
                                   boost::hash<Predicate> > cacheMap;
    typedef cacheMap::iterator cacheMapIt;

    /**
     * Constructors
     */
    WorldWrapperUtilCache();

    /**
     * Call f (which is assumed to be called on predicate pred) and
     * cache pred if not already cached.
     */
    template<typename Fun>
    float call(const Fun& f, unsigned long timestamp, const Predicate& pred) {
        float value = find(timestamp, pred);
        if (value == CACHE_MISS) {
            value = f();
            add(timestamp, pred, value);
        }
        return value;
    }

    /**
     * Add information to cache.
     *
     * @param timestamp The information timestamp.
     * @param predicate The predicate information (name and arguments)
     * @param value The predicate cached value
     * @return true if the information was cached and false otherwise.
     */
    bool add(unsigned long timestamp, const Predicate& pred, float value);

    /**
     * Lookup for a cached value for the given predicate.
     *
     * @param timestamp The timestamp where the information is valid.
     * @param predicate The predicate whose cached value is to be
     * searched.
     * @return The cached value or -1.0 if cache miss.
     *
     * Note that this method is not const as it can clear the cache if
     * the timestamp is different.
     */
    float find(unsigned long timestamp, const Predicate& pred);

    /**
     * Get cache's current timestamp. It holds information only for the
     * current timestamp. If timestamp changes, then all cached data is
     * removed.
     */
    inline unsigned long getTimestamp() const {
        return timestamp;
    }

    inline unsigned int size() {
        return cache.size();
    }

private:

    // the timestamp for the current cached information.
    unsigned long timestamp;

    // the hash map that holds cached information
    cacheMap cache;

    // profiling variables. total find and cache miss
    unsigned finds;
    unsigned notFoundMiss;
    unsigned emptyCacheMiss;
    unsigned timestampChangeMiss;


    // Clear the cache. During cache clear, the new valid timestamp is
    // set. Cache clear will be called every time one of the two happens:
    //
    // 1. search for data with newer timestamp than WWUtil current one.
    // 2. insert data with newer timestamp than WWCacheUtil current one.
    //
    // @param timestamp The new timestamp valid
    void clearCache(unsigned long timestamp);

}; // class WorldWrapperUtilCache

} } // namespace opencog::world

#endif
