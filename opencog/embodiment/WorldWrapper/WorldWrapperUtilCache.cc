/*
 * opencog/embodiment/WorldWrapper/WorldWrapperUtilCache.cc
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

#include "WorldWrapperUtilCache.h"

using namespace opencog::world;

Predicate::Predicate() {
    this->name = std::string("");
}

Predicate::Predicate(const std::string& name,
                     const std::vector<std::string>& arguments) {
    this->name = name;
    this->arguments = arguments;
}

bool Predicate::operator==(const Predicate& p) const {
    return this->name == p.name && this->arguments == p.arguments;
}

WorldWrapperUtilCache::WorldWrapperUtilCache()
{
    this->notFoundMiss = 0;
    this->emptyCacheMiss = 0;
    this->timestampChangeMiss = 0;

    this->finds = 0;
    this->timestamp = 0;
}

bool WorldWrapperUtilCache::add(unsigned long timestamp, const Predicate& pred, float value)
{
    if (this->timestamp != timestamp) {
        clearCache(timestamp);
    }

    std::pair<cacheMapIt, bool> result;
    result = cache.insert(cacheMap::value_type(pred, value));

    return result.second;
}

float WorldWrapperUtilCache::find(unsigned long timestamp, const Predicate& pred)
{
    this->finds++;

    if (cache.empty()) {
        this->emptyCacheMiss++;
        return (CACHE_MISS);
    }

    if (this->timestamp != timestamp) {
        this->timestampChangeMiss++;
        clearCache(timestamp);
        return (CACHE_MISS);
    }

    cacheMapIt result = cache.find(pred);
    if (result == cache.end()) {
        this->notFoundMiss++;
        return (CACHE_MISS);
    }

    return result->second;
}

void WorldWrapperUtilCache::clearCache(unsigned long timestamp)
{
//    double missRate = (double)(notFoundMiss + emptyCacheMiss + timestampChangeMiss) / (double)finds;
//    std::cout << "Cache - timestamp '" << timestamp << std::endl;
//    std::cout << "' finds '" << finds << "' emptyCacheMiss '" << emptyCacheMiss <<
//                 "' timestampChangeMiss '" << timestampChangeMiss << "' notFound miss '" << notFoundMiss << std::endl;
//    std::cout << "' hit rate '" << ((1 - missRate) * 100)
//              << "%' miss hate '" << (missRate * 100) << "%'." << std::endl;

    this->cache.clear();
    this->timestamp = timestamp;

    this->finds = 0;
    this->notFoundMiss = 0;
    this->emptyCacheMiss = 0;
    this->timestampChangeMiss = 0;
}

