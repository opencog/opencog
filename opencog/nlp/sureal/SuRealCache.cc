/*
 * SuRealCache.cc
 *
 * Copyright (C) 2016 OpenCog Foundation
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

#include "SuRealCache.h"

#include <opencog/atoms/proto/NameServer.h>
#include <opencog/atoms/base/Atom.h>

using namespace opencog::nlp;
using namespace opencog;
using namespace std;


SuRealCache::SuRealCache()
{
}

SuRealCache::~SuRealCache()
{
}

SuRealCache *SuRealCache::instance_ = NULL;

SuRealCache& SuRealCache::instance() 
{
    if (instance_ == NULL) {
        instance_ = new SuRealCache();
    }
    return *instance_;
}

void SuRealCache::add_match(HandleCacheMap &map, const Handle &h1, const Handle &h2, bool value) 
{

    SuRealCache::HandleCacheMap::iterator it1 = map.find(h1);
    if (it1 == map.end()) {
        HandleBoolMap new_map;
        new_map.insert(HandleBoolMap::value_type(h2, value));
        map.insert(HandleCacheMap::value_type(h1, new_map));
    } else {
        (*it1).second.insert(HandleBoolMap::value_type(h2, value));
    }
}

int SuRealCache::match(HandleCacheMap &map, const Handle &h1, const Handle &h2) 
{

    int answer = -1;

    SuRealCache::HandleCacheMap::iterator it1 = map.find(h1);
    if (it1 != map.end()) {
        HandleBoolMap::iterator it2 = (*it1).second.find(h2);
        if (it2 != (*it1).second.end()) {
            answer = ((*it2).second ? 1 : 0);
        }
    }

    return answer;
}

void SuRealCache::add_grounding_match(const HandleMap &m1, bool value) 
{
    std::string key1 = build_map_hash_key(m1);
    SuRealCache::StringMap::iterator it1 = m_partial_grounding_cache.find(key1);
    if (it1 == m_partial_grounding_cache.end()) {
        m_partial_grounding_cache.insert(StringMap::value_type(key1, value));
    }
}

void SuRealCache::add_grounding_match(const HandleMap &m1, const HandleMap &m2, bool value) 
{

    std::string key1 = build_map_hash_key(m1);
    std::string key2 = build_map_hash_key(m2);
    SuRealCache::StringCacheMap::iterator it1 = m_grounding_cache.find(key1);
    if (it1 == m_grounding_cache.end()) {
        StringMap new_map;
        new_map.insert(StringMap::value_type(key2, value));
        m_grounding_cache.insert(StringCacheMap::value_type(key1, new_map));
    } else {
        (*it1).second.insert(StringMap::value_type(key2, value));
    }
}

int SuRealCache::grounding_match(const HandleMap &m1, const HandleMap &m2) 
{

    int answer = -1;

    std::string key1 = build_map_hash_key(m1);
    SuRealCache::StringMap::iterator it1 = m_partial_grounding_cache.find(key1);
    if (it1 != m_partial_grounding_cache.end()) {
        answer = ((*it1).second ? 1 : 0);
    } else {
        std::string key2 = build_map_hash_key(m2);
        SuRealCache::StringMap::iterator it2 = m_partial_grounding_cache.find(key2);
        if (it2 != m_partial_grounding_cache.end()) {
            answer = ((*it2).second ? 1 : 0);
        } else {
            SuRealCache::StringCacheMap::iterator it3 = m_grounding_cache.find(key1);
            if (it3 != m_grounding_cache.end()) {
                StringMap::iterator it4 = (*it3).second.find(key2);
                if (it4 != (*it3).second.end()) {
                    answer = ((*it4).second ? 1 : 0);
                }
            }
        }
    }

    return answer;
}

void SuRealCache::add_variable_match(const Handle &h1, const Handle &h2, bool value) 
{
    return add_match(m_variable_cache, h1, h2, value);
}

int SuRealCache::variable_match(const Handle &h1, const Handle &h2) 
{
    return match(m_variable_cache, h1, h2);
}

void SuRealCache::add_clause_match(const Handle &h1, const Handle &h2, bool value) 
{
    return add_match(m_clause_cache, h1, h2, value);
}

int SuRealCache::clause_match(const Handle &h1, const Handle &h2) 
{
    return match(m_clause_cache, h1, h2);
}

bool SuRealCache::get_node_list(const Handle &h, HandleSeq &list)
{
    bool answer = false;

    SuRealCache::HandleSeqCache::iterator it = m_node_list_cache.find(h);
    if (it != m_node_list_cache.end()) {
        list = (*it).second;
        answer = true;
    }

    return answer;
}

void SuRealCache::add_node_list(const Handle &h, const HandleSeq &list)
{
    SuRealCache::HandleSeqCache::iterator it = m_node_list_cache.find(h);
    if (it == m_node_list_cache.end()) {
        m_node_list_cache.insert(HandleSeqCache::value_type(h, list));
    }
}

void SuRealCache::reset() 
{

    for (auto it = m_variable_cache.begin(); it != m_variable_cache.end(); it++) {
        (*it).second.clear();
    }

    for (auto it = m_clause_cache.begin(); it != m_clause_cache.end(); it++) {
        (*it).second.clear();
    }

    for (auto it = m_grounding_cache.begin(); it != m_grounding_cache.end(); it++) {
        (*it).second.clear();
    }

    m_variable_cache.clear();
    m_clause_cache.clear();
    m_grounding_cache.clear();
    m_partial_grounding_cache.clear();
    m_node_list_cache.clear();
}

std::string SuRealCache::handle_to_hash_key(const Handle &h) 
{

    std::string answer = "";
    Type type = h->get_type();
    if (nameserver().isNode(type)) {
        answer += nameserver().getTypeName(type);
        answer += ":";
        answer += h->get_name();
    } else {
        answer += nameserver().getTypeName(type);
        answer += "(";
        for (const Handle& target_handle: h->getOutgoingSet()) {
            answer += handle_to_hash_key(target_handle);
            answer += ",";
        }
        answer += ")";
    }

    return answer;
}

std::string SuRealCache::build_map_hash_key(const HandleMap &m) 
{

    std::string answer = "";
    for (auto it = m.begin(); it != m.end(); it++) {
        answer += handle_to_hash_key(it->first);
        answer += "-";
        answer += handle_to_hash_key(it->second);
        answer += "#";
    }

    return answer;
}
