/*
 * SuRealCache.h
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

#ifndef _OPENCOG_SUREAL_CACHE_H
#define _OPENCOG_SUREAL_CACHE_H

#include <unordered_map>
#include <opencog/atoms/base/Handle.h>

namespace opencog
{
namespace nlp
{

/**
 * A Cache between SuReal and PatternMatcher. This class is a Singleton.
 *
 * XXX THIS IS A BROKEN DESIGN! -- FIXME! (The fix is easy) The cache
 * needs to be kept in the atomspace; just put the atoms there, keep
 * them there, put them in well-known locations!  The problem with this
 * cache is that it does NOT work with multiple atomspaces; see for
 * example SuRealUTest, which bombs when multiple atomspces are used!!
 * FIXME by getting rid of this class!!!
 *
 * This cache stores the results of calls to differents methods of PatternMatcherCallBack
 * in separate Caches. This makes sense because such calls in SuReal are
 * time expensive and in some scenarios (in particular when SuReal is being used
 * by microplanner) sureal is called A LOT of times with very similar
 * parameters.
 *
 * This class have a reset() method which is supposed to be called when the
 * bunch of similar SuReal requests have ended.
 *
 * A major deficiency of this cache is that is is not ready to be used in a
 * multi-thread scenario.
 */
class SuRealCache
{

public:
    ~SuRealCache();

    static SuRealCache& instance();

    typedef std::unordered_map<Handle, HandleSeq> HandleSeqCache;
    typedef std::unordered_map<std::string, bool> StringMap;
    typedef std::unordered_map<std::string, StringMap> StringCacheMap;
    typedef std::unordered_map<Handle, bool> HandleBoolMap;
    typedef std::unordered_map<Handle, HandleBoolMap> HandleCacheMap;

    int variable_match(const Handle &h1, const Handle &h2);
    void add_variable_match(const Handle &h1, const Handle &h2, bool value);

    int clause_match(const Handle &h1, const Handle &h2);
    void add_clause_match(const Handle &h1, const Handle &h2, bool value);

    int grounding_match(const HandleMap &m1, const HandleMap &m2);
    void add_grounding_match(const HandleMap &m1, bool value);
    void add_grounding_match(const HandleMap &m1, const HandleMap &m2, bool value);

    bool get_node_list(const Handle &h, HandleSeq &list);
    void add_node_list(const Handle &h, const HandleSeq &list);

    void reset();


private:

    SuRealCache();

    static SuRealCache *instance_;

    HandleCacheMap m_variable_cache;
    HandleCacheMap m_clause_cache;
    StringCacheMap m_grounding_cache;
    StringMap m_partial_grounding_cache;
    HandleSeqCache m_node_list_cache;

    int match(HandleCacheMap &map, const Handle &h1, const Handle &h2);
    void add_match(HandleCacheMap &map, const Handle &h1, const Handle &h2, bool value);
    std::string build_map_hash_key(const HandleMap &m);
    std::string handle_to_hash_key(const Handle &h);
};

}
}

#endif // _OPENCOG_SUREAL_CACHE_H
