/*
 * PatternIndexAPI.h
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

#ifndef _OPENCOG_PATTERNINDEXAPI_H
#define _OPENCOG_PATTERNINDEXAPI_H

#include <map>
#include <vector>
#include <string>
#include <limits>

#include <opencog/guile/SchemeEval.h>
#include <opencog/guile/SchemeSmob.h>
#include <opencog/atomspace/AtomSpace.h>
#include <opencog/atoms/base/Handle.h>

#include "TypeFrameIndex.h"

namespace opencog
{

/**
 * This class is a wrapper around the actual implementation of the pattern index
 * (which is done in TypeFrameIndex). The purpose of this class is to provide an
 * user's C++ API for all the functionalities of this module. Thus, there are no
 * relevant algorithms implemented here.
 *
 * The functionalities are:
 *
 * (1) Create and destroy pattern indexes
 * (2) Query an index for sub-graphs that satisfy a given pattern
 * (3) Mine "interesting patterns" in the index
 *
 * The user can create several different indexes (each with different
 * customization parameters) and query/mine these indexes individually. However,
 * notice that current implementation is not thread-safe, meaning that if you
 * plan to access the methods of this class concurrently, you must provide
 * mutual exclusion by yourself.
 *
 */
class PatternIndexAPI 
{
    public:

        // These types are used by some API methods to return querying/mining
        // results
        typedef HandlePairSeq VariableMapping;
        typedef std::pair<HandleSeq, VariableMapping> QueryResult;
        typedef std::pair<float, Handle> MiningResult;

        // Singleton
        static PatternIndexAPI& getInstance()
        {
            static PatternIndexAPI instance;
            return instance;
        }
        ~PatternIndexAPI();
        // public to provide better error messages
        PatternIndexAPI(PatternIndexAPI const&) = delete;
        void operator=(PatternIndexAPI const&) = delete;

        // Public API

        /*
         * Create a new index using the atoms listed in the passed SCM file.
         * Returns the Handle of an AnchorNode wich must be used as a key to
         * access this index in further calls to the other methods of this API.
         *
         * The atoms in the file are actually inserted in the Atomspace (if they
         * are not already).
         */
        Handle createIndex(const std::string &scmPath);

        /*
         * Create a new index using the atoms passed in the HandleSeq
         * Returns the Handle of an AnchorNode wich must be used as a key to
         * access this index in further calls to the other methods of this API.
         */
        Handle createIndex(const HandleSeq &handles);

        /*
         * Delete a previously created index. Raises an exception if the passed
         * key does not denote an index or if the index have been previously
         * deleted.
         */
        void deleteIndex(Handle key);

        /*
         * The methods for querying and mining an index behave differently
         * according to a set of parameters. Use this method to set these
         * parameters for a specific index.
         *
         * The list of all possible parameters as well as an explanation for
         * each of them is provided in the implementation of 
         * PatternIndexAPI::setDefaultProperties().
         */
        void setProperty(Handle key, const std::string &propertyName, const std::string &propertyValue);

        /*
         * Query an index for sub-graphs that satisfies the passed pattern.
         *
         * Example:
         *
         * Suppose we created an index with the following atoms
         * 
         * (SimilarityLink (ConceptNode "human") (ConceptNode "monkey"))
         * (SimilarityLink (ConceptNode "human") (ConceptNode "chimp"))
         * (SimilarityLink (ConceptNode "chimp") (ConceptNode "monkey"))
         * 
         * If we call query() passing the following queryLink
         * 
         * (AndLink 
         *   (SimilarityLink (VariableNode "V1") (VariableNode "V2"))
         *   (SimilarityLink (VariableNode "V2") (VariableNode "V3"))
         * )
         * 
         * We'll have the following answer
         * 
         * (ListLink
         *   (ListLink
         *     (ListLink
         *       (SimilarityLink (ConceptNode "monkey") (ConceptNode "chimp"))
         *       (SimilarityLink (ConceptNode "monkey") (ConceptNode "human"))
         *     )
         *     (ListLink
         *       (ListLink (VariableNode "V1") (ConceptNode "chimp"))
         *       (ListLink (VariableNode "V2") (ConceptNode "monkey"))
         *       (ListLink (VariableNode "V3") (ConceptNode "human"))
         *     )
         *   )
         * ) 
         *
         * The answer is a list of query results. Each result is a pair
         * <L1, L2>. L1 is a satisfying sub-graph, so each of its elements is
         * a list of atoms.  L2 is a list of variable assigments, so each of its
         * elements is a pair <variable, assigned atom>.
         *
         * The exact rules to match patterns and assign values to variables may
         * vary depending on the parameters setup.
         */
        Handle query(Handle key, Handle queryLink);

        /*
         * Just an alternative way to perform a query. This method expects a
         * query string (which is a scm definition of a queryLink like the one
         * described in the example above). Instead of creating a ListLink and
         * returning it as the answer, this method will populate the passed 
         * vector of QueryResult, which is a C++ structured type with the same
         * information described in the example above.
         */
        void query(std::vector<QueryResult> &result,
                   Handle key,
                   const std::string &queryStr);


        /*
         * Mine "interesting" patterns in the given index.
         *
         * "Interesting" patterns are patterns that return high values for the
         * metric function defined by the parameter "PatternRankingMetric".
         *
         * minePatterns() return the top "MaximumNumberOfMiningResults" patterns
         * discovered by the mining algorithm wraped in a ListLink like this:
         *
         * (ListLink
         *   (ListLink
         *     (NumberNode "0.98")
         *     (AndLink
         *       ...
         *     )
         *   )
         *   (ListLink
         *     ...
         *   )
         *   ...
         * )
         *
         * Many aspects of the mining algorithm can be customized using the
         * parameters setup.
         */
        Handle minePatterns(Handle key);

        /*
         * The same as the above but populates a vector of MiningResult instead
         * of returning a ListLink. MiningResult is a structured C++ type with
         * the same information described above.
         */
        void minePatterns(std::vector<MiningResult> &answer, Handle key);


    private:

        typedef std::map<std::string, std::string> StringMap;
        typedef std::map<int, std::pair<TypeFrameIndex *, StringMap>> IndexMap;

        int lastUsedTicket;
        IndexMap indexes;
        AtomSpace *atomSpace;
        SchemeEval *schemeEval;

        PatternIndexAPI();
        void setDefaultProperties(StringMap &properties);
        void applyProperties(Handle key);
        void setDefaultProperty(StringMap &map, const std::string &propertyKey);
        void query(std::vector<QueryResult> &answer,
                   Handle key,
                   const TypeFrame &query);
        std::string getStringProperty(StringMap &map, const std::string &key);
        int getIntProperty(StringMap &map,
                           const std::string &key,
                           int min = std::numeric_limits<int>::min(),
                           int max = std::numeric_limits<int>::max());
        bool getBoolProperty(StringMap &map, const std::string &key);

};

// Compliance with OpenCog's style of singleton access.
// using patternindex() or PatternIndexAPI::getInstance() is equivalent
PatternIndexAPI &patternindex();

}
#endif // _OPENCOG_PATTERNINDEXAPI_H
