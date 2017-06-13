/*
 * PatternIndexAPI.h
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

#ifndef _OPENCOG_PATTERNINDEXAPI_H
#define _OPENCOG_PATTERNINDEXAPI_H

#include <map>
#include <vector>
#include <string>

#include <opencog/guile/SchemeEval.h>
#include <opencog/guile/SchemeSmob.h>
#include <opencog/atomspace/AtomSpace.h>
#include <opencog/atoms/base/Handle.h>

#include "TypeFrameIndex.h"

namespace opencog
{

/**
 *
 */
class PatternIndexAPI 
{
    public:

        typedef std::vector<std::pair<Handle, Handle>> VariableMapping;
        typedef std::pair<HandleSeq, VariableMapping> QueryResult;
        typedef std::pair<float, Handle> MiningResult;
        typedef enum MinedPatternsRankingMetric {I_SURPRISINGNESS, N_I_SURPRISINGNESS, II_SURPRISINGNESS, N_II_SURPRISINGNESS} MinedPatternsRankingMetric;


        static PatternIndexAPI& getInstance()
        {
            static PatternIndexAPI instance;
            return instance;
        }

        ~PatternIndexAPI();
        // Public to allow better error messages
        PatternIndexAPI(PatternIndexAPI const&) = delete;
        void operator=(PatternIndexAPI const&) = delete;

        int createNewIndex(const std::string &scmPath);
        void setProperty(int key, const std::string &propertyName, const std::string &propertyValue);
        void query(std::vector<QueryResult> &result, int key, const std::string &queryStr, bool distinct, bool noPermutations);
        void minePatterns(std::vector<MiningResult> &answer, int key, unsigned int components, unsigned int maxResults, MinedPatternsRankingMetric metric);

    private:

        typedef std::map<std::string, std::string> PropertyMap;

        std::vector<TypeFrameIndex *> indexVector;
        std::vector<PropertyMap> properties;
        AtomSpace *atomSpace;
        SchemeEval *schemeEval;

        PatternIndexAPI();
        void setDefaultProperties(PropertyMap &properties);
        void checkKey(int key);
        void applyProperties(int key);
        TypeFrameIndex::RankingMetric selectMetric(MinedPatternsRankingMetric src);



};

}

#endif // _OPENCOG_PATTERNINDEXAPI_H
