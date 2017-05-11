/*
 * loadscm.cc
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

#include "SCMLoader.h"
#include "TypeFrameIndex.h"
#include "TypeFrameIndexBuilder.h"
#include <opencog/guile/SchemeEval.h>

using namespace opencog;

bool loadFile(char *fileName)
{
    TypeFrameIndex index;
    TypeFrameIndexBuilder builder(&index);
    AtomSpace atomSpace;
    SchemeEval::init_scheme();
    SchemeEval::set_scheme_as(&atomSpace);
    SchemeEval *schemeEval = new SchemeEval(&atomSpace);
    schemeEval->eval("(use-modules (opencog nlp relex2logic))");
    bool returnValue = SCMLoader::load(fileName, atomSpace, &builder);
    //bool returnValue = SCMLoader::load(fileName, atomSpace);
    index.buildSubPatternsIndex();
    std::vector<TypeFrameIndex::ResultPair> result;

    //std::string query2[1] = {"(AndLink (InheritanceLink (VariableNode \"V2\") (VariableNode \"V3\")) (SimilarityLink (VariableNode \"V1\") (VariableNode \"V2\")))"};
    //std::string query6[1] = {"(AndLink (InheritanceLink (VariableNode \"V2\") (VariableNode \"V2\")) (SimilarityLink (VariableNode \"V1\") (VariableNode \"V2\")))"};

    std::string query7[9] = {
        "(AndLink (SimilarityLink (VariableNode \"X\") (VariableNode \"Y\")) (InheritanceLink (VariableNode \"Y\") (VariableNode \"Z\")))",
        "(AndLink (SimilarityLink (VariableNode \"X\") (VariableNode \"Y\")) (InheritanceLink (VariableNode \"Z\") (VariableNode \"Y\")))",
        "(OrLink (SimilarityLink (VariableNode \"X\") (VariableNode \"Y\")) (InheritanceLink (VariableNode \"Y\") (VariableNode \"Z\")))",
        "(OrLink (NotLink (NotLink (SimilarityLink (VariableNode \"X\") (VariableNode \"Y\")))) (NotLink (NotLink (InheritanceLink (VariableNode \"Y\") (VariableNode \"Z\")))))",
        "(OrLink (AndLink (SimilarityLink (VariableNode \"X\") (VariableNode \"Y\")) (InheritanceLink (VariableNode \"Y\") (VariableNode \"Z\"))) (AndLink (InheritanceLink (VariableNode \"X\") (VariableNode \"Y\")) (SimilarityLink (VariableNode \"Y\") (VariableNode \"Z\"))))",
        "(OrLink (NotLink (NotLink (AndLink (SimilarityLink (VariableNode \"X\") (VariableNode \"Y\")) (InheritanceLink (VariableNode \"Y\") (VariableNode \"Z\"))))) (AndLink (InheritanceLink (VariableNode \"X\") (VariableNode \"Y\")) (SimilarityLink (VariableNode \"Y\") (VariableNode \"Z\"))))",
        "(AndLink (SimilarityLink (VariableNode \"X\") (VariableNode \"Y\")) (OrLink (InheritanceLink (VariableNode \"Y\") (VariableNode \"Z\")) (InheritanceLink (VariableNode \"Z\") (VariableNode \"Y\"))))",
        "(AndLink (SimilarityLink (VariableNode \"X\") (VariableNode \"Y\")) (SimilarityLink (VariableNode \"Y\") (VariableNode \"Z\")))",
        "(AndLink (SimilarityLink (VariableNode \"X\") (VariableNode \"Y\")) (NotLink (AndLink (SimilarityLink (VariableNode \"X\") (VariableNode \"Y\")) (SimilarityLink (VariableNode \"Y\") (VariableNode \"Z\")))))"
    };

                

    for (unsigned int i = 0; i < (sizeof(query7) / sizeof(query7[0])); i++) {
        index.query(result, query7[i]);
        printf("================================================================================\n");
        printf("Query: %s\n", query7[i].c_str());
        for (unsigned int i = 0; i < result.size(); i++) {
            printf("Query solution %u\n" , i);
            for (TypeFrameIndex::TypeFrameSet::iterator it = result.at(i).first.begin(); it != result.at(i).first.end(); it++) {
                (*it).printForDebug("", "\n", true);
            }
            for (TypeFrameIndex::VarMapping::iterator it = result.at(i).second.begin(); it != result.at(i).second.end(); it++) {
                printf("%s = ", (*it).first.c_str());
                (*it).second.printForDebug("", "\n", true);
            }
        }
    }

    return returnValue;
}

int main(int argc, char *argv[]) {

    int exitValue = 0;

    if (argc != 2) {
        fprintf(stderr, "Usage: %s <SCM file>\n", argv[0]);
        exitValue = 1;
    } else {
        if (! loadFile(argv[1])) {
            exitValue = 1;
        }
    }

    return exitValue;
}
