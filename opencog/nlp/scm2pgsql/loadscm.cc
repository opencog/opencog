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

#include <chrono>

#include "SCMLoader.h"
#include "TypeFrameIndex.h"
#include "TypeFrameIndexBuilder.h"
#include <opencog/guile/SchemeEval.h>

#include "CombinationGenerator.h"
#include "PartitionGenerator.h"
#include "CartesianProductGenerator.h"

using namespace opencog;

bool loadFile(char *fileName)
{
    /*
    bool returnValue = true;
    TypeFrame f;
    //std::vector<int> v1; v1.push_back(true ); v1.push_back(true ); v1.push_back(false); v1.push_back(false);
    //std::vector<int> v2; v2.push_back(true ); v2.push_back(false); v2.push_back(true ); v2.push_back(false);
    //std::vector<int> v3; v3.push_back(true ); v3.push_back(false); v3.push_back(false); v3.push_back(true );
    //std::vector<int> v4; v4.push_back(false); v4.push_back(false); v4.push_back(true ); v4.push_back(true );
    //std::vector<int> v1; v1.push_back(true ); v1.push_back(false); v1.push_back(false); v1.push_back(false);
    //std::vector<int> v2; v2.push_back(false); v2.push_back(true ); v2.push_back(false); v2.push_back(false);
    //std::vector<int> v3; v3.push_back(false); v3.push_back(false); v3.push_back(true ); v3.push_back(false);
    //std::vector<int> v4; v4.push_back(false); v4.push_back(false); v4.push_back(false); v4.push_back(true );
    //std::vector<int> v1; v1.push_back(true ); v1.push_back(false); v1.push_back(false); v1.push_back(false);
    //std::vector<int> v2; v2.push_back(false); v2.push_back(true ); v2.push_back(false); v2.push_back(false);
    //std::vector<int> v3; v3.push_back(false); v3.push_back(false); v3.push_back(true ); v3.push_back(false);
    //std::vector<int> v4; v4.push_back(false); v4.push_back(false); v4.push_back(false); v4.push_back(true );
    //std::vector<int> v1; v1.push_back(true ); v1.push_back(true ); v1.push_back(false); v1.push_back(false);
    //std::vector<int> v2; v2.push_back(true ); v2.push_back(false); v2.push_back(true ); v2.push_back(false);
    //std::vector<int> v3; v3.push_back(true ); v3.push_back(false); v3.push_back(false); v3.push_back(true );
    //std::vector<int> v4; v4.push_back(true ); v4.push_back(false); v4.push_back(false); v4.push_back(true );
    //std::vector<int> v1; v1.push_back(false); v1.push_back(true ); v1.push_back(true ); v1.push_back(true );
    //std::vector<int> v2; v2.push_back(false); v2.push_back(true ); v2.push_back(true ); v2.push_back(false);
    //std::vector<int> v3; v3.push_back(false); v3.push_back(false); v3.push_back(true ); v3.push_back(true );
    //std::vector<int> v4; v4.push_back(false); v4.push_back(false); v4.push_back(true ); v4.push_back(false);
    //std::vector<int> v1; v1.push_back(true ); v1.push_back(true ); v1.push_back(false); v1.push_back(true );
    //std::vector<int> v2; v2.push_back(true ); v2.push_back(true ); v2.push_back(false); v2.push_back(true );
    //std::vector<int> v3; v3.push_back(true ); v3.push_back(true ); v3.push_back(false); v3.push_back(true );
    //std::vector<int> v4; v4.push_back(true ); v4.push_back(true ); v4.push_back(false); v4.push_back(true );
    //std::vector<int> v1; v1.push_back(true ); v1.push_back(true ); v1.push_back(true ); v1.push_back(true );
    //std::vector<int> v2; v2.push_back(true ); v2.push_back(true ); v2.push_back(true ); v2.push_back(true );
    //std::vector<int> v3; v3.push_back(true ); v3.push_back(true ); v3.push_back(true ); v3.push_back(true );
    //std::vector<int> v4; v4.push_back(false); v4.push_back(false); v4.push_back(false); v4.push_back(false);
    //std::vector<int> v1; v1.push_back(true ); v1.push_back(false); v1.push_back(false); v1.push_back(false);
    //std::vector<int> v2; v2.push_back(true ); v2.push_back(false); v2.push_back(false); v2.push_back(false);
    //std::vector<int> v3; v3.push_back(true ); v3.push_back(false); v3.push_back(false); v3.push_back(false);
    //std::vector<int> v4; v4.push_back(true ); v4.push_back(true ); v4.push_back(true ); v4.push_back(true );
    std::vector<int> v1; v1.push_back(true ); v1.push_back(true ); v1.push_back(false); v1.push_back(false);
    std::vector<int> v2; v2.push_back(true ); v2.push_back(true ); v2.push_back(false); v2.push_back(false);
    std::vector<int> v3; v3.push_back(true ); v3.push_back(true ); v3.push_back(false); v3.push_back(false);
    std::vector<int> v4; v4.push_back(true ); v4.push_back(true ); v4.push_back(true ); v4.push_back(true );
    std::vector<std::vector<int>> m; m.push_back(v1); m.push_back(v2); m.push_back(v3); m.push_back(v4);
    if (f.isFeasible(m, (int) m.size())) {
        printf("FEASIBLE\n");
    } else {
        printf("NOT FEASIBLE\n");
    }
    */

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
    //index.printForDebug(true);

    /*
    CombinationGenerator comb(4, true, true);
    while (! comb.depleted()) {
        comb.printForDebug("", "\n");
        comb.generateNext();
    }
    */

    /*
    CombinationGenerator comb(5, (unsigned int) 3);
    while (! comb.depleted()) {
        comb.printForDebug("", "\n");
        comb.generateNext();
    }
    */

    /*
    PartitionGenerator part(4);

    while (! part.depleted()) {
        part.printForDebug("", "\n");
        part.generateNext();
    }

    printf("------------------\n");
    PartitionGenerator part2(4);

    while (! part2.depleted()) {
        printf("{");
        unsigned int count1 = 0;
        PartitionGenerator::IntegerSetSet partition = part2.getPartition();
        for (PartitionGenerator::IntegerSetSet::const_iterator itComp = partition.begin(); itComp != partition.end(); itComp++) {
            printf("{");
            unsigned int count2 = 0;
            for (PartitionGenerator::IntegerSet::const_iterator it = (*itComp).begin(); it != (*itComp).end(); it++) {
                printf("%u", (*it));
                if (count2++ != ((*itComp).size() - 1)) {
                    printf(" ");
                }
            }
            printf("}");
            if (count1++ != (partition.size() - 1)) {
                printf(" ");
            }
        }
        printf("}\n");
        part2.generateNext();
    }
    */

    /*
    std::vector<unsigned int> base;
    base.push_back(4);
    base.push_back(1);
    base.push_back(1);
    base.push_back(3);
    base.push_back(3);
    CartesianProductGenerator cart(base);
    while (! cart.depleted()) {
        cart.printForDebug("", "\n");
        cart.generateNext();
    }
    */

    /*
    std::vector<unsigned int> base;
    base.push_back(5);
    base.push_back(5);
    base.push_back(5);
    CartesianProductGenerator cart(base);
    while (! cart.depleted()) {
        if ((cart.at(2) == 0) || (cart.at(2) == 4)) {
            cart.drop(2);
            //cart.printForDebug("", "\n");
        } else {
            cart.printForDebug("", "\n");
        }
        cart.generateNext();
    }
    */

    /*
    CartesianProductGenerator cart(3, 4, true, true);
    while (! cart.depleted()) {
        if ((cart.at(1) == 1) || (cart.at(1) == 1)) {
            cart.drop(1);
            cart.printForDebug("DROP: ", "\n");
            //cart.printForDebug("", "\n");
        } else {
            cart.printForDebug("", "\n");
        }
        cart.generateNext();
    }
    */

    std::vector<std::pair<float,TypeFrame>> patterns;
    std::chrono::high_resolution_clock::time_point t1 = std::chrono::high_resolution_clock::now();
    index.minePatterns(patterns, 3, 10, TypeFrameIndex::N_I_SURPRISINGNESS);
    std::chrono::high_resolution_clock::time_point t2 = std::chrono::high_resolution_clock::now();
    unsigned int duration = std::chrono::duration_cast<std::chrono::seconds>(t2 - t1).count();
    for (unsigned int i = 0; i < patterns.size(); i++) {
        printf("%f: ", patterns.at(i).first);
        patterns.at(i).second.printForDebug("", "\n");
    }
    printf("Total mining time: %u\n", duration);

    /*
    std::vector<TypeFrameIndex::ResultPair> result;
    //std::string query2[1] = {"(AndLink (InheritanceLink (VariableNode \"V2\") (VariableNode \"V3\")) (SimilarityLink (VariableNode \"V1\") (VariableNode \"V2\")))"};
    //std::string query6[1] = {"(AndLink (InheritanceLink (VariableNode \"V2\") (VariableNode \"V2\")) (SimilarityLink (VariableNode \"V1\") (VariableNode \"V2\")))"};
    //std::string query7[10] = {
    //    "(AndLink (SimilarityLink (VariableNode \"X\") (VariableNode \"Y\")) (InheritanceLink (VariableNode \"Y\") (VariableNode \"Z\")))",
    //    "(AndLink (SimilarityLink (VariableNode \"X\") (VariableNode \"Y\")) (InheritanceLink (VariableNode \"Z\") (VariableNode \"Y\")))",
    //    "(OrLink (SimilarityLink (VariableNode \"X\") (VariableNode \"Y\")) (InheritanceLink (VariableNode \"Y\") (VariableNode \"Z\")))",
    //    "(OrLink (NotLink (NotLink (SimilarityLink (VariableNode \"X\") (VariableNode \"Y\")))) (NotLink (NotLink (InheritanceLink (VariableNode \"Y\") (VariableNode \"Z\")))))",
    //    "(OrLink (AndLink (SimilarityLink (VariableNode \"X\") (VariableNode \"Y\")) (InheritanceLink (VariableNode \"Y\") (VariableNode \"Z\"))) (AndLink (InheritanceLink (VariableNode \"X\") (VariableNode \"Y\")) (SimilarityLink (VariableNode \"Y\") (VariableNode \"Z\"))))",
    //    "(OrLink (NotLink (NotLink (AndLink (SimilarityLink (VariableNode \"X\") (VariableNode \"Y\")) (InheritanceLink (VariableNode \"Y\") (VariableNode \"Z\"))))) (AndLink (InheritanceLink (VariableNode \"X\") (VariableNode \"Y\")) (SimilarityLink (VariableNode \"Y\") (VariableNode \"Z\"))))",
    //    "(AndLink (SimilarityLink (VariableNode \"X\") (VariableNode \"Y\")) (OrLink (InheritanceLink (VariableNode \"Y\") (VariableNode \"Z\")) (InheritanceLink (VariableNode \"Z\") (VariableNode \"Y\"))))",
    //    "(AndLink (SimilarityLink (VariableNode \"X\") (VariableNode \"Y\")) (SimilarityLink (VariableNode \"Y\") (VariableNode \"Z\")))",
    //    "(AndLink (SimilarityLink (VariableNode \"X\") (VariableNode \"Y\")) (NotLink (AndLink (SimilarityLink (VariableNode \"X\") (VariableNode \"Y\")) (SimilarityLink (VariableNode \"Y\") (VariableNode \"Z\")))))",
    //    "(AndLink (NotLink (AndLink (SimilarityLink (VariableNode \"X\") (VariableNode \"Y\")) (SimilarityLink (VariableNode \"Y\") (VariableNode \"Z\")))) (SimilarityLink (VariableNode \"X\") (VariableNode \"Y\")))"
    //};

    //std::string query9[10] = {
    //    "(AndLink (SimilarityLink (VariableNode \"X\") (VariableNode \"Y\")) (SimilarityLink (VariableNode \"Y\") (VariableNode \"Z\")))",
    //    "(OrLink (AndLink (SimilarityLink (VariableNode \"X\") (VariableNode \"Y\")) (SimilarityLink (VariableNode \"Y\") (VariableNode \"Z\"))) (AndLink (InheritanceLink (VariableNode \"X\") (VariableNode \"Y\")) (InheritanceLink (VariableNode \"Y\") (VariableNode \"Z\"))))",
    //    "(AndLink (InheritanceLink (VariableNode \"X\") (VariableNode \"Y\")) (InheritanceLink (VariableNode \"Y\") (VariableNode \"Z\")) (InheritanceLink (VariableNode \"Z\") (VariableNode \"W\")))",
    //    "(AndLink (SimilarityLink (VariableNode \"X\") (VariableNode \"Y\")) (NotLink (AndLink (InheritanceLink (VariableNode \"X\") (VariableNode \"Z\")) (InheritanceLink (VariableNode \"Y\") (VariableNode \"Z\")))))",
    //    "(AndLink (SimilarityLink (VariableNode \"X\") (VariableNode \"Y\")) (SimilarityLink (VariableNode \"Y\") (VariableNode \"Z\")) (SimilarityLink (VariableNode \"Z\") (VariableNode \"W\")))",
    //    "(AndLink (InheritanceLink (VariableNode \"X\") (VariableNode \"Y\")) (InheritanceLink (VariableNode \"Y\") (VariableNode \"X\")))",
    //    "(AndLink (NotLink (OrLink (InheritanceLink (VariableNode \"X\") (ConceptNode \"mammal\")) (InheritanceLink (VariableNode \"Y\") (ConceptNode \"mammal\")))) (SimilarityLink (VariableNode \"X\") (VariableNode \"Y\")))",
    //    "(AndLink (SimilarityLink (VariableNode \"X\") (VariableNode \"Y\")) (SimilarityLink (VariableNode \"Y\") (VariableNode \"Z\")) (SimilarityLink (VariableNode \"Z\") (VariableNode \"X\")))",
    //    "(AndLink (AndLink (InheritanceLink (VariableNode \"X\") (VariableNode \"Z\")) (InheritanceLink (VariableNode \"Y\") (VariableNode \"Z\"))) (NotLink (SimilarityLink (VariableNode \"X\") (VariableNode \"Y\"))))",
    //    "(AndLink (OrLink (SimilarityLink (VariableNode \"$var_1\") (ConceptNode \"chimp\") ) (SimilarityLink (VariableNode \"$var_1\") (VariableNode \"IV1\"))) (AndLink (InheritanceLink (ConceptNode \"chimp\") (VariableNode \"IV2\")) (InheritanceLink (VariableNode \"IV1\") (VariableNode \"IV2\"))))"
    //};

    std::string query10[1] = {
        "(AndLink (SimilarityLink (VariableNode \"X\") (VariableNode \"Y\")) (SimilarityLink (VariableNode \"Y\") (VariableNode \"Z\")))"
    };

    for (unsigned int i = 0; i < (sizeof(query10) / sizeof(query10[0])); i++) {
    //for (unsigned int i = 9; i == 9; i++) {
        //index.query(result, query9[i]);
        index.query(result, query10[i], true, true);
        printf("================================================================================\n");
        printf("Query: %s\n", query10[i].c_str());
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
    */

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
