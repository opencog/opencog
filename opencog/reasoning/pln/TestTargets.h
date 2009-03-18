#include <opencog/nlp/wsd/atom_types.h>
map<int, Btr<vtree > > tests;

void initTests()
{
    AtomSpaceWrapper* atw = GET_ATW;
    int testi = 0;
    tests.clear();
/*  tests[31] = Btr<vtree > (new vtree(
        mva(IMP,
            mva(EXECUTION_LINK, CreateVar(ASW())),
            make_vtree(reward))));
*/
            /// Requires test/reasoning/bigdemo.xml 
printf("Insert test %d\n", testi++);
            tests[0] = Btr<vtree > (new vtree(mva(AND_LINK,
                    NewNode(CONCEPT_NODE, "Osama"),
                    NewNode(CONCEPT_NODE, "terrorist")
            )));

printf("Insert test %d\n", testi++);
            tests[1] = Btr<vtree > (new vtree(mva(EVALUATION_LINK,
                    NewNode(PREDICATE_NODE, "friendOf"),
                    mva(LIST_LINK,
                                NewNode(CONCEPT_NODE, "Amir"),
                                NewNode(CONCEPT_NODE, "Osama")
                            )
            )));
printf("Insert test %d\n", testi++);
            tests[2] = Btr<vtree > (new vtree(mva(EVALUATION_LINK,
                    NewNode(PREDICATE_NODE, "wasKilled"),
                    mva(LIST_LINK,
                                NewNode(CONCEPT_NODE, "Osama")
                            )
            )));
printf("Insert test %d\n", testi++);
            tests[3] = Btr<vtree >(new vtree(mva(EVALUATION_LINK,
                    NewNode(PREDICATE_NODE, "friendOf"),
                    mva(LIST_LINK,
                                NewNode(FW_VARIABLE_NODE, "$OsamaFriend"),
                                NewNode(CONCEPT_NODE, "Osama")
                            )
            )));
printf("Insert test %d\n", testi++);
            tests[4] = Btr<vtree > (new vtree(mva(INHERITANCE_LINK,
                    NewNode(CONCEPT_NODE, "Osama"),
                    NewNode(CONCEPT_NODE, "AlQaeda")
            )));
printf("Insert test %d\n", testi++);
            tests[5] = Btr<vtree > (new vtree(mva(INHERITANCE_LINK,
                    NewNode(CONCEPT_NODE, "Osama"),
                    NewNode(CONCEPT_NODE, "Abu")
            )));
printf("Insert test %d\n", testi++);
            tests[6] = Btr<vtree >(new vtree(mva(INHERITANCE_LINK,
                    NewNode(CONCEPT_NODE, "AlQaeda"),
                    NewNode(CONCEPT_NODE, "terrorist")
            )));
printf("Insert test %d\n", testi++);
            tests[7] = Btr<vtree > (new vtree(mva(INHERITANCE_LINK,
                    NewNode(CONCEPT_NODE, "Muhammad"),
                    NewNode(CONCEPT_NODE, "Osama")
            )));
printf("Insert test %d\n", testi++);
            tests[8] = Btr<vtree > (new vtree(mva(INHERITANCE_LINK,
                    NewNode(CONCEPT_NODE, "Muhammad"),
                    NewNode(CONCEPT_NODE, "terrorist")
            )));
printf("Insert test %d\n", testi++);
            tests[9] = Btr<vtree > (new vtree(mva(EVALUATION_LINK,
                    NewNode(PREDICATE_NODE, "killed"),
                    mva(LIST_LINK,
                                NewNode(FW_VARIABLE_NODE, "$killeri"),
                                NewNode(CONCEPT_NODE, "Osama")
                            )
            )));
            
            /// Maybe broken:
printf("Insert test %d\n", testi++);
            tests[10] = Btr<vtree > (new vtree(mva(EVALUATION_LINK,
                    NewNode(PREDICATE_NODE, "+++") /*,
                    mva(LIST_LINK)*/
            )));
            
            /// Requires test/reasoning/fetchdemo.xml 
printf("Insert test %d\n", testi++);
            tests[11] = Btr<vtree > (new vtree(mva(IMPLICATION_LINK,
                mva(AND_LINK,
                    mva(EVALUATION_LINK,
                        NewNode(PREDICATE_NODE, "teacher_say"),
                        mva(LIST_LINK,
                            NewNode(WORD_NODE, "fetch")
                        )
                    ),
                    NewNode(FW_VARIABLE_NODE, "$1")                 
                ),
                mva(EVALUATION_LINK,
                    NewNode(PREDICATE_NODE, "+++")
                )
            )));

printf("Insert test %d\n", testi++);
            tests[12] = Btr<vtree > (new vtree(mva(IMPLICATION_LINK,
                mva(AND_LINK,
                    NewNode(CONCEPT_NODE, "C"),
                    NewNode(CONCEPT_NODE, "A")
                ),
                mva(AND_LINK,
                    NewNode(CONCEPT_NODE, "C"),
                    NewNode(CONCEPT_NODE, "B")
                )
            )));

printf("Insert test %d\n", testi++);
            tests[13] = Btr<vtree > (new vtree(mva(IMPLICATION_LINK,
                mva(AND_LINK,
                    NewNode(CONCEPT_NODE, "A"),
                    NewNode(CONCEPT_NODE, "C")
                ),
                mva(AND_LINK,
                    NewNode(CONCEPT_NODE, "C"),
                    NewNode(CONCEPT_NODE, "B")
                )
            )));
printf("Insert test %d\n", testi++);
            tests[14] = Btr<vtree > (new vtree(mva(IMPLICATION_LINK,
                mva(AND_LINK,
                    NewNode(CONCEPT_NODE, "A"),
                    NewNode(CONCEPT_NODE, "C")
                ),
                mva(AND_LINK,
                    NewNode(CONCEPT_NODE, "B"),
                    NewNode(CONCEPT_NODE, "C")
                )
            )));

printf("Insert test %d\n", testi++);
            tests[15] = Btr<vtree > (new vtree(
                mva(IMPLICATION_LINK,
                    NewNode(FW_VARIABLE_NODE, "$1"),
                    mva(EVALUATION_LINK,
                        NewNode(PREDICATE_NODE, "teacher_say"),
                        mva(LIST_LINK,
                            NewNode(WORD_NODE, "fetch")
                        )                   
                    )
            )));

printf("Insert test %d\n", testi++);
            tests[16] = Btr<vtree > (new vtree(
                mva(IMPLICATION_LINK,
                    NewNode(FW_VARIABLE_NODE, "$1"),
                    mva(AND_LINK,
                        mva(EVALUATION_LINK,
                            NewNode(PREDICATE_NODE, "teacher_say"),
                            mva(LIST_LINK,
                                NewNode(WORD_NODE, "fetch")
                            )                   
                        ),
                        mva(EVALUATION_LINK,
                            NewNode(PREDICATE_NODE, "just_done"),
                            mva(LIST_LINK,
                                mva(EVALUATION_LINK,
                                    NewNode(SCHEMA_NODE, "give"),
                                    mva(LIST_LINK,
                                        NewNode(CONCEPT_NODE, "ball"),
                                        NewNode(CONCEPT_NODE, "teacher")
                                )
                            )                                       
                        )
                    )
                    )                   
                )
            ));

printf("Insert test %d\n", testi++);
            tests[17] = Btr<vtree > (new vtree(
                mva(IMPLICATION_LINK,
                    NewNode(FW_VARIABLE_NODE, "$1"),
                    mva(AND_LINK,
                    mva(EVALUATION_LINK,
                        NewNode(PREDICATE_NODE, "teacher_say"),
                        mva(LIST_LINK,
                            NewNode(WORD_NODE, "fetch")
                        )                   
                    ),
                    mva(EVALUATION_LINK,
                        NewNode(PREDICATE_NODE, "can_do"),
                        mva(LIST_LINK,
                            mva(EVALUATION_LINK,
                                NewNode(SCHEMA_NODE, "walktowards"),
                                mva(LIST_LINK,
                                    NewNode(CONCEPT_NODE, "teacher")
                                )
                            )                                       
                        )
                    )
                    )                   
                )
            ));

printf("Insert test %d\n", testi++);
            tests[18] = Btr<vtree > (new vtree(
                    mva(EVALUATION_LINK,
                        NewNode(PREDICATE_NODE, "just_done"),
                        mva(LIST_LINK,
                            mva(EVALUATION_LINK,
                                NewNode(SCHEMA_NODE, "give"),
                                mva(LIST_LINK,
                                    NewNode(CONCEPT_NODE, "ball"),
                                    NewNode(CONCEPT_NODE, "teacher")
                                )
                            )                                       
                        )
                    )
            ));

printf("Insert test %d\n", testi++);
            tests[19] = Btr<vtree > (new vtree(
                    mva(EVALUATION_LINK,
                        NewNode(PREDICATE_NODE, "do"),
                        mva(LIST_LINK,
                            mva(EVALUATION_LINK,
                                NewNode(SCHEMA_NODE, "give"),
                                mva(LIST_LINK,
                                    NewNode(CONCEPT_NODE, "ball"),
                                    NewNode(CONCEPT_NODE, "teacher")
                                )
                            )                                       
                        )
                    )
            ));
            
printf("Insert test %d\n", testi++);
        tests[20] = Btr<vtree > (new vtree(
                mva(EVALUATION_LINK,
                    NewNode(PREDICATE_NODE, "+++")
                )
            ));

printf("Insert test %d\n", testi++);
        tests[21] = Btr<vtree > (new vtree(
                mva(EVALUATION_LINK,
                    NewNode(PREDICATE_NODE, "test2"),
                    mva(LIST_LINK,
                        NewNode(CONCEPT_NODE, "Osama")
                    )
                )
            ));
printf("Insert test %d\n", testi++);
        tests[22] = Btr<vtree > (new vtree(
            mva(EVALUATION_LINK,
                        NewNode(PREDICATE_NODE, "just_done"),
                        mva(LIST_LINK,
                            mva(EVALUATION_LINK,
                                NewNode(SCHEMA_NODE, "give"),
                                mva(LIST_LINK,
                                    NewNode(CONCEPT_NODE, "ball"),
                                    NewNode(CONCEPT_NODE, "teacher")
                                )
                            )                                       
                        )
                    )
                ));

printf("Insert test %d\n", testi++);
            tests[23] = Btr<vtree > (new vtree(
                    mva(EVALUATION_LINK,
                        NewNode(PREDICATE_NODE, "near"),
                        mva(LIST_LINK,
                                NewNode(CONCEPT_NODE, "teacher")
                        )
                    )                   
                )
            );

printf("Insert test %d\n", testi++);
            tests[24] = Btr<vtree > (new vtree(
                    mva(SIMULTANEOUS_AND_LINK,
                        NewNode(WORD_NODE, "blockword"),
                        NewNode(FW_VARIABLE_NODE, "$blockword_associatee")
                    )                   
                )
            );

printf("Insert test %d\n", testi++);
            tests[25] = Btr<vtree > (new vtree(mva(IMPLICATION_LINK,
                NewNode(FW_VARIABLE_NODE, "$1"),
                mva(EVALUATION_LINK,
                    NewNode(PREDICATE_NODE, "+++")
                )
            )));

printf("Insert test %d\n", testi++);
            tests[26] = Btr<vtree > (new vtree(mva(EVALUATION_LINK,
                NewNode(PREDICATE_NODE, "found_under"),
                    mva(LIST_LINK,
                        NewNode(CONCEPT_NODE, "toy_6"),
                        NewNode(FW_VARIABLE_NODE, "$1")
                    )
            )));
printf("Insert test %d\n", testi++);
              tests[27] = Btr<vtree > (new vtree(mva(AND_LINK,
                mva(INHERITANCE_LINK,               
                    NewNode(CONCEPT_NODE, "toy_6"),
                    NewNode(CONCEPT_NODE, "toy")),
                mva(INHERITANCE_LINK,               
                    NewNode(CONCEPT_NODE, "red_bucket_6"),
                    NewNode(CONCEPT_NODE, "bucket")),
                mva(EVALUATION_LINK,
                    NewNode(PREDICATE_NODE, "placed_under"),
                        mva(LIST_LINK,
                            NewNode(CONCEPT_NODE, "toy_6"),
                            NewNode(CONCEPT_NODE, "red_bucket_6")
                        )
                )
            )));

printf("Insert test %d\n", testi++);
            tests[28] = Btr<vtree > (new vtree(mva(EVALUATION_LINK,
                    NewNode(PREDICATE_NODE, "friendOf"),
                    mva(LIST_LINK,
                                NewNode(CONCEPT_NODE, "Britney"),
                                NewNode(CONCEPT_NODE, "Amir")
            ))));

printf("Insert test %d\n", testi++);
            tests[29] = Btr<vtree > (new vtree(mva(FORALL_LINK,
                mva(LIST_LINK), //empty dummy
                mva(INHERITANCE_LINK,
                    NewNode(FW_VARIABLE_NODE, "$i"),
                    NewNode(CONCEPT_NODE, "terrorist")
            ))));

printf("Insert test %d\n", testi++);
            tests[30] = Btr<vtree > (new vtree(mva(VARIABLE_SCOPE_LINK,
                mva(LIST_LINK), //empty dummy
                mva(INHERITANCE_LINK,
                    NewNode(FW_VARIABLE_NODE, "$i"),
                    NewNode(CONCEPT_NODE, "terrorist")
            ))));

printf("Insert test %d\n", testi++);
            tests[31] = Btr<vtree > (new vtree(mva(EVALUATION_LINK,
                        NewNode(CONCEPT_NODE, "Possible"),
                        mva(LIST_LINK,
                            NewNode(FW_VARIABLE_NODE, "$elmerist")
                        )
                    )));

printf("Insert tests init\n");
}

