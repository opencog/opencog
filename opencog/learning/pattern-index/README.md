# Pattern Index - an index for fast lookup of patterns in the AtomSpace

This module provides two interfaces (C++ and Guile) for a data abstraction that
alows the user to create indexes for subsets of Atoms from the AtomSpace and
then submit queries to retrieve subgraphs that matches a given pattern.

_*NOTE*: this is not a wrapper around OpenCog's PatternMatcher._

## Why having a Pattern Index?

The motivation behind this implementation are the applications that use
Opencog's AtomSpace as data container. Such applications usually load a
(potentially huge) bunch of pre-processed information into the AtomSpace and then
perform some sort of computation using this information (eventually generating
more information which is stored back in the AtomSpace)

Using AtomSpace information may be time-expensive specially if the number of
atoms involved is huge. So having an index to perform faster lookup of patterns
may be essential to allow the algorithms to run in acceptable time.

So this module allows the user to do something like:

1. Load a large SCM with lots of atoms
1. Build an index with only the relevant atoms
1. Use this index to gather information from the AtomSpace as required by the algorithm
1. Optionally discard the index when it is no longer necessary (this will have no effect in the AtomSpace)

Although it's not the main purpose of this module, Pattern Index have
additional algorithms to perform pattern mining in the indexed patterns. Those
algorithms follow the methodology described in the documents:

* http://wiki.opencog.org/w/Pattern_miner#Tutorial_of_running_Pattern_Miner_in_Opencog
* http://wiki.opencog.org/w/Measuring_Surprisingness

_*NOTE*: this functionality is not a wrapper around OpenCog's PatternMiner_

## Additional motivation for the Pattern Index

The Pattern Index internal structure have been designed to allow building the
index in the disk (rather in in RAM) and without the need af having an actual
AtomSpace in memory.

This is relevant because one would be able to build an index (entirely in the
disk) given a huge SCM file which would never fit in RAM.  This functionality
IS NOT IMPLEMENTED YET but it is an fairly-easy-to-implement new feature.

Once we have this feature we would be able to do fairly fast lookup for
patterns in huge datasets in disk.

## Relevant files for potential users (examples and API)

If you plan to use this module you want to take a look at the following files:

* **patternIndexQueryExample.cc and patternIndexMiningExample.cc:** Small C++ programs with a complete use case of the Pattern Index being used to perform queries and pattern mining. If you plan to use Pattern Index from C++ code this is the right place to start.
* **pattern-index-query-example.scm and pattern-index-mining-example.scm:** Small Scheme programs with a complete use case of the Pattern Index being used to perform queries and pattern mining. If you plan to use Pattern Index from Scheme code this is the right place to start.
* **toy-example-query.scm and toy-example-mining.scm:** Datasets used by the example programs above
* **ExampleConfig.conf:** Opencog's configuration file used by the example programs above
* **PatternIndexAPI.h:** This is the C++ API.
* **PatternIndexSCM.{h,cc}:** This is the Scheme API.

## Other files (actual implementation of functionalities)

* **TypeFrameIndex.{h,cc}:** Actual implementation of all the index algorithms (creation, queries and mining)
* **TypeFrame.{h,cc}:** Basic data abstraction used in the index. It's a simplified representation of Atom
* **CartesianProductGenerator.{h,cc}, CombinationGenerator.{h,cc} and PartitionGenerator.{h,cc}:** Helper classes used to iterate through combinations of set(s) elements.
* **SCMLoader.{h,cc}, SCMLoaderCallback.{h,cc} and TypeFrameIndexBuilder.{h,cc}:** Helper classes used to load SCM files.

## How to use the Pattern Index

Reading the example programs mentioned above are the best way to understand how to use the Pattern Index.

### Creating a new index

#### C++

```
    Handle indexKey = patternindex().createIndex("toy-example-query.scm");
```

#### Scheme

First you need to load the pattern-index module

```
    (use-modules (opencog learning pattern-index))
```

then create an index given a file name

```
    (define indexKey (pi-create-index (ConceptNode "toy-example-query.scm")))
```

indexKey will be used in further calls to query or mine the newly created
index. You can create many indexes using different sets of atoms and query them
separately.

### Quering requests

Let us use "toy-example-query.scm" to illustrate queries examples.

```
    (SimilarityLink (ConceptNode "human") (ConceptNode "monkey"))
    (SimilarityLink (ConceptNode "human") (ConceptNode "chimp"))
    (SimilarityLink (ConceptNode "chimp") (ConceptNode "monkey"))
    (SimilarityLink (ConceptNode "snake") (ConceptNode "earthworm"))
    (SimilarityLink (ConceptNode "rhino") (ConceptNode "triceratops"))
    (SimilarityLink (ConceptNode "snake") (ConceptNode "vine"))
    (SimilarityLink (ConceptNode "human") (ConceptNode "ent"))
    (InheritanceLink (ConceptNode "human") (ConceptNode "mammal"))
    (InheritanceLink (ConceptNode "monkey") (ConceptNode "mammal"))
    (InheritanceLink (ConceptNode "chimp") (ConceptNode "mammal"))
    (InheritanceLink (ConceptNode "mammal") (ConceptNode "animal"))
    (InheritanceLink (ConceptNode "reptile") (ConceptNode "animal"))
    (InheritanceLink (ConceptNode "snake") (ConceptNode "reptile"))
    (InheritanceLink (ConceptNode "dinosaur") (ConceptNode "reptile"))
    (InheritanceLink (ConceptNode "triceratops") (ConceptNode "dinosaur"))
    (InheritanceLink (ConceptNode "earthworm") (ConceptNode "animal"))
    (InheritanceLink (ConceptNode "rhino") (ConceptNode "mammal"))
    (InheritanceLink (ConceptNode "vine") (ConceptNode "plant"))
    (InheritanceLink (ConceptNode "ent") (ConceptNode "plant"))
```

#### C++

In C++ one can query an index either passing a Handle or a std::string representing the query.

```
    std::string queryStr = "(AndLink (SimilarityLink (VariableNode \"X\") (VariableNode \"Y\")) (SimilarityLink (VariableNode \"Y\") (VariableNode \"Z\")))";
    Handle queryHandle = schemeEval->eval_h(queryStr);
    Handle resultHandle = patternindex().query(indexKey, queryHandle);
```

resultHandle will point to a ListLink with the query result. This link is a
list with all the satisfying subgraphs (each represented by a ListLink) and the
respective variable assignments (also represented by ListLinks to pairs
```<variable, assigned atom>```).

```
    (ListLink
        (ListLink
            (ListLink
                (SimilarityLink (ConceptNode "monkey") (ConceptNode "chimp"))
                (SimilarityLink (ConceptNode "monkey") (ConceptNode "human"))
            )
            (ListLink
                (ListLink (VariableNode "X") (ConceptNode "chimp"))
                (ListLink (VariableNode "Y") (ConceptNode "monkey"))
                (ListLink (VariableNode "Z") (ConceptNode "human"))
            )
        )
        (ListLink
            (ListLink
                (SimilarityLink (ConceptNode "ent") (ConceptNode "human"))
                (SimilarityLink (ConceptNode "monkey") (ConceptNode "human"))
            )
            (ListLink
                (ListLink (VariableNode "X") (ConceptNode "ent"))
                (ListLink (VariableNode "Y") (ConceptNode "human"))
                (ListLink (VariableNode "Z") (ConceptNode "monkey"))
            )
        )
        (ListLink
            (ListLink
                (SimilarityLink (ConceptNode "human") (ConceptNode "chimp"))
                (SimilarityLink (ConceptNode "ent") (ConceptNode "human"))
            )
            (ListLink
                (ListLink (VariableNode "X") (ConceptNode "ent"))
                (ListLink (VariableNode "Y") (ConceptNode "human"))
                (ListLink (VariableNode "Z") (ConceptNode "chimp"))
            )
        )
        (ListLink
            (ListLink
                (SimilarityLink (ConceptNode "earthworm") (ConceptNode "snake"))
                (SimilarityLink (ConceptNode "vine") (ConceptNode "snake"))
            )
            (ListLink
                (ListLink (VariableNode "X") (ConceptNode "vine"))
                (ListLink (VariableNode "Y") (ConceptNode "snake"))
                (ListLink (VariableNode "Z") (ConceptNode "earthworm"))
            )
        )
    )
```

Optionally one can pass the query in a std::string.  Using this syntax,
```query()``` will populate a vector of QueryResult with all the subgraphs (and
respective variable assigments) that satisfies the passed pattern.

```
    std::string queryStr = "(AndLink (SimilarityLink (VariableNode \"X\") (VariableNode \"Y\")) (NotLink (AndLink (InheritanceLink (VariableNode \"X\") (VariableNode \"Z\")) (InheritanceLin
    std::vector<PatternIndexAPI::QueryResult> queryResult;
    patternindex().query(queryResult, indexKey, queryStr);
```

In our example, this query would return:

```
    Result #1:
    
    (SimilarityLink
        (ConceptNode "earthworm")
        (ConceptNode "snake")
    )
    
    Mapping:
    
    (VariableNode "X")
    (ConceptNode "earthworm")
    
    (VariableNode "Y")
    (ConceptNode "snake")
    
    Result #2:
    
    (SimilarityLink
        (ConceptNode "triceratops")
        (ConceptNode "rhino")
    )
    
    Mapping:
    
    (VariableNode "X")
    (ConceptNode "rhino")
    
    (VariableNode "Y")
    (ConceptNode "triceratops")
    
    Result #3:
    
    (SimilarityLink
        (ConceptNode "vine")
        (ConceptNode "snake")
    )
    
    Mapping:
    
    (VariableNode "X")
    (ConceptNode "snake")
    
    (VariableNode "Y")
    (ConceptNode "vine")
    
    Result #4:
    
    (SimilarityLink
        (ConceptNode "ent")
        (ConceptNode "human")
    )
    
    Mapping:
    
    (VariableNode "X")
    (ConceptNode "ent")
    
    (VariableNode "Y")
    (ConceptNode "human")
```

#### Scheme

In scheme we use only one syntax:

```
    (pi-query index-key (AndLink (SimilarityLink (VariableNode "X") (VariableNode "Y")) (SimilarityLink (VariableNode "Y") (VariableNode "Z"))))
```

Which return the same answer as the first syntax described above for C++.

### Mining requests

To illustrate the pattern mining using Pattern Index, we'll use
"toy-example-mining.scm" which is exactly the same dataset
```ugly_male_soda-drinker_corpus.scm``` used as example in OpenCog's PatternMiner.

#### C++

```
    patternindex().minePatterns(resultPatterns, indexKey);
```

resultPatterns is populated with the best found patterns. Actually with pairs ```<quality measure, pattern toplevel atom>```. In our example, the best pattern is:

```
    (AndLink
        (InheritanceLink
            (VariableNode "V0")
            (ConceptNode "ugly")
        )
        (InheritanceLink
            (VariableNode "V0")
            (ConceptNode "man")
        )
        (InheritanceLink
            (VariableNode "V0")
            (ConceptNode "soda drinker")
        )
    ) 
```


#### Scheme

```
    (pi-mine-patterns index-key)
```

Results are basically the same as the above but returned as a nested ListLink instead of a structured type.

## Running the example programs

To run the examples just execute one of these command lines (from ```build/```):

```
./opencog/learning/pattern-index/patternIndexQueryExample ../opencog/learning/pattern-index/toy-example-query.scm ../opencog/learning/pattern-index/ExampleConfig.conf
./opencog/learning/pattern-index/patternIndexMiningExample ../opencog/learning/pattern-index/toy-example-mining.scm ../opencog/learning/pattern-index/ExampleConfig.conf
guile -l ../opencog/learning/pattern-index/pattern-index-query-example.scm
guile -l ../opencog/learning/pattern-index/pattern-index-mining-example.scm
```

_*NOTE*: The Scheme API read the configuration file in ```lib/opencog.conf``` so you need to add Pattern Index parameters to it. ```cat ../opencog/learning/pattern-index/ExampleConfig.conf >> ./lib/opencog.conf```_

## Design details

Let us see an example to explain how the Pattern Index is built and how queries are processed.
Again, lets take "toy-example-query.scm".

```
    0: (SimilarityLink (ConceptNode "human") (ConceptNode "monkey"))
    1: (SimilarityLink (ConceptNode "human") (ConceptNode "chimp"))
    2: (SimilarityLink (ConceptNode "chimp") (ConceptNode "monkey"))
    3: (SimilarityLink (ConceptNode "snake") (ConceptNode "earthworm"))
    4: (SimilarityLink (ConceptNode "rhino") (ConceptNode "triceratops"))
    5: (SimilarityLink (ConceptNode "snake") (ConceptNode "vine"))
    6: (SimilarityLink (ConceptNode "human") (ConceptNode "ent"))
    7: (InheritanceLink (ConceptNode "human") (ConceptNode "mammal"))
    8: (InheritanceLink (ConceptNode "monkey") (ConceptNode "mammal"))
    9: (InheritanceLink (ConceptNode "chimp") (ConceptNode "mammal"))
    10: (InheritanceLink (ConceptNode "mammal") (ConceptNode "animal"))
    11: (InheritanceLink (ConceptNode "reptile") (ConceptNode "animal"))
    12: (InheritanceLink (ConceptNode "snake") (ConceptNode "reptile"))
    13: (InheritanceLink (ConceptNode "dinosaur") (ConceptNode "reptile"))
    14: (InheritanceLink (ConceptNode "triceratops") (ConceptNode "dinosaur"))
    15: (InheritanceLink (ConceptNode "earthworm") (ConceptNode "animal"))
    16: (InheritanceLink (ConceptNode "rhino") (ConceptNode "mammal"))
    17: (InheritanceLink (ConceptNode "vine") (ConceptNode "plant"))
    18: (InheritanceLink (ConceptNode "ent") (ConceptNode "plant"))
```

The numbers preceeding the links are any sort of identifier. It may be the
actual Handle of the link in the AtomSpace or a pointer to a file entry in
disk. In current implementation we use an int indicating the position of the
root link in the input file, e.g. "07" is the 7th root link in the scm file.

To implement some of the new features indicated in the section "TODO" below,
one would need to change the meaning of this to represent the actual desired
behaviour.

It actually doesn't matter for the index implementation. The idea behind the
Pattern Index is to build a kind of inverted list of sub-patterns pointing to
wherever the actual data is. If data is in AtomSpace, disk or elsewhere it
doesn't really matter.

Thus, given the input above, here are a some examples of index entries:

```
    (ConceptNode "chimp") : 1 2 9
    (SimilarityLink * (ConceptNode 0 "chimp")) : 1 2
    (InheritanceLink (ConceptNode "chimp") (ConceptNode "mammal")) : 9 
    (InheritanceLink (ConceptNode "chimp") *) : 7 
    (InheritanceLink * (ConceptNode "mammal")) : 7 8 9 16
    (SimilarityLink * *) : 0 1 2 3 4 5 6
```

Once the index is built, it's possible to query it for patterns. Let us
detail how the two example queries (provided in the example programs) work.

First query is:

```
    Q: 
        (AndLink
            (SimilarityLink
                (VariableNode "X")
                (VariableNode "Y")
            )
            (SimilarityLink
                (VariableNode "Y")
                (VariableNode "Z")
            )
        )
```

So we are searching the input database for X, Y and Z | exist (SimilarityLink X
Y) and (SimilarityLink Y Z).

The querying algorithm interprets AndLink, OrLink and NotLink as logical
operators. The resulting sub-queries are recursively processed and then joined
according to the operator. Recursion stops when the first non-"logical
operator" root atom is found.

_*NOTE*: See the section "Known issues" below.  These atom types should be
changed._

In our example, we have two recursive calls for each of these sub-queries:

```
    Q1:

    (SimilarityLink
        (VariableNode "X")
        (VariableNode "Y")
    )
```

and

```
    Q2:

    (SimilarityLink
        (VariableNode "Y")
        (VariableNode "Z")
    )
```

Q1 is pre-processed to replace VariableNodes by index wildcards. So it becomes
(SimilarityLink \* \*). This query is submitted to the index and the matching
results R1 are recorded:

```
    R1:

        (SimilarityLink (ConceptNode "human") (ConceptNode "monkey"))
        (SimilarityLink (ConceptNode "human") (ConceptNode "chimp"))
        (SimilarityLink (ConceptNode "chimp") (ConceptNode "monkey"))
        (SimilarityLink (ConceptNode "snake") (ConceptNode "earthworm"))
        (SimilarityLink (ConceptNode "rhino") (ConceptNode "triceratops"))
        (SimilarityLink (ConceptNode "snake") (ConceptNode "vine"))
        (SimilarityLink (ConceptNode "human") (ConceptNode "ent"))
```

Similarly, Q2 returns:

```
    R2:

        (SimilarityLink (ConceptNode "human") (ConceptNode "monkey"))
        (SimilarityLink (ConceptNode "human") (ConceptNode "chimp"))
        (SimilarityLink (ConceptNode "chimp") (ConceptNode "monkey"))
        (SimilarityLink (ConceptNode "snake") (ConceptNode "earthworm"))
        (SimilarityLink (ConceptNode "rhino") (ConceptNode "triceratops"))
        (SimilarityLink (ConceptNode "snake") (ConceptNode "vine"))
        (SimilarityLink (ConceptNode "human") (ConceptNode "ent"))
```

Either R1 and R2 have 7 valid results each. Each of these results are actually a tuple
(S, M, F) where S is the atom sub-graph that satisfies the query, M is the subjacent
variable mapping and F is a set of mappings that are forbidden from this point
up in the recursion. We're showing only S for the sake of simplicity.

E.g. in the first result in R1: 

```
M = {[X, (ConceptNode "human") or (ConceptNode "monkey")], [Y, (ConceptNode "human") or (ConceptNode "monkey")]}
```

In R1 and R2 F is empty for all results.

Given R1 and R2 the "AND" operator is processed searching in R1 and R2 for
combinations of results that gives valid variable mappings.  (the concept of
"valid" variable mapping deppends on the configured parameters as described in
the sections above).

Each element (S, M, F) of R = R1 AND R2 is computed using the corresponding
elements in R1 and R2:

```
    S = (AndLink S1 S2)
    M = M1 U M2
    F = F1 U F2
```

In this case, the valid combination of results are:

```
    R:
        S = (AndLink
            (SimilarityLink (ConceptNode "human") (ConceptNode "monkey"))
            (SimilarityLink (ConceptNode "human") (ConceptNode "chimp"))
        )
        M = {[X, (ConceptNode "monkey")], [Y, (ConceptNode "human")], [Z, (ConceptNode "chimp")]}
        F = EMPTY

        S = (AndLink
            (SimilarityLink (ConceptNode "human") (ConceptNode "monkey"))
            (SimilarityLink (ConceptNode "human") (ConceptNode "ent"))
        )
        M = {[X, (ConceptNode "monkey")], [Y, (ConceptNode "human")], [Z, (ConceptNode "ent")]}
        F = EMPTY

        S = (AndLink
            (SimilarityLink (ConceptNode "human") (ConceptNode "chimp"))
            (SimilarityLink (ConceptNode "human") (ConceptNode "ent"))
        )
        M = {[X, (ConceptNode "chimp")], [Y, (ConceptNode "human")], [Z, (ConceptNode "ent")]}
        F = EMPTY

        S = (AndLink
            (SimilarityLink (ConceptNode "snake") (ConceptNode "earthworm"))
            (SimilarityLink (ConceptNode "snake") (ConceptNode "vine"))
        )
        M = {[X, (ConceptNode "earthworm")], [Y, (ConceptNode "snake")], [Z, (ConceptNode "vine")]}
        F = EMPTY
```

Which is the result of the original query Q. Again, each result in R is a tuple
(S, M, F). F is empty in all these results. M is the combination of the
mappings of each combined result.

The second query is slightly more interesting:

```
    (AndLink
        (SimilarityLink
            (VariableNode "X")
            (VariableNode "Y")
        )
        (NotLink
            (AndLink
                (InheritanceLink
                    (VariableNode "X")
                    (VariableNode "Z")
                )
                (InheritanceLink
                    (VariableNode "Y")
                    (VariableNode "Z")
                )
            )
        )
    )
```

We are searching the input database for X similar to Y but X and Y should not inherit from the same Z.
Again, the query Q is split in two sub-queries that will be processed recursively Q = Q1 AND Q2:

```
    Q1:
        (SimilarityLink
            (VariableNode "X")
            (VariableNode "Y")
        )

    Q2:
        (NotLink
            (AndLink
                (InheritanceLink
                    (VariableNode "X")
                    (VariableNode "Z")
                )
                (InheritanceLink
                    (VariableNode "Y")
                    (VariableNode "Z")
                )
            )
        )
```

Results for Q1 are exaclty the same R1 of the previous example. Q2 is
recursively split again Q2 = NOT Q3:

```
    Q3:
        (AndLink
            (InheritanceLink
                (VariableNode "X")
                (VariableNode "Z")
            )
            (InheritanceLink
                (VariableNode "Y")
                (VariableNode "Z")
            )
        )
```

Finally, Q3 is split again Q3 = Q4 AND Q5:

```
    Q4:
        (InheritanceLink
            (VariableNode "X")
            (VariableNode "Z")
        )

    Q5:
        (InheritanceLink
            (VariableNode "Y")
            (VariableNode "Z")
        )
```

Results for Q4 and Q5 are all the InheritanceLinks in the input database. To
compute R3 which is the result of Q3 = Q4 AND Q5, we need to combine (Q4 AND
Q5) these links and pick up the combinations that give valid variable mappings
(as we did in the previous example). Thus (ommiting the variable mappings):

```
    R3:
        (AndLink
            (InheritanceLink (ConceptNode "human") (ConceptNode "mammal"))
            (InheritanceLink (ConceptNode "monkey") (ConceptNode "mammal"))
        )
        (AndLink
            (InheritanceLink (ConceptNode "human") (ConceptNode "mammal"))
            (InheritanceLink (ConceptNode "chimp") (ConceptNode "mammal"))
        )
        (AndLink
            (InheritanceLink (ConceptNode "human") (ConceptNode "mammal"))
            (InheritanceLink (ConceptNode "rhino") (ConceptNode "mammal"))
        )
        (AndLink
            (InheritanceLink (ConceptNode "monkey") (ConceptNode "mammal"))
            (InheritanceLink (ConceptNode "chimp") (ConceptNode "mammal"))
        )
        (AndLink
            (InheritanceLink (ConceptNode "monkey") (ConceptNode "mammal"))
            (InheritanceLink (ConceptNode "rhino") (ConceptNode "mammal"))
        )
        (AndLink
            (InheritanceLink (ConceptNode "chimp") (ConceptNode "mammal"))
            (InheritanceLink (ConceptNode "rhino") (ConceptNode "mammal"))
        )
        (AndLink
            (InheritanceLink (ConceptNode "mammal") (ConceptNode "animal"))
            (InheritanceLink (ConceptNode "reptile") (ConceptNode "animal"))
        )
        (AndLink
            (InheritanceLink (ConceptNode "mammal") (ConceptNode "animal"))
            (InheritanceLink (ConceptNode "earthworm") (ConceptNode "animal"))
        )
        (AndLink
            (InheritanceLink (ConceptNode "reptile") (ConceptNode "animal"))
            (InheritanceLink (ConceptNode "earthworm") (ConceptNode "animal"))
        )
        (AndLink
            (InheritanceLink (ConceptNode "snake") (ConceptNode "reptile"))
            (InheritanceLink (ConceptNode "dinosaur") (ConceptNode "reptile"))
        )
        (AndLink
            (InheritanceLink (ConceptNode "vine") (ConceptNode "plant"))
            (InheritanceLink (ConceptNode "ent") (ConceptNode "plant"))
        )
```

Back from recursion we can now compute R2 which is the results for Q2 = NOT Q3.
At this point, the querying algorithm will just record the resulting mappings
of Q3 and define a set of "forbidden" mappings. S and M are empty for
all results in R2.

```
    R2:
        F = {[X, (ConceptNode "human")], [Y, (ConceptNode "monkey"]}
        F = {[X, (ConceptNode "human")], [Y, (ConceptNode "chimp"]}
        F = {[X, (ConceptNode "human")], [Y, (ConceptNode "rhino"]}
        F = {[X, (ConceptNode "monkey")], [Y, (ConceptNode "chimp"]}
        F = {[X, (ConceptNode "monkey")], [Y, (ConceptNode "rhino"]}
        F = {[X, (ConceptNode "chimp")], [Y, (ConceptNode "rhino"]}
        F = {[X, (ConceptNode "mammal")], [Y, (ConceptNode "reptile"]}
        F = {[X, (ConceptNode "mammal")], [Y, (ConceptNode "earthworm"]}
        F = {[X, (ConceptNode "snake")], [Y, (ConceptNode "dinosaur"]}
        F = {[X, (ConceptNode "vine")], [Y, (ConceptNode "ent"]}
```

Thus the recursion gets us back to compute Q = Q1 AND Q2. Results of Q1 are R1.
Results of Q2 are the forbidden mappings in R2. Thus combination of R1 and R2
is:

```
    R: 
        (SimilarityLink (ConceptNode "human") (ConceptNode "ent"))
        (SimilarityLink (ConceptNode "snake") (ConceptNode "earthworm"))
        (SimilarityLink (ConceptNode "human") (ConceptNode "ent"))
```

## TODO

1. Implement C++ and Scheme API to allow creation of empty index followed by addition of atoms and finally actual creation of the index.
2. Incorporate AttentionValue and TruthValue in the index
3. Implement multi-thread version of TypeframeIndex::query()
4. Implement optional creation of TypeFrameIndex pointing to elements in disk
5. Implement optional creation of TypeFrameIndex storing the index itself in disk
6. Implement distributed version of the mining algorithm
7. Implement distributed version of TypeFrameIndex
8. Add scheme bindings for the configuration options

## Known issues

1. Replace VariableNode, AndLink, OrLink and NotLink by PatternVariableNode,
PatternAndLink, PatternOrLink and PatternNotLink respectively.
2. TypeFrameIndex::query() is terribly (time) inefficient. It needs a complete
refatoring to make rational use of allocated memory and avoid unnecessary
copying of data among recursive calls.
3. Current implementation is adding all the possible permutations of
UNORDERED_LINK (only links with arity <= 5). Thus if we have a (SimilarityLink
A B), all the patterns (SimilarityLink * *), (SimilarityLink A *),
(SimilarityLink * A), (SimilarityLink B *) and (SimilarityLink * B) are being
inserted in the index. The proper way would be to insert only three patterns
(SimilarityLink * *), (SimilarityLink A *) and (SimilarityLink B *) and deal
with ORDERED_LINK permutations in the query() method.
4. User should be able to control the Atom types that should be "expanded" when
creating the patterns. For example, the user may want to prevent (ListLink A B)
from becoming patterns like (ListLink A *) etc. This sort of customization may
be crucial when trying to do pattern mining.
