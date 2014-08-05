### Summary:

This is an implementation of a modified version of hobbs algorithm which does anaphora resolutions and zero-pronoun 
resolutions.

### Algorithm:

- /agents/hobbs.py

    ```
    It does Breadth-first search on the dependency tree for each word which needs to be resolved, each antecedent encountered
    in the traversal will go through a set of filters which filter out unqualified antecedents(i.e. gender mismatches, plura-singular mismatches, etc.)
    
    Each antecedent will be linked to the current target(the word we are trying to solve for) with a reference link:
    
    (ReferenceLink (some TV values)
        (target)
        (antecedent)
    )
    
    Unqualified antecedents will be linked to the target with ReferenceLinks with TV of (stv 0.020000 0.900000).
    
    Qualified antecedents will be linked to the target with ReferenceLinks with TV of (stv 0.98 ConfidenceValue)
    ConfidenceValue is defined as following:
    
    Suppose the decreasing rate is x, then
    the ith accepted candidate(in Breadth-first traversal order) will have confidence value of
    (x^(i-1))(1-x)  i starts at 1.
    
    ```
       
- rules

    ```
    /rules/filters:
    
    A set of filters which are used to filter out unqualified antecedents.
    
    /rules/pleonastic-it:
    
    A set of patterns which are used to identify pleonastic it(s).
    
    /rules/pre-process:
    
    A set of patterns which are used to identify words which need to be resolved.
    ```
 
    
### Prerequisites:

- Adding python library path

    ```
    Your_path/opencog/opencog/nlp
    ```
- Adding the path of hobbs agent to opencog.conf

    ```
    ../opencog/nlp/anaphora/agents
    ```

### Example #1:

- Example sentences:

    ```
    (relex-parse "Tom saw an apple under a tree, he ate it")
    ```
    
- Load hobbs agent:

    ```
    loadpy hobbs
    ```
- run a single cycle of hobbs agent

    ```
    agents-step hobbs.HobbsAgent
    ```

- Testing result(displayed on cogserver server console):

    ```
    Resolving...........
    (WordInstanceNode "he@0b328c6e-3a84-431d-9856-534d5a2640c6") ; [112]

    accepted Tom@e68745b7-6266-45c1-b020-de4d7c81dccf

    Resolving...........
    (WordInstanceNode "it@31c03890-5ffa-4af9-a536-ee25ff0bfcd8") ; [124]

    accepted apple@6f6bd594-952b-491b-8b6a-691e7c56e4ce
    accepted tree@a482013b-0279-41f2-a5d5-e15820da3616

    ```

### Example #2:

- Example sentences:

    ```
    (relex-parse "The window had a crack in it.")
    ```

- Load hobbs agent:

    ```
    loadpy hobbs
    ```
- run a single cycle of hobbs agent

    ```
    agents-step hobbs.HobbsAgent
    ```

- Testing result(displayed on cogserver server console):

    ```
    Resolving...........
    (WordInstanceNode "it@1e86217f-14d1-43bc-9e37-b4aef73dd7b6") ; [100]

    accepted window@5d75f283-ec1f-4db7-86bc-984d00cc18c9
    accepted crack@35d449fa-deeb-46a3-bf94-58a746aa5784

    ```

- Where to find the log file?

    ```
    The log file is located at "tmp/hobbs.log"

    ```

### More examples:

- /tests/hobbs-testing-results.txt:

    ```
    Sentences taken from the original paper of Hobbs' algorithm.
    ```
    
- /tests/simple-sentences-testing-results.txt:

    ```
    Simple sentences taken from different categories of sentence structures.

- /tests/testing-results.txt:

    ```
    Part of sentences in it are taken from Relex's hobbs testing data.
    The rest of the sentences are taken randomly from Wikipedia.
    ``` 
- /tests/Winograd-Schema-Challenge-testing-results.txt:

    ```
    Sentences taken from http://commonsensereasoning.org/2011/papers/Levesque.pdf
    ``` 