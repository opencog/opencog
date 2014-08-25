### Summary:

This is an implementation of a modified version of hobbs algorithm which does anaphora resolutions and zero-pronoun 
resolutions.

## Algorithm:

- /agents/hobbs.py

    It does Breadth-first search on the dependency tree for each word which needs to be resolved, each antecedent encountered
    in the traversal will go through a set of filters which filter out unqualified antecedents(i.e. gender mismatches, plura-singular mismatches, etc.)
    
    Each antecedent will be linked to the current target(the word we are trying to solve for) with a reference link:
    
    ```
    (ReferenceLink (some TV values)
        (target)
        (antecedent)
    )
    ```
    
    Unqualified antecedents will be linked to the target with ReferenceLinks with TV of (stv 0.020000 0.900000).
    
    Qualified antecedents will be linked to the target with ReferenceLinks with TV of (stv 0.98 ConfidenceValue)
    ConfidenceValue is defined as following:
    
    Suppose the decreasing rate is x, then
    the ith accepted candidate(in Breadth-first traversal order) will have confidence value of
    (x^(i-1))(1-x), i starts at 1.
       
- rules


    - /rules/filters:
    
        A set of filters which are used to filter out unqualified antecedents.
    
        Why do filters exist?
    
        There are restrictions which can be used to disqualify antecedent candidates immediately, such as gender mismatches, plural-singular mismatches.
    
    - /rules/pleonastic-it:
    
        A set of patterns which are used to identify pleonastic it(s).
    
    - /rules/pre-process:
    
        A set of patterns which are used to identify references which need to be resolved.
    
        Three sets of rules need to be applied in the following order:
        1. pre-process
        for each antecedent candidate:
            2. pleonastic-it
            3. filters
    
    - Note that rules in each set are independent with each other, or rules in the same set can be run in parallel.
 
- pleonastic-it

    
    - Pleonastic it(s) will be connected to (AnchorNode "Pleonastic-it"). For example,
    
    ```
    [2014-08-05 03:34:05:997] [FINE] (ReferenceLink (stv 0.980000 0.300000)
    (WordInstanceNode "it@36845a59-09d9-40c9-a07e-e4cf442edecf") ; [8606]
    (AnchorNode "Pleonastic-it") ; [8717]
    ) ; [9070]
    ```

## Prerequisites:

- Adding python library path

    ```
    Your_path/opencog/opencog/nlp
    ```
- Adding the path of hobbs agent to opencog.conf

    ```
    ../opencog/nlp/anaphora/agents
    ```

## Example #1:

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
    Resolving....
    (WordInstanceNode "he@c300d23e-e837-44f8-95b3-25e04f150153") ; [2790]

    accepted Tom@bccfc991-be8e-4624-b03f-22aac01ff872
    accepted apple@8b05b856-44d4-494d-917f-06f299375d3f
    accepted tree@38e7cdad-3430-4176-bffe-e484bb267eef
    Resolving....
    (WordInstanceNode "it@4dfaa49a-6b77-4a3e-b83e-f86f2a261921") ; [2802]

    accepted apple@8b05b856-44d4-494d-917f-06f299375d3f
    accepted tree@38e7cdad-3430-4176-bffe-e484bb267eef
    ```

## Example #2:

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

## Example #3:

- Example sentences:

    ```
    (relex-parse "waitresses and the cook sigh and roll their eyes.")
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
    (WordInstanceNode "their@9c845be5-461e-419b-94f7-fbd72f3a314c") ; [5686]

    accepted
    [(WordInstanceNode "waitresses@55cf46d0-9243-43dd-b1dd-133e9b43fab3" (av 0 0 0) (stv 1.000000 0.000000)) ; [5645]
    , (WordInstanceNode "cook@e9eb2ba8-9f1b-4fbc-a82f-c3e552c503f0" (av 0 0 0) (stv 1.000000 0.000000)) ; [5663]
    ]
    
    ([a,b] stands for a and b)
    
    accepted waitresses@55cf46d0-9243-43dd-b1dd-133e9b43fab3
    ```
## Example #4:

- Example sentences:

    ```
    (relex-parse "It seems that there is a car.")
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
    (WordInstanceNode "it@36845a59-09d9-40c9-a07e-e4cf442edecf") ; [8606]
    accepted Pleonastic-it
    
    ```
    
## Debugging

- Where to find the log file?

    ```
    The log file is located at "tmp/hobbs.log"
    ```

## More examples:

- /tests/hobbs-testing-results.txt:

    ```
    Sentences taken from the original paper of Hobbs' algorithm.
    ```
    
- /tests/simple-sentences-testing-results.txt:

    ```
    Simple sentences taken from different categories of sentence structures.
    ```

- /tests/testing-results.txt:

    ```
    Part of sentences in it are taken from Relex's hobbs testing data.
    The rest of the sentences are taken randomly from Wikipedia.
    ``` 
- /tests/Winograd-Schema-Challenge-testing-results.txt:

    ```
    Sentences taken from http://commonsensereasoning.org/2011/papers/Levesque.pdf
    ``` 