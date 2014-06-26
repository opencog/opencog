### Prerequisites:

- Adding python library path

    ```
    "Your_path/opencog/opencog/nlp"
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

Note that accepted antecedents are sorted according to the order of Breadth-first traversal of the dependency tree.
Suppose word A comes before word B, A should have a higher probability of being an antecedent to a anaphor.
