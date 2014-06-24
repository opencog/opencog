### Adding python library path

- Path

    ```
    "Your_path/opencog/opencog/nlp"
    ```


### Example:

- Example sentences:

    ```
    (relex-parse "Tom ate an apple under a tree.")
    (relex-parse "It was delicious.")
    (relex-parse "It was small.")
    ```
    
- Load rules:

    ```
    (load-scm-from-file "nlp/anaphora/rules/anaphora_resolution.scm")
    (load-scm-from-file "nlp/anaphora/rules/pronoun_finder.scm")
    ```

- Finding pronouns:

     ```
    (cog-bind pronoun-finder)
     ```

- Doing anaphora resolution:

    ```
    (cog-bind anaphora-resolution)
    ```


- Testing result:

```
(ListLink
   (ReferenceLink
      (WordInstanceNode "tree@98b1ee02-2dcc-46cc-8494-3fdd1ba0b3b9")
      (WordInstanceNode "it@094c245d-543b-4fdd-aae1-ac6dcdebaac2")
   )
   (ReferenceLink
      (WordInstanceNode "Tom@8d1f1511-eee0-4bc1-a6b1-69ce13e754bf")
      (WordInstanceNode "it@094c245d-543b-4fdd-aae1-ac6dcdebaac2")
   )
   (ReferenceLink
      (WordInstanceNode "apple@4fd1556d-32f5-4ec5-8477-87eca9451fd6")
      (WordInstanceNode "it@094c245d-543b-4fdd-aae1-ac6dcdebaac2")
   )
   (ReferenceLink
      (WordInstanceNode "it@885025c5-a1ac-43bf-81de-03fd8b940866")
      (WordInstanceNode "it@094c245d-543b-4fdd-aae1-ac6dcdebaac2")
   )
   (ReferenceLink
      (WordInstanceNode "tree@98b1ee02-2dcc-46cc-8494-3fdd1ba0b3b9")
      (WordInstanceNode "it@885025c5-a1ac-43bf-81de-03fd8b940866")
   )
   (ReferenceLink
      (WordInstanceNode "Tom@8d1f1511-eee0-4bc1-a6b1-69ce13e754bf")
      (WordInstanceNode "it@885025c5-a1ac-43bf-81de-03fd8b940866")
   )
   (ReferenceLink
      (WordInstanceNode "apple@4fd1556d-32f5-4ec5-8477-87eca9451fd6")
      (WordInstanceNode "it@885025c5-a1ac-43bf-81de-03fd8b940866")
   )
)
```
