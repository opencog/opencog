;"She sings with an accent." ,

(define adverbialpp
    (BindLink
        (VariableList
            (TypedVariableLink
                (VariableNode "$a-parse")
                (TypeNode "ParseNode")
            )
            (TypedVariableLink
                (VariableNode "$prep")
                (TypeNode "WordInstanceNode")
            )
            (TypedVariableLink
                (VariableNode "$noun")
                (TypeNode "WordInstanceNode")
            )
            (TypedVariableLink
                (VariableNode "$verb")
                (TypeNode "WordInstanceNode")
            )
        )
        (AndLink
            (WordInstanceLink
                (VariableNode "$prep")
                (VariableNode "$a-parse")
            )
            (WordInstanceLink
                (VariableNode "$noun")
                (VariableNode "$a-parse")
            )
            (WordInstanceLink
                (VariableNode "$verb")
                (VariableNode "$a-parse")
            )
            (EvaluationLink
                (DefinedLinguisticRelationshipNode "_pobj")
                (ListLink
                    (VariableNode "$prep")
                    (VariableNode "$noun")
                )
            )
           (EvaluationLink
                (DefinedLinguisticRelationshipNode "_advmod")
                (ListLink
                    (VariableNode "$verb")
                    (VariableNode "$prep")
                )
           )
        )
        (ExecutionOutputLink
       	   (GroundedSchemaNode "scm: pre-adverbialpp-rule")
       	      (ListLink
       	         (VariableNode "$prep")
       	         (VariableNode "$noun")
                 (VariableNode "$verb")
            )
        )
    )
)


(define (pre-adverbialpp-rule prep noun verb)
  (ListLink
    (adverbialpp-rule (cog-name (word-inst-get-lemma  verb)) (cog-name verb)
              (cog-name (word-inst-get-lemma prep)) (cog-name prep)
              (cog-name (word-inst-get-lemma noun)) (cog-name noun)
    )
  )
)
