; This rule is for nouns which are posessed by words marked as posessives by relex,
; such as "my doggy" or "your mama"
; (AN June 2015)

(define poss
    (BindLink
        (VariableList
            (TypedVariableLink
                (VariableNode "$a-parse")
                (TypeNode "ParseNode")
            )
            (TypedVariableLink
                (VariableNode "$noun")
                (TypeNode "WordInstanceNode")
            )
            (TypedVariableLink
                (VariableNode "$poss")
                (TypeNode "WordInstanceNode")
            )
        )
        (AndLink
            (WordInstanceLink
                (VariableNode "$noun")
                (VariableNode "$a-parse")
            )
            (WordInstanceLink
                (VariableNode "$poss")
                (VariableNode "$a-parse")
            )
            (EvaluationLink
                (DefinedLinguisticRelationshipNode "_poss")
                (ListLink
                    (VariableNode "$noun")
                    (VariableNode "$poss")
                )
            )
        )
        (ExecutionOutputLink
       	   (GroundedSchemaNode "scm: pre-poss-rule")
       	      (ListLink
       	         (VariableNode "$noun")
       	         (VariableNode "$poss")
            )
        )
    )
)

; This is function is not needed. It is added so as not to break the existing
; r2l pipeline.
(define (pre-poss-rule noun poss)
  (ListLink
    (possessive-rule (cog-name (word-inst-get-lemma noun)) (cog-name noun)
              (cog-name (word-inst-get-lemma poss)) (cog-name poss)
    )
  )
)
