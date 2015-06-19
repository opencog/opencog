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

(InheritanceLink (stv 1 .99) (ConceptNode "poss-Rule") (ConceptNode "Rule"))

(ReferenceLink (stv 1 .99) (ConceptNode "poss-Rule") poss)

; This is function is not needed. It is added so as not to break the existing
; r2l pipeline.
(define (pre-poss-rule noun poss)
    (possessive-rule (word-inst-get-word-str noun) (cog-name noun)
              (word-inst-get-word-str poss) (cog-name poss)
    )
)


