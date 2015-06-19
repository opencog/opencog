; This handles adverbs and many adverbial phrases, because I rigged relex to treat a lot of adverbial phrases as adverbs.
; so, for example "She sings well" is treated the same way as "She sings with an accent."  'with an qccent' is treated
; as an adverb.  This is why I changed the individual triadic prepositional relations into two rules each -- one to assign
; the preposition as an adverbial or adjectival or predicate, and the other to assign the object of the preposition.
; (AN June 2015)

(define advmod
    (BindLink
        (VariableList
            (TypedVariableLink
                (VariableNode "$a-parse")
                (TypeNode "ParseNode")
            )
            (TypedVariableLink
                (VariableNode "$verb")
                (TypeNode "WordInstanceNode")
            )
            (TypedVariableLink
                (VariableNode "$adv")
                (TypeNode "WordInstanceNode")
            )
        )
        (AndLink
            (WordInstanceLink
                (VariableNode "$verb")
                (VariableNode "$a-parse")
            )
            (WordInstanceLink
                (VariableNode "$adv")
                (VariableNode "$a-parse")
            )
            (EvaluationLink
                (DefinedLinguisticRelationshipNode "_advmod")
                (ListLink
                    (VariableNode "$verb")
                    (VariableNode "$adv")
                )
            )
        )
        (ExecutionOutputLink
       	   (GroundedSchemaNode "scm: pre-advmod-rule")
       	      (ListLink
       	         (VariableNode "$verb")
       	         (VariableNode "$adv")
            )
        )
    )
)

(InheritanceLink (stv 1 .99) (ConceptNode "advmod-Rule") (ConceptNode "Rule"))

(ReferenceLink (stv 1 .99) (ConceptNode "advmod-Rule") advmod)

; This is function is not needed. It is added so as not to break the existing
; r2l pipeline.
(define (pre-advmod-rule verb adv)
    (advmod-rule (word-inst-get-word-str verb) (cog-name verb)
              (word-inst-get-word-str adv) (cog-name adv)
    )
)


