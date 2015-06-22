; NB: there seems to be a bit of a bug in Relex for assigning multiple adjectives to the same noun,
; which works partially -- it seems to get some of the multiple adjectives but not necessarily all 
; of them.
;
; This rule treats adjectival phrases as well as adjectives, because I rigged relex that way.  So, 
; amod handles "The tall man" and "the man in the hat."  'in the hat' is an adjectival phrase here.
; this is why I changed the individual triadic prepositional relations into two rules, one to assign
; the role of the prepositional phrase and the other to assign the object of the preposition.
; (AN June 2015)

(define amod
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
                (VariableNode "$adj")
                (TypeNode "WordInstanceNode")
            )
        )
        (AndLink
            (WordInstanceLink
                (VariableNode "$noun")
                (VariableNode "$a-parse")
            )
            (WordInstanceLink
                (VariableNode "$adj")
                (VariableNode "$a-parse")
            )
            (EvaluationLink
                (DefinedLinguisticRelationshipNode "_amod")
                (ListLink
                    (VariableNode "$noun")
                    (VariableNode "$adj")
                )
            )
        )
        (ExecutionOutputLink
       	   (GroundedSchemaNode "scm: pre-amod-rule")
       	      (ListLink
       	         (VariableNode "$noun")
       	         (VariableNode "$adj")
            )
        )
    )
)

(InheritanceLink (stv 1 .99) (ConceptNode "amod-Rule") (ConceptNode "Rule"))

(ReferenceLink (stv 1 .99) (ConceptNode "amod-Rule") amod)

; This is function is not needed. It is added so as not to break the existing
; r2l pipeline.
(define (pre-amod-rule noun adj)
    (amod-rule (word-inst-get-word-str noun) (cog-name noun)
              (word-inst-get-word-str adj) (cog-name adj)
    )
)


