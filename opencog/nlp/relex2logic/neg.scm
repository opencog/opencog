; This rule is for negation of predicates, as in "You do not smell good, sir."
; (AN June 2015)


(define neg
    (BindLink
        (VariableList
            (TypedVariableLink
                (VariableNode "$a-parse")
                (TypeNode "ParseNode")
            )
            (TypedVariableLink
                (VariableNode "$pred")
                (TypeNode "WordInstanceNode")
            )
        )
        (AndLink
            (WordInstanceLink
                (VariableNode "$pred")
                (VariableNode "$a-parse")
            )
	    (InheritanceLink
   		(VariableNode "$pred")
   		(DefinedLinguisticConceptNode "negative")
	    )
        )
        (ExecutionOutputLink
       	   (GroundedSchemaNode "scm: pre-neg-rule")
       	      (ListLink
       	         (VariableNode "$pred")
            )
        )
    )
)

(InheritanceLink (stv 1 .99) (ConceptNode "neg-Rule") (ConceptNode "Rule"))

(ReferenceLink (stv 1 .99) (ConceptNode "neg-Rule") neg)

; This is function is not needed. It is added so as not to break the existing
; r2l pipeline.
(define (pre-neg-rule pred)
    	(negative-rule (word-inst-get-word-str pred) (cog-name pred)   
    )
)


