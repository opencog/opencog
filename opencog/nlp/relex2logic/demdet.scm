; This rule is for demonstrative determiners such as "that bozo" and "this job."  It used to be called the det rule
; but I changed it because there are lots of other determiners.
; (AN June 2015)


(define demdet
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
                (VariableNode "$demdet")
                (TypeNode "WordInstanceNode")
            )
        )
        (AndLink
            (WordInstanceLink
                (VariableNode "$noun")
                (VariableNode "$a-parse")
            )
            (WordInstanceLink
                (VariableNode "$demdet")
                (VariableNode "$a-parse")
            )
	    (EvaluationLink
   		(DefinedLinguisticRelationshipNode "_det")
   			(ListLink
      				(VariableNode "$noun")
      				(VariableNode "$demdet")
   			)
		)

        )
        (ExecutionOutputLink
       	   (GroundedSchemaNode "scm: pre-demdet-rule")
       	      (ListLink
       	         (VariableNode "$demdet")
       	         (VariableNode "$noun")
            )
        )
    )
)

(InheritanceLink (stv 1 .99) (ConceptNode "demdet-Rule") (ConceptNode "Rule"))

(ReferenceLink (stv 1 .99) (ConceptNode "demdet-Rule") demdet)

; This is function is not needed. It is added so as not to break the existing
; r2l pipeline.
(define (pre-demdet-rule demdet noun)
    	(demdet-rule (word-inst-get-word-str noun) (cog-name noun)
	(word-inst-get-word-str demdet) (cog-name demdet)
              
    )
)


