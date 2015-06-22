; This is for which-subjects of SV sentences, as in
; "Which movie sucks more, "Enemy Mine" or "Event Horizon"?
; (definitely Enemy Mine)
; (AN June 2015)


(define whichsubjSVQ
    (BindLink
        (VariableList
            (TypedVariableLink
                (VariableNode "$a-parse")
                (TypeNode "ParseNode")
            )
            (TypedVariableLink
                (VariableNode "$subj")
                (TypeNode "WordInstanceNode")
            )
            (TypedVariableLink
                (VariableNode "$verb")
                (TypeNode "WordInstanceNode")
            )
            (TypedVariableLink
                (VariableNode "$obj")
                (TypeNode "WordInstanceNode")
            )
            (TypedVariableLink
                (VariableNode "$qVar")
                (TypeNode "WordInstanceNode")
            )		
        )
        (AndLink
            (WordInstanceLink
                (VariableNode "$subj")
                (VariableNode "$a-parse")
            )
            (WordInstanceLink
                (VariableNode "$verb")
                (VariableNode "$a-parse")
            )
            (EvaluationLink
                (DefinedLinguisticRelationshipNode "_subj")
                (ListLink
                    (VariableNode "$verb")
                    (VariableNode "$subj")
                )
            )
		(AbsentLink            
			(EvaluationLink
        	        	(DefinedLinguisticRelationshipNode "_obj")
        	       		 (ListLink
        	           		 (VariableNode "$verb")
        	           		 (VariableNode "$obj")
        	        	)
        	   	 )
		)
			(EvaluationLink
   				(DefinedLinguisticRelationshipNode "_det")
  			 	(ListLink
     					(VariableNode "$subj")
      					(VariableNode "$qVar")
				)
			)
			(InheritanceLink
				(VariableNode "$qVar")
				(DefinedLinguisticConceptNode "which")
			)
        )
        (ExecutionOutputLink
       	   (GroundedSchemaNode "scm: pre-whichsubjSVQ-rule")
       	      (ListLink
       	         (VariableNode "$subj")
       	         (VariableNode "$verb")

            )
        )
    )
)

(InheritanceLink (stv 1 .99) (ConceptNode "whichsubjSVQ-Rule") (ConceptNode "Rule"))

(ReferenceLink (stv 1 .99) (ConceptNode "whichsubjSVQ-Rule") whichsubjSVQ)

; This is function is not needed. It is added so as not to break the existing
; r2l pipeline.
(define (pre-whichsubjSVQ-rule subj verb)
    (whichsubjSVQ-rule (word-inst-get-word-str subj) (cog-name subj)
              (word-inst-get-word-str verb) (cog-name verb)
    )
)


