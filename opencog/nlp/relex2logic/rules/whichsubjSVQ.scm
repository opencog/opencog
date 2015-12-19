; This is for which-subjects of SV sentences, as in
; "Which movie sucks more, "Enemy Mine" or "Event Horizon"?
; (definitely Enemy Mine)
; (AN June 2015)


(define whichsubjSVQ
    (BindLink
        (VariableList
			(var-decl "$a-parse" "ParseNode")
			(var-decl "$subj" "WordInstanceNode")
			(var-decl "$verb" "WordInstanceNode")
			(var-decl "$obj" "WordInstanceNode")
			(var-decl "$qVar" "WordInstanceNode")
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

; XXX FIXME: define the whichsubjSVQ-rule
; This is function is not needed. It is added so as not to break the existing
; r2l pipeline.
(define (pre-whichsubjSVQ-rule subj verb)
    (whichsubjSVQ-rule (cog-name (word-inst-get-lemma  subj)) (cog-name subj)
              (cog-name (word-inst-get-lemma  verb)) (cog-name verb)
    )
)
