; This rule is for which-subject sof SVO sentences, as in
; "Which one of you ate all the ice cream?"
; (AN june 2015)

(define whichsubjSVOQ
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
            (WordInstanceLink
                (VariableNode "$obj")
                (VariableNode "$a-parse")
            )
            (EvaluationLink
                (DefinedLinguisticRelationshipNode "_subj")
                (ListLink
                    (VariableNode "$verb")
                    (VariableNode "$subj")
                )
            )
            (EvaluationLink
                (DefinedLinguisticRelationshipNode "_obj")
                (ListLink
                    (VariableNode "$verb")
                    (VariableNode "$obj")
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
       	   (GroundedSchemaNode "scm: pre-whichsubjSVOQ-rule")
       	      (ListLink
       	         (VariableNode "$subj")
       	         (VariableNode "$verb")
       	         (VariableNode "$obj")
            )
        )
    )
)

; This is function is not needed. It is added so as not to break the existing
; r2l pipeline.
(define (pre-whichsubjSVOQ-rule subj verb obj)
    (whichsubjSVOQ-rule (cog-name (word-inst-get-lemma  subj)) (cog-name subj)
              (cog-name (word-inst-get-lemma  verb)) (cog-name verb)
              (cog-name (word-inst-get-lemma  obj)) (cog-name obj)
    )
)
