; This rule is for which-subjects of SVIO sentences, such as
; "Which agent sent you this message?"
; (AN June 2015)

(define whichsubjSVIOQ
    (BindLink
        (VariableList
			(var-decl "$a-parse" "ParseNode")
			(var-decl "$subj" "WordInstanceNode")
			(var-decl "$verb" "WordInstanceNode")
			(var-decl "$obj" "WordInstanceNode")
			(var-decl "$iobj" "WordInstanceNode")
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
	    (WordInstanceLink
                (VariableNode "$iobj")
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
                (DefinedLinguisticRelationshipNode "_iobj")
                (ListLink
                    (VariableNode "$verb")
                    (VariableNode "$iobj")
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
       	   (GroundedSchemaNode "scm: pre-whichsubjSVIOQ-rule")
       	      (ListLink
       	         (VariableNode "$subj")
       	         (VariableNode "$verb")
       	         (VariableNode "$obj")
            )
        )
    )
)
;ToDo: XXX FIXME define whichsubjSVIOQ-rule
; This is function is not needed. It is added so as not to break the existing
; r2l pipeline.
(define (pre-whichsubjSVIOQ-rule subj verb obj iobj)
    (whichsubjSVIOQ-rule (cog-name (word-inst-get-lemma  subj)) (cog-name subj)
              (cog-name (word-inst-get-lemma verb)) (cog-name verb)
              (cog-name (word-inst-get-lemma obj)) (cog-name obj)
              (cog-name (word-inst-get-lemma  iobj)) (cog-name iobj)
    )
)
