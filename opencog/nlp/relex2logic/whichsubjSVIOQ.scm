; This rule is for which-subjects of SVIO sentences, such as
; "Which agent sent you this message?"
; (AN June 2015)

(define whichsubjSVIOQ
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
                (VariableNode "$iobj")
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
       (ListLink
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
)
;ToDo: define whichsubjSVIOQ-rule
; This is function is not needed. It is added so as not to break the existing
; r2l pipeline.
(define (pre-whichsubjSVIOQ-rule subj verb obj iobj)
  (ListLink
    (whichsubjSVIOQ-rule (word-inst-get-word-str subj) (cog-name subj)
              (word-inst-get-word-str verb) (cog-name verb)
              (word-inst-get-word-str obj) (cog-name obj)
		(word-inst-get-word-str iobj) (cog-name iobj)
    )
  )
)
