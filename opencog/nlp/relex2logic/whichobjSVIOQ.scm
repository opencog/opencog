; This rule is for which objects in indirect object sentences,
; such as "Which book did you give to your friend?"
; (AN June 2015)

(define whichobjSVIOQ
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
     				(VariableNode "$obj")
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
       	   (GroundedSchemaNode "scm: pre-whichobjSVIOQ-rule")
       	      (ListLink
       	         (VariableNode "$subj")
       	         (VariableNode "$verb")
       	         (VariableNode "$obj")
       	         (VariableNode "$iobj")
            )
        )
      )
    )
)
;;ToDo: define whichobjSVIOQ
; This is function is not needed. It is added so as not to break the existing
; r2l pipeline.
(define (pre-whichobjSVIOQ-rule subj verb obj iobj)
  (ListLink
    (whichobjSVIOQ-rule (word-inst-get-word-str obj) (cog-name obj)
              (word-inst-get-word-str verb) (cog-name verb)
              (word-inst-get-word-str subj) (cog-name subj)
		          (word-inst-get-word-str iobj) (cog-name iobj)
    )
  )
)
