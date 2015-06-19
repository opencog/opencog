; This rule is for which-subjects of prepositional objects, as in
; "Which toy is in this box?"
; (AN June 2015)


(define whichsubjpobjQ
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
                (VariableNode "$prep")
                (TypeNode "WordInstanceNode")
            )
            (TypedVariableLink
                (VariableNode "$pobj")
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
                (VariableNode "$prep")
                (VariableNode "$a-parse")
            )
            (WordInstanceLink
                (VariableNode "$pobj")
                (VariableNode "$a-parse")
            )
            (EvaluationLink
                (DefinedLinguisticRelationshipNode "_predadj")
                (ListLink 
                    (VariableNode "$subj")
		    (VariableNode "$prep")
                )
            )
            (EvaluationLink
                (DefinedLinguisticRelationshipNode "_pobj")
                (ListLink
                    (VariableNode "$prep")
                    (VariableNode "$pobj")
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
       	   (GroundedSchemaNode "scm: pre-whichsubjpobjQ-rule")
       	      (ListLink
       	         (VariableNode "$subj")
       	         (VariableNode "$prep")
       	         (VariableNode "$pobj")
            )
        )
    )
)

(InheritanceLink (stv 1 .99) (ConceptNode "whichsubjpobjQ-Rule") (ConceptNode "Rule"))

(ReferenceLink (stv 1 .99) (ConceptNode "whichsubjpobjQ-Rule") whichsubjpobjQ)

; This is function is not needed. It is added so as not to break the existing
; r2l pipeline.
(define (pre-whichsubjpobjQ-rule subj prep pobj)
    (whichsubjpobjQ-rule (word-inst-get-word-str pobj) (cog-name pobj)
              (word-inst-get-word-str prep) (cog-name prep)
              (word-inst-get-word-str subj) (cog-name subj)
    )
)


