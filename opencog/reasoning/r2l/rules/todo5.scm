(define todo5
    (BindLink
        (VariableList
            (TypedVariableLink
                (VariableNode "$a-parse")
                (TypeNode "ParseNode")
            )
            (TypedVariableLink
                (VariableNode "$subj1")
                (TypeNode "WordInstanceNode")
            )
            (TypedVariableLink
                (VariableNode "$verb1")
                (TypeNode "WordInstanceNode")
            )
            (TypedVariableLink
                (VariableNode "$verb2")
                (TypeNode "WordInstanceNode")
            )		
        )
        (ImplicationLink
            (AndLink
                (WordInstanceLink
                    (VariableNode "$subj1")
                    (VariableNode "$a-parse")
                )
                (WordInstanceLink
                    (VariableNode "$verb1")
                    (VariableNode "$a-parse")
                )
		(WordInstanceLink
                    (VariableNode "$verb2")
                    (VariableNode "$a-parse")
                )
                (EvaluationLink
                    (DefinedLinguisticRelationshipNode "_subj")
                    (ListLink
                        (VariableNode "$verb1")
                        (VariableNode "$subj1")
                    )
                )
		(EvaluationLink
                    (DefinedLinguisticRelationshipNode "_to-do")
                    (ListLink
                        (VariableNode "$verb1")
                        (VariableNode "$verb2")
                    )
                )
            )
            (ExecutionOutputLink
           	   (GroundedSchemaNode "scm: pre-todo5-rule")
           	      (ListLink
           	         (VariableNode "$verb1")
			(VariableNode "$verb2")
			(VariableNode "$subj1")
                )
            )
        )
    )
)

(InheritanceLink (stv 1 .99) (ConceptNode "todo5-Rule") (ConceptNode "Rule"))

(ReferenceLink (stv 1 .99) (ConceptNode "todo5-Rule") todo5)

; This is function is not needed. It is added so as not to break the existing
; r2l pipeline.
(define (pre-todo5-rule verb1 verb2 subj1)
    (to-do-rule-5 (word-inst-get-word-str verb1) (cog-name verb1)
		(word-inst-get-word-str verb2) (cog-name verb2)
		(word-inst-get-word-str subj1) (cog-name subj1)
    )
)


