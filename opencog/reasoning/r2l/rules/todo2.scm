(define todo2
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
                	(VariableNode "$subj2")
                	(TypeNode "WordInstanceNode")
            )
            (TypedVariableLink
                (VariableNode "$verb1")
                (TypeNode "WordInstanceNode")
            )
            (TypedVariableLink
                (VariableNode "$obj")
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
                    (VariableNode "$subj2")
                    (VariableNode "$a-parse")
                )
                (WordInstanceLink
                    (VariableNode "$verb1")
                    (VariableNode "$a-parse")
                )
                (WordInstanceLink
                    (VariableNode "$obj")
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
                    (DefinedLinguisticRelationshipNode "_subj")
                    (ListLink
                        (VariableNode "$verb2")
                        (VariableNode "$subj2")
                    )
                )
                (EvaluationLink
                    (DefinedLinguisticRelationshipNode "_obj")
                    (ListLink
                        (VariableNode "$verb2")
                        (VariableNode "$obj")
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
           	   (GroundedSchemaNode "scm: pre-todo2-rule")
           	      (ListLink
           	         (VariableNode "$verb1")
			(VariableNode "$verb2")
			(VariableNode "$subj1")
			(VariableNode "$subj2")
           	         (VariableNode "$obj")
                )
            )
        )
    )
)

(InheritanceLink (stv 1 .99) (ConceptNode "todo2-Rule") (ConceptNode "Rule"))

(ReferenceLink (stv 1 .99) (ConceptNode "todo2-Rule") todo2)

; This is function is not needed. It is added so as not to break the existing
; r2l pipeline.
(define (pre-todo2-rule verb1 verb2 subj1 subj2 obj)
    (to-do-rule-2 (word-inst-get-word-str verb1) (cog-name verb1)
		(word-inst-get-word-str verb2) (cog-name verb2)
		(word-inst-get-word-str subj1) (cog-name subj1)
		(word-inst-get-word-str subj2) (cog-name subj2)
 		(word-inst-get-word-str obj) (cog-name obj)      
    )
)


