; This rule hooks up a complementizer with the verb it complementizes.
; E.g. "I run if I have energy."  This rule relates "run" to "if"; the _comp
; rule constructs the "if" phrase.  It may be hard to predict when this rule
; will fire because there are other rules for some specific complementizers,
; that work sometimes, and it seems right now, relex is giving _rel sometimes
; when it should give _comp, which doesn't really make any difference . . .
; (AN June 21015)

(define compmod
    (BindLink
        (VariableList
            (TypedVariableLink
                (VariableNode "$a-parse")
                (TypeNode "ParseNode")
            )
            (TypedVariableLink
                (VariableNode "$pred")
                (TypeNode "WordInstanceNode")
            )
            (TypedVariableLink
                (VariableNode "$comp")
                (TypeNode "WordInstanceNode")
            )
        )
        (AndLink
            (WordInstanceLink
                (VariableNode "$pred")
                (VariableNode "$a-parse")
            )
            (WordInstanceLink
                (VariableNode "$comp")
                (VariableNode "$a-parse")
            )
            (EvaluationLink
                (DefinedLinguisticRelationshipNode "_compmod")
                (ListLink
                    (VariableNode "$pred")
                    (VariableNode "$comp")
                )
            )
        )
        (ExecutionOutputLink
       	   (GroundedSchemaNode "scm: pre-compmod-rule")
       	      (ListLink
       	         (VariableNode "$pred")
		             (VariableNode "$comp")
            )
        )
    )
)

; This is function is not needed. It is added so as not to break the existing
; r2l pipeline.
(define (pre-compmod-rule pred comp)
    (compmod-rule (cog-name (word-inst-get-lemma pred)) (cog-name pred)
		(cog-name (word-inst-get-lemma comp)) (cog-name comp)
	 )
)
