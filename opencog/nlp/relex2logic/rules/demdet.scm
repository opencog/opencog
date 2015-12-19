; This rule is for demonstrative determiners such as "that bozo" and "this job."  It used to be called the det rule
; but I changed it because there are lots of other determiners.
; (AN June 2015)


(define demdet
    (BindLink
        (VariableList
			(var-decl "$a-parse" "ParseNode")
			(var-decl "$noun" "WordInstanceNode")
			(var-decl "$demdet" "WordInstanceNode")
        )
        (AndLink
			(word-in-parse "$noun" "$a-parse")
			(word-in-parse "$demdet" "$a-parse")
            (EvaluationLink
               (DefinedLinguisticRelationshipNode "_det")
               (ListLink
                   (VariableNode "$noun")
                   (VariableNode "$demdet")
               )
            )
        )
        (ExecutionOutputLink
       	   (GroundedSchemaNode "scm: pre-demdet-rule")
       	      (ListLink
       	         (VariableNode "$demdet")
       	         (VariableNode "$noun")
            )
        )
    )
)

; This is function is not needed. It is added so as not to break the existing
; r2l pipeline.

;; XXX FIXME ToDo: define demdet-rule
(define (pre-demdet-rule demdet noun)
  (demdet-rule (cog-name (word-inst-get-lemma noun)) (cog-name noun)
	(cog-name (word-inst-get-lemma demdet)) (cog-name demdet)

  )
)
