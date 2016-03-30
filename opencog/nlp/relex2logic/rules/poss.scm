; This rule is for nouns which are posessed by words marked as posessives by relex,
; such as "my doggy" or "your mama"
; (AN June 2015)

(define poss
    (BindLink
        (VariableList
			(var-decl "$a-parse" "ParseNode")
			(var-decl "$noun" "WordInstanceNode")
			(var-decl "$poss" "WordInstanceNode")
        )
        (AndLink
			(word-in-parse "$noun" "$a-parse")
			(word-in-parse "$poss" "$a-parse")
			(dependency "_poss" "$noun" "$poss")
        )
        (ExecutionOutputLink
       	   (GroundedSchemaNode "scm: pre-poss-rule")
       	      (ListLink
       	         (VariableNode "$noun")
       	         (VariableNode "$poss")
            )
        )
    )
)

; This is function is not needed. It is added so as not to break the existing
; r2l pipeline.
(define-public (pre-poss-rule noun poss)
    (possessive-rule (cog-name (word-inst-get-lemma noun)) (cog-name noun)
              (cog-name (word-inst-get-lemma poss)) (cog-name poss)
    )
)
