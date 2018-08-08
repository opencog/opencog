;"She sings with an accent." ,

(define adverbialpp
	(BindLink
		(VariableList
			(var-decl "$a-parse" "ParseNode")
			(var-decl "$prep" "WordInstanceNode")
			(var-decl "$noun" "WordInstanceNode")
			(var-decl "$verb" "WordInstanceNode")
		)
		(AndLink
			(word-in-parse "$prep" "$a-parse")
			(word-in-parse "$noun" "$a-parse")
			(word-in-parse "$verb" "$a-parse")
			(dependency "_pobj" "$prep" "$noun")
			(dependency "_advmod" "$verb" "$prep")
		)
		(ExecutionOutputLink
			(GroundedSchemaNode "scm: pre-adverbialpp-rule")
			(ListLink
				(VariableNode "$prep")
				(VariableNode "$noun")
				(VariableNode "$verb")
			)
		)
	)
)


(define-public (pre-adverbialpp-rule prep noun verb)
	(adverbialpp-rule (cog-name (word-inst-get-lemma  verb)) (cog-name verb)
			(cog-name (word-inst-get-lemma prep)) (cog-name prep)
			(cog-name (word-inst-get-lemma noun)) (cog-name noun)
	)
)
