
(define conj
	(BindLink
		(VariableList
			(var-decl "$a-parse" "ParseNode")
			(var-decl "$var1" "WordInstanceNode")
			(var-decl "$var2" "WordInstanceNode")
			(var-decl "$pos" "DefinedLinguisticConceptNode")
			(var-decl "$var3" "PrepositionalRelationshipNode")
		)
		(AndLink
			(word-in-parse "$var1" "$a-parse")
			(word-in-parse "$var2" "$a-parse")
			(PartOfSpeechLink
				(VariableNode "$var1")
				(VariableNode "$pos")
			)
			(PartOfSpeechLink
				(VariableNode "$var2")
				(VariableNode "$pos")
			)
			(EvaluationLink
				(VariableNode "$var3")
					(ListLink
						(VariableNode "$var1")
						(VariableNode "$var2")
					)
			)
			(OrLink
				(EqualLink
					(VariableNode "$var3")
					(PrepositionalRelationshipNode "conj_and")
				)
				(EqualLink
					(VariableNode "$var3")
					(PrepositionalRelationshipNode "conj_but")
				)
				(EqualLink
					(VariableNode "$var3")
					(PrepositionalRelationshipNode "conj_or")
				)
			))
		(ExecutionOutputLink
			(GroundedSchemaNode "scm: pre-conj-rule")
			(ListLink
				(VariableNode "$var1")
				(VariableNode "$var2")
				(VariableNode "$var3")
				(VariableNode "$pos")
			)
		)
	)
)

(define-public (pre-conj-rule var1 var2 var3 pos)
  (cond
	((string=? (cog-name var3) "conj_and")
		(and-rule (cog-name (word-inst-get-lemma  var1)) (cog-name var1)
			(cog-name (word-inst-get-lemma  var2)) (cog-name var2)
			(cog-name pos)
		)
	)
	((string=? (cog-name var3) "conj_or")
		(or-rule (cog-name (word-inst-get-lemma  var1)) (cog-name var1)
			(cog-name (word-inst-get-lemma  var2)) (cog-name var2)
			(cog-name pos)
		)
	)
	((string=? (cog-name var3) "conj_but")
		(but-rule (cog-name (word-inst-get-lemma  var1)) (cog-name var1)
			(cog-name (word-inst-get-lemma  var2)) (cog-name var2)
			(cog-name pos)
		)
	)
  )
)
