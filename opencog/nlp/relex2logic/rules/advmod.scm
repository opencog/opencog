; This handles adverbs and many adverbial phrases, because I rigged
; relex to treat a lot of adverbial phrases as adverbs.  So, for
; example, "She sings well" is treated the same way as "She sings with
; an accent."  'with an accent' is treated as an adverb.  This is why
; I changed the individual triadic prepositional relations into two
; rules each -- one to assign the preposition as an adverbial or
; adjectival or predicate, and the other to assign the object of the
; preposition. -- (AN June 2015)

(define (advmod-template EXTRA-STUFF RESULT)
	(BindLink
		(VariableList
			(var-decl "$a-parse" "ParseNode")
			(var-decl "$verb" "WordInstanceNode")
			(var-decl "$adv" "WordInstanceNode")
			(var-decl "$verb-lemma" "WordNode")
			(var-decl "$adv-lemma" "WordNode")
		)
		(AndLink
			(word-in-parse "$verb" "$a-parse")
			(word-in-parse "$adv" "$a-parse")
			(dependency "_advmod" "$verb" "$adv")
			(word-lemma "$verb" "$verb-lemma")
			(word-lemma "$adv" "$adv-lemma")
			EXTRA-STUFF
		)
		RESULT
	)
)

(define advmod-maybe
	(advmod-template
		(ChoiceLink
			(Lemma (Variable "$adv") (Word "maybe"))
			(Lemma (Variable "$adv") (Word "possibly"))
			(Lemma (Variable "$adv") (Word "perhaps"))
			(Lemma (Variable "$adv") (Word "probably"))
		)
		(ExecutionOutputLink
			(GroundedSchemaNode "scm: maybe-rule")
			(ListLink
				(VariableNode "$verb-lemma")
				(VariableNode "$verb")
			)
		)
	))

(define advmod
	(advmod-template
		(list
			(Absent (Lemma (Variable "$adv") (Word "maybe")))
			(Absent (Lemma (Variable "$adv") (Word "possibly")))
			(Absent (Lemma (Variable "$adv") (Word "perhaps")))
			(Absent (Lemma (Variable "$adv") (Word "probably")))
		)
		(ExecutionOutputLink
			(GroundedSchemaNode "scm: advmod-rule")
			(ListLink
				(VariableNode "$verb-lemma")
				(VariableNode "$verb")
				(VariableNode "$adv-lemma")
				(VariableNode "$adv")
			)
		)
	))
