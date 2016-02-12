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

(define advmod
	(advmod-template
		'()
		(ExecutionOutputLink
			(GroundedSchemaNode "scm: pre-advmod-rule")
			(ListLink
				(VariableNode "$verb-lemma")
				(VariableNode "$verb")
				(VariableNode "$adv-lemma")
				(VariableNode "$adv")
			)
		)
	))

; This is function is not needed. It is added so as not to break the existing
; r2l pipeline.
(define (pre-advmod-rule verb-lemma verb adv-lemma adv)
  (cond
	((or (string=? (cog-name adv-lemma) "maybe")
	 (string=? (cog-name adv-lemma) "possibly")
	 (string=? (cog-name adv-lemma) "perhaps")
	 (string=? (cog-name adv-lemma) "probably"))
	 (maybe-rule (cog-name verb-lemma) (cog-name verb))
	 )
  (else
	 (advmod-rule (cog-name verb-lemma) (cog-name verb)
			 (cog-name adv-lemma) (cog-name adv)
	 )
  ))
)
