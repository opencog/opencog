; NB: there seems to be a bit of a bug in Relex for assigning multiple
; adjectives to the same noun, which works partially -- it seems to get
; some of the multiple adjectives but not necessarily all of them.
;
; This rule treats adjectival phrases as well as adjectives, because
; I rigged relex that way.  So, amod handles "The tall man" and
; "the man in the hat."  'in the hat' is an adjectival phrase here.
; this is why I changed the individual triadic prepositional
; relations into two rules, one to assign the role of the prepositional
; phrase and the other to assign the object of the preposition.
; (AN June 2015)

(define amod
    (BindLink
        (VariableList
			(var-decl "$a-parse" "ParseNode")
			(var-decl "$noun" "WordInstanceNode")
			(var-decl "$adj" "WordInstanceNode")
        )
        (AndLink
			(word-in-parse "$noun" "$a-parse")
			(word-in-parse "$adj" "$a-parse")
			(dependency "_amod" "$noun" "$adj")
        )
        (ExecutionOutputLink
       	   (GroundedSchemaNode "scm: pre-amod-rule")
       	      (ListLink
       	         (VariableNode "$noun")
       	         (VariableNode "$adj")
            )
        )
    )
)

; This is function is not needed. It is added so as not to break the existing
; r2l pipeline.
(define-public (pre-amod-rule noun adj)
    (amod-rule (cog-name (word-inst-get-lemma  noun)) (cog-name noun)
              (cog-name (word-inst-get-lemma  adj)) (cog-name adj)
    )
)
