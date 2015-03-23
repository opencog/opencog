
(define (stv mean conf) (cog-new-stv mean conf))
;;
;; The matching solution, and the bind link are in the middle 
;; of this file, so that the pattern matcher doesn't accidentally
;; start searching with the correct solution first, just because
;; we loaded it into the atomspace first. Its down in the middle
;; of this file ...

;; this should not match.
(InheritanceLink (stv 1.0 1.0)
	(GroundedSchemaNode "ActivationModulatorUpdater")
	(NumberNode "0.24")
	(SetLink (stv 1.0 1.0)
		(GroundedSchemaNode "ActivationModulatorUpdater")
		(ListLink
			(LemmaNode "thing1")
			(LemmaNode "thing2")
			(LemmaNode "thing3")
		)
	)
)

;; this should not match.
(MemberLink (stv 1.0 1.0)
	(GroundedSchemaNode "ActivationModulatorUpdater")
	(NumberNode "0.24")
	(SetLink (stv 1.0 1.0)
		(GroundedSchemaNode "ActivationModulatorUpdater")
		(ListLink
			(LemmaNode "thing1")
			(LemmaNode "thing3")
		)
	)
)

;; this should not match.
(MemberLink (stv 1.0 1.0)
	(GroundedSchemaNode "ActivationModulatorUpdater")
	(NumberNode "0.24")
	(AtTimeLink (stv 1.0 1.0)
		(GroundedSchemaNode "ActivationModulatorUpdater")
		(ListLink
			(LemmaNode "thing1")
			(LemmaNode "thing2")
			(LemmaNode "thing3")
		)
	)
)

;; this should not match.
(MemberLink (stv 1.0 1.0)
	(GroundedSchemaNode "ActivationModulatorUpdater")
	(NumberNode "0.24")
	(SetLink (stv 1.0 1.0)
		(PrepositionalRelationshipNode "Big Red Button")
		(ListLink
			(LemmaNode "thing1")
			(LemmaNode "thing2")
			(LemmaNode "thing3")
		)
	)
)

;; this should not match.
(MemberLink (stv 1.0 1.0)
	(GroundedSchemaNode "ActivationModulatorUpdater")
	(FeatureNode "here kitty kitty")
	(EvaluationLink (stv 1.0 1.0)
		(GroundedSchemaNode "ActivationModulatorUpdater")
		(ListLink
			(LemmaNode "thing1")
			(LemmaNode "thing2")
			(LemmaNode "thing3")
		)
	)
)

;; this should not match.
(MemberLink (stv 1.0 1.0)
	(GroundedSchemaNode "ActivationModulatorUpdater")
	(ConceptNode "big idea")
	(SetLink (stv 1.0 1.0)
		(FeatureNode "ActivationModulatorUpdater")
		(ListLink
			(LemmaNode "thing1")
			(LemmaNode "thing2")
			(LemmaNode "thing3")
		)
	)
)

;; Should match to this, and only this.
;; Put this in the middle of the file, so that its kind-of
;; randomized in the atomspace -- i.e. not at the begining (lowest
;; handle uuid's) nor at the end (highest handle numbers) of the
;; atomspace
(MemberLink (stv 1.0 1.0)
	(GroundedSchemaNode "ActivationModulatorUpdater")
	(NumberNode "0.24")
	(SetLink (stv 1.0 1.0)
		(GroundedSchemaNode "ActivationModulatorUpdater")
		(ListLink
			(LemmaNode "thing1")
			(LemmaNode "thing2")
			(LemmaNode "thing3")
		)
	)
)

;; this should not match.
(MemberLink (stv 1.0 1.0)
	(GroundedSchemaNode "ActivationModulatorUpdater")
	(NumberNode "0.24")
	(SetLink (stv 1.0 1.0)
		(GroundedSchemaNode "ActivationModulatorUpdater")
		(ListLink
			(LemmaNode "thing2")
			(LemmaNode "thing3")
		)
	)
)

;; this should not match.
(FeatureLink (stv 1.0 1.0)
	(GroundedSchemaNode "ActivationModulatorUpdater")
	(NumberNode "0.24")
	(SetLink (stv 1.0 1.0)
		(GroundedSchemaNode "ActivationModulatorUpdater")
		(ListLink
			(LemmaNode "thing1")
			(LemmaNode "thing2")
			(LemmaNode "thing3")
		)
	)
)

;; this should not match.
(MemberLink (stv 1.0 1.0)
	(GroundedSchemaNode "ActivationModulatorUpdater")
	(NumberNode "0.24")
	(SetLink (stv 1.0 1.0)
		(WordNode "bird is the word")
		(ListLink
			(LemmaNode "thing1")
			(LemmaNode "thing2")
			(LemmaNode "thing3")
		)
	)
)

;; this should not match.
(MemberLink (stv 1.0 1.0)
	(SemeNode "ActivationModulatorUpdater")
	(ConceptNode "big idea")
	(SetLink (stv 1.0 1.0)
		(GroundedSchemaNode "ActivationModulatorUpdater")
		(ListLink
			(LemmaNode "thing1")
			(LemmaNode "thing2")
			(LemmaNode "thing3")
		)
	)
)

;; this should not match.
(MemberLink (stv 1.0 1.0)
	(GroundedSchemaNode "ActivationModulatorUpdater")
	(NumberNode "0.24")
	(HebbianLink (stv 1.0 1.0)
		(GroundedSchemaNode "ActivationModulatorUpdater")
		(ListLink
			(LemmaNode "thing1")
			(LemmaNode "thing2")
			(LemmaNode "thing3")
		)
	)
)

(define (bind_ou)
	(BindLink
		;; variable decls
		(ListLink
			(VariableNode "$var_number")
			(VariableNode "$var_schema")
		)
		(ImplicationLink
			;; body
			(MemberLink (stv 1.0 1.0)
				(VariableNode "$var_schema")
				(VariableNode "$var_number")
				(SetLink (stv 1.0 1.0)
					(VariableNode "$var_schema")
					(ListLink
						(LemmaNode "thing1")
						(LemmaNode "thing2")
						(LemmaNode "thing3")
					)
				)
			)
			;; implicand -- result
			(ListLink
				(VariableNode "$var_number")
				(VariableNode "$var_schema")
			)
		)
	)
)

;; this should not match.
(SubsetLink (stv 1.0 1.0)
	(GroundedSchemaNode "ActivationModulatorUpdater")
	(NumberNode "0.24")
	(SetLink (stv 1.0 1.0)
		(GroundedSchemaNode "ActivationModulatorUpdater")
		(ListLink
			(LemmaNode "thing1")
			(LemmaNode "thing2")
			(LemmaNode "thing3")
		)
	)
)

;; this should not match.
(MemberLink (stv 1.0 1.0)
	(WordNode "bird is the word")
	(NumberNode "0.24")
	(SetLink (stv 1.0 1.0)
		(GroundedSchemaNode "ActivationModulatorUpdater")
		(ListLink
			(LemmaNode "thing1")
			(LemmaNode "thing2")
			(LemmaNode "thing3")
		)
	)
)

;; this should not match.
(MemberLink (stv 1.0 1.0)
	(GroundedSchemaNode "ActivationModulatorUpdater")
	(NumberNode "0.24")
	(ParseLink (stv 1.0 1.0)
		(GroundedSchemaNode "ActivationModulatorUpdater")
		(ListLink
			(LemmaNode "thing1")
			(LemmaNode "thing2")
			(LemmaNode "thing3")
		)
	)
)

;; this should not match.
(MemberLink (stv 1.0 1.0)
	(GroundedSchemaNode "ActivationModulatorUpdater")
	(NumberNode "0.24")
	(SetLink (stv 1.0 1.0)
		(GroundedSchemaNode "ActivationModulatorUpdater")
		(ListLink
			(LemmaNode "thing ohh noo")
			(LemmaNode "thing2")
			(LemmaNode "thing3")
		)
	)
)

;; this should not match.
(MemberLink (stv 1.0 1.0)
	(PrepositionalRelationshipNode "Big Red Button")
	(NumberNode "0.24")
	(SetLink (stv 1.0 1.0)
		(GroundedSchemaNode "ActivationModulatorUpdater")
		(ListLink
			(LemmaNode "thing1")
			(LemmaNode "thing2")
			(LemmaNode "thing3")
		)
	)
)

