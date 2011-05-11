
(define (stv mean conf) (cog-new-stv mean conf))

;; this should not match.
(SetLink (stv 1.0 1.0)
	(NumberNode "0.24")
	(ExecutionOutputLink (stv 1.0 1.0)
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
	(NumberNode "0.24")
	(ExecutionOutputLink (stv 1.0 1.0)
		(GroundedSchemaNode "ActivationModulatorUpdater")
		(ListLink
			(LemmaNode "thing1")
			(LemmaNode "thing3")
		)
	)
)

;; this should not match.
(MemberLink (stv 1.0 1.0)
	(NumberNode "0.24")
	(LatestLink (stv 1.0 1.0)
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
	(NumberNode "0.24")
	(ExecutionOutputLink (stv 1.0 1.0)
		(CWColorNode "Big Red Button")
		(ListLink
			(LemmaNode "thing1")
			(LemmaNode "thing2")
			(LemmaNode "thing3")
		)
	)
)

;; this should not match.
(MemberLink (stv 1.0 1.0)
	(PetNode "here kitty kitty")
	(ExecutionOutputLink (stv 1.0 1.0)
		(GroundedSchemaNode "ActivationModulatorUpdater")
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
	(NumberNode "0.24")
	(ExecutionOutputLink (stv 1.0 1.0)
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
	(NumberNode "0.24")
	(ExecutionOutputLink (stv 1.0 1.0)
		(GroundedSchemaNode "ActivationModulatorUpdater")
		(ListLink
			(LemmaNode "thing2")
			(LemmaNode "thing3")
		)
	)
)

;; this should not match.
(FeatureLink (stv 1.0 1.0)
	(NumberNode "0.24")
	(ExecutionOutputLink (stv 1.0 1.0)
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
	(NumberNode "0.24")
	(ExecutionOutputLink (stv 1.0 1.0)
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
	(ConceptNode "big idea")
	(ExecutionOutputLink (stv 1.0 1.0)
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

(define (bindlink)
	(BindLink
		;; variable decls
		(ListLink
			(TypedVariableLink
				(VariableNode "$var_number")
				(VariableTypeNode "NumberNode")
			)
			(TypedVariableLink
				(VariableNode "$var_schema")
				(VariableTypeNode "GroundedSchemaNode")
			)
		)
		(ImplicationLink
			;; body
			(MemberLink (stv 1.0 1.0)
				(VariableNode "$var_number")
				(ExecutionOutputLink (stv 1.0 1.0)
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
	(NumberNode "0.24")
	(ExecutionOutputLink (stv 1.0 1.0)
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
	(NumberNode "0.24")
	(ExecutionOutputLink (stv 1.0 1.0)
		(GroundedSchemaNode "ActivationModulatorUpdater")
		(ListLink
			(LemmaNode "thing ohh noo")
			(LemmaNode "thing2")
			(LemmaNode "thing3")
		)
	)
)

