
;; The matching solution, and the bind link are in the middle 
;; of this file, so that the pattern matcher doesn't accidentally
;; start searching with the correct solution first, just because
;; we loaded it into the atomspace first. Its down in the middle
;; of this file ...

;; Just like stackmore-o-uu.scm except that the top link is unoprderd

;; this should not match.
(InheritanceLink
	(SchemaNode "ActivationModulatorUpdater")
	(NumberNode "0.24")
	(SetLink
		(SchemaNode "ActivationModulatorUpdater")
		(LemmaNode "thing1")
		(LemmaNode "thing2")
		(LemmaNode "thing3")
	)
	(SetLink
		(NumberNode "0.24")
		(NumberNode "1.0")
		(NumberNode "2.0")
		(NumberNode "3.0")
	)
)

;; this should not match.
(SimilarityLink
	(SchemaNode "ActivationModulatorUpdater")
	(NumberNode "0.24")
	(SetLink
		(SchemaNode "ActivationModulatorUpdater")
		(LemmaNode "thing1")
		(LemmaNode "thing3")
	)
	(SetLink
		(NumberNode "0.24")
		(NumberNode "1.0")
		(NumberNode "2.0")
		(NumberNode "3.0")
	)
)

;; this should not match.
(SimilarityLink
	(SchemaNode "ActivationModulatorUpdater")
	(NumberNode "0.24")
	(AtTimeLink
		(SchemaNode "ActivationModulatorUpdater")
		(LemmaNode "thing1")
		(LemmaNode "thing2")
		(LemmaNode "thing3")
	)
	(SetLink
		(NumberNode "0.24")
		(NumberNode "1.0")
		(NumberNode "2.0")
		(NumberNode "3.0")
	)
)

;; this should not match.
(SimilarityLink
	(SchemaNode "ActivationModulatorUpdater")
	(NumberNode "0.24")
	(SetLink
		(PrepositionalRelationshipNode "Big Red Button")
		(LemmaNode "thing1")
		(LemmaNode "thing2")
		(LemmaNode "thing3")
	)
	(SetLink
		(NumberNode "0.24")
		(NumberNode "1.0")
		(NumberNode "2.0")
		(NumberNode "3.0")
	)
)

;; this should not match.
(SimilarityLink
	(SchemaNode "ActivationModulatorUpdater")
	(FeatureNode "here kitty kitty")
	(EvaluationLink
		(SchemaNode "ActivationModulatorUpdater")
		(LemmaNode "thing1")
		(LemmaNode "thing2")
		(LemmaNode "thing3")
	)
	(SetLink
		(NumberNode "0.24")
		(NumberNode "1.0")
		(NumberNode "2.0")
		(NumberNode "3.0")
	)
)

;; this should not match.
(SimilarityLink
	(SchemaNode "ActivationModulatorUpdater")
	(ConceptNode "big idea")
	(SetLink
		(FeatureNode "ActivationModulatorUpdater")
		(LemmaNode "thing1")
		(LemmaNode "thing2")
		(LemmaNode "thing3")
	)
	(SetLink
		(NumberNode "0.24")
		(NumberNode "1.0")
		(NumberNode "2.0")
		(NumberNode "3.0")
	)
)

;; Should match to this, and only this.
;; Put this in the middle of the file, so that its kind-of
;; randomized in the atomspace -- i.e. not at the begining (lowest
;; handle uuid's) nor at the end (highest handle numbers) of the
;; atomspace
(SimilarityLink
	(SchemaNode "ActivationModulatorUpdater")
	(NumberNode "0.24")
	(SetLink
		(SchemaNode "ActivationModulatorUpdater")
		(LemmaNode "thing1")
		(LemmaNode "thing2")
		(LemmaNode "thing3")
	)
	(SetLink
		(NumberNode "0.24")
		(NumberNode "1.0")
		(NumberNode "2.0")
		(NumberNode "3.0")
	)
)

;; this should not match.
(SimilarityLink
	(SchemaNode "ActivationModulatorUpdater")
	(NumberNode "0.24")
	(SetLink
		(SchemaNode "ActivationModulatorUpdater")
		(LemmaNode "thing2")
		(LemmaNode "thing3")
	)
	(SetLink
		(NumberNode "0.24")
		(NumberNode "1.0")
		(NumberNode "2.0")
		(NumberNode "3.0")
	)
)

;; this should not match.
(FeatureLink
	(SchemaNode "ActivationModulatorUpdater")
	(NumberNode "0.24")
	(SetLink
		(SchemaNode "ActivationModulatorUpdater")
		(LemmaNode "thing1")
		(LemmaNode "thing2")
		(LemmaNode "thing3")
	)
	(SetLink
		(NumberNode "0.24")
		(NumberNode "1.0")
		(NumberNode "2.0")
		(NumberNode "3.0")
	)
)

;; this should not match.
(SimilarityLink
	(SchemaNode "ActivationModulatorUpdater")
	(NumberNode "0.24")
	(SetLink
		(WordNode "bird is the word")
		(LemmaNode "thing1")
		(LemmaNode "thing2")
		(LemmaNode "thing3")
	)
	(SetLink
		(NumberNode "0.24")
		(NumberNode "1.0")
		(NumberNode "2.0")
		(NumberNode "3.0")
	)
)

;; this should not match.
(SimilarityLink
	(SemeNode "ActivationModulatorUpdater")
	(ConceptNode "big idea")
	(SetLink
		(SchemaNode "ActivationModulatorUpdater")
		(LemmaNode "thing1")
		(LemmaNode "thing2")
		(LemmaNode "thing3")
	)
	(SetLink
		(NumberNode "0.24")
		(NumberNode "1.0")
		(NumberNode "2.0")
		(NumberNode "3.0")
	)
)

;; this should not match.
(SimilarityLink
	(SchemaNode "ActivationModulatorUpdater")
	(NumberNode "0.24")
	(HebbianLink
		(SchemaNode "ActivationModulatorUpdater")
		(LemmaNode "thing1")
		(LemmaNode "thing2")
		(LemmaNode "thing3")
	)
	(SetLink
		(NumberNode "0.24")
		(NumberNode "1.0")
		(NumberNode "2.0")
		(NumberNode "3.0")
	)
)

(define (bind_uuu)
	(BindLink
		;; variable decls
		(VariableList
			(VariableNode "$var_number")
			(VariableNode "$var_schema")
		)
		(ImplicationLink
			;; body
			(SimilarityLink
				(VariableNode "$var_schema")
				(VariableNode "$var_number")
				(SetLink
					(LemmaNode "thing3")
					(LemmaNode "thing1")
					(VariableNode "$var_schema")
					(LemmaNode "thing2")
				)
				(SetLink
					(NumberNode "2.0")
					(VariableNode "$var_number")
					(NumberNode "3.0")
					(NumberNode "1.0")
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
(SubsetLink
	(SchemaNode "ActivationModulatorUpdater")
	(NumberNode "0.24")
	(SetLink
		(SchemaNode "ActivationModulatorUpdater")
		(LemmaNode "thing1")
		(LemmaNode "thing2")
		(LemmaNode "thing3")
	)
	(SetLink
		(NumberNode "0.24")
		(NumberNode "1.0")
		(NumberNode "2.0")
		(NumberNode "3.0")
	)
)

;; this should not match.
(SimilarityLink
	(WordNode "bird is the word")
	(NumberNode "0.24")
	(SetLink
		(SchemaNode "ActivationModulatorUpdater")
		(LemmaNode "thing1")
		(LemmaNode "thing2")
		(LemmaNode "thing3")
	)
	(SetLink
		(NumberNode "0.24")
		(NumberNode "1.0")
		(NumberNode "2.0")
		(NumberNode "3.0")
	)
)

;; this should not match.
(SimilarityLink
	(SchemaNode "ActivationModulatorUpdater")
	(NumberNode "0.24")
	(ParseLink
		(SchemaNode "ActivationModulatorUpdater")
		(LemmaNode "thing1")
		(LemmaNode "thing2")
		(LemmaNode "thing3")
	)
	(SetLink
		(NumberNode "0.24")
		(NumberNode "1.0")
		(NumberNode "2.0")
		(NumberNode "3.0")
	)
)

;; this should not match.
(SimilarityLink
	(SchemaNode "ActivationModulatorUpdater")
	(NumberNode "0.24")
	(SetLink
		(SchemaNode "ActivationModulatorUpdater")
		(LemmaNode "thing ohh noo")
		(LemmaNode "thing2")
		(LemmaNode "thing3")
	)
	(SetLink
		(NumberNode "0.24")
		(NumberNode "1.0")
		(NumberNode "2.0")
		(NumberNode "3.0")
	)
)

;; this should not match.
(SimilarityLink
	(PrepositionalRelationshipNode "Big Red Button")
	(NumberNode "0.24")
	(SetLink
		(SchemaNode "ActivationModulatorUpdater")
		(LemmaNode "thing1")
		(LemmaNode "thing2")
		(LemmaNode "thing3")
	)
	(SetLink
		(NumberNode "0.24")
		(NumberNode "1.0")
		(NumberNode "2.0")
		(NumberNode "3.0")
	)
)

