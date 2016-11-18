
(define-public (pln-load-rules RULE-NAME)
"
  Loads the different variations of the rules known by RULE-NAME.
"
  ; NOTE:
  ; 1.  If a rule needs formula defined in formulas.scm then the rule file
  ;     should load it.
  ; 2. Rule files are assumed to be named as "RULE-NAME-rule.scm"
  ; 3. load-from-path is used so as to be able to use build_dir/opencog/scm,
  ;    even when the module isn't installed.

  (load-from-path (string-append "opencog/pln/rules/"  RULE-NAME "-rule.scm"))
)

; ----------------------------------------------------------------------------
(define-public (filter-for-pln a-list)
"
  Takes a list of atoms and return a SetLink containing atoms that pln can
  reason on. Some of the patterns that make the returned SetLink are,
          (Inheritance
              (TypeChoice
                  (Type \"ConceptNode\")
                  (Type \"SatisfyingSetLink\"))
              (TypeChoice
                  (Type \"ConceptNode\")
                  (Type \"SatisfyingSetLink\")))
          (Evaluation
              (Type \"PredicateNode\")
              (ListLink
                  (Type \"ConceptNode\")
                  (Type \"ConceptNode\")))
          (Evaluation
              (Type \"PredicateNode\")
              (ListLink
                  (Type \"ConceptNode\")))
          (Member
              (TypeChoice
                  (Type \"ConceptNode\")
                  (Type \"SatisfyingSetLink\"))
              (TypeChoice
                  (Type \"ConceptNode\")
                  (Type \"SatisfyingSetLink\")))
          (Implication
              (Type \"PredicateNode\")
              (Type \"PredicateNode\"))

  a-list:
  - This is a list of atoms, for example a list of r2l outputs
"
; TODO: Move this to an (opencog pln) module, when there is one.
    (define filter-in-pattern
        (ScopeLink
            (TypedVariable
                (Variable "$x")
                (TypeChoice
                    (Signature
                        (Inheritance
                            (TypeChoice
                                (Type "ConceptNode")
                                (Type "SatisfyingSetLink"))
                            (TypeChoice
                                (Type "ConceptNode")
                                (Type "SatisfyingSetLink"))))
                    (Signature
                        (Implication
                            (Type "PredicateNode")
                            (Type "PredicateNode")))
                    (Signature
                        (Member
                            (TypeChoice
                                (Type "ConceptNode")
                                (Type "SatisfyingSetLink"))
                            (TypeChoice
                                (Type "ConceptNode")
                                (Type "SatisfyingSetLink"))))
                    (Signature
                        (Evaluation
                            (Type "PredicateNode")
                            (ListLink
                                (Type "ConceptNode")
                                (Type "ConceptNode"))))
                    (Signature
                        (Evaluation
                            (Type "PredicateNode")
                            (ListLink
                                (Type "ConceptNode"))))
                ))
            ; Return atoms with the given signatures
            (Variable "$x")
        ))

    (define filter-from (SetLink  a-list))

    ; Do the filtering
    (define result (cog-execute! (MapLink filter-in-pattern filter-from)))

    ; Cleanup garbage
    (cog-delete-recursive filter-from)

    result
)

; ----------------------------------------------------------------------------
