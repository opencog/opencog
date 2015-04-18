;
; buggy-equal.scm
;
; Unit test for github bug report #1520
;
(use-modules (opencog))
(use-modules (opencog query))

; (load-from-path "utilities.scm")

; --------------------------------------------------------------------
; First, some data to populate the atomspace

; Dogs are a kind of cat
(InheritanceLink (ConceptNode "dog") (ConceptNode "cat"))

; Cats are a kind of dog.
(InheritanceLink (ConceptNode "cat") (ConceptNode "dog"))

; Cats and dogs are mammals, mammals are animals
(InheritanceLink (ConceptNode "cat") (ConceptNode "mammal"))
(InheritanceLink (ConceptNode "dog") (ConceptNode "mammal"))
(InheritanceLink (ConceptNode "mammal") (ConceptNode "animal"))


; --------------------------------------------------------------------
;; The Bindlink from bug #1520, with one change:
;; Use NotLink instead of AbsentLink.
;;
;; Absence is looking for the lack of a pattern.
;; Not is inverting the sense of a GPN.  This is what we want here.
;;
;; See also below, `pln-alt` which s the same as this, re-written to
;; use EqualLink instead of the GPN.

(define pln-rule-deduction
    (BindLink
        (VariableList
            (TypedVariableLink
                (VariableNode "$A")
                (TypeNode "ConceptNode")
            )
            (TypedVariableLink
                (VariableNode "$B")
                (TypeNode "ConceptNode")
            )
            (TypedVariableLink
                (VariableNode "$C")
                (TypeNode "ConceptNode")
            )
        )
        (ImplicationLink
            (AndLink
                (InheritanceLink
                    (VariableNode "$A")
                    (VariableNode "$B")
                )
                (InheritanceLink
                    (VariableNode "$B")
                    (VariableNode "$C")
                )
                ; To avoid matching (Inheritance A B) and (Inheritance B A)
                (NotLink
                    (EvaluationLink
                        (GroundedPredicateNode "scm: cog-equal?")
                        (ListLink
                            (VariableNode "$A")
                            (VariableNode "$C")
                        )
                    )
                )
            )
            (ExecutionOutputLink
                (GroundedSchemaNode "scm: pln-xxx")
                (ListLink
                    (InheritanceLink
                        (VariableNode "$A")
                        (VariableNode "$B"))
                    (InheritanceLink
                        (VariableNode "$B")
                        (VariableNode "$C")
                    )
                    ; Don't screw-up in-progress searches
                    ; by using InheritanceLink here.
                    (ListLink
                        (VariableNode "$A")
                        (VariableNode "$C")
                    )
                )
            )
        )
    )
)

; --------------------------------------------------------------------
; Check whether two nodes are equal.
;
; If they are equal then it will return TRUE_TV else it returns
; FALSE_TV.
;
; --------------------------------------------------------------------
(define (cog-equal? atom-1 atom-2)
    (if (equal? atom-1 atom-2)
        (stv 1 1)
        (stv 0 1)
    )
)

; Do nothing except for returning the arguments wrapped in a QuoteLink
(define (pln-xxx a b c) (QuoteLink a b c))

; --------------------------------------------------------------------
; Same as above, but using the built-in EqualLink for atom equality.

(define pln-alt
    (BindLink
        (VariableList
            (TypedVariableLink
                (VariableNode "$A")
                (TypeNode "ConceptNode")
            )
            (TypedVariableLink
                (VariableNode "$B")
                (TypeNode "ConceptNode")
            )
            (TypedVariableLink
                (VariableNode "$C")
                (TypeNode "ConceptNode")
            )
        )
        (ImplicationLink
            (AndLink
                (InheritanceLink
                    (VariableNode "$A")
                    (VariableNode "$B")
                )
                (InheritanceLink
                    (VariableNode "$B")
                    (VariableNode "$C")
                )
                ; To avoid matching (Inheritance A B) and (Inheritance B A)
                (NotLink
                    (EqualLink
                        (VariableNode "$A")
                        (VariableNode "$C")
                    )
                )
            )
            (ExecutionOutputLink
                (GroundedSchemaNode "scm: pln-xxx")
                (ListLink
                    (InheritanceLink
                        (VariableNode "$A")
                        (VariableNode "$B"))
                    (InheritanceLink
                        (VariableNode "$B")
                        (VariableNode "$C")
                    )
                    ; Don't screw-up in-progress searches
                    ; by using InheritanceLink here.
                    (ListLink
                        (VariableNode "$A")
                        (VariableNode "$C")
                    )
                )
            )
        )
    )
)

