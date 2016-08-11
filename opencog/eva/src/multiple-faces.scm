(use-modules (opencog))

; -----------------------------------------------------------------------------
; usage:
;(cog-evaluate! (PutLink
;    (DefinedPredicate "Set face priority")
;    (List (Number "12") (Number ".12"))))
(DefineLink
	(DefinedPredicate "Set face priority")
    (Lambda
        (VariableList (Variable "face-id") (Variable "priority"))
        (True (StateLink
                (List
                    (Concept "visual priority")
                    (Variable "face-id"))
                (Variable "priority")))))
