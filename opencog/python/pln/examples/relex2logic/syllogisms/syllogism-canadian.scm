; All Canadians are right-handed.
; All right-handed are opticians.
; |- Some right-handed people are opticians.

(EvaluationLink (PredicateNode "inputs")
    (ListLink
        ; All Canadians are right-handed.
        (ForAllLink (stv 1.000000 0.000000) ; tv needs to be set properly
            (VariableNode "$X") ; [2040]
            (ImplicationLink (stv 0.990000 0.990000)
                (InheritanceLink (stv 0.990000 0.990000)
                    (VariableNode "$X") ; [2040]
                    (ConceptNode "Canadians@c5669c61-1069-478c-abb9-aaa69c4f2217") ; [2041]
                ) ; [2042]
                (InheritanceLink (stv 0.990000 0.990000)
                    (VariableNode "$X") ; [2040]
                    (ConceptNode "right-handed@54ed2540-87ba-4c5f-9cce-02053ea91857") ; [2043]
                ) ; [2044]
            ) ; [2045]
        ) ; [2046]
        (InheritanceLink (stv 0.990000 0.990000)
            (ConceptNode "right-handed@54ed2540-87ba-4c5f-9cce-02053ea91857") ; [2043]
            (ConceptNode "right-handed") ; [2047]
        ) ; [2048]
        (InheritanceLink (stv 0.990000 0.990000)
            (ConceptNode "Canadians@c5669c61-1069-478c-abb9-aaa69c4f2217") ; [2041]
            (ConceptNode "Canadian") ; [2049]
        ) ; [2050]
        ; All right-handed people are opticians.
        ; (needs another r2l all-rule to generate appropriate representation.)
        ; (exact representation is still up for discussion)
        (ForAllLink (stv 1.000000 0.000000) ; tv needs to be set properly
            (VariableNode "$X") ; [2040]
            (ImplicationLink (stv 0.990000 0.990000)
                (InheritanceLink (stv 0.990000 0.990000)
                    (VariableNode "$X") ; [2040]
                    (AndLink
                        (ConceptNode "people@AAA"); [2041]
                        (ConceptNode "right-handed@AAA")
                    )
                ) ; [2042]
                (InheritanceLink (stv 0.990000 0.990000)
                    (VariableNode "$X") ; [2040]
                    (ConceptNode "opticians@AAA") ; [2043]
                ) ; [2044]
            ) ; [2045]
        ) ; [2046]
        (InheritanceLink (stv 0.990000 0.990000)
            (ConceptNode "people@AAA")
            (ConceptNode "people")
        )
        (InheritanceLink (stv 0.990000 0.990000)
            (ConceptNode "right-handed@AAA")
            (ConceptNode "right-handed")
        )
        (InheritanceLink (stv 0.990000 0.990000)
            (ConceptNode "opticians@AAA")
            (ConceptNode "optician")
        )
        ; Besides that, the system also needs to know that
        ; Canadians are people.
        (InheritanceLink (stv 0.990000 0.990000)
            (ConceptNode "Canadian")
            (ConceptNode "people")
        )

    )
)

(EvaluationLink (PredicateNode "rules")
	(ListLink
		(ConceptNode "ModusPonensRule:ImplicationLink")
		; rule to handle ForAllRule is needed
	)
)

(EvaluationLink (PredicateNode "desired_outputs")
	(ListLink
	    ; "Some right-handed people are opticians"
	    (EvaluationLink (stv 0.99 0.99)
	        (PredicateNode "some")
	        (ListLink
	            (AndLink
	                (InheritanceLink
	                    (ConceptNode "people")
	                    (ConceptNode "right-handed")
	                )
	                (InheritanceLink
	                    (ConceptNode "people")
	                    (ConceptNode "optician")
	                )
	            )
	        )
	    )
	)
)
