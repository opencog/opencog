; A RuleBase includes different rules written for different purposes, as well
; as the priority and exclusion relationship among the rules. As such,
; it is a specification of the rules that are to be used in a particular
; context, and applied by a particular control-policy. Thus, in this case, it
; is specifying the rules that are to be used by the R2L pipeline and how they
; have to be applied for the english language.

(InheritanceLink (1, .99) 
    (ConceptNode "R2L-en-RuleBase") 
    (ConceptNode "RuleBase")
)

; This is to avoid repeatitive definition of mutually exclusive rules.
(ContextLink
    (ConceptNode "R2l-en-RuleBase")
    (ForAllLink
        (ListLink
            (VariableNode "$X")
            (VariableNode "$Y")
        )
        (EquivalenceLink
            (EvaluationLink
                (PredicateNode "MutuallyExclusive")
                (ListLink
                    (VariableNode "$X")
                    (VariableNode "$Y")
                )
            )
            (EvaluationLink
                (PredicateNode "MutuallyExclusive")
                (ListLink
                    (VariableNode "$Y")
                    (VariableNode "$X")
                )
            )
        )
    )
)

; --------------------------------------------------------------------
; Specifiyication of the SVO-Rule in the context of R2L-en-
; Examples: "Alice ate the mushroom."

(MemberLink (1, .99) 
    (ConceptNode "SVO-Rule") 
    (ConceptNode "R2L-en-RuleBase") 
)

(ContextLink
    (ConceptNode "R2L-en-RuleBase")
    (EvaluationLink
        (PredicateNode "RulePriority")
        (ListLink
            (ConceptNode "SVO-Rule")
            (NumberNode "2")
        )
    )
)

(ContextLink
    (ConceptNode "R2L-en-RuleBase")
    (EvaluationLink
        (PredicateNode "MutuallyExclusive")
        (ListLink
            (ConceptNode "SVO-Rule")
            (ConceptNode "BE-Rule")
        )
    )
)


; --------------------------------------------------------------------
; Examples: "Socrates is a man", "Cats are animals", "Trees are plants"

(MemberLink (1, .99) 
    (ConceptNode "BE-Rule") 
    (ConceptNode "R2L-en-RuleBase") 
)

(ContextLink
    (ConceptNode "R2L-en-RuleBase")
    (EvaluationLink
        (PredicateNode "RulePriority")
        (ListLink
            (ConceptNode "SVO-Rule")
            (NumberNode "1")
        )
    )
)

