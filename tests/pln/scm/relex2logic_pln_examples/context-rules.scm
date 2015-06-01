; Examples to test the contextualize and decontextualize rules
; using the pattern matcher
; The rules can be loaded from opencog/learning/engine/pln/rules
; together with this input data.

(EvaluationLink
    (PredicateNode "inputs")
    (ListLink
        ; 1) input for (pln-rule-contextualize-inheritance)
        ; "Ben is competent in doing mathematics."
        (InheritanceLink (stv .8 .8)
            (AndLink
                (ConceptNode "Ben")
                (ConceptNode "doing_mathematics"))
            (AndLink
                (ConceptNode "competent")
                (ConceptNode "doing_mathematics")))
        ; 2) input for (pln-rule-contextualize-evaluation)
        ; "The sky is blue in the context of the earth.
        ; The sky is not blue in the context of the moon."
        (EvaluationLink (stv .8 .7)
            (PredicateNode "isBlue")
            (ListLink
                (AndLink
                    (ConceptNode "sky")
                    (ConceptNode "earth"))))
        (EvaluationLink (stv .0 .2)
            (PredicateNode "isBlue")
            (ListLink
                (AndLink
                    (ConceptNode "sky")
                    (ConceptNode "moon"))))
        ; 3) input for (pln-rule-contextualize-subset)
        ; "Dogs are animals."
        (SubsetLink (stv .8 .8)
            (ConceptNode "dogs")
            (ConceptNode "animals"))
        ; 4) input for (pln-rule-decontextualize-inheritance)
        ; "Ben is competent in the domain of mathematics.
        (ContextLink (stv .8 .8)
            (ConceptNode "doing_mathematics")
            (InheritanceLink
                (ConceptNode "Ben")
                (ConceptNode "competent")))
        ; 5) input for (pln-rule-decontextualize-evaluation)
        ; "In the context of the earth, the sky is blue."
        (ContextLink (stv .8 .8)
            (ConceptNode "earth")
            (EvaluationLink
                (PredicateNode "isBlue")
                (ListLink
                    (ConceptNode "sky"))))
        ; 6) input for (pln-rule-decontextualize-subset)
        ; "Dogs are animals."
        (ContextLink (stv .8 .8)
            (ConceptNode "dogs")
            (ConceptNode "animals"))))

(define check-output
    (BindLink
        (AndLink
            ; 1) output
            (ContextLink (stv .8 .8)
                (ConceptNode "doing_mathematics")
                (InheritanceLink
                    (ConceptNode "Ben")
                    (ConceptNode "competent")))
            ; 2) output 
            (ContextLink (stv .8 .8)
                (ConceptNode "earth")
                (EvaluationLink (stv .8 .7)
                    (PredicateNode "isBlue")
                    (ListLink
                        (ConceptNode "sky"))))
            (ContextLink (stv .8 .8)
                (ConceptNode "moon")
                (EvaluationLink (stv .0 .2)
                    (PredicateNode "isBlue")
                    (ListLink
                        (ConceptNode "sky"))))
            ; 3) output
            (ContextLink (stv .8 .8)
                (ConceptNode "dogs")
                (ConceptNode "animals"))
            ; output of decontextualize-rules is
            ; input 1-3
            ; 4) output                
            (InheritanceLink (stv .8 .8)
                (AndLink
                    (ConceptNode "Ben")
                    (ConceptNode "doing_mathematics"))
                (AndLink
                    (ConceptNode "competent")
                    (ConceptNode "doing_mathematics")))
            ; 5) output
            (EvaluationLink (stv .8 .8)
                (PredicateNode "isBlue")
                (ListLink
                    (AndLink
                        (ConceptNode "sky")
                        (ConceptNode "earth"))))
            ; 6) output
            (SubsetLink (stv .8 .8)
                (ConceptNode "dogs")
                (ConceptNode "animals")))
        (ConceptNode "true")))
