; Examples in Scheme format; not usable without the cogserver due to
; https://github.com/opencog/opencog/issues/530
; Therefore, these are also defined in the Python code using the rather
; verbose syntax

; Socrates is a man.
; Todo: The Relex2Logic rules require modification to properly translate
; this relationship; see:
; https://github.com/opencog/opencog/issues/530
(EvaluationLink (stv 1.0 1.0)
  (PredicateNode "be")
  (ListLink
    (ConceptNode "Socrates")
    (ConceptNode "man")))

; Men breathe air.
(EvaluationLink (stv 1.0 1.0)
  (PredicateNode "breathe")
  (ListLink
    (ConceptNode "man")
    (ConceptNode "air")))

; Example predicate with only 1 argument
(EvaluationLink (stv 1.0 1.0)
    (PredicateNode "smokes")
    (ListLink
        (ConceptNode "Anna")))
