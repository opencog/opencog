; define concept nodes and elements.
; sub and super must have TVs, or they will be ignored
(define sub (ConceptNode "sub" (stv 0.5 0.999)))
(define super (ConceptNode "super" (stv 0.5 0.999)))
(define e0 (Node "e0"))
(define e1 (Node "e1"))
(define e2 (Node "e2"))
(define e3 (Node "e3"))
(define e4 (Node "e4"))
(define e5 (Node "e5"))
(define e6 (Node "e6"))
(define e7 (Node "e7"))
(define e8 (Node "e8"))
(define e9 (Node "e9"))
; define default confidence
(define dc 0.5)
; define member links of sub
(MemberLink (stv 0.1 dc) e0 sub)
(MemberLink (stv 0.5 dc) e2 sub)
(MemberLink (stv 0.2 dc) e4 sub)
(MemberLink (stv 0.9 dc) e5 sub)
(MemberLink (stv 1.0 dc) e7 sub)
(MemberLink (stv 0.4 dc) e8 sub)
; define member links of super
(MemberLink (stv 0.2 dc) e1 super)
(MemberLink (stv 0.9 dc) e2 super)
(MemberLink (stv 0.8 dc) e3 super)
(MemberLink (stv 0.4 dc) e6 super)
(MemberLink (stv 0.6 dc) e8 super)
(MemberLink (stv 0.0 dc) e9 super)

(define target (SubsetLink sub super))
(EvaluationLink (PredicateNode "query") (ListLink target))
(EvaluationLink (PredicateNode "rules") (ListLink (ConceptNode "SubsetEvaluationRule")))

;(EvaluationLink (stv 0.0 dc) super
;    (ListLink e9)
;)

