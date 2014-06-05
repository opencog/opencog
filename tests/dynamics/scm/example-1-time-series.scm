; This demonstrates the expected time series dynamical evolution of Example #1

; t=0
(EvaluationLink (stv 1 0.99999982)
   (PredicateNode "smokes" (av 1000 0 0))
   (ListLink
      (ConceptNode "Anna" (stv 0.1 0.99999982))))

; t=1
(EvaluationLink (stv 1 0.99999982) (av 600 0 0)
   (PredicateNode "smokes" (av 400 0 0))
   (ListLink
      (ConceptNode "Anna" (stv 0.1 0.99999982))))

; t=2
(EvaluationLink (stv 1 0.99999982) (av 480 0 0)
   (PredicateNode "smokes" (av 340 0 0))
   (ListLink (av 180 0 0)
      (ConceptNode "Anna" (stv 0.1 0.99999982))))

; t=3
(EvaluationLink (stv 1 0.99999982) (av 450 0 0)
   (PredicateNode "smokes" (av 280 0 0))
   (ListLink (av 216 0 0)
      (ConceptNode "Anna" (stv 0.1 0.99999982) (av 54 0 0))))

; t=4
(EvaluationLink (stv 1 0.99999982) (av 413 0 0)
   (PredicateNode "smokes" (av 247 0 0))
   (ListLink (av 253 0 0)
      (ConceptNode "Anna" (stv 0.1 0.99999982) (av 87 0 0))))

; t=5
(EvaluationLink (stv 1 0.99999982) (av 389 0 0)
   (PredicateNode "smokes" (av 223 0 0))
   (ListLink (av 277 0 0)
      (ConceptNode "Anna" (stv 0.1 0.99999982) (av 111 0 0))))

; t=6
(EvaluationLink (stv 1 0.99999982) (av 361 0 0)
   (PredicateNode "smokes" (av 194 0 0))
   (ListLink (av 306 0 0)
      (ConceptNode "Anna" (stv 0.1 0.99999982) (av 139 0 0))))

; t=7
(EvaluationLink (stv 1 0.99999982) (av 353 0 0)
   (PredicateNode "smokes" (av 186 0 0))
   (ListLink (av 313 0 0)
      (ConceptNode "Anna" (stv 0.1 0.99999982) (av 148 0 0))))

; t=8
(EvaluationLink (stv 1 0.99999982) (av 347 0 0)
   (PredicateNode "smokes" (av 180 0 0))
   (ListLink (av 320 0 0)
      (ConceptNode "Anna" (stv 0.1 0.99999982) (av 153 0 0))))

; t=9
(EvaluationLink (stv 1 0.99999982) (av 343 0 0)
   (PredicateNode "smokes" (av 176 0 0))
   (ListLink (av 324 0 0)
      (ConceptNode "Anna" (stv 0.1 0.99999982) (av 157 0 0))))

; t=10
(EvaluationLink (stv 1 0.99999982) (av 340 0 0)
   (PredicateNode "smokes" (av 173 0 0))
   (ListLink (av 327 0 0)
      (ConceptNode "Anna" (stv 0.1 0.99999982) (av 160 0 0))))
