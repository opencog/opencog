;; Use the backward chainer to prove that a non negative real number
;; is not a negative real number.
;;
;; The informal reasoning goes as follows
;;
;; 1. The sign of a non negative real number is 0 or 1
;;
;; 2. Therefore the sign is not -1
;;
;; 3. Therefore it is not a negative real number

;; TODO need to have the backward chainer fully supports meta-rules,
;; so that conditional instantiation and contraposition can be
;; back-chained to produce the target. The other option would be to
;; implement contrapositive conditional instantiation, but we won't
;; escape full meta-rule support.

;; Set logger to debug level
(ure-logger-set-level! "debug")

;; The sumo knowledge base (without compiling)
(primitive-load "Merge.scm")

;; Load PLN rule base
(load "pln-config3.scm")

;; Add axioms pertaining to the test
;; KB-test3-1
(MemberLink (stv 1.000000 1.000000)
  (ConceptNode "Number3-1" (stv 0.010000 1.000000)) ; [6502352803782467190][1]
  (ConceptNode "NonnegativeRealNumber" (stv 0.010000 1.000000)) ; [854426703771326157][1]
) ; [12939358011030762013][1]

;; Define taget
(define target
  (NotLink
    (MemberLink
      (ConceptNode "Number3-1" (stv 0.010000 1.000000)) ; [6502352803782467190][1]
      (ConceptNode "NegativeRealNumber" (stv 0.010000 1.000000)) ; [2370306305334504801][1]
    ) ; [14455237612593940657][1]
  ) ; [14475556587474805702][1]
)

;; Prove target
(pln-bc target)
