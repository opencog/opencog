;; PLN states

(load "states.scm")

(define pln-answers (Anchor (chat-prefix "PLNAnswers")))
(define pln-qa (Anchor (chat-prefix "PLNQA")))
(define pln-inferred-atoms (Anchor (chat-prefix "PLNInferredAtoms")))
(State pln-answers default-state)
(State pln-qa default-state)
(State pln-inferred-atoms default-state)
