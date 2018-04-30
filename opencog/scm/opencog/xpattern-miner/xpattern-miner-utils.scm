;;
;; xpattern-miner-utils.scm
;;
;;;; Commentary:
;;
;; Handy utilities for working with the ure pattern miner. In
;; particular to configure the rule engine.
;;
;; Utilities include:
;;
;; If you add more utilities don't forget to add them in the
;; export-xpattern-miner-utils function.
;;
;;;; Code:
;; Copyright (c) 2018, OpenCog Foundation
;;

(use-modules (opencog))
(use-modules (opencog exec))
(use-modules (srfi srfi-1))

(define (fill-texts-cpt texts-cpt texts)
"
  For each element, text, of texts create

  MemberLink
    text
    texts-cpt
"
  (let ((mk-member (lambda (text) (Member text texts-cpt))))
    (for-each mk-member texts)))

(define (cog-mine texts-cpt minsup)
"
  
")

(define (export-rule-engine-utils)
  (export
    fill-texts-cpt
    cog-mine
  )
)
