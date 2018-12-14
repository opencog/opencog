(use-modules (srfi srfi-1))

(use-modules (opencog))
(use-modules (opencog exec))
(use-modules (opencog rule-engine))

; ----------------------------------------------------------------------------
(define-public (pln-load-rules RULE-TYPE)
"
  pln-load-rules RULE-TYPE

  Loads the different variations of the rules known by RULE-TYPE. A RULE-TYPE
  may include the categorization of the rule. For example, 'term/deduction'
  implies that the rule to be loaded is the term-logic deduction rule.
"
  ; NOTE:
  ; 1. If a rule needs formula defined in formulas.scm then the rule file
  ;    should load it.
  ; 2. Rule files are assumed to be named as "RULE-TYPE.scm"
  ; 3. load-from-path is used so as to be able to use build_dir/opencog/scm,
  ;    even when the module isn't installed.

  (load-from-path (string-append "opencog/pln/rules/" RULE-TYPE ".scm"))
)
