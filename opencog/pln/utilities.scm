
(define-public (pln-load-rules RULE-NAME)
"
  Loads the different variations of the rules known by RULE-NAME.
"
  ; NOTE:
  ; 1.  If a rule needs formula defined in formulas.scm then the rule file
  ;     should load it.
  ; 2. Rule files are assumed to be named as "RULE-NAME-rule.scm"
  ; 3. load-from-path is used so as to be able to use build_dir/opencog/scm,
  ;    even when the module isn't installed.

  (load-from-path (string-append "opencog/pln/rules/"  RULE-NAME "-rule.scm"))
)
