(define-module (opencog learning pattern-index))

(use-modules (opencog oc-config))
(load-extension (string-append opencog-ext-path-pattern-index "libpatternindex") "opencog_patternindex_init")

(define-public (pi-create-index path)
    (scm-api-create-index path) ; Defined in PatternIndexSCM.cc
)

(define-public (pi-create-index-scm path)
    (scm-api-create-index-scm path) ; Defined in PatternIndexSCM.cc
)

(define-public (pi-query index-key query-link)
    (scm-api-query index-key query-link) ; Defined in PatternIndexSCM.cc
)

(define-public (pi-mine-patterns index-key)
    (scm-api-mine-patterns index-key) ; Defined in PatternIndexSCM.cc
)
