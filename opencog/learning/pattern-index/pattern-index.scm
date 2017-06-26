(define-module (opencog learning pattern-index))
(load-extension "libpatternindex" "opencog_patternindex_init")

(use-modules (opencog))

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
