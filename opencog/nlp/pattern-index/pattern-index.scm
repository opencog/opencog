(define-module (opencog nlp pattern-index))
(load-extension "libpatternindex" "opencog_patternindex_init")

(use-modules (opencog))

(define-public (create-new-index path)
    (scm-api-create-new-index path) ; Defined in PatternIndexSCM.cc
)

(define-public (query index-key query-link)
    (scm-api-query index-key query-link) ; Defined in PatternIndexSCM.cc
)

(define-public (mine-patterns index-key)
    (scm-api-mine-patterns index-key) ; Defined in PatternIndexSCM.cc
)
