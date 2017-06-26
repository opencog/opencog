(use-modules (opencog))
(use-modules (opencog nlp pattern-index))

(let (
    (index-key  (pi-create-index (ConceptNode "../opencog/nlp/pattern-index/toy-example-mining.scm")))
) (begin
    (display "Results: ")
    (display (pi-mine-patterns index-key))
))
