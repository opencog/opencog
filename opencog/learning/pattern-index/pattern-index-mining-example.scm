(use-modules (opencog))
(use-modules (opencog learning pattern-index))

(let (
    (index-key  (pi-create-index (ConceptNode "../opencog/learning/pattern-index/toy-example-mining.scm")))
) (begin
    (display "Results: ")
    (display (pi-mine-patterns index-key))
))
