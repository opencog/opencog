;
; collect-stats.scm
;
; Process parsed text through the prepositional-triples code.
;
; This will run the preposition-triple rules through the forward
; chainer, and then go through the results, updating the 
; CountTruthValue assoiated with each, and then storing the 
; updated count in the OpenCog persistence backend.
;
; Linas Vepstas April 2009
;


(define triple-list-link (cog-ad-hoc "do-implication" frame-rule-0))
(define triple-list (cog-outgoing-set triple-list-link))


