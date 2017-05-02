;
; base-states.scm
;
; Return assorted database statistics
;
; Copyright (c) 2017 Linas Vepstas
;
; ---------------------------------------------------------------------
; The statistics reported here are those collected via the code
; in `link-pipeline.scm` and computed in `sompute-mi.scm`.  Therefore
; structure defintions there and here need to be maintained
; consistently.
; ---------------------------------------------------------------------
;
(use-modules (srfi srfi-1))
(use-modules (opencog))
(use-modules (opencog persist))

; ---------------------------------------------------------------------
(define-public (get-sentence-count)
"
  get-sentence-count -- get the number of sentences observed.
  This does fetch the count from the database. 

  This count is maintained by the link-pipeline code.
"
	(get-count (fetch-atom (SentenceNode "ANY")))
)

(define-public (get-parse-count)
"
  get-parse-count -- get the number of parses observed.
  This does fetch the count from the database. 

  This count is maintained by the link-pipeline code.
"
	(get-count (fetch-atom (ParseNode "ANY")))
)

; ---------------------------------------------------------------------
