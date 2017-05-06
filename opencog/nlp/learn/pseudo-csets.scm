;
; pseudo-csets.scm
;
; Compute the cosine distance between two pseuo-connector-sets.
;
; Copyright (c) 2017 Linas Vepstas
;
; ---------------------------------------------------------------------
; OVERVIEW
; --------
; The scripts below compute the cosine-distance between pseudo
; connector-set vectors.
;
; An example connector-set, for the word "playing", illustrating
; that it can connect to the word "level" on the left, and "field"
; on the right, is shown below. In terms of ENglish grammar, it is
; a bad example, because English should not connect this way. But
; it is an example...
;
;    (LgWordCset
;       (WordNode "playing")
;       (LgAnd
;          (PseudoConnector
;             (WordNode "level")
;             (LgConnDirNode "-"))
;          (PseudoConnector
;             (WordNode "field")
;             (LgConnDirNode "+"))))
;
; Any given word may have dozens or hundreds or thousands of these
; connector sets. The totality of these sets, for a given, fixed word
; form a vector.  The `LgAnd` is a basis element, and the raw
; observational count on the `LgWordCset` is the magnitude of the
; of the vector in that basis direction.
;
; As vectors, dot-products can be taken. The most interesting of these
; is the cosine distance between two words. This distance indicates how
; similar two words are, grammatically-speaking. 
;
; ---------------------------------------------------------------------
;
(use-modules (opencog))
(use-modules (opencog persist))

; ---------------------------------------------------------------------
; ---------------------------------------------------------------------

(define-public (fetch-pseudo-connectors)
"
  fetch-pseudo-cpnnectors - fetch all pseudo-connectors for all
  WordNodes from the database backend.
"
	(define start-time (current-time))
	(load-atoms-of-type 'WordNode)
	(format #t "Elapsed time to load words: ~A secs\n"
		(- (current-time) start-time))
)

; ---------------------------------------------------------------------
