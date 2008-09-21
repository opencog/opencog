scm
;
; cog-stream.scm
;
; Streams and stream-processors of cog atoms. See wiring.scm for details.
;
; Copyright (c) 2008 Linas Vepstas <linasvepstas@gmail.com>
;

; Place all atoms in the atomspace, of type 'atom-type', onto wire
(define (cog-wire-atoms wire atom-type)

	(wire-source-list wire (cog-get-atoms atom-type))
)


.
exit
