#!/usr/bin/env sh
export LTDL_LIBRARY_PATH=../../build/opencog/guile
cat - << XXX | /usr/bin/env guile
;;;
;;; The above is a hacky work-around to a weird linux bug.  The below
;;; works, but does not set the library search path.
;;; #!/usr/bin/env guile
;;; !#
;;;
;;; The below hangs; it should not. See
;;; https://bugs.launchpad.net/ubuntu/+source/coreutils/+bug/1421760
;;; #!/usr/bin/env LTDL_LIBRARY_PATH=../../build/opencog/guile guile
;
; Basic guile atom creation benchmark.
;
; See opencog/guile/README or http://wiki.opencog.org/w/Scheme
; for additional documentation.
;
; To run this, say:
;
; $ export LTDL_LIBRARY_PATH=build/opencog/guile
; $ ./gperf.scm
;
; where "build" is where-ever you built opencog.

; Add search paths for the opencog module.
(add-to-load-path "../../bin")
(add-to-load-path "../../build")
(add-to-load-path "../../opencog/scm")

(use-modules (opencog))
(use-modules (ice-9 format))
(use-modules (srfi srfi-19))

; Define a function that creates a link with two nodes in it.
; Things that waste CPU time here are:
; 1) conversion of number to string
; 2) concatenation of two strings
; 3) creation of the ConceptNode in the AtomSpace
; 4) creation of the ListLink in the AtomSpace.
; 5) tail recursion, to loop and repeat the process again.
(define (make-node-and-link prefix n)
	(define (make-atoms prefix node n)
		(define cpt (ConceptNode (string-append/shared prefix (number->string n))))
		; Hmmm.. formatted printing is kind-of slow.
		; (ConceptNode (format #f "Object_~d" n))
		(ListLink node cpt)
		(if (< 0 n)
			(make-atoms prefix cpt (- n 1))
			'()
		)
	)
	(make-atoms prefix (ConceptNode prefix) n)
)

; Define a function that creates large tree of links.
; We expect this to run much faster than the above, because it does a
; lot less.  Things that waste CPU time here are:
; 1) creation of the ListLink in the AtomSpace.
; 2) tail recursion, to loop and repeat the process again.
(define (make-link-tree n)
	(define (make-atoms atom n)
		(define linky (ListLink atom atom))
		(if (< 0 n)
			(make-atoms linky (- n 1))
			'()
		)
	)
	(make-atoms (ConceptNode "nil") n)
)

; A handy utility to report the elapsed time and the rate.
(define (report-perf id start stop niter)
	(define elapsed (time-difference stop start))
	(define delta 
		(+ (time-second elapsed) 
			(/ (time-nanosecond elapsed) 1000000000.0)))
	(define rate (round (/ niter delta)))
	(display id) (newline)
	(display "Elapsed time (secs): ") (display delta) (newline)
	(display "Loops per second: ") (display rate) (newline)
	(newline)
)


; Measure how long it takes to create a bunch of links.
(define niter 250000)
(define start (current-time))
(make-node-and-link "hello world" niter)
(define stop (current-time))
(report-perf "node and link, first time:" start stop niter)

; Do it again. This time, its twice as fast, because the atoms already
; exist in the atomspace, and are not being made for the first time.
(define start (current-time))
(make-node-and-link "hello world" niter)
(define stop (current-time))
(report-perf "node and link, second time:" start stop niter)

; Make a large binary tree.
(define start (current-time))
(make-link-tree niter)
(define stop (current-time))
(report-perf "binary tree, first time:" start stop niter)

; Make a large binary tree, again.
(define start (current-time))
(make-link-tree niter)
(define stop (current-time))
(report-perf "binary tree, second time:" start stop niter)

