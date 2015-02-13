;
; Basic guile atom creation benchmark.
;
; See opencog/guile/README or http://wiki.opencog.org/w/Scheme
; for additional documentation.
;
; If you have installed opencog, then start guile simply by saying
; "guile" at the bash prompt $.  Otherwise, you will need to do this:
;
; $ export LTDL_LIBRARY_PATH=build/opencog/guile
; $ guile -L opencog/scm -L build
;
; where "build" is where-ever you built opencog.
;
; Another possibility: add stuff to your ~/.guile file, for example:
; (add-to-load-path "/home/yourname/opencog/build")
; (add-to-load-path "/home/yourname/opencog/opencog/scm")
;

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
(define c1 (ConceptNode "first one ever"))
(define (make-atoms prefix n)
	(define c2 c1)
	(define c1 (ConceptNode (string-append/shared prefix (number->string n))))
	; Hmmm.. formatted printing is kind-of slow.
	; (ConceptNode (format #f "Object_~d" n))
	(ListLink c1 c2)
	(if (< 0 n)
		(make-atoms prefix (- n 1))
		'()
	)
)

; A handy utility to report the elapsed time and the rate.
(define (report-perf start stop niter)
	(define elapsed (time-difference stop start))
	(define delta 
		(+ (time-second elapsed) 
			(/ (time-nanosecond elapsed) 1000000000.0)))
	(define rate (round (/ niter delta)))
	(display "Elapsed time (secs): ") (display delta) (newline)
	(display "Loops per second: ") (display rate) (newline)
)

; Measure how long it takes to create a bunch of links.
(define niter 150000)
(define start (current-time))
(make-atoms "hello world" niter)
(define stop (current-time))
(report-perf start stop niter)

; Do it again. This time, its twice as fast, because the atoms already
; exist in the atomspace, and are not being made for the first time.
(define start (current-time))
(make-atoms "hello world" niter)
(define stop (current-time))
(report-perf start stop niter)

