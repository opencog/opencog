;
; utilities.scm
;
; Generic utilities.
; These do NOT assume opencog; they'll work outside of opencog.
;
; Utilites include:
; * A parallel version of the srfi-1 find function
; * A for-all-pairs function
; * A fold-over-all-pairs function
;
; Copyright (c) 2017, 2018 Linas Vepstas
;
; ---------------------------------------------------------------------

(use-modules (srfi srfi-1))
(use-modules (ice-9 threads))
(use-modules (ice-9 receive))   ; for partition

; ---------------------------------------------------------------------
;
(define (par-find PRED LST)
"
  par-find PRED LST

  Apply PRED to elements in LST, and return the first element for
  which PRED evaluates to #t; else return #f.

  This is similar to the srfi-1 `find` function, except that the
  processing is done in parallel, over multiple threads.
"
	; If the number of threads was just 1, then do exactly this:
	; (find PRED LST)

	(define NTHREADS 4)
	(define SLEEP-TIME 1)

	; Design issues:
	; 1) This uses sleep in a hacky manner, to poll for finished
	;    threads. This is OK for the current application but is
	;    hacky, and needs to be replaced by some semaphore.
	; 2) Guile threads suck.  There's some kind of lock contention,
	;    somewhere, which leads to lots of thrashing, when more
	;    than 2-3 threads are run. The speedup from even 2 threads
	;    is lackluster and barely acceptable.

	; Return #t if thr is a thread, and if its still running
	(define (is-running? thr)
		(and (thread? thr) (not (thread-exited? thr))))

	; Return exit value of the thread, if its actually a thread.
	(define (get-result thr)
		(if (thread? thr) (join-thread thr) #f))

	; Convenience wrapper for srfi-1 partition
	(define (parton PRED LST)
		(receive (y n) (partition PRED LST) (list y n)))

	; thread launcher - return a list of the launched threads.
	; Note: CNT musr be equal to or smaller than (length ITM-LST)
	(define (launch ITM-LST CNT)
		(if (< 0 CNT)
			(cons
				(call-with-new-thread
					(lambda () (if (PRED (car ITM-LST)) (car ITM-LST) #f)))
				(launch (cdr ITM-LST) (- CNT 1)))
			'()))

	; Check the threads to see if any got an answer.
	; If so, return the answer.
	; If not, examine ITEMS in some threads.
	(define (check-threads THRD-LST ITEMS)
		; Partition list of threads into those that are running,
		; and those that are stopped.
		(define status (parton is-running? THRD-LST))
		(define running (car status))
		(define stopped (cadr status))

		; Did any of the stopped threads return a value
		; other than #f? If so, then return that value.
		(define found
			(find (lambda (v) (not (not v))) (map get-result stopped)))

		; Compute number of new threads to start
		(define num-to-launch
			(min (- NTHREADS (length running)) (length ITEMS)))

		; Return #t if we've looked at them all.
		(define done
			(and (= 0 num-to-launch)
				(= 0 (length running))
				(= 0 (length ITEMS))))

		; If we found a value, we are done.
		; If list is exhausted, we are done.
		; Else, launch some threads and wait.
		(if found found
			(if done #f
				(let ((thrd-lst (append! (launch ITEMS num-to-launch) running)))
					(if (= 0 num-to-launch) (sleep SLEEP-TIME))
					(check-threads thrd-lst (drop ITEMS num-to-launch)))))
	)

	(check-threads (make-list NTHREADS #f) LST)
)

; ---------------------------------------------------------------------
;
(define (for-all-unordered-pairs FUNC LST)
"
  for-all-unordered-pairs FUNC LST

  Call function FUNC on all possible unordered pairs created from LST.
  That is, given the LST of N items, create all possible pairs of items
  from this LST. There will be N(N-1)/2 such pairs.  Then call FUNC on
  each of these pairs.  This means that the runtime is O(N^2).

  The function FUNC must accept two arguments. The return value of FUNC
  is ignored.

  The return value is unspecified.
"
	(define (make-next-pair primary rest)
		(define more (cdr primary))
		(if (not (null? more))
			(if (null? rest)
				(make-next-pair more (cdr more))
				(let ((item (car primary))
						(next-item (car rest)))
					(format #t "~A ~A " (length primary) (length rest))
					(FUNC item next-item)
					(make-next-pair primary (cdr rest))))))

	(make-next-pair LST (cdr LST))
)

; ---------------------------------------------------------------------
;
(define (fold-unordered-pairs ACC FUNC LST)
"
  Call function FUNC on all possible unordered pairs created from LST.
  That is, given the LST of N items, create all possible pairs of items
  from this LST. There will be N(N-1)/2 such pairs.  Then call FUNC on
  each of these pairs.  This means that the runtime is O(N^2).

  The function FUNC must accept three arguments: the first two
  are the pair, and the last is the accumulated (folded) value.
  It must return the (modified) accumulated value.

  This returns the result of folding on these pairs.
"
	(define (make-next-pair primary rest accum)
		(define more (cdr primary))
		(if (null? more) accum
			(if (null? rest)
				(make-next-pair more (cdr more) accum)
				(let ((item (car primary))
						(next-item (car rest)))
					(format #t "~A ~A " (length primary) (length rest))
					(make-next-pair primary (cdr rest)
						(FUNC item next-item accum))
				))))

	(make-next-pair LST (cdr LST) ACC)
)

; ---------------------------------------------------------------
