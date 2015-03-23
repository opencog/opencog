;
; Guile ExecutionOutputLink example.
; The cog-execute! function is demonstrated.  See also the except.scm
; example to see what happens when things
;

(use-modules (opencog))

; Using python code in an execution link. Be sure to set the PYTHONPATH
; environment varialbe first, so that the cython and python libraries
; can be found. Something similar to this should do:
;
; export PYTHONPATH=build/opencog/cython:./opencog/python:./opencog/python/opencog:./examples/guile
;
; Also, be sure that my_py_func.py is loaded.
; XXX FIXME -- this doesn't work!???  Don't know how to fix.
;
(cog-execute!
   (ExecutionOutputLink
      (GroundedSchemaNode "py:my_py_func")
      (ListLink
         (ConceptNode "1")
         (ConceptNode "2"))))

; Similar example, for embedded scheme code
;
(define (my-scm-func atoma atomb)
	(display "My func called with atom arguments\n")
	(display atoma) (display atomb)
	(newline)
	(ConceptNode "I'm returning this atom")
)
	
(cog-execute!
   (ExecutionOutputLink
      (GroundedSchemaNode "scm:my-scm-func")
      (ListLink
         (ConceptNode "1")
         (ConceptNode "2"))))


