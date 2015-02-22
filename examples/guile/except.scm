;
; Guile exception handling example.  This demonstrates how to catch
; exceptions, and also how what happens when a bad ExecutionOutputLink
; is used.  See execute.scm for more cog-execute! examples.
;

(use-modules (opencog))

; First, just give it some broken junk.  See what happens.
(cog-execute!
   (ExecutionOutputLink
      (GroundedSchemaNode "py:b0rk3n_junk")
      (ListLink
         (ConceptNode "1")
         (ConceptNode "2"))))

; C++ exceptions are converted into scheme exceptions, and can be
; caught, as usual.
(catch
   #t
   (lambda ()
      (cog-execute!
         (ExecutionOutputLink
            (GroundedSchemaNode "py:b0rk3n_junk")
            (ListLink
               (ConceptNode "1")
               (ConceptNode "2")))))
   (lambda (key . args)
      (display "Ohhh noooo Mr. Bill!!! ") (display key)
      (newline)
      (display "Sluggo says to ... ") (display args)
      (newline) (newline)
   ))


; Exception-producing code, but for mal-formed scheme.
;
(cog-execute!
   (ExecutionOutputLink
      (GroundedSchemaNode "scm:(((((uber-badf")
      (ListLink
         (ConceptNode "1")
         (ConceptNode "2"))))
