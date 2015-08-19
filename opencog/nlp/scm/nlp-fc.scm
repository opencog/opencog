;recomended way
;1.
(define (load-r2l-rules rules-file)
    (load-scm-from-file rules-file)
)
;then call nlp-fc
(define (nlp-fc sent)
  (define mylist '())
;load rules
;  (load-scm-from-file rules-file)
; run forward chaining on sentence
  (let* ((temp (run-fc sent)); temp contains list of cog-fc results and parse node
        (result1 (car temp));result1 contains Listlink nested 3 times
        (parse-node (cadr temp))
        (result2 (cog-outgoing-set result1));result2 contains ListLink nested 2 times
        (parse-name (cog-name parse-node))
        )


     (for-each (lambda (x)
                 (let* ((t1 (cog-outgoing-set x));t1 contains ListLink List of results
                       )
                       (for-each (lambda (b)
                                   (let* ((t2 (cog-outgoing-set b)));t2 contains atoms contained in ListLink
                                    (for-each (lambda(c)
                                             ;  (display c)
                                                (set! mylist (append mylist (list c)))
                                              )
                                     t2)
                                   )
                                 )
                       t1
                       )
                 )
               )
      result2)


  (ReferenceLink
    (InterpretationNode (string-append parse-name "_interpretation_$X"))
    ; The function in the SetLink returns a list of outputs that
    ; are the results of the evaluation of the relex-to-logic functions,
    ; on the relex-opencog-outputs.
  (SetLink (delete-duplicates mylist))
  )

  (InterpretationLink
    (InterpretationNode (string-append parse-name "_interpretation_$X"))
    parse-node
  )
 )
#t
)

;below function is not recomended for normal use
(define (nlp-parse-fc sent rules-file)
  (define mylist '())
;load rules
  (load-scm-from-file rules-file)
; run forward chaining on sentence
  (let* ((temp (run-fc sent)); temp contains list of cog-fc results and parse node
        (result1 (car temp));result1 contains Listlink nested 3 times
        (parse-node (cadr temp))
        (result2 (cog-outgoing-set result1));result2 contains ListLink nested 2 times
        (parse-name (cog-name parse-node))
        )


     (for-each (lambda (x)
                 (let* ((t1 (cog-outgoing-set x));t1 contains ListLink List of results
                       )
                       (for-each (lambda (b)
                                   (let* ((t2 (cog-outgoing-set b)));t2 contains atoms contained in ListLink
                                    (for-each (lambda(c)
                                             ;  (display c)
                                                (set! mylist (append mylist (list c)))
                                              )
                                     t2)
                                   )
                                 )
                       t1
                       )
                 )
               )
      result2)


  (ReferenceLink
    (InterpretationNode (string-append parse-name "_interpretation_$X"))
    ; The function in the SetLink returns a list of outputs that
    ; are the results of the evaluation of the relex-to-logic functions,
    ; on the relex-opencog-outputs.
  (SetLink (delete-duplicates mylist))
  )

  (InterpretationLink
    (InterpretationNode (string-append parse-name "_interpretation_$X"))
    parse-node
  )
 )
#t
)


(define (run-fc sent)
  (define parse-node (car (sentence-get-parses (nlp-parse sent))))
  (list (cog-fc
    (SetLink (parse-get-relations parse-node))
    r2l-rules
   )
  parse-node
  )
)
(define (run-fc-dbg sent)
  (define parse-node (car (sentence-get-parses (nlp-parse sent))))
  (display (cog-fc
    (SetLink (parse-get-relations parse-node))
    r2l-rules
   )
  )
)
