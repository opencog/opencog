; Links relex-to-logic output with relex-opencog-output
;
; Test sentence : "This is a sentence."

; modules needed for call-with-input-file and get-string-all
;  (use-modules (rnrs io ports))
;  (use-modules (ice-9 rdelim))

(use-modules (srfi srfi-1)   ; needed for delete-duplicates
             ; (ice-9 threads) ; needed for par-map
             (ice-9 rdelim) (ice-9 regex) (ice-9 receive))
(use-modules (opencog))
(use-modules (opencog exec))
(use-modules (opencog nlp)
             (opencog nlp oc)
             (opencog nlp lg-dict)
             (opencog nlp relex2logic))

; (use-modules (opencog logger))

; ---------------------------------------------------------------------
; Creates a single list  made of the elements of lists within it with
; the exception of empty-lists.
(define (list-squash lst member-output)
    (receive (list-lst member-lst) (partition list? lst)
        (if (nil? list-lst)
            (lset-union equal? member-output member-lst)
            (lset-union equal? member-lst
                (append-map
                    (lambda (x) (list-squash x '()))
                    list-lst
                )
            )
        )
    )
)

; ---------------------------------------------------------------------
(define-public (reset-sureal-cache dummy)
    (reset-cache)
)

(define-public (sureal a-set-link)
"
  sureal SETLINK -- main entry point for sureface realization

  Expect SETLINK to be a SetLink -- since it is assumed that the
  output of the micro-planner is unordered.
"
    ;; (cog-logger-info "sureal a-set-link = ~a" a-set-link)
    (if (equal? 'SetLink (cog-type a-set-link))
        (let ((interpretations (cog-chase-link 'ReferenceLink 'InterpretationNode a-set-link)))
            (if (nil? interpretations)
                (delete-duplicates (create-sentence a-set-link #f))
                (get-sentence interpretations)
            )
        )
        (display "Please input a SetLink only")
    )
)

;; This "cached" version of sureal is used by Microplanner.
;;
;; The idea is to cache the results from the calls to SuRealPCMB,
;; which is the PaternMatcher callback object (see PatternMatcher documentation).
;;
;; This cached version makes sense for Microplanner because it performs a lot of
;; sureal queries with very similar inputs.
;;
;; The cache lifetime is a single call of a Microplanner query
(define-public (cached-sureal a-set-link)
"
  sureal SETLINK -- main entry point for surface realization

  Expect SETLINK to be a SetLink -- since it is assumed that the
  output of the micro-planner is unordered.
"
    (if (equal? 'SetLink (cog-type a-set-link))
        (let ((interpretations (cog-chase-link 'ReferenceLink 'InterpretationNode a-set-link)))
            (if (nil? interpretations)
                (create-sentence a-set-link #t)
                (get-sentence interpretations)
            )
        )
        (display "Please input a SetLink only")
    )
)

; Returns a possible set of SuReals from an input SetLink
; * 'a-set-link' : A SetLink which is to be SuRealed
; * 'use-cache' : A flag to specify that the cached version of SuReal should be used
(define (create-sentence a-set-link use-cache)
    (define (construct-sntc mappings)
        ; get the words, skipping ###LEFT-WALL###
        (define words-seq (cdr (parse-get-words-in-order (interp-get-parse (caar mappings)))))
        ; helper to generate sentence using one mapping
        (define (construct-sntc-mapping w-seq vars mapping)
            ; make a clone of the w-seq to avoid changing when list-set!
            (define w-seq-copy (list-copy w-seq))
            (for-each
                (lambda (old-logic-node new-logic-node)
                    (let ((old-word-inst (r2l-get-word-inst old-logic-node))
                          (new-word-inst (r2l-get-word-inst new-logic-node))
                          (new-word (r2l-get-word new-logic-node))
                          )
                        ; if old-logic-node is actually a word (could be not for VariableNode or InterpretationNode)
                        (if (not (nil? old-word-inst))
                            ; find all locations in the w-seq this word-inst appear
                            (for-each
                                (lambda (x idx)
                                    (if (equal? x old-word-inst)
                                        (if (nil? new-word-inst)
                                            (list-set! w-seq-copy idx new-word)
                                            (list-set! w-seq-copy idx new-word-inst)
                                        )
                                    )
                                )
                                w-seq
                                (list-tabulate (length w-seq) values)
                            )
                        )
                    )
                )
                mapping
                vars
            )
            ; change from node to word string
            (map
                (lambda (w)
                    (if (equal? (cog-type w) 'WordInstanceNode)
                        (cog-name (word-inst-get-word w))
                        (cog-name w)
                    )
                )
                w-seq-copy
            )
        )

        (map construct-sntc-mapping (circular-list words-seq) (circular-list (cdar mappings)) (map cdr (cdr mappings)))
    )

    ;; (cog-logger-info "create-sentence a-set-link = ~a, use-cache = ~a"
    ;;                  a-set-link use-cache)

    ; Perform LG dictionary lookup on each word, if it's not already
    ; in the atomspace.
    ;
    ; Doing this in parallel with par-map seems like a good idea at
    ; first, but the guile implementation of par-map is so terrible
    ; that it actually makes things slower, by getting stuck in some
    ; live-lock.  So don't use par-map, use plain map.
    (map
        lg-dict-entry
        (filter-map
            (lambda (n)
                (if (nil? (r2l-get-word-inst n))
                    (if (nil? (r2l-get-word n))
                        #f
                        (begin
                            (if (equal? (cog-type n) 'PredicateNode)
                                (map
                                    (lambda (p)
                                        ; TODO: There could be too many... skip if seen before?
                                        (lg-dict-entry (word-inst-get-word p))
                                    )
                                    (cog-chase-link 'LemmaLink 'WordInstanceNode (r2l-get-word n))
                                )
                            )
                            (r2l-get-word n)
                        )
                    )
                    (word-inst-get-word (r2l-get-word-inst n))
                )
            )
            (cog-get-all-nodes a-set-link)
        )
    )

    ;; (cog-logger-info "Still there?")

    (if use-cache
        (cached-sureal-match a-set-link)
        (let* ((results (sureal-match a-set-link))
            (interps (delete-duplicates (map car results))))
            (append-map construct-sntc (map (lambda (i) (filter (lambda (r) (equal? (car r) i)) results)) interps))
        )
    )
)

; Returns a number that could be used to compare atoms of the same type. For eg.
; if the nodes are instaces of the same concept such as (ConceptNode "fly@123")
; (ConceptNode "fly@345") . Then using this funciton one can rank the atoms in terms
; of 'truthfulness'.
; * 'constant' : A parameter used to adjust the calcualtion of the heuristic value.
;                The choice of value (where x = constant) will go like,
;                       0<x<1 if higher confidence is preferred
;                       x > 1 if higher strength is preferred
(define (cog-stv-heuristic-value constant atom)
    (* (expt (cog-mean atom) constant) (expt (cog-confidence atom) (- 2 constant)))
)

; This just takes the one of the many InterpretationNode.
(define (get-sentence interpret-node-lst)
    (define parse (list-ref (cog-chase-link 'InterpretationLink 'ParseNode (list-ref interpret-node-lst 0)) 0))

    ;; (cog-logger-info "create-sentence interpret-node-lst = ~a"
    ;;                  interpret-node-lst)

    (string-join (cdr (map
        (lambda (winst) (cog-name (word-inst-get-word winst)))
        (parse-get-words-in-order parse))))
)


; ---------------------------------------------------------------------

; Return a match structure describing the results of the match on the node's name,
; for the given pattern or #f if no match could be found.
; * 'pattern' : It is the compiled regular expression of the pattern being matched.
; * 'node' : The node with name  being matched.
(define (cog-name-match pattern node)
    (if (cog-node? node)
        (regexp-exec pattern (cog-name node))
        #f
    )
)

; Return the name of a node without any UUID or indexes. It is assumed that '@'
; is used as a delimiter.
(define (cog-name-clean node)
    (define pattern (make-regexp "@"))
    (define has-match (regexp-exec pattern (cog-name node)))
    (if has-match (match:prefix has-match) (cog-name node))
)

; Checks if the outgoing set of an InheritanceLink or an ImplicationLink are similar
; and returns #t if they are or #f if they are not. An abstraction is the linking of an
; instance verison of an atom with it full abstracted version.
(define (is-abstraction? link)
    (let ((out-set (cog-outgoing-set link)))
        (and
            (nil? (filter-map cog-link? out-set))
            (equal? (cog-name-clean (car out-set)) (cog-name-clean (cadr out-set)))
        )
    )
)

; ---------------------------------------------------------------------
(define-public (filter-for-sureal a-list)
"
  filter-for-sureal A-LIST

  Takes a list of atoms A-LIST and returns a SetLink containing atoms that
  sureal can process.
"
    (define filter-in-pattern
        (ScopeLink
            (TypedVariable
                (Variable "$filter-for-sureal")
                (TypeChoice
                    (Signature
                        (Inheritance
                            (Type "ConceptNode")
                            (Type "ConceptNode")))
                    (Signature
                        (Implication
                            (Type "PredicateNode")
                            (Type "PredicateNode")))
                    (Signature
                        (Evaluation
                            (Type "PredicateNode")
                            (List
                                (Type "ConceptNode")
                                (Type "ConceptNode"))))
                    (Signature
                        (Evaluation
                            (Type "PredicateNode")
                            (ListLink
                                (Type "ConceptNode"))))
                ))
            ; Return atoms with the given signatures
            (Variable "$filter-for-sureal")
        ))

    (define filter-from (SetLink  a-list))

    ; Do the filtering
    (define result (cog-execute! (MapLink filter-in-pattern filter-from)))

    ; Delete the filter-from SetLink and its encompasing MapLink.
    ; FIXME: This results in 'result' being 'Invalid handle' sometimes.
    ;(cog-extract-recursive! filter-from)

    result
)
