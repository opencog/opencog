; Links relex-to-logic output with relex-opencog-output
; It is temporary until the r2l rules are moved into the URE
; XXX Huh ???

; Test sentence : "This is a sentence."

; modules needed for call-with-input-file and get-string-all
;  (use-modules (rnrs io ports))
;  (use-modules (ice-9 rdelim))

; ---------------------------------------------------------------------
; Creates a single list  made of the elements of lists within it with the exception
; of empty-lists.
(define (list-squash lst member-output)
    (receive (list-lst member-lst) (partition list? lst)
        (if (null? list-lst)
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
(define-public (sureal a-set-link)
"
  sureal SETLINK -- main entry point for sureface realization

  Expect SETLINK to be a SetLink -- since it is assumed that the
  output of the micro-planner is unordered.
"
    (if (equal? 'SetLink (cog-type a-set-link))
        (let ((interpretations (cog-chase-link 'ReferenceLink 'InterpretationNode a-set-link)))
            (if (null? interpretations)
                (delete-duplicates (create-sentence a-set-link))
                (get-sentence interpretations)
            )
        )
        (display "Please input a SetLink only")
    )
)

; Returns a possible set of SuReals from an input SetLink
; * 'a-set-link' : A SetLink which is to be SuRealed
(define (create-sentence a-set-link)
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
                        (if (not (null? old-word-inst))
                            ; find all locations in the w-seq this word-inst appear
                            (for-each
                                (lambda (x idx)
                                    (if (equal? x old-word-inst)
                                        (if (null? new-word-inst)
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
                        (word-inst-get-word-str w)
                        (cog-name w)
                    )
                )
                w-seq-copy
            )
        )

        (map construct-sntc-mapping (circular-list words-seq) (circular-list (cdar mappings)) (map cdr (cdr mappings)))
    )

    ; add LG dictionary on each word if not already in the atomspace
    (par-map
        lg-get-dict-entry
        (filter-map
            (lambda (n)
                (if (null? (r2l-get-word-inst n))
                    (if (null? (r2l-get-word n))
                        #f
                        (r2l-get-word n)
                    )
                    (car (word-inst-get-word (r2l-get-word-inst n)))
                )
            )
            (cog-get-all-nodes a-set-link)
        )
    )

    (let* ((results (sureal-match a-set-link))
           (interps (delete-duplicates (map car results))))
        (append-map construct-sntc (map (lambda (i) (filter (lambda (r) (equal? (car r) i)) results)) interps))
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
    (* (expt (cog-stv-strength atom) constant) (expt (cog-stv-confidence atom) (- 2 constant)))
)

; This just takes the one of the many InterpretationNode.
(define (get-sentence interpret-node-lst)
    (define parse (list-ref (cog-chase-link 'InterpretationLink 'ParseNode (list-ref interpret-node-lst 0)) 0))
    (string-join (cdr (map word-inst-get-word-str (parse-get-words-in-order parse))))
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
            (null? (filter-map cog-link? out-set))
            (equal? (cog-name-clean (car out-set)) (cog-name-clean (cadr out-set)))
        )
    )
)
