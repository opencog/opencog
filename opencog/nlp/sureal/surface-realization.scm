;
; Links relex-to-logic output with relex-opencog-output
; It is temporary until the r2l rules are moved into the URE

; Test sentence : "This is a sentence."

; modules needed for call-with-input-file and get-string-all
;  (use-modules (rnrs io ports))
;  (use-modules (ice-9 rdelim))

(use-modules (ice-9 rdelim))
(use-modules (ice-9 regex))
(use-modules (ice-9 receive))
; ---------------------------------------------------------------------
; Patterns used for spliting the string passed from relex server
(define pattern1 "; ##### END OF A PARSE #####")
(define pattern2 "; ##### START OF R2L #####")

; ---------------------------------------------------------------------
; Splits a string into substring delimited by the a pattern and returns a list
; of the substrings.
; TODO: make tail recursive, for efficency
(define (split-string a-pattern a-string)
    (let ((a-match (string-match a-pattern a-string)))
        (if (not a-match)
            (list a-string)
            (append (list (match:prefix a-match))
                (split-string a-pattern (match:suffix a-match)))
        )
    )
)

; ---------------------------------------------------------------------
; returns the name of a ParseNode if the string has a ParseNode entry.
; returns "sentence@dbce9f0d-8b8a-4ea7-a8c8-d69abf05f810_parse_1" from the string
; "84590fb2e46c (ParseNode \"sentence@dbce9f0d-8b8a-4ea7-a8c8-d69abf05f810_parse_1\""
(define (parse-str a-string)
    (substring (match:substring
        (string-match "ParseNode \"sentence@[[:alnum:]_-]*" a-string)) 11)
)

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
; Evaluate the string and returns a list with SetLink of the initial r2l rule application.
; (ReferenceLink 
;   (InterpretationNode "sentence@1d220-c7ace2f_parse_2_interpretation_$X")
;   (SetLink
;       different links that are a result of r2l rule-functions being applied
;   )
; )
;
; A ReferenceLink is used instead of InterpretationLink so as to differentiate
; the final word-sense-disambiguated , anaphore and cataphor resolved version from
; the initial r2l output. The final version will have structure that is detailed
; @ http://wiki.opencog.org/w/Linguistic_Interpretation.
; Having the output from this function should help during garbage-removal from the
; atomspace as well as the generation of the final set of interpretations for the
; the sentence.

(define (set-link a-string)
    ; z-list is a list containing a set of lists, with firts elements of the
    ; sub-list being a relex-opencog-output string and the second element being
    ; a string of the relex-to-logic function calls that is to be applied on the
    ; relex-opencog-output in the atomspace. The last sub-list is just the AnchorNode.
    (define z-list (map (lambda (x) (split-string pattern2 x)) 
                                    (split-string pattern1 a-string)))

    ; helper function that prune away atoms that no longer exists from subsequent
    ; rules, or atoms that are wrapped inside another link
    (define (pruner x)
        (define deref-x (cog-atom (cog-handle x)))
        ; if 'deref-x' is #<Invalid handle>, than both cog-node?
        ; and cog-link? will return false
        (if (and (or (cog-node? deref-x) (cog-link? deref-x))
                 (null? (cog-incoming-set x)))
            x
            #f
        )
    )

    ; Given any one of the sub-lists from the z-lists it will evaluate the strings
    ; elements from left to right resulting in the creation of the Atoms of the 
    ; relex-to-logic pipeline.
    (define (eval-list a-list)
        (if (= 1 (length a-list))
            (begin (eval-string (list-ref a-list 0)) '())
            (begin
                (eval-string (list-ref a-list 0))
                (let* ((parse-name (parse-str (list-ref a-list 0)))
                       (parse-node (ParseNode parse-name)))
                    ; generate the LG dictionary entries for each word
                    (let ((words (parse-get-words parse-node)))
                        (map-word-instances
                            (lambda (word-inst) (map-word-node lg-get-dict-entry word-inst))
                            parse-node
                        )
                    )
                    (ReferenceLink 
                        (InterpretationNode (string-append parse-name "_interpretation_$X"))
                        ; The function in the SetLink returns a list of outputs that
                        ; are the results of the evaluation of the relex-to-logic functions,
                        ; on the relex-opencog-outputs.
                        (SetLink
                            (filter-map pruner
                                (delete-duplicates 
                                    (apply append 
                                        (map-in-order eval-string
                                            (filter (lambda (x) (not (string=? "" x)))
                                                (split-string "\n" (list-ref a-list 1))
                                            )
                                        )
                                    )
                                )
                            )
                        )
                    )
                    (InterpretationLink
                        (InterpretationNode (string-append parse-name "_interpretation_$X"))
                        parse-node
                    )
                ))
        )
    )

    (par-map eval-list z-list)
)


; ---------------------------------------------------------------------
; A helper function 
(define (set-interpret port)
    (let ((string-read (get-string-all port)))
        (if (eof-object? string-read)
            #f
            (set-link string-read)
        )
    )
)

; ---------------------------------------------------------------------
; A copy of relex-parse funtion modified for relex-to-logic(r2l) purposes.
(define (r2l plain-txt)
    (define (do-sock-io sent-txt)
        (let ((s (socket PF_INET SOCK_STREAM 0)))
            (connect s AF_INET (inet-pton AF_INET relex-server-host) relex-server-port)

            (display sent-txt s)
            (display "\n" s)
            (system (string-join (list "echo \"Info: send to parser: " sent-txt "\"")))
            (set-interpret s)
            (system (string-join (list "echo Info: close socket to parser" )))
            (close-port s)
        )
    )

    (if (string=? plain-txt "")
        (display "Please enter a valid sentence.")
        (do-sock-io plain-txt)
    )
)

; ---------------------------------------------------------------------
; A SetLink is the input because it is assumed that the output of the micro-planner
; is unordered.
(define (sureal a-set-link)
    (if (equal? 'SetLink (cog-type a-set-link))
        (let ((interpretations (cog-chase-link 'ReferenceLink 'InterpretationNode a-set-link)))
            (if (null? interpretations)
            (create-sentence a-set-link)
            (get-sentence interpretations)
            )
        )
        (display "Please input a SetLink only")
    )
)

; Returns a possible set of SuReals from an input SetLink
; * 'a-set-link' : A SetLink which is to be SuRealed
(define (create-sentence a-set-link)
    (define (construct-sntc itpr)
        ; get the words, skipping ###LEFT-WALL###
        (define words-seq (cdr (parse-get-words-in-order (interp-get-parse itpr))))
        ; get the new query to old interpretation mappings (multiple)
        (define mappings (sureal-get-mapping itpr))
        ; helper to generate sentence using one mapping
        (define (construct-sntc-mapping w-seq vars mapping)
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
                                            (list-set! w-seq idx new-word)
                                            (list-set! w-seq idx new-word-inst)
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
                w-seq
            )
        )

        (map construct-sntc-mapping (circular-list words-seq) (circular-list (car mappings)) (cdr mappings))
    )

    ; add LG dictionary on each word if not already in the atomspace
    (map
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

    (append-map construct-sntc (sureal-match a-set-link))
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
; Some Utilities : These are likely to be moved to a separate file.


; Returns a list of the r2l logic outputs associated with the InterpretationNode.
(define (interp-get-logic-outputs interp-node)
    (cog-outgoing-set (list-ref (cog-chase-link 'ReferenceLink 'SetLink interp-node) 0))
)

; Returns a list of the r2l logic outputs associated with the ParseNode.
(define (parse-get-logic-outputs parse-node)
    (let ((inter (car (cog-chase-link 'InterpretationLink 'InterpretationNode parse-node))))
        (interp-get-logic-outputs inter)
    )
)

; Returns the InterpretationNode associated with the SetLink.
(define (logic-get-interp a-set-link)
    (car (cog-chase-link 'ReferenceLink 'InterpretationNode a-set-link))
)

; Returns the ParseNode associated with an InterpretationNode
; * 'interp' : An InterpretationNode
(define (interp-get-parse interp)
    (car (cog-chase-link 'InterpretationLink 'ParseNode interp))
)

; ---------------------------------------------------------------------
; The functions below are used for the pattern/knowledgebase based approach to
; SuReal.

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


