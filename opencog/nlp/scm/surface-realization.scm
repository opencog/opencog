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
            (begin (eval-string (list-ref a-list 0))
            (let ((parse-name (parse-str (list-ref a-list 0))))
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
                    (ParseNode parse-name)
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

; Returns a possible set of SuReals from an output SetLink
; * 'a-set-link' : A SetLink which is to be SuRealed
(define (create-sentence a-set-link)
    (define clean-set (SetLink (remove is-abstraction? (cog-outgoing-set a-set-link))))
    (define (replace-wins-with-str a-list)
        (map (lambda (x)
                (if (and (cog-atom? x) (equal? 'WordInstanceNode (cog-type x)))
                    (word-inst-get-word-str x)
                    x
                ))
            a-list
        )
    )

    (let ((output-lst (par-map replace-wins-with-str (get-chunks clean-set))))
        (map delete (make-list (length output-lst) "###LEFT-WALL###") output-lst)
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

; It will return a list  of atoms that are similar. Similar links are atoms that are of the
; same type , have the same subgraph and have part of the node name match for
; at least one atom. This only work first-order links. Similar nodes are atoms
; that have similar type and naming. A similar name is a name that matches before
; UUID or other indexes or index markers. For e.g.,
;   (WordInstanceNode "sentence@1ba8b794-93ba-47d9-93f4-a0db91dfc9d4")
;   (WordInstanceNode "sentence")
; * 'atom' : is the atom for which similar atoms are being searched for.
; * 'lst' : is the list of atoms being checked for similarity.
(define (similar-atoms atom lst)
    (define (similar-node? node)
        (and (equal? (cog-type node) (cog-type atom))
            (cog-name-match (make-regexp (cog-name-clean atom) regexp/icase) node)
        )
    )
    (define (similar-link? link)
        (let ((link-out-set (cog-outgoing-set link))
                (atom-out-set (cog-outgoing-set atom)))
            (and (equal? (cog-type link) (cog-type atom))
                (not (null?
                    (append-map similar-atoms atom-out-set (make-list (length atom-out-set) link-out-set))))
            )
        )
    )
    (cond
        ((cog-node? atom) (filter-hypergraph similar-node? lst))
        ((cog-link? atom) (remove (negate similar-link?) lst))
    )
)


; Returns #t if node1 and node2 are equivalent. The Nodes are equivalent if
; they are of the same type and have similar names. A similar name is a name
; that matches before UUID or other indexes or index markers.
; * 'node1' : node with a name being used for matching.
; * 'node2' ; node which is being checked if it has a name which matchs node1's name.
(define (eqv-node? node1 node2)
    (if
        (and (equal? (cog-type node1) (cog-type node2))
            (cog-name-match (make-regexp (cog-name-clean node1) regexp/icase) node2)
        )
        #t
        #f
    )
)

; Returns SetLinks associated with the dialogue system
(define (get-dialogue-sets)
    (delete '()
        (par-map (lambda (x)
                    (if (or (not (null? (cog-chase-link 'ReferenceLink 'InterpretationNode x)))
                            (not (null? (cog-chase-link 'ReferenceLink 'SentenceNode x)))
                        )
                        x
                        '()
                    )
                 ) (cog-get-atoms 'SetLink)
         )
     )
)

; Check if any of the atoms in the outgoing set of SetLinks, relevant for dialogue,
; are similar to 'an-atom', and return a list containing those which do.
; Should one want to define a different criteria for changes can be made here.
(define (filter-dialogue-sets an-atom)
    (delete '()
        (par-map (lambda (x) (if (null? (similar-atoms an-atom (cog-outgoing-set x))) '() x)
                 ) (get-dialogue-sets)
        )
    )
)

; Returns a alist in which the keys are the handles of the outgoing-set of
; the SuRealizable SetLink and the values are a list of handles of the SetLinks
; which have atoms similar to the keys.
; * 'output-set' is the SetLink to be surealed.
(define (index output-set)
    (define out-set (remove is-abstraction? (cog-outgoing-set output-set)))
    (define dict '())
        (append-map
            (lambda (x) (acons (cog-handle x) (par-map cog-handle (filter-dialogue-sets x)) dict))
            out-set
        )
)

; creates value to key alist of key-value pair, where the cdr of a-pair is a list.
; It has a weired side-effect in that it both modifies 'a-dict' as well as
; returning a new alist which is equal (as in #t for equal? and #f for eq?)
(define (alist-inverse a-pair a-dict)
    (delete-duplicates (append-map (lambda (x)
            (if (null? x)
                (if (assoc-ref a-dict "NO_PATTERN")
                    (acons "NO_PATTERN" (delete-duplicates! (append (assoc-ref a-dict "NO_PATTERN") (list (car a-pair)))) a-dict)
                    (acons "NO_PATTERN" (list (car a-pair)) a-dict)
                )
                (if (assoc-ref a-dict x)
                    (acons x (delete-duplicates! (append! (assoc-ref a-dict x) (list (car a-pair)))) a-dict)
                    (acons x (list (car a-pair)) a-dict)
                )
            )
        )
        (delete-duplicates (cdr a-pair))))
)

; Returns an alist of key-value pairs structured as handles of
; (r2l-SetLink . (output-link1 output-link2 ...))
; * 'old-dict' : It is an alist of key-value pairs structured as
;                (output-link1 . (r2l-SetLink1 .... r2l-SetLinkn))
; * 'new-dict' : It could be an empty list or an existing index to be updated.
(define (reorder old-dict new-dict)
    (if (null? (cdr old-dict))
        (alist-inverse (car old-dict) new-dict)
        (reorder (cdr old-dict) (alist-inverse (car old-dict) new-dict))
    )
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

; Returns a pair of atom handles structured as (r2l-node . output-node) provided
; they are equivalent as in (eqv-node? arg1 arg2).
; or an empty list other wise.
; * 'output-atom' : It is a link we want to surealize
; * 'r2l-atom' ; It is a link we are using for extracting relations between words.
(define (get-mapping output-atom r2l-atom)
    (if (equal? (cog-type output-atom) (cog-type r2l-atom))
        (cond
            ((cog-node? output-atom)
                (if (eqv-node? output-atom r2l-atom)
                    (cons (cog-handle r2l-atom) (cog-handle output-atom))
                    'not-eqv-node
                )
            )
            ((cog-link? output-atom)
                ; if the arity of the links are equal then assuming that the order
                ; of the nodes is associated with the property/functionality of
                ; of the node with in the language, a mapping is made. If the
                ; arity are not equal then a mapping of equvalent-nodes is only made.
                (if (equal? (cog-arity output-atom) (cog-arity r2l-atom))
                        (map
                            (lambda (x y)
                                (let ((mapping (get-mapping x y)))
                                    (if (equal? mapping 'not-eqv-node)
                                        (cons (cog-handle y) (cog-handle x))
                                        mapping
                                    )
                                )
                            )
                            (cog-outgoing-set output-atom) (cog-outgoing-set r2l-atom)
                        )
                        (remove null? (map
                            (lambda (x y)
                                (let ((mapping (get-mapping x y)))
                                    (if (equal? mapping 'not-eqv-node)
                                        '()
                                        mapping
                                    )
                                )
                            )
                            (cog-outgoing-set output-atom) (cog-outgoing-set r2l-atom)
                        ))
                )
            )
        )
        '()
    )
)

; Return chunks of a sentence. It ranks sentences by swaping equivalent structures
; depending on the name of the constituent Node names.
; TODO : Figure out an appropirate ranking formula. The ranking should
; be across all the outgoing-set of the output SetLink (I think???)
(define (get-chunks output-set)
    (define dict (reorder (index output-set) '()))
    (define (get-sntc r2l-set)
        (parse-get-words-in-order (interp-get-parse (logic-get-interp r2l-set)))
    )

    ; Takes a pair input structured as the handles of (r2l-SetLink . (output-link1 output-link2))
    ; and returns a list of lists, containing pairs of (r2l-node . output-node)
    ; handles. This is the function that should be edited to gain multiple possible
    ; output per r2l-SetLink.
    ; TODO: 1. have to remove tag atoms while mapping.
    ;       2. When there is muliple mapping for a r2l-node merge them to a list so as multiple alternative
    ;        sentences could be generated . OR , make a choice of the most likely pair.
    (define (get-mapping-pair a-pair)

        (let
            ((r2l-set (remove is-abstraction? (cog-outgoing-set (cog-atom (car a-pair)))))
            )
            (delete-duplicates (list-squash
                   (par-map (lambda (x)
                        (remove null?
                        (append-map get-mapping (make-list (length r2l-set) (cog-atom x)) r2l-set))
                        ) (cdr a-pair)
                    ) '() )
            )
        )
    )

    ; Replaces the WordInstanceNodes with names matching the name of the car atom
    ; with the clean name of the cdr of the pair
    (define (replace-word r2l-sntc a-pair)
        (let ((an-index (list-index (lambda (x)
                    (if (cog-atom? x)
                        (equal? (cog-name (cog-atom (car a-pair))) (cog-name x)) #f)) r2l-sntc))
            )
            (if (number? an-index)
                (begin
                    (list-set! r2l-sntc an-index (cog-name-clean (cog-atom (cdr a-pair))))
                    #t
                )
                #f
            )
        )
    )

    ; Returns a random merge of possible sentences that can be formed from one r2l SetLink
    ; TODO : Ranking of all possible alternative.
    ; * 'a-pair' : the pair passed as input has to be a mapping sturctured as handles of
    ;               (r2l-SetLink . (output-link1 output-link2))
    (define (chunk a-pair)
        (let ((mapping (get-mapping-pair a-pair)) (sntc (get-sntc (cog-atom (car a-pair)))))
                (map-in-order (lambda (x) (replace-word sntc x)) mapping)
             sntc
        )
    )
    (map chunk dict)
)

