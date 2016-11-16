(use-modules (ice-9 threads))

(use-modules (opencog))

; -----------------------------------------------------------------------
; From TruthValue::DEFAULT_TV()
(define default-stv (stv 1 0))

; -----------------------------------------------------------------------
; An alist from atom-types to the number of times they are encountered
; in the r2l step of the nlp pipeline. This gives a resonable estimate
; of the type, as compared to just doing a count of the types from other
; inputs.
; NOTE: StateLink isn't used so as to avoid putting garbage NumberNodes.
(define r2l-type-counts '())

(define (get-type-count atom)
"
  Returns the number of times an atom of same type had been encountered.
  If it the first type this function is called then it makes a crude estimate
  by counting the number of atoms of same type.

  atom:
  - An atom is taken as a whole instead of its type, because in the future we
  might want to count by pattern for more accuracy(for learning for eg).
"
    (let* ((type (cog-type atom))
        (count (assoc-ref r2l-type-counts type)))

        (if count
            (begin
                (set! r2l-type-counts
                    (assoc-set! r2l-type-counts type (+ 1 count)))
                (assoc-ref r2l-type-counts type)
            )
            (begin
                (set! r2l-type-counts
                    (assoc-set! r2l-type-counts type
                        (cog-count-atoms (cog-type atom))))
                (assoc-ref r2l-type-counts type)
            )
        )
    )
)

; -----------------------------------------------------------------------
(define (update-stv atom)
"
  Used to update the stv of an atom

  atom:
  - An atom with stv for tv type.
"
    (let ((default-k 800) ; From TruthValue::DEFAULT_K
        (type-count (get-type-count atom)))

        (if (equal? (cog-tv atom) default-stv)
            (let ((new-mean (/ 1 type-count))
                  (new-conf (/ 1 (+ 1 default-k))))
                (cog-set-tv! atom (cog-new-stv new-mean new-conf))
            )
            (let* ((current-count (tv-count (cog-tv atom)))
                   (new-count (+ current-count 1))
                   (new-mean (/ new-count type-count))
                   (new-conf (/ new-count (+ new-count default-k))))
                (cog-set-tv! atom (cog-new-stv new-mean new-conf))
            )
        )
    )
)

; -----------------------------------------------------------------------
(define (create-or-update-etv atom)
"
  Used to replace stv tv-type to etv, or update the etv of the given atom.
  The positive-evidence of the atom is incremented by one and then returned.
"
    (let ((tv (cog-tv atom)))

        (cog-set-tv! atom (etv (+ 1 (tv-positive-count tv)) (tv-count tv)))
    )
)

; -----------------------------------------------------------------------
(define-public (r2l-count sent-list)
"
  r2l-count SENT -- maintain counts of R2L atoms for SENT-LIST.

  sent-list:
  - A list of SentenceNodes
"
    (define (update-tv nodes)
        (par-map update-stv nodes)
    )

    ; Increment the R2L's node count value
    (parallel-map-parses
        (lambda (p)
            ; The preferred algorithm is
            ; (1) get all non-abstract nodes
            ; (2) delete duplicates
            ; (3) get the corresponding abstract nodes
            ; (4) update count
            (let* ((all-nodes
                    (append-map cog-get-all-nodes (parse-get-r2l-outputs p)))
                    ; XXX FIXME this is undercounting since each abstract
                    ; node can have multiple instances in a sentence.  Since
                    ; there is no clean way to get to the abstracted node
                    ; from an instanced node yet, such repeatition are
                    ; ignored for now
                    (abst-nodes (delete-duplicates
                        (filter is-r2l-abstract? all-nodes)))
                    (word-nodes
                        (append-map word-inst-get-word (parse-get-words p))))

                ; Updating all-nodes without deleting multiple occurence of
                ; a node is made so as to count every relation an atom is
                ; involved in as a separate observation of the concept.
                ; This form of counting is not precise but is assumed
                ; to be proptional to the size of the incoming-set.This is a
                ; crude fix for the issue raised in the FIXME on abst-nodes,
                ; above.
                (update-tv all-nodes)
                (update-tv word-nodes)
            )
        )
        sent-list
    )
)
