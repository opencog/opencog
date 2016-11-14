(use-modules (ice-9 threads))

; -----------------------------------------------------------------------
(define-public (r2l-count sent-list)
"
  r2l-count SENT -- maintain counts of R2L atoms for SENT-LIST.

  sent-list:
  - A list of SentenceNodes
"
    (define (update-tv nodes)
        ; DEFAULT_TV and DEFAULT_K as defined in TruthValue.cc
        (let ((default-stv (stv 1 0))
              (default-k 800))
            (par-map
                (lambda (n)
                    (if (equal? (cog-tv n) default-stv)
                        (let ((new-mean (/ 1 (cog-count-atoms (cog-type n))))
                              (new-conf (/ 1 (+ 1 default-k))))
                            (cog-set-tv! n (cog-new-stv new-mean new-conf))
                        )
                        (let* ((current-count (round (assoc-ref (cog-tv->alist (cog-tv n)) 'count)))
                               (new-count (+ current-count 1))
                               (new-mean (/ new-count (cog-count-atoms (cog-type n))))
                               (new-conf (/ new-count (+ new-count default-k))))
                            (cog-set-tv! n (cog-new-stv new-mean new-conf))
                        )
                    )
                )
                nodes
            )
        )
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
