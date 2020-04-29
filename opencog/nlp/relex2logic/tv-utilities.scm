(use-modules (ice-9 threads))

(use-modules (opencog))

; -----------------------------------------------------------------------
(define-public (load-gtwc)
"
  Populate the atomspace with ConceptNodes drived from Google's Trillion Word
  Corpus.
"
; TODO: move this to (opencog data) module when it is created.
    (let ((path "/var/opencog/data/gtwc-en-333333-words.scm"))
        (if (file-exists? path)
            (primitive-load path)
            (display "The data file hasn't been downloaded.")
        )
    )
)

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
            (let* ((current-count (cog-count atom))
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

        (cog-set-tv! atom (etv (+ 1 (tv-positive-count tv)) (cog-count atom)))
    )
)

; -----------------------------------------------------------------------
; From TruthValue::DEFAULT_K)
(define default-k 800)
; We are using Google's Trillion Word Corpus
; See github.com/opencog/external-tools/blob/master/gtwc/atomize.sh#L27
(define total-count 1024908267229.0)

(define (update-node-stv r2l-node)
"
  If the given r2l-node is linked to WordNode then it updates its stv by
  copying the stv of the ConceptNodes with the same name, else returns the r2l-node with calculated stv.
"
    (let ((winst (cog-chase-link 'ReferenceLink 'WordInstanceNode r2l-node)))
        (if (nil? winst)
            (let* ((name (string-downcase (cog-name r2l-node)))
                (concept (cog-node 'ConceptNode name)))

                (if (nil? concept)
                    (cog-set-tv! r2l-node
                        (stv (/ 1 total-count) (/ 1 (+ 1 default-k))))
                    (cog-set-tv! r2l-node (cog-tv concept))
                )
            )

            (let* ((name (string-downcase (cog-name
                        (word-inst-get-lemma (car winst)))))
                (concept (cog-node 'ConceptNode name)))

                (if (nil? concept)
                    (cog-set-tv! r2l-node
                        (stv (/ 1 total-count) (/ 1 (+ 1 default-k))))
                    (cog-set-tv! r2l-node (cog-tv concept))
                )
            )
        )
    )
)

; -----------------------------------------------------------------------
(define-public (r2l-count sent-list)
"
  r2l-count SENT -- maintain counts of R2L atoms for SENT-LIST.

  XXX FIXME ... this does NOT maintain counts in the same way as
  all the other code ... i.e. it does NOT use CountTV.

  This is some experimental? code and it might be obsolete?

  sent-list:
  - A list of SentenceNodes
"
    (define (update-tv atom)
        (if (cog-node? atom)
            (update-node-stv atom)
            ; Set all links as true.
            (cog-set-tv! atom (stv 1 1))
        ))

    (define (count-one-parse p)
        (let* ((r2l-outputs (parse-get-r2l-outputs p))
            (all-nodes (delete-duplicates
                (append-map cog-get-all-nodes r2l-outputs))))

            (map update-tv all-nodes)
            (map update-tv r2l-outputs)
        )
    )
    (for-each
        (lambda (sent)
            (for-each count-one-parse (sentence-get-parses sent)))
        sent-list)
)
