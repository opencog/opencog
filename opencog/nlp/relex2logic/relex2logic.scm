
(define-module (opencog nlp relex2logic))

; (add-to-load-path "/usr/local/share/opencog/scm")

(use-modules (srfi srfi-1))
(use-modules (opencog))
(use-modules (opencog spacetime))
(use-modules (opencog nlp))
(use-modules (opencog rule-engine))

(load "relex2logic/rule-utils.scm")
(load "relex2logic/r2l-utilities.scm")
(load "relex2logic/tv-utilities.scm")
(load "relex2logic/post-processing.scm")

; -----------------------------------------------------------------------
(define-public (r2l-parse SENT)
"
  r2l-parse SENT -- perform relex2logic processing on sentence SENT.

  Runs the rules found in `R2L-en-RuleBase` over the RelEx output,
  creating the logical representation of sentence in the atomspace.
  Returns a list of `SetLinks`, each `SetLink` holding the R2L
  interpretation of one parse of the sentence.

  This can't handle  mutliple thread execution (Why??? This is a bug,
  I would think ...). Thus, mapping this function over a list of
  sentences, even though possible, is not advised.

  SENT must be a SentenceNode.
"
    (define (unwrap-list-link A-LINK IS-FROM-FC)
        ; Many rules return a ListLink of the results that
        ; they generated. Some rules return singletons. The
        ; Forward Chainer uses a SetLink to wrap all these
        ; results. So, if A-LINK is a ListLink or is directly
        ; from the FC, then delete it and return a list of
        ; its contents, else return a list holding A-LINK.
        ;
        ; XXX maybe this should be part of the ure module??
        (if (or (equal? 'ListLink (cog-type A-LINK)) IS-FROM-FC)
            (let ((returned-list (cog-outgoing-set A-LINK)))
                    (cog-extract A-LINK)
                    returned-list)
            (list A-LINK))
    )

    (define (apply-r2l-rules PARSE-NODE INTERP-LINK)
        ; This applies all the rules in the R2L-en-RuleBase to the
        ; RelEx parse, and returns a cleaned and de-duplicated list.
        ; The RelEx outputs associated with `PARSE-NODE` are the
        ; focus-set. Thus, if there are multiple parses, then
        ; each is handled independently, by passing it seperately,
        ; since each is likely to exist in a seperate semantic-universe.
        (define focus-set
            (SetLink (parse-get-relex-outputs PARSE-NODE) INTERP-LINK))
        (define outputs
            (unwrap-list-link (cog-fc r2l-rules (Set) #:focus-set focus-set) #t))

        (append-map (lambda (o) (unwrap-list-link o #f)) outputs)
    )

    (define (interpret PARSE-NODE)
        ; FIXME: Presently, only a single interpretation is created for
        ; each parse. Multiple interpreation should be handled, when
        ; word-sense-disambiguation, anaphora-resolution and other
        ; post-processing are added to the pipeline.
        (let* (
                (interp-name (string-append
                    (cog-name PARSE-NODE) "_interpretation_$X"))

                (interp-node (InterpretationNode interp-name))

                ; Associate the interpretation with a parse, as there
                ; could be multiplie interpretations for the same parse.
                (interp-link (InterpretationLink interp-node PARSE-NODE))

                (pre-result (remove
                    (lambda (a) (equal? (cog-type a) 'ReferenceLink))
                    (delete-duplicates
                        (apply-r2l-rules PARSE-NODE interp-link))))

                (result (SetLink pre-result)))

            ; Construct a ReferenceLink to the output
            (ReferenceLink interp-node result)

            ; Return the SetLink of R2L rule outputs.
            result
        )
    )

    ;; Can this safely be made parallel ???
    (map interpret (sentence-get-parses SENT))
)
; -----------------------------------------------------------------------

; This loads all the rules into the atomspace
(define-public (load-r2l-rulebase)

	; "." in case the cogserver is started from in-source build directory.
	(add-to-load-path ".")
	(load "relex2logic/rule-utils.scm")  ; XXX
	(load "relex2logic/rule-helpers.scm")
	(load "relex2logic/loader/load-rules.scm")  ; XXX
	(load "relex2logic/loader/gen-r2l-en-rulebase.scm")

	*unspecified*  ; no return value, avoids printing gunk.
)

(load-r2l-rulebase)

; -----------------------------------------------------------------------
*unspecified*
