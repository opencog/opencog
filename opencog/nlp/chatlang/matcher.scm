;; ChatLang DSL for chat authoring rules
;;
;; This is the custom action selector that allows OpenPsi to find the authored
;; rules.
;; TODO: This is not needed in the long run as the default action selector in
;; OpenPsi should be able to know how and what kinds of rules it should be
;; looking for at a particular point in time.

(use-modules (opencog logger)
             (opencog exec))

; For storing the groundings
(define globs-word '())
(define globs-lemma '())

(define-public (show-globs)
  "For debugging only."
  (display "globs-word: ") (display globs-word) (newline)
  (display "globs-lemma: ") (display globs-lemma) (newline))

(define (get-bindlinks SEQ)
  "Get the BindLink that contains SEQ."
  ; TODO: Delete the Dual-sets
  (delete-duplicates (cog-filter 'BindLink (concatenate (map cog-get-trunk SEQ)))))

(define-public (chat-find-rules SENT)
  "The action selector. It first searches for the rules using DualLink,
   and then does the filtering by evaluating the context of the rules.
   Eventually returns a list of weighted rules that can satisfy the demand."
  ; Clear any previous groundings
  (set! globs-word '()) (set! globs-lemma '())
  (let* ((input-lemmas (cdr (sent-get-word-seqs SENT)))
         ; The ones that contains no variables/globs
         (exact-match (get-bindlinks (list input-lemmas)))
         ; The ones that contains no constant terms in the term-seq
         (no-const (get-bindlinks
           (cog-chase-link 'MemberLink 'ListLink chatlang-no-constant)))
         ; The ones found by the recognizer
         (dual-match (get-bindlinks
           (cog-outgoing-set (cog-execute! (Dual input-lemmas)))))
         ; The ones that can actually be grounded (a match)
         (bind-grd (filter (lambda (b)
           (not (equal? (Set) (cog-execute! b))))
             (append no-const dual-match exact-match)))
         ; Get the psi-rules associate with them
         (rules-matched (append-map (lambda (b)
           (cog-chase-link 'ReferenceLink 'ImplicationLink b)) bind-grd)))

        (cog-logger-debug "For input:\n~a" input-lemmas)
        (cog-logger-debug "Rules with no constant:\n~a" no-const)
        (cog-logger-debug "Exact match:\n~a" exact-match)
        (cog-logger-debug "Dual match:\n~a" dual-match)
        (cog-logger-debug "Grounded:\n~a" bind-grd)
        (cog-logger-debug "Rules matched:\n~a" rules-matched)

        ; TODO: Pick the one with the highest weight
        (List (append-map
          ; TODO: "psi-satisfiable?" doesn't work here (?)
          (lambda (r)
            (if (equal? (stv 1 1)
                        (cog-evaluate! (car (psi-get-context r))))
                (list r)
                '()))
          rules-matched))))

(Define
  (DefinedSchema "Get Current Input")
  (Get (State chatlang-anchor
              (Variable "$x"))))

(Define
  (DefinedSchema "Find Chat Rules")
  (Lambda (VariableList (TypedVariable (Variable "sentence")
                                       (Type "SentenceNode")))
          (ExecutionOutput (GroundedSchema "scm: chat-find-rules")
                           (List (Variable "sentence")))))

; The action selector for OpenPsi
(psi-set-action-selector
  (Put (DefinedSchema "Find Chat Rules")
       (DefinedSchema "Get Current Input"))
  default-topic)
