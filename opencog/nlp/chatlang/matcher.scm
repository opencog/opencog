;; ChatLang DSL for chat authoring rules
;;
;; This is the custom action selector that allows OpenPsi to find the authored
;; rules.
;; TODO: This is not needed in the long run as the default action selector in
;; OpenPsi should be able to know how and what kinds of rules it should be
;; looking for at a particular point in time.

(use-modules (opencog logger))

(define-public (chat-find-rules sent-node)
  "The action selector. It first searches for the rules using DualLink,
   and then does the filtering by evaluating the context of the rules.
   Eventually returns a list of weighted rules that can satisfy the demand"
  ; Get the lemma of the words associate with sent-node
  (define (get-sent-lemmas sent-node)
    (List (append-map
      (lambda (w)
        ; Ignore LEFT-WALL and punctuations
        (if (or (string-prefix? "LEFT-WALL" (cog-name w))
                (word-inst-match-pos? w "punctuation")
                (null? (cog-chase-link 'LemmaLink 'WordNode w)))
            '()
            ; For proper names, e.g. Jessica Henwick,
            ; RelEx converts them into a single WordNode, e.g.
            ; (WordNode "Jessica_Henwick"). Codes below try to
            ; split it into two WordNodes, "Jessica" and "Henwick",
            ; so that the matcher will be able to find the rules
            (let* ((wn (car (cog-chase-link 'LemmaLink 'WordNode w)))
                   (name (cog-name wn)))
              (if (integer? (string-index name #\_))
                (map Word (string-split name  #\_))
                (list wn)))))
      (car (sent-get-words-in-order sent-node)))))
  (let* ((lemma-list (get-sent-lemmas sent-node))
         (rules-matched (append (psi-get-dual-match lemma-list)
                                (psi-get-exact-match lemma-list))))
    (cog-logger-debug "For input:\n~aRules found:\n~a" lemma-list rules-matched)
    ; TODO: Pick the ones with the highest weight
    (List (append-map
      ; TODO: "psi-satisfiable?" doesn't work here (?)
      (lambda (r)
        (if (equal? (stv 1 1)
                    (cog-evaluate! (car (psi-get-context (gar r)))))
            (list (gar r))
            '()))
      rules-matched))))

(Define
  (DefinedSchema "Get Current Input")
  (Get (State (Anchor "Currently Processing")
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
  yakking)
