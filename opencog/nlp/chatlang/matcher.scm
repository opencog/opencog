;; ChatLang DSL for chat authoring rules
;;
;; This is the custom action selector that allows OpenPsi to find the authored
;; rules.
;; TODO: This is not needed in the long run as the default action selector in
;; OpenPsi should be able to know how and what kinds of rules it should be
;; looking for at a particular point in time.

(use-modules (opencog logger))

(define globs '())
(define (ground-globs pattern input)
  "Use MapLink to ground the GlobNodes in the pattern and save them in 'globs'."
  ; For example, if the pattern is:
  ;   (ListLink (WordNode "A") (GlobNode "$x"))
  ;
  ; and the input is:
  ;   (ListLink (WordNode "A") (WordNode "B"))
  ;
  ; Create an association list:
  ;   (("$x" . (WordNode "B")))
  ;
  ; MapLink returns things like e.g.:
  ; (SetLink
  ;   (ListLink
  ;     (ListLink ...)  ; GlobNode-1
  ;     (ListLink ...)  ; GlobNode-2
  ;     ...))
  (define grds (cog-execute! (Map pattern (Set input))))
  (for-each
    (lambda (gn grd)
      (set! globs (assoc-set! globs (cog-name gn) grd)))
    (cog-filter 'GlobNode (cog-outgoing-set pattern))
    ; TODO: There may be more than one possible groundings?
    (cog-outgoing-set (gar grds))))

(define-public (chat-find-rules sent-node)
  "The action selector. It first searches for the rules using DualLink,
   and then does the filtering by evaluating the context of the rules.
   Eventually returns a list of weighted rules that can satisfy the demand"
  (let* ((input-lemmas (get-sent-lemmas sent-node))
         (no-constant (append-map psi-get-exact-match
           (cog-chase-link 'InheritanceLink 'ListLink chatlang-no-constant)))
         (dual-match (psi-get-dual-match input-lemmas))
         (exact-match (psi-get-exact-match input-lemmas))
         (rules-matched (append dual-match exact-match no-constant)))
    (cog-logger-debug "For input:\n~aRules found:\n~a" input-lemmas rules-matched)

    ; Clear any previous groundings before grounding the above GlobNodes
    (set! globs '())
    (if (not (null? dual-match)) (ground-globs dual-match input-lemmas))
    (if (not (null? no-constant)) (ground-globs no-constant input-lemmas))
    (cog-logger-debug "GlobNode groundings: ~a" globs)

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
