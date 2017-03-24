(define (chat-find-rules sent-node)
  "The action selector. It first searches for the rules using DualLink,
   and then does the filtering by evaluating the context of the rules.
   Eventually returns a list of weighted rules that can satisfy the demand"
  (let* ((word-list (get-sent-words sent-node))
         (rules-matched (psi-get-dual-match word-list)))
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
