(define-public (ghost-reuse LABEL)
  "The Schema function being called by (DefinedSchemaNode \"reuse\")."
  (define rule (cog-chase-link 'ListLink 'ImplicationLink
                               (Concept (string-append "OpenPsi: " (cog-name LABEL)))))
  (if (null? rule)
      (cog-logger-error ghost "Failed to find the GHOST rule \"~a\"" (cog-name LABEL))
      (psi-get-action (car rule))))

(Define
  (DefinedSchema "reuse")
  (Lambda (VariableList (TypedVariable (Variable "$x")
                                       (Type "WordNode")))
          (ExecutionOutput (GroundedSchema "scm: ghost-reuse")
                           (List (Variable "$x")))))
