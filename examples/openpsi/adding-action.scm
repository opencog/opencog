; Copyright (C) 2016 OpenCog Foundation

(use-modules (opencog) (opencog openpsi))

; XXX This example is incomplete! XXX FIXME!

; When defining a new demand, the default action should be an
; ExecutionOutputLink with a scm/python function that takes, as
; arguments, the demand-node. The function should modify the
; truthvalue of the demand node. The demand node MUST be
; (VariableNode "Demand"). Aditional arguments are allowed.

(define sociality-action
    (ExecutionOutputLink
        (GroundedSchemaNode "scm: sociality-behavior")
        (ListLink (VariableNode "Demand")))
)

; The node that represents the number of faces.
(define faces-node (ConceptNode "Total number of faces around"))

(define (sociality-behavior demand-node)
"
  This function characterizes what sociality is.

  Sociality can be measured by how much people stayed near the robot,
  as opposed to leaving.  If faces are present, then the demand of
  sociality is being fulfilled. The more faces the better! When there
  are no (visible) faces, the sociality is zero.

  The atomese representation used for the number of percieved faces
  is assumed to be
      (StateLink
          (ConceptNode \"Total number of faces around\")
          (NumberNode 0))

  demand-node:
  - The Node that represents Sociality.
"

    ; A function to return the number of faces perceived
    (define (num-of-faces)
        (string->number (cog-name (car
            (cog-chase-link 'StateLink 'NumberNode faces-node)))))

    ; Update the strength of the demand-node using `n/(n+1)` where
    ; `n` is the `face-num` variable below.
    (let ((face-num (num-of-faces))
          (conf (tv-conf (cog-tv demand-node))))
        (cog-set-tv! demand-node (stv (/ face-num (+ face-num  1))  conf))
    )
)

; NOTE: There is nothing preventing one from using the above function
; for ; use with another demand. But, in this case, it would be best to
; make the function generic.
