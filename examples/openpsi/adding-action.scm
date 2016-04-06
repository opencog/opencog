; Copyright (C) 2016 OpenCog Foundation

(load-from-path "openpsi/demand.scm")

; When defining a new demand the default action should be an ExecutionOutputLink
; with a scm/python function that at-least takes in the demand-node and modify
; its truthvalue. The demand-node that is passed to the functions must be
; represented by 'demand-var'. The variable 'demand-var' is defined in the
; "openpsi/demand.scm".

(define (sociality-action)
    (ExecutionOutputLink
        (GroundedSchemaNode "scm: sociality-behavior")
        (ListLink demand-var))
)

; The node that represents the number of faces.
; This is a helper function
(define (faces-node) (ConceptNode "Total number of faces around"))

(define (sociality-behavior demand-node)
"
  The function that characterizes what sociality is.

  Sociality can be measured by whether people stayed around the robot or went
  away.  So in the current scheme of things: if faces are present, sociality
  is being fulfilled. The more faces the better!. Thus when there are no faces
  sociality is zero.

  The atomese representation used for following on the number of faces percived
  currently is assumed to be
      (StateLink
          (ConceptNode \"Total number of faces around\")
          (NumberNode 0))

  demand-node:
  - The Node that represents Sociality.
"

    ; A function to return the number of faces perceived
    (define (num-of-faces)
        (string->number (cog-name (car
            (cog-chase-link 'StateLink 'NumberNode (faces-node))))))

    ; Update the strength of the demand-node using 'n/(n+1)' where n is face-num
    ; variable below.
    (let ((face-num (num-of-faces))
          (conf (tv-conf (cog-tv demand-node))))
        (cog-set-tv! demand-node (stv (/ face-num (+ face-num  1))  conf))
    )
)

; NOTE: Ther is nothing preventing one from using the above function for
; use with another demand. But, in such a case it is best to make the functions
; generic.
