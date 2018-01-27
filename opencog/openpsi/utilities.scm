;
; utilities.scm
; Helper functions for OpenPsi
;
; Copyright (C) 2015-2016 OpenCog Foundation
; Copyright (C) 2017 MindCloud

(use-modules (ice-9 regex)) ; For string-match
(use-modules (srfi srfi-1)) ; For fold, delete-duplicates

(use-modules (opencog) (opencog exec))

; --------------------------------------------------------------
; XXX TODO: does this really need to be public? change into atom.
(define psi-prefix-str "OpenPsi: ")

; --------------------------------------------------------------
; XXX TODO: does this really need to be public?
(define (psi-suffix-str a-string)
"
  psi-suffix-str STRING

  Given the string STRING, this removes the psi prefix string.
"
    (let ((z-match (string-match psi-prefix-str a-string)))
        (if z-match
            (match:suffix z-match)
            (error (string-append "The string argument must have the prefix: "
                "\"" psi-prefix-str "\". " "Instead got:" a-string) )
        )
    )
)

; --------------------------------------------------------------
; It appears that the predicate psi-member? is not defined anywhere
; and so thecode below seems to be dead...
; 
;; DEAD CODE ;;
;; DEAD CODE ;;(define (psi-get-exact-match ATOM)
;; DEAD CODE ;;"
;; DEAD CODE ;;  psi-get-exact-match ATOM - Return list of all of the MemberLinks
;; DEAD CODE ;;  holding rules whose context or action apply exactly (without
;; DEAD CODE ;;  any variables) to the ATOM. In other words, the ATOM appears
;; DEAD CODE ;;  directly in the context of the rule.
;; DEAD CODE ;;
;; DEAD CODE ;;  All psi rules are members of some ruleset; this searches for and
;; DEAD CODE ;;  finds such MemberLinks.
;; DEAD CODE ;;"
;; DEAD CODE ;;    ;; Get all exact matches
;; DEAD CODE ;;    (define inset (cog-get-trunk ATOM))
;; DEAD CODE ;;
;; DEAD CODE ;;    ;; Keep only those links that are of type MemberLink...
;; DEAD CODE ;;    ;; and, more precisely, a MemberLink that is of a valid
;; DEAD CODE ;;    ;; psi-fule form.
;; DEAD CODE ;;    (filter psi-member?
;; DEAD CODE ;;        (delete-duplicates (cog-filter 'MemberLink inset)))
;; DEAD CODE ;;)
;; DEAD CODE ;;
;; DEAD CODE ;;(define (psi-get-dual-match ATOM)
;; DEAD CODE ;;"
;; DEAD CODE ;;  psi-get-dual-match ATOM - Return list of the MemberLinks
;; DEAD CODE ;;  holding rules whose context or action might apply to ATOM,
;; DEAD CODE ;;  as a generalized case (i.e. containining variables).
;; DEAD CODE ;;
;; DEAD CODE ;;  All psi rules are members of some ruleset; this searches for and
;; DEAD CODE ;;  finds such MemberLinks.
;; DEAD CODE ;;"
;; DEAD CODE ;;    (define set-of-duals (cog-execute! (DualLink ATOM)))
;; DEAD CODE ;;
;; DEAD CODE ;;    ;; Get all patterned rules
;; DEAD CODE ;;    (define duset
;; DEAD CODE ;;        (concatenate
;; DEAD CODE ;;            (map cog-get-trunk (cog-outgoing-set set-of-duals))))
;; DEAD CODE ;;
;; DEAD CODE ;;    ; Avoid garbaging up the atomspace.
;; DEAD CODE ;;    (cog-delete set-of-duals)
;; DEAD CODE ;;
;; DEAD CODE ;;    ;; Keep only those links that are of type MemberLink...
;; DEAD CODE ;;    ;; and, more precisely, a MemberLink that is of a valid
;; DEAD CODE ;;    ;; psi-fule form.
;; DEAD CODE ;;    (filter psi-member?
;; DEAD CODE ;;        (delete-duplicates (cog-filter 'MemberLink duset)))
;; DEAD CODE ;;)
;; DEAD CODE ;;
;; DEAD CODE ;;(define (psi-get-members ATOM)
;; DEAD CODE ;;"
;; DEAD CODE ;;  psi-get-members ATOM - Return list of all of the MemberLinks
;; DEAD CODE ;;  holding rules whose context or action might apply to ATOM.
;; DEAD CODE ;;
;; DEAD CODE ;;  All psi rules are members of some ruleset; this searches for and
;; DEAD CODE ;;  finds such MemberLinks.
;; DEAD CODE ;;"
;; DEAD CODE ;;    (delete-duplicates (concatenate! (list
;; DEAD CODE ;;        (psi-get-exact-match ATOM)
;; DEAD CODE ;;        (psi-get-dual-match ATOM)
;; DEAD CODE ;;    )))
;; DEAD CODE ;;)
;; DEAD CODE ;;
; --------------------------------------------------------------
(define (psi-set-func! function is-eval component-node function-name)
"
  psi-set-func! FUNC IS-EVAL COMPONENT FUNC-NAME

  Associate a function with a particular component.

  FUNC is an atom that can be executed or evaluated. It will perform
    the function for the particular component.

  IS-EVAL is a the string '#t' if the function is evaluatable and '#f' if
    it is executable.

  COMPONENT should be a component node that the function will
    be assocaited with.

  FUNC-NAME is the type of function.
"
  ; Record whether the function is evaluatable or executable.
  (cog-set-value!
    function
    (Predicate "is_evaluatable?")
    (StringValue is-eval))

  ; Record the function with the component-node used to represent it.
  (cog-set-value!
    component-node
    (Predicate function-name)
    function)
)

; --------------------------------------------------------------
(define (psi-func component-node function-name)
"
  psi-func COMPONENT FUNC-NAME

  Return the node that represents the function for the given component
  or nil if it doesn't exist.

  COMPONENT should be a component node that the function is set for.

  FUNC-NAME should be the type of function.
"
  (cog-value component-node (Predicate function-name))
)

; --------------------------------------------------------------
(define (psi-func-evaluatable? component-node function-name)
"
  psi-func-evaluatable? COMPONENT FUNC-NAME

  Return '#t' if the function is evaluatable and '#f' if
    it is executable.

  COMPONENT should be a component node that the function is set for.

  FUNC-NAME should be the type of function.
"
  (let ((func (psi-func component-node function-name)))
    (if (null? func)
      (error (format "A function called \"~a\" hasn't been set for ~a\n"
          function-name component-node))
      (equal? "#t"
        (cog-value-ref (cog-value func (Predicate "is_evaluatable?")) 0))
    )
  )
)

; --------------------------------------------------------------
