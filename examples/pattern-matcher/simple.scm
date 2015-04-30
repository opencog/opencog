;
; Define a hypergraphs using BindLink. These are the queries and
; the action to be taken on the result of the query
;
; The run this, you probably need to do this:
;
; export LTDL_LIBRARY_PATH=build/opencog/guile:build/opencog/query
; guile -L build -L opencog/scm
;
; (add-to-load-path "/home/yourname/opencog/build")
; (add-to-load-path "/home/yourname/opencog/opencog/scm")
; (add-to-load-path ".")
;
; and then load this file:
; (load-from-path "simple.scm")
;
; Then, scroll to the bottom, and some of the commented-out
; examples.

(use-modules (opencog))
(use-modules (opencog query))

(load-from-path "utilities.scm")

(define human
  (BindLink
     (VariableNode "$H") ; 
     (ImplicationLink
        ; This is the pattern that will be matched ...
        (InheritanceLink
           (VariableNode "$H")
           (ConceptNode "human")
        )
        ; This is what is returned if the above pattern is found.
        (VariableNode "$H")    
     )
  )
)


(define human-implies-animal
  (BindLink
     (VariableNode "$H") ; 
     (ImplicationLink
        ; This is the pattern that will be matched ...
        (InheritanceLink
           (VariableNode "$H")
           (ConceptNode "human")
        )
        ; This is the hypergraph that will be created if the 
        ; above pattern is found.
        (InheritanceLink
           (VariableNode "$H")
           (ConceptNode "animal")
        )
     )
  )
)


(define human-implies-animal-stv
  (BindLink
     (VariableNode "$H") ; 
     (ImplicationLink
        ; This is the pattern that will be matched ...
        (InheritanceLink
           (VariableNode "$H")
           (ConceptNode "human")
        )
        (ExecutionOutputLink
           (GroundedSchemaNode "scm: modify-stv")
           ; This is the list of arguments to pass to the formula.
           ; Notice that *two* arguments are passed.  The first
           ; argument is a repeat of the pattern that was matched,
           ; and the second argument is a hypergraph that will be
           ; created.
           (ListLink
              ; The schema will take the truth value from this link ...
              (InheritanceLink
                 (VariableNode "$H")
                 (ConceptNode "human")
              )
              ; and it will set the truth value here, after scaling by 0.3.
              (InheritanceLink
                 (VariableNode "$H")
                 (ConceptNode "animal")
              )
           )
        )
     )
  )
)

; Return a truth value where the strength was multiplied by 'val'
(define (scale-tv-strength val tv)
  (cog-new-stv
    (* val (cdr (assoc 'mean (cog-tv->alist tv))))
    (cdr (assoc 'confidence (cog-tv->alist tv)))
  )
)
 
; Define a formula that computes a truth value for atom2 based on atom1's stv.
(define (modify-stv atom1 atom2)
  ; Set the strength of the truth value on atom hb
  ; to be just 0.3 of the strength of atom ha.
  (cog-set-tv! atom2 (scale-tv-strength 0.3 (cog-tv atom1)))
  atom2  ; return atom hb
)

; Some data to populate the atomspace:
(InheritanceLink (stv 1 0.99)  ; a non-zero truth value is needed!
  (ConceptNode "Ben")
  (ConceptNode "human")
)

(InheritanceLink (stv 1 0.99)  ; a non-zero truth value is needed!
  (ConceptNode "Linas")
  (ConceptNode "human")
)

; Run the Pattern-Mather by invoking either of the following.
; (cog-bind human)
; (cog-bind human-implies-animal)
; (cog-bind human-implies-animal-stv)

;Expected output in the same order as the above invokation
; (SetLink
;    (ConceptNode "Ben")
;    (ConceptNode "Linas") 
; ) 

; (SetLink 
;     (InheritanceLink (ConceptNode "Linas") (ConceptNode "animal"))
;     (InheritanceLink (ConceptNode "Ben") (ConceptNode "animal"))
; )

; (SetLink
;    (InheritanceLink (stv 0.30000001 0.99000001)
;       (ConceptNode "Linas")
;       (ConceptNode "animal")
;    )
;    (InheritanceLink (stv 0.30000001 0.99000001)
;       (ConceptNode "Ben")
;       (ConceptNode "animal")
;    )
; )

