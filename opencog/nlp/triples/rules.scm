;
; rules.scm
;
; An experimental set of copula/preposition-mangling rules.
;
; Some quick notes on evaluation: conjuncts are evaluated from 
; first to last, and so the order of the terms matters. Terms that
; narrow down the search the most dramatically should come first,
; so as to avoid an overly-broad search of the atomspace.
;
; All of these rules are structured so that a search is performed
; only over sentences That are tagged with a link to the node
; "# APPLY TRIPLE RULES". Since this rule is first, this prevents 
; a search over the entire atomspace.
;
;
; -----------------------------------------------------------------
; get-anon-var-id! given a string, return a new unique string
; Given an input string, this returns a new, more-or-less unique
; new string.  Only minimal measures are taken to ensure uniqueness;
; so the result is not "strong". This is intended for use in generating
; "unique" variable names in ImplicatinLinks
;
; Example usage:
;     > (get-anon-var-id! "beee")
;     > $anon-beee-3

(define *anon-var-id* 0)
(define *anon-prefix* "$anon-")
(define (get-anon-var-id! str)
	(set! *anon-var-id* (1+ *anon-var-id*))
	(string-concatenate/shared
		(list *anon-prefix* str "-"
			(call-with-output-string 
				(lambda (port)
					(display *anon-var-id* port)
				)
			)
		)
	)
)

(define (set-anon-prefix! str) (set! *anon-prefix* str))

; -----------------------------------------------------------------
; r-isvar? -- return #t if arg is a variable name, i.e. starts with $
; Returns #t if the string starts with a $

(define (r-isvar? v)
	(eq? #\$ (string-ref v 0))
)

; -----------------------------------------------------------------
; r-rlx -- format a RelEx-like expression
; The r-rlx routine takes a RelEx-like expression and outputs a list
; of appropriate OpenCog linbk structures.  Thus, for example
; _subj(be, $var) is exprssed as (r-rlx "_subj" "be" "$var")
; which is then processed into the appropriate OpenCog graph.
; An appropriate indirection for "be" as a word-instance of the 
; word "be" is made.
;
; Example:
;      (r-rlx "_subj" "be" "$var0")   ;; _subj(be, $var0)
;
(define (r-rlx rel a b)

	; The lemma-link needed for joining words to wod-instances.
	(define (lem var wrd)
		(LemmaLink (stv 1 1)
			(VariableNode var)
			(WordNode wrd)
		)
	)

	; Create a basic variable typing declaration
	(define (var-type var type)
		(TypedVariableLink
			(VariableNode var)
			(VariableTypeNode type)
		)
	)

	; prepend to list, maybe
	(define (vpend bool name lst)
		(define (pend bool item lst)
			(if bool (cons item lst) lst)
		)
		(pend bool (var-type name "WordInstanceNode") lst)
	)

	(let* (
			; av, bv are true if a,b start with $
			(av (r-isvar? a))
			(bv (r-isvar? b))
			(rv (r-isvar? rel))

			; avn is the variable name to use
			(avn (if av a (get-anon-var-id! a)))
			(bvn (if bv b (get-anon-var-id! b)))
			(rvn (if rv rel (get-anon-var-id! rel)))

			; The basic RelEx predicate structure
			(pred
				(EvaluationLink (stv 1 1)
					(if rv
						(VariableNode rvn)
						(DefinedLinguisticRelationshipNode rel)
					)
					(ListLink
						(VariableNode avn)
						(VariableNode bvn)
					)
				)
			)
		)

		; The clauses needed to work with this relex expression
		(define clauses
			(cond
				((and av bv) (list pred))
				((and (not av) bv)
					(list pred (lem avn a))
				)
				((and av (not bv))
					(list pred (lem bvn b))
				)
				((and (not av) (not bv))
					(list pred (lem avn a) (lem bvn b))
				)
			)
		)
		
		; The variables appearing in the clauses
		(define vartypes
			(vpend (not av) avn
				(vpend (not bv) bvn '())
			)
		)

		; Return the variables and clauses in an association list
		(alist-cons 'vardecls vartypes
			(alist-cons 'clauses clauses '())
		)
	)
)

; -----------------------------------------------------------------
; r-and -- concatenate a list of r-clauses together.
; Accepts a variable number of clauses and returns a concatenation 
; of them into one clause structure.
;
; Example usage:
;    (r-and
;       (r-rlx "_subj" "be" "$var0")
;       (r-rlx "_obj" "be" "$var1")
;    )
;
(define (r-and . alst)

	; Merge two alists together into one.
	(define (merge item otem)
		(let* (
				; ivp == item variable pair
				(ivp (assoc 'vardecls item))
				; iv == item variables
				(iv (if ivp (cdr ivp) '()))
				
				; icp == item clauses pair
				(icp (assoc 'clauses item))
				; ic == item clauses
				(ic (if icp (cdr icp) '()))
				
				; ovp == other item variable pair
				(ovp (assoc 'vardecls otem))
				; ov == other item variables
				(ov (if ovp (cdr ovp) '()))
				
				; ocp == other item clauses pair
				(ocp (assoc 'clauses otem))
				; oc == other item clauses
				(oc (if ocp (cdr ocp) '()))

				;; concatenate
				(varbles (append iv ov))
				(clauses (append ic oc))
			)


			; Return the variables and clauses in an association list
			(alist-cons 'vardecls varbles
				(alist-cons 'clauses clauses '())
			)
		)
	)

	; concatenate a bunch of alists into one
	(define (do-merge lst reslt)
		(if (not (null? lst))
			(do-merge (cdr lst)
				(merge (car lst) reslt)
			)
			reslt
		)
	)

	; And now do it.
	(do-merge alst '())
)

; -----------------------------------------------------------------
; r-link -- declare a simple opencog link connecting two items
; 
; Example usage:
;   (r-link WordInstanceLink "$var1" "$sent")
;
(define (r-link lnk-type item-1 item-2)
	(define lnk
		(lnk-type (stv 1 1)
			(VariableNode item-1)
			(VariableNode item-2)
		)
	)
	(alist-cons 'clauses (list lnk) '())
)

; -----------------------------------------------------------------
; r-decl-var -- declare a variable type
;
(define (r-decl-var vartype varname)
	(define (vd vtype vname)
		(TypedVariableLink
			(VariableNode vname)
			(VariableTypeNode vtype)
		)
	)
	(alist-cons 'vardecls (list (vd vartype varname)) '())
)

; -----------------------------------------------------------------
; r-anchor -- link a variable to an anchor
;
(define (r-anchor anchor-name var)
	(define lnk
		(ListLink (stv 1 1)
			(AnchorNode anchor-name)
			(VariableNode var)
		)
	)
	(alist-cons 'clauses (list lnk) '())
)

; -----------------------------------------------------------------
; r-anchor-trips -- declare a sentence anchored to a rule
;
(define (r-anchor-trips sent)
	(r-and 
		(r-anchor "# APPLY TRIPLE RULES" sent)
		(r-decl-var "SentenceNode" sent)
	)
)

; -----------------------------------------------------------------
; r-decl-word-inst -- declare a word instance belonging to a sentence
;
(define (r-decl-word-inst word-inst sent)
	(r-and
		(r-link WordInstanceLink word-inst sent)
		(r-decl-var "WordInstanceNode" word-inst)
	)
)

; -----------------------------------------------------------------
; r-decl-lemma -- declare a lemma of a word-instance
;
(define (r-decl-lemma word-inst lemma)
	(r-and
		(r-link LemmaLink word-inst lemma)
		(r-decl-var "WordNode" lemma)
	)
)
; -----------------------------------------------------------------
(define (x)
	; Sentence: "Lisbon is the capital of Portugaul"
	; The RelEx parse is:
	;   _subj(be, Lisbon)
	;   _obj(be, capital)
	;   of(capital, Portugaul)
	;
	; We expect the following variable grounding:
	; var0=Lisbon, var1=capital var2=Portugaul
	;
	(r-and
		; We are looking for sentences anchored to the 
		; triples-processing node.
		(r-anchor-trips "$sent")
	
		; $var0 and $var1 are word-instances that must 
		; belong to the same sentence.
		(r-decl-word-inst "$var0" "$sent") 
		(r-decl-word-inst "$var1" "$sent")
	
		; Match subject and object as indicated above.
		(r-rlx "_subj" "be" "$var0")
		(r-rlx "_obj" "be" "$var1")

		; Match the proposition
		(r-rlx "$prep" "$var1" "$var2")

		; Get the lemma form of the word instance
		(r-decl-lemma "$var1" "$word1")

		; Convert to a phrase
		(r-rlx "$phrase" "$word1" "$prep")
	)
)

(define (r-ifthen P Q)
	(ImplicationLink  P Q)
)

