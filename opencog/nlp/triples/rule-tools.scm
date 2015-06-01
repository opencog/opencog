;
; rules.scm
; Copyright (c) 2009 Linas Vepstas
;
; A set of routines for constructing ImplicationLinks and 
; BindLinks in a "simplified" manner. The goal here is
; to make it easier to write, read and debug hand-written 
; Implication/VarScope links.
;
; All of the names of the routines here begin with the string "r-".
; Most of the routines take as input, or create as output, an object
; called an "r-expression".  The "r-expression" is a very simple object:
; it is just a scheme association-list with three items: "clauses", 
; "vardecls" and "freevars".  The "clauses" are just a list of clauses
; that will eventually be and'ed together to create an ImplicationLink.
; The "vardecls" is a list of variable declarations that will appear in
; the final BindLink that is constructed.  The "freevars" is a 
; list of variables that were found along the way; these will be promoted
; to bound variables in the final varscope.
;
; A "real-life" example is given below; it constructs the
; BindLink needed to convert the parsed sentence 
; "Lisbon is the capital of Portugaul" into the semantic triple
; "capital_of(Porutgaul, Lisbon)".
;
;   (define my-varscope-link
;
;      ; Sentence: "Lisbon is the capital of Portugaul"
;      ; The RelEx parse is:
;      ;   _subj(be, Lisbon)
;      ;   _obj(be, capital)
;      ;   of(capital, Portugaul)
;      ;
;      ; We expect the following variable grounding:
;      ; var0=Lisbon, var1=capital var2=Portugaul
;      ;
;      (r-varscope
;         (r-and
;            ; We are looking for sentences anchored to the 
;            ; triples-processing node.
;            (r-anchor-trips "$sent")
;         
;            ; $var0 and $var1 are word-instances that must 
;            ; belong to the same sentence.
;            (r-decl-word-inst "$var0" "$sent") 
;            (r-decl-word-inst "$var1" "$sent")
;         
;            ; Match subject and object as indicated above.
;            (r-rlx "_subj" "be" "$var0")
;            (r-rlx "_obj" "be" "$var1")
;      
;            ; Match the proposition
;            (r-rlx "$prep" "$var1" "$var2")
;   
;            ; Get the lemma form of the word instance
;            (r-decl-lemma "$var1" "$word1")
;   
;            ; Convert to a phrase
;            (r-rlx "$phrase" "$word1" "$prep")
;         )
;         ; The implicand
;         (r-rlx "$phrase" "$var2" "$var0")
;      )
;   )
;
; The comparable IF...THEN expression to the above is:
;
; # IF %ListLink("# APPLY TRIPLE RULES", $sent)
;       ^ %WordInstanceLink($var0,$sent)  ; $var0 and $var1 must be
;       ^ %WordInstanceLink($var1,$sent)  ; in the same sentence
;       ^ _subj(be,$var0)
;       ^ _obj(be,$var1)
;       ^ $prep($var1,$var2)              ; preposition 
;       ^ %LemmaLink($var1,$word1)        ; word of word instance
;       ^ $phrase($word1, $prep)          ; convert to phrase
;       THEN ^3_$phrase($var2, $var0) 
;
; Some quick notes on evaluation: conjuncts are evaluated from 
; first to last, and so the order of the terms matters. Terms that
; narrow down the search the most dramatically should come first,
; so as to avoid an overly-broad search of the atomspace.
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
;
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
; This routine returns a r-expression containing the clause and the
; variables appearing in that clause.
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
			(TypeNode type)
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
		; XXX FIXME: really, if a or b are vars, then they are WordInstanceNodes.
		; XXX However, to fix this, we will need to modify the varscope code to
		; merge together lits of possibly duplicate var decls!
		; XXX err, well, no, since b can sometimes be a 
		; DefinedLinguisticRelationshipNode when building a prep-phrase
		; XXX SemeNodes can appear here as well.
		(r-new-expr vartypes clauses (r-fv rel a b))
	)
)

; -----------------------------------------------------------------
; r-new-expr -- constructor for the r-expression object
;
; Create a new r-expression object. Tkes three arguments: 
; a list of variable declarations, a list of clauses, and a list
; of free variables.
;
(define (r-new-expr vdcl claus freev)
	(alist-cons 'vardecls vdcl
		(alist-cons 'clauses claus 
			(alist-cons 'freevars  freev '())
		)
	)
)

; -----------------------------------------------------------------
; Getters for the r-expression object, returning the vardecls, clauses
; and freevars.

(define (r-get-vardecls expr)
	(let ((evp (assoc 'vardecls expr)))
		(if evp (cdr evp) '())
	)
)

(define (r-get-freevars expr)
	(let ((evp (assoc 'freevars expr)))
		(if evp (cdr evp) '())
	)
)

(define (r-get-clauses expr)
	(let ((evp (assoc 'clauses expr)))
		(if evp (cdr evp) '())
	)
)

; -----------------------------------------------------------------
; r-not -- wrap each clause of an r-expression with a NotLink
;
; Caution: because this wraps *each clause* in the r-expression
; in a NotLink, you may not quite get what you were expecting,
; since more complex r-expressions may have many caluses in them!
; This works best when the r-expression just contains a *single* 
; clause!
;
; Returns an r-expression.
;
; Example usage:
;    (r-not (r-link InheritanceLink "$var1" "$var2"))
;
(define (r-not expr)

	; split out the variables and clauses;
	; invert the clauses 
	(let* (
			; ev == expr variables
			(ev (r-get-vardecls expr))

			; ec == expr clauses
			(ec (r-get-clauses expr))

			; nc == inverteted clauses
			(nc (map NotLink ec))
		)

		; Return the variables and clauses in an association list
		(r-new-expr ev nc (r-get-freevars expr))
	)
)

; -----------------------------------------------------------------
; r-and -- concatenate a list of r-expressions together.
;
; Accepts a variable number of r-expressions and returns a concatenation 
; of them into one r-expression structure. The order of the clauses is
; preserved.
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
				; iv == item variables
				(iv (r-get-vardecls item))
				(ic (r-get-clauses  item))
				(ir (r-get-freevars item))
				
				; ov == other item variables
				(ov (r-get-vardecls otem))
				(oc (r-get-clauses  otem))
				(or (r-get-freevars otem))

				;; concatenate
				(varbles (append ov iv))
				(clauses (append oc ic))
				(freevrs (append or ir))
			)

			; Return the variables and clauses in an association list
			(r-new-expr varbles clauses (delete-duplicates freevrs))
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
; r-link -- declare a simple opencog link holding variables
;
; Returns an r-expression defining the link.
; Caution: at this time, assumes that *all* arguments are 
; variables!  May need to fix this as appropriate!!
; 
; Example usage:
;   (r-link WordInstanceLink "$var1" "$sent")
;
; will expand into the following:
;    (WordInstanceLink (stv 1 1)
;       (VariableNode "$var1")
;       (VariableNode "$sent")
;    )
;
(define (r-link lnk-type . items)
	(define lnk
		(lnk-type (stv 1 1)
			(map VariableNode items)
		)
	)
	(r-new-expr '() (list lnk) (r-fvl items))
)

; -----------------------------------------------------------------
; r-decl-var -- declare a (bound) variable
;
; Returns an r-expression holding variable declarations
; 
; Example usage:
;   (r-decl-var "$var1" "$sent")
;
(define (r-decl-var . items)
	(r-new-expr (list (map VariableNode items)) '() '())
)

; -----------------------------------------------------------------
; r-decl-freevar -- declare a (free) variable
;
; Returns an r-expression holding free-variable declarations.
; The declared variables *must* begine with a $, else they will not
; actually be treted as variables.
; 
; Example usage:
;   (r-decl-freevar "$var1" "$sent")
;
(define (r-decl-freevar . items)
	(r-decl-freevarlist items)
)

(define (r-decl-freevarlist vlist)
	(r-new-expr '() '() (r-fvl vlist))
)

; A nearly-equivalent internal-use-only routine.
(define (r-fv . items) 
	(r-fvl items)
)

(define (r-fvl items)
	(map VariableNode 
		(filter r-isvar? items)  ;; only if its actually a var..
	)
)

; -----------------------------------------------------------------
; r-decl-vartype -- declare a variable type
;
; During pattern matching, a variable can be constrained to be of a 
; a certain type, so that it is not matched too freely. This is done
; by using TypedVariableLink's and TypeNode's. 
;
; This routine returns an r-expression.
;
; Example usage:
;	(r-decl-vartype "WordInstanceNode" "$var0")
;
(define (r-decl-vartype vartype varname)
	(define (vd vtype vname)
		(TypedVariableLink
			(VariableNode vname)
			(TypeNode vtype)
		)
	)
	(r-new-expr (list (vd vartype varname)) '() '())
)

; -----------------------------------------------------------------
; r-varscope -- create a varscope structure from r-expressions
;
; A BindLink consists of an implication link P->Q, with the
; predicate P being a sequence of disjuncts (to be conjoined together)
; and Q the implicand.  Here, both P and Q are taken to be r-expressions,
; constructed with the r-* routines.  A BindLink also contains
; a list of the bound variables which will be grounded when the 
; implication is evaluated. The variables are taken from the predicate 
; P r-expression. Any variables appearing in Q must also appear in P.
;
; During the construction of the varscope, all variables -- i.e. nodes
; with names that start with a $ -- are promoted to bound variables. 
; This is done "for convenience", so that the user does not have to 
; explicitly declare variables.
;
; Returns the constructed BindLink.
;
(define (r-varscope predicates implicand)

	; Return #t if the var appears in the variable-type declaration
	; else return #f.  The vartype is assumed to be of the form:
	; (TypedVariableLink (VariableNode "$abc") (TypeNode "WhateverNode"))
	; so that car of outgoping set identifies the variable.
	(define (is-vdecl? vartype var)
		(equal? var
			(car (cog-outgoing-set vartype))
		)
	)

	; Return #f if the var appears in the list of variable-type
	; declarations, else  return #t
	(define (in-vdecls? typelist var)
		(null? (filter (lambda (x) (is-vdecl? x var)) typelist))
	)

	; Delete from varlist any variables that appear in typelist
	(define (delvar typelist varlist)
		(filter (lambda (x) (in-vdecls? typelist x)) varlist)
	)

	(let* (
			; pv == predicates variables
			(pv (r-get-vardecls predicates))
				
			; pc == predicates clauses
			(pc (r-get-clauses predicates))

			; pf == predicates free-vars
			(pf (r-get-freevars predicates))

			; ic == implicand clauses
			(ic (r-get-clauses implicand))

			; ff == freevars not appearing in pv
			(ff (delvar pv pf))
		)

		; The Big Kahuna -- a list of variables, and the implication.
		(BindLink
			(VariableList pv ff)
			(AndLink pc)
			ic
		)
	)
)

; -----------------------------------------------------------------
; r-schema -- declare a GroundedSchemaNode
;
; Returns an r-expression defining the schema
; 
; Example usage:
;   (r-schema "scm:make-prep-phrase" "$word1" "$prep")
;
(define (r-schema schema-name . args)
	(define lnk
		(ExecutionOutputLink
			(GroundedSchemaNode schema-name)
			(ListLink
				(map VariableNode args)
			)
		)
	)
	(r-new-expr '() (list lnk) (r-fvl args))
)

; -----------------------------------------------------------------
; -----------------------------------------------------------------
; -----------------------------------------------------------------
; The routines below are "utilities" aimed primarily with the 
; specifics of the triples-processing code.  These are not "core"
; routines like those above.
;
; -----------------------------------------------------------------
; r-anchor -- link a variable to an AnchorNode
;
; Returns an r-expression.
;
(define (r-anchor-node anchor var)
	(define lnk
		(ListLink (stv 1 1)
			anchor
			(VariableNode var)
		)
	)
	(r-new-expr '() (list lnk) (r-fv var))
)

(define (r-anchor anchor-name var)
	(r-anchor-node (AnchorNode anchor-name) var)
)

; -----------------------------------------------------------------
; r-anchor-trips -- declare a sentence anchored to the triples anchor
;
; Returns an r-expression
;
(define (r-anchor-trips sent)
	(r-and 
		; This anchor is not yet defined when this file is loaded,
		; So do it manually. Should probably split up this file.
		; (r-anchor-node *ready-for-triples-anchor* sent)
		(r-anchor-node (AnchorNode "# APPLY TRIPLE RULES") sent)
		(r-decl-vartype "ParseNode" sent)
	)
)

; -----------------------------------------------------------------
; r-decl-word-inst -- declare a word instance belonging to a sentence
;
; This routine presumes that "word-inst" is a variable, and declares
; it as such. (It does NOT make sense to declare a non-variable to be 
; a part of a sentence -- this was already done during parse and
; shouldn't be redone).
;
; Returns an r-expression.
;
(define (r-decl-word-inst word-inst sent)
	(r-and
		(r-link WordInstanceLink word-inst sent)
		(r-decl-vartype "WordInstanceNode" word-inst)
	)
)

; -----------------------------------------------------------------
; r-decl-lemma -- declare a lemma of a word-instance
;
; If "lemma" is a variable, then it is declared to be typed as
; a WordNode.
;
; Returns an r-expression.
;
(define (r-decl-lemma word-inst lemma)
	(let* ((is-lem-var (r-isvar? lemma))
			(lem-lnk
				; If lemma is NOT a variable, then it MUST be a word node
				(LemmaLink (stv 1 1)
					(VariableNode word-inst)
					(if is-lem-var 
						(VariableNode lemma)
						(WordNode lemma)
					)
				)
			)
			(r-exp (r-new-expr '() (list lem-lnk) (r-fv word-inst lemma)))
		)

		; If lemma is a string begining with $, then declare it
		; to be a variable that must be a WordNode.
		(if is-lem-var
			(r-and r-exp (r-decl-vartype "WordNode" lemma))
			r-exp
		)
	) 
)

; -----------------------------------------------------------------
; r-decl-prep -- declare a preposition relation
; In order to get a meaningful result here, the "preps.scm" 
; preposition dictionary must be loaded.
;
; Assumes that both "prep" and "prep-word" are vairables, 
; and declares "prep" to be of type DefinedLinguisticRelationshipNode
; XXX this is wrong, it should be PrepositionalRelationshipNode ??? XXX
;
; Returns an r-expression.
;
(define (r-decl-prep prep prep-word)

	(r-and 
		(r-link ListLink prep prep-word)
		(r-decl-vartype "DefinedLinguisticRelationshipNode" prep)
	)
)

; -----------------------------------------------------------------
; r-rlx-flag -- create a ReleEx flag-style relationship
;
; Returns an r-expression defining the flag.
; 
; Example usage:
;   (r-rlx-flag "definite" "$var2")
;
; creates the equivalent to the RelEx DEFINITE-FLAG($var2)
;
(define (r-rlx-flag flag var)
	(define lnk
		(InheritanceLink (stv 1 1)
			(VariableNode var)
			(DefinedLinguisticConceptNode flag)
		)
	)
	(r-new-expr '() (list lnk) (r-fv var))
)

; -----------------------------------------------------------------
; Find the seme for this word-instance. Such an InheritanceLink
; will exist if and only if the seme is *definintely* correct for
; this word instance.
;
(define (r-seme-of-word-inst word-inst seme)
	(r-and 
		(r-link InheritanceLink word-inst seme)
		(r-decl-vartype "WordInstanceNode" word-inst)
		(r-decl-vartype "SemeNode" seme)
	)
)

; ------------------------ END OF FILE ----------------------------
; -----------------------------------------------------------------
