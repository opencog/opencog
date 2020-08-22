;
; post-processing.scm
;
; Assorted functions for post-processing relex2logic output.
;

(use-modules (ice-9 receive) (srfi srfi-1))

(use-modules (opencog))
(use-modules (opencog exec))

(load "r2l-utilities.scm")
(load "tv-utilities.scm")

; =======================================================================
; Helper utilities for post-processing.
; =======================================================================

; -----------------------------------------------------------------------
; word-get-r2l-node -- Retrieve corresponding R2L created node
;
; Given a WordInstanceNode/WordNode created by RelEx, retrieve the
; corresponding ConceptNode or PredicateNode or NumberNode or
; DefinedLinguisticPredicateNode created by R2L helper.
;
; XXX FIXME this method is really bad because for each new type of
; node that R2L uses, it needs to be added here.  There needs some
; different way for linking R2L nodes to WordInstanceNodes other
; than node name! e.g. maybe an R2LLink ?
;
(define (word-get-r2l-node node)
	(define name
		(if (not (nil? node))
			(cog-name node)
		)
	)

	(cond ((nil? node) '())
		((not (nil? (cog-node 'ConceptNode name))) (cog-node 'ConceptNode name))
		((not (nil? (cog-node 'PredicateNode name))) (cog-node 'PredicateNode name))
		((not (nil? (cog-node 'NumberNode name))) (cog-node 'NumberNode name))
		((not (nil? (cog-node 'DefinedLinguisticPredicateNode name))) (cog-node 'DefinedLinguisticPredicateNode name))
		(else '())
	)
)

; -----------------------------------------------------------------------
; r2l-is-unary? -- Check if 'link' is only about one word instance
;
; Given a link, check if it only contains one instanced node.  Used to
; ignore links that do not need to be post-processed.
;
; XXX FIXME except that we can have (EvaluationLink "not" "run@1234") which
;     appears unary but should be post-processed.  A more long term
;     solution is needed.
;
(define (r2l-is-unary? link)
	(define nodes (cog-get-all-nodes link))

	(<= (count is-r2l-inst? nodes) 1)
)

; -----------------------------------------------------------------------
; pairwise-combination -- Create all possible pairwise combination
;
; Given a list of lists, create all possible pairwise combination
; between lists.
;
(define (pairwise-combination sets)
	(define (create-with-index index rest)
		(map (lambda (r) (list index r)) rest))
	(define (recursive-helper index rest)
		(if (nil? (cdr rest))
			(create-with-index index rest)
			(append (create-with-index index rest)
				(recursive-helper (car rest) (cdr rest))
			)
		)
	)

	(recursive-helper (car sets) (cdr sets))
)

; -----------------------------------------------------------------------
; lset-pairwise-intersection -- Pairwise intersection
;
; Given a list of lists, do pairwise intersection, keeping any items
; that appear in more than one list.
;
(define (lset-pairwise-intersection sets)
	(define all-pairs (pairwise-combination sets))
	(append-map (lambda (pair-list) (lset-intersection equal? (car pair-list) (cadr pair-list)))
			all-pairs)
)

; -----------------------------------------------------------------------
; member? -- Custom member? function to return the item instead of a list
;
; Same as 'member' but return the original item instead of a list.
;
(define (member? x lst)
	(if (member x lst)
		x
		#f
	)
)

; -----------------------------------------------------------------------
; definite? -- Check if a ConceptNode, check if it is DEFINITE
;
; Check if a word-inst ConceptNode has definite-rule applied.
;
(define (definite? word-concept-inst)
	(define definite (DefinedLinguisticPredicateNode "definite"))
	(define llink (cog-link 'ListLink word-concept-inst))
	(and
		(not (nil? llink))
		(not (nil? (cog-link 'EvaluationLink definite llink))))
)

; -----------------------------------------------------------------------
; random-hex-string -- Generate a random string of hex
;
; Returns a random hex string of length 'str-length'.
;
(define (random-hex-string str-length)
	(define alphanumeric "abcdef0123456789")
	(define str "")
	(while (> str-length 0)
		(set! str (string-append str (string (string-ref alphanumeric (random (string-length alphanumeric))))))
		(set! str-length (- str-length 1))
	)
	str
)

; -----------------------------------------------------------------------
; random-UUID -- Generate a new UUID version 4
;
; Returns UUID version 4 (ie, mostly just random hex with some fixed values)
; XXX FIXME should be changed to just use sha-256 -- that would make it
; faster, better.
;
(define (random-UUID)
	(define part1 (random-hex-string 8))
	(define part2 (random-hex-string 4))
	(define part3 (string-append "4" (random-hex-string 3)))
	(define reserve (string (string-ref "89ab" (random 4))))
	(define part4 (string-append reserve (random-hex-string 3)))
	(define part5 (random-hex-string 12))
	(string-append part1 "-" part2 "-" part3 "-" part4 "-" part5)
)

; -----------------------------------------------------------------------
; create-unique-word-name -- Generate a new unique name for a word
;
; Given an instanced word's ConceptNode, generate a new instance,
; appending @ UUID.
;
(define (create-unique-word-name word)
	(define (create-new-name w)
		(define tail-name (random-UUID))
		(string-append (cog-name (word-inst-get-word (r2l-get-word-inst w))) "@" tail-name)
	)
	(define new-name (create-new-name word))

	(while (check-name? new-name 'ConceptNode)
		(set! new-name (create-new-name word))
	)
	new-name
)


; =======================================================================
; Post-processsing functions for markers created from pre-processing.
; =======================================================================

; -----------------------------------------------------------------------
; r2l-marker-processing -- Entry point of markers post-processing
;
; Call all markers' post-processing steps.
;
(define (r2l-marker-processing)
	(define results
		(append
			(marker-cleaner "allmarker" allmarker-helper)
			(marker-cleaner "maybemarker" maybemarker-helper)
		)
	)
	; helper function that prune away atoms that no longer exists
	; from subsequent cleaners, or atoms that are wrapped inside
	; another link
	(define (pruner x)
		(define deref-x (cog-atom (cog-handle x)))
		; if 'deref-x' is #<Invalid handle>, than both cog-node?
		; and cog-link? will return false
		(if (and (or (cog-node? deref-x) (cog-link? deref-x))
			 (nil? (cog-incoming-set x)))
			x
			#f
		)
	)
	(filter-map pruner results)
)

; -----------------------------------------------------------------------
; allmarker-helper -- Helper function of allmarker post-processing
;
; The allmarker helper function for post-processing one specific
; allmarker.
;
; Given an allmarker of the form as 'orig-link':
;
;	EvaluationLink
;		DefinedLinguisticPredicateNode "allmarker"
;		ListLink
;			ConceptNode noun_instance
;
; It creates:
;
;	ForAllLink
;		VariableNode "$X"
;		ImplicationLink
;			InheritanceLink "$X" noun_instance
;			AndLink
;				** links involving noun_instance **
;
; meaning all links involving noun_instance (except maybe other markers,
; or Inherit n n_inst) are included.  Each noun_instance will be
; replaced as (VariableNode "$X").
;
(define (allmarker-helper orig-link)
	(define listlink (car (cog-filter 'ListLink (cog-outgoing-set orig-link))))
	(define word (gar listlink))
	(define root-links (cog-get-root word))
	; get rid of links with only one instanced word
	(define clean-links (remove r2l-is-unary? root-links))
	; helper recursive function to rebuild a link, replacing 'word' node with $X
	(define (rebuild-with-x link)
		(define old-oset (cog-outgoing-set link))
		(define (rebuild-helper atom)
			(if (cog-link? atom)
				(rebuild-with-x atom)
				(if (equal? atom word)
					(VariableNode "$X")
					atom
				)
			)
		)
		(define new-oset (map rebuild-helper old-oset))

		(apply cog-new-link (append (list (cog-type link) (cog-tv link)) new-oset))
	)

	; rebuild each link, replace 'word' with (VariableNode "$X")
	(define final-links (map rebuild-with-x clean-links))
	(define results-list
		(list
			(ForAllLink
				(VariableNode "$X")
				(ImplicationLink
					(InheritanceLink
						(VariableNode "$X")
						word
					)
					; new rebuilt links
					(if (= (length final-links) 1)
						final-links
						(AndLink final-links)
					)
				)
			)
		)
	)

	; delete old rebuilt links
	(for-each extract-hypergraph clean-links)

	results-list
)

; -----------------------------------------------------------------------
; maybemarker-helper -- Helper function of maybemarker post-processing
;
; The maybemarker helper function for post-processing one specific
; maybemarker.
;
; Given an maybemarker of the form as 'orig-link':
;
;	EvaluationLink
;		DefinedLinguisticPredicateNode "maybemarker"
;		ListLink
;			ConceptNode word_instance
;
; we find all root links containing 'word_instance' and change the
; confidence to 0.5.
;
(define (maybemarker-helper orig-link)
	(define listlink (car (cog-filter 'ListLink (cog-outgoing-set orig-link))))
	(define word (gar listlink))
	(define root-links (cog-get-root word))
	; get rid of links with only one instanced word
	(define clean-links (remove r2l-is-unary? root-links))
	(define (change-tv l)
		(cog-set-tv! l (cog-new-stv 0.99 0.5))
	)

	; modify the TV
	(map change-tv clean-links)
)

; -----------------------------------------------------------------------
; marker-cleaner -- Common code for calling marker's helper function
;
; A general purpose marker function that calls a specific 'helper' function
; to clean each instance of a marker with 'name' in the atomspace, and
; delete them.
;
(define (marker-cleaner name helper)
	(define marker (cog-node 'DefinedLinguisticPredicateNode name))
	(define (call-helper)
		; get the list of all unprocessed marker of type "name"
		(define marker-list (cog-get-link 'EvaluationLink 'ListLink marker))
		; call helper function to process them
		(define results-list (append-map helper marker-list))
		; delete the markers links and the marker itself
		(for-each extract-hypergraph marker-list)
		(cog-extract! marker)
		; return the results
		results-list
	)

	(if (cog-atom? marker)
		(call-helper)
	)
)


; =======================================================================
; Functions to create partially and fully abstract version of the
; representations.
; =======================================================================

; -----------------------------------------------------------------------
(define (rebuild ilink lone-nodes non-lone-alist update)
"
  rebuild -- Main recursive function to build the new abstracted links

  A helper function for building a new version of 'ilink' with abstraction.

  ilink:
  - a link to be abstracted should it have the 'lone-nodes'

  lone-nodes:
  - contains nodes that should be abstracted, while 'non-lone-alist'
    is an a-list whose keys are nodes that should be cloned with new instance
    name, and whose data values are the new instance name.

  update:
  - #t or #f to signal the update of etv in the abstracted r2l outputs returned
"
;XXX FIXME using the hacky word-get-r2l-node, bad idea!
	; get all the nodes linked by this link
	(define old-oset (cog-outgoing-set ilink))
	(define (replace-old old-atom)
		(define a-pair (assoc old-atom non-lone-alist))
		(cond ((cog-link? old-atom)
					(rebuild old-atom lone-nodes non-lone-alist update))
		      ; if node needed to be abstracted
		      ((member? old-atom lone-nodes)
					(if (equal? 'VariableNode (cog-type old-atom))
						; XXX what would an abstracted VariableNode be like?
						old-atom
						; fail-safe for when R2L rule is incomplete and
						; never created the abstract node
						(if (nil? (word-get-r2l-node
								(word-inst-get-lemma (r2l-get-word-inst old-atom))))
							(cog-new-node
								(cog-type old-atom)
								(cog-name (word-inst-get-lemma
									(r2l-get-word-inst old-atom))))
							(word-get-r2l-node (word-inst-get-lemma
								(r2l-get-word-inst old-atom)))
						)
					)
		      )
			;  ; FIXME: Why create a node with new-instance name? Is this for
			; anaphora-resolution?
		    ;  ; If node needed to be cloned with new instance name
		    ;  (a-pair
			;(let ((abstract-node
			;	; fail-safe for when R2L rule is incomplete and never created the abstract node
			;	(if (nil? (word-get-r2l-node (word-inst-get-lemma (r2l-get-word-inst old-atom))))
			;		(cog-new-node (cog-type old-atom) (cog-name (word-inst-get-lemma (r2l-get-word-inst old-atom))))
			;		(word-get-r2l-node (word-inst-get-lemma (r2l-get-word-inst old-atom)))
			;	)
			;     ))
			;	(if (equal? 'PredicateNode (cog-type old-atom))
			;		(ImplicationLink
			;			(cog-new-node (cog-type old-atom) (cdr a-pair) (cog-tv old-atom))
			;			abstract-node
			;		)
			;		(InheritanceLink
			;			(cog-new-node (cog-type old-atom) (cdr a-pair) (cog-tv old-atom))
			;			abstract-node
			;		)
			;	)
			;	(cog-node (cog-type old-atom) (cdr a-pair))
			;)
		    ;  )
		      (else old-atom)
		)
	)
	; Create new outgoing-set
	(define new-oset (map replace-old old-oset))

	; Create a new link with the new outgoing-set
	(define new-link
		(apply cog-new-link (append (list (cog-type ilink)) new-oset)))

	(if update
		(create-or-update-etv new-link)
		new-link
	)
)

; -----------------------------------------------------------------------
; Represents the set of InterpretationNodes with their abstracted r2l-outputs
; counted.
(define abstracted-interp-node (Concept "r2l-abstracted-interp"))

; -----------------------------------------------------------------------
(define (create-abstract-version interpretation-node update)
"
  Given an 'interpretation-node', it finds out which r2l links can be
  abstracted and returns the newly abstracted atoms. For example, the
  following link which is part of the an r2l output for the sentence,
  \"Men breathe air\"

	(EvaluationLink
		(PredicateNode \"breathe@959e0d2f-dcb2-4105-b897-faefbe4d5149\")
		(ListLink
			(ConceptNode \"men@728c977c-2b4e-4b53-aeca-4a9d4d751357\")
			(ConceptNode \"air@2d2b04b1-4502-4ffa-ac36-1c83006c6f4f\")
		)
	)

	is converted to

	(EvaluationLink
		(PredicateNode \"breathe\")
		(ListLink
			(ConceptNode \"man\")
			(ConceptNode \"air\")
		)
	)

  interpretation-node:
  - An InterpretationNode

  update:
  - #t or #f to signal the update of etv in the abstracted r2l outputs returned
"
	(define r2l-set (cog-outgoing-set
		(car (cog-chase-link 'ReferenceLink 'SetLink interpretation-node))))

	; remove all unary links
	(define r2l-cleaned-set (remove r2l-is-unary? r2l-set))

	; for each link, retrieve its nodes
	(define r2l-set-nodes (append-map
		(lambda (lnk) (delete-duplicates (cog-get-all-nodes lnk)))
		r2l-cleaned-set))

	; partition the set of nodes: one set all lone nodes and the other the rest
	(receive (lone-nodes other-nodes)
		(partition
			(lambda (n)
				(and
					(is-r2l-inst? n)
					;FIXME: Why occurence of a node in more than one relation
					; matter? One reason is if the instance-node is renamed
					; then one wouldn't want to rename the same instance to
					; different nodes, but then again why create new
					; instance-nodes.
					;(= (count (lambda (x) (equal? x n)) r2l-set-nodes) 1)
					; Because a definite article may imply that the noun is,
					; known in current context, and as such might not
					; necessarily be a generializable(aka fully-abstracted)
					; concept.
					(not (definite? n))
				)
			)
			r2l-set-nodes
		)

		;(let ((non-lone-alist
		;		(filter-map
		;			(lambda (n)
		;				(if (is-r2l-inst? n)
		;					(cons n (create-unique-word-name n))
		;					#f
		;				)
		;			)
		;			(delete-duplicates other-nodes))))
		;	(append
		;		(map (lambda (lnk) (rebuild lnk lone-nodes non-lone-alist)) r2l-cleaned-set)
		;		(map (lambda (lnk) (rebuild lnk (filter is-r2l-inst? (delete-duplicates r2l-set-nodes)) '())) r2l-cleaned-set)
		;	)
		;)

		(if update (Member interpretation-node  abstracted-interp-node))

		(map (lambda (lnk) (rebuild lnk lone-nodes '() update)) r2l-cleaned-set)
	)
)



(define (has-been-counted? interp)
"
  Check if the abstracted-version of the given InterpretationNode's r2l output
  has been counted or not.
"
	(define interp-list
		(cog-chase-link 'MemberLink 'InterpretationNode abstracted-interp-node))

	(if (member? interp interp-list)
		#t
		#f
	)
)


(define-public (get-abstract-version interp)
"
  Returns a list with the abstracted-version of the r2l outputs. The returned
  atoms are counted only once.

  The abstracted r2l outputs are primarily used for reasoning.

  interp:
  - An InterpretationNode
"
	(create-abstract-version interp (not (has-been-counted? interp)))
)
