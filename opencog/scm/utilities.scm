;
; utilities.scm
;
; Miscellaneous handy utilities for working with atoms.
; The most useful utilities in here are probably 'cog-chase-link'
; for finding atoms connected by a given link type, and 'cog-get-pred'
; which is useful for working with EvaluationLink's.
;
; Utilities include:
; -- abbreviations for working with truth values (stv, ctv, etc.)
; -- simple traversal of outgoing set (gar, gdr, etc.)
; -- for-each-except loop.
; -- cog-atom-incr --  Increment count truth value on "atom" by "cnt".
; -- delete-hypergraph -- delete a hypergraph and everything "under" it.
; -- delete-type -- delete all atoms of type 'atom-type'.
; -- clear -- delete all atoms in the atomspace.
; -- cog-get-atoms -- Return a list of all atoms of type 'atom-type'
; -- cog-count-atoms -- Count of the number of atoms of given type.
; -- cog-report-counts -- Return an association list of counts.
; -- cog-get-partner -- Return other atom of a link conecting two atoms.
; -- cog-pred-get-partner -- Get the partner in an EvaluationLink.
; -- cog-filter-map -- call proceedure on list of atoms.
; -- cog-filter -- return a list of atoms of given type.
; -- cog-filter-incoming -- filter atoms of given type from incoming set.
; -- cog-filter-outgoing -- filter atoms of given type from outgoing set.
; -- cog-chase-link -- Return other atom of a link conecting two atoms.
; -- cog-map-chase-link -- Invoke proc on atoms connected through type.
; -- cog-map-chase-links -- Invoke proc on atoms connected through type.
; -- cog-map-chase-link-dbg -- Debugging version of above.
; -- cog-map-apply-link -- call proc on link between atom and atom type.
; -- cog-get-link -- Get list of links connecting atom to atom type.
; -- cog-get-pred -- Find all EvaluationLinks of given form.
; -- filter-hypergraph -- recursively traverse outgoing links of graph.
; -- cartesian-prod -- create Cartesian product from tuple of sets.
;
;
; Copyright (c) 2008, 2013 Linas Vepstas <linasvepstas@gmail.com>
;

(use-modules (srfi srfi-1))

(define (av sti lti vlti) (cog-new-av sti lti vlti))    

(define (stv mean conf) (cog-new-stv mean conf))
(define (itv lower upper conf) (cog-new-itv lower upper conf))
(define (ctv mean conf count) (cog-new-ctv mean conf count))
; Helper to create CompositeTV
; one can use it 2 ways
; 1) (mtv vh tv) where vh is a versionHandle and tv is TruthValue
; 2) (mtv (cons vh1 tv1) ... (cons vhn tvn))
(define (mtv . x)
  (let ((head (car x)) (tail (cdr x)))
    (if (pair? head) (cog-set-vtv! (mtv tail) (car head) (cdr head))
        (cog-new-mtv head (cadr x)))))

; -----------------------------------------------------------------------
; analogs of car, cdr, etc. but for atoms.
; (define (gar x) (if (cog-atom? x) (car (cog-outgoing-set x)) (car x)))
; (define (gdr x) (if (cog-atom? x) (cadr (cog-outgoing-set x)) (cdr x)))

(define (gar x) (car (cog-outgoing-set x)) )
(define (gdr x) (cadr (cog-outgoing-set x)) )
(define (gadr x) (gar (gdr x)) ) 
(define (gddr x) (gdr (gdr x)) )
(define (gaddr x) (gar (gddr x)) )
(define (gdddr x) (gdr (gddr x)) )

; A more agressive way of doing the above:
; (define car (let ((oldcar car)) (lambda (x) (if (cog-atom? x) (oldcar (cog-outgoing-set x)) (oldcar x)))))
; But this would probably lead to various painful debugging situations.

; -----------------------------------------------------------------------
; for-each-except 
; standard for-each loop, except that anything matching "except" is skipped
(define (for-each-except exclude proc lst)
	(define (loop items)
		(cond
			((null? items) #f)
			((eq? exclude (car items))
				(loop (cdr items))
			)
			(else
				(proc (car items))
				(loop (cdr items))
			)
		)
	)
	(loop lst)
)

; --------------------------------------------------------------
;
; cog-atom-incr --  Increment count truth value on "atom" by "cnt"
;
; If the current truth value on the atom is not a CountTruthValue,
; then the truth value is replaced by a CountTruthValue, with the 
; count set to "cnt".
;
; XXX this implementation is slow/wasteful, a native C++ would
; be considerably faster.
;
; Example usage:
; (cog-atom-incr (ConceptNode "Answer") 42)
;
(define (cog-atom-incr atom cnt) 
	(let* (
			(tv (cog-tv atom))
			(atv (cog-tv->alist tv))
			(mean (assoc-ref atv 'mean))
			(conf (assoc-ref atv 'confidence))
			(count (assoc-ref atv 'count))

			; non-CountTV's will not have a 'count in the a-list
			; so its enough to test for that.
			(ntv (if count
					(cog-new-ctv mean conf (+ count cnt))
					(cog-new-ctv mean conf cnt))
			)
		)
		(cog-set-tv! atom ntv)
	)
)

; --------------------------------------------------------------------
; delete-hypergraph -- delete a hypergraph and everything "under" it
;
; If the indicated atom has no incoming links, then delete it. Repeat
; recursively downwards, following the *outgoing* set of any links 
; encountered.

(define (delete-hypergraph atom)
	(if (cog-node? atom) 
		(cog-delete atom)
		(let* ((oset (cog-outgoing-set atom))
				(flg (cog-delete atom))
			)
			(if flg ;; halt recursion if link was not delete-able
				(for-each delete-hypergraph oset)
			)
		)
	)
)

; --------------------------------------------------------------------
; delete-type -- delete all atoms of type 'atom-type'
;
; If any atoms of that type have incoming links, those links will be
; deleted, and so on recursively.
(define (delete-type atom-type)
    (cog-map-type
        (lambda (x) (cog-delete-recursive x) #f)
        atom-type
    )
)

; --------------------------------------------------------------------
; clear -- delete all atoms in the atomspace
(define (clear)
    (for-each 
		delete-type
		(cog-get-types)
	)
)

(define (count-all)
 (define cnt 0)
 (define (ink a) (set! cnt (+ cnt 1)) #f)
 (define (cnt-type x) (cog-map-type ink x)) 
 (map cnt-type (cog-get-types))
 cnt
)

; -----------------------------------------------------------------------
; cog-get-atoms -- Return a list of all atoms of type 'atom-type'
;
; cog-get-atoms atom-type
; Return a list of all atoms in the atomspace that are of type 'atom-type'
;
; Example usage:
; (display (cog-get-atoms 'ConceptNode))
; will return and display all atoms of type 'ConceptNode
;
(define (cog-get-atoms atom-type)
	(let ((lst '()))
		(define (mklist atom)
			(set! lst (cons atom lst))
			#f
		)
		(cog-map-type mklist atom-type)
		lst
	)
)

; -----------------------------------------------------------------------
; cog-count-atoms -- Count of the number of atoms of given type
;
; cog-count-atoms atom-type
; Return a count of the number of atoms of the given type 'atom-type'
;
; Example usage:
; (display (cog-count-atoms 'ConceptNode))
; will display a count of all atoms of type 'ConceptNode
;
(define (cog-count-atoms atom-type)
	(let ((cnt 0))
		(define (inc atom)
			(set! cnt (+ cnt 1))
			#f
		)
		(cog-map-type inc atom-type)
		cnt
	)
)

; -----------------------------------------------------------------------
; cog-report-counts -- Return an association list of counts
;
; Return an association list holding a report of the number of atoms
; of each type currently in the atomspace. Counts are included only
; for types with non-zero atom counts.
;
(define (cog-report-counts)
	(let ((tlist (cog-get-types)))
		(define (rpt type)
			(let ((cnt (cog-count-atoms type)))
				(if (not (= 0 cnt))
					(cons type cnt)
					#f
				)
			)
		)
		(filter-map rpt tlist)
	)
)

; -----------------------------------------------------------------------
; cog-get-partner -- Return other atom of a link conecting two atoms
; cog-get-partner pair atom
;
; If 'pare' is a link containing two atoms, and 'wrd' is one of the
; two atoms, then this returns the other atom in the link.
;
; See also cog-chase-link which does not require the link to be
; explicitly specified; instead, only the pare type is needed.
;
(define (cog-get-partner pare atom)
	(let ((plist (cog-outgoing-set pare)))
		(if (equal? atom (car plist))
			(cadr plist)
			(car plist)
		)
	)
)

; -----------------------------------------------------------------------
; cog-pred-get-partner -- Get the partner in an EvaluationLink
;
; cog-pred-get-partner pred atom
;
; Get the partner to the atom 'atom' in the opencog predicate 'pred'.
; An opencog predicate is assumed to be structured as follows:
;
;    EvaluationLink
;        SomeAtom  "relation-name"
;        ListLink
;            AnotherAtom  "some atom"
;            AnotherAtom  "some other atom"
;
; Assuming this structure, then, given the top-level link, and one
; of the two atoms in the ListLink, this routine returns the other
; atom in the listLink.
;
(define (cog-pred-get-partner rel atom)
	; The 'car' appears here because 'cog-filter-outgoing' is returning
	; a list, and we want just one atom (the only one in the list)
	(cog-get-partner (car (cog-filter-outgoing 'ListLink rel)) atom)
)

; -----------------------------------------------------------------------
; cog-filter-map -- call proceedure on list of atoms
;
; cog-filter-map atom-type proc atom-list
;
; Apply the proceedure 'proc' to every atom of 'atom-list' that is
; of type 'atom-type'. Application halts if proc returns any value 
; other than #f. Return the last value returned by proc; that is,
; return #f if proc always returned #f, otherwise return the value
; that halted the application.
;
; Exmaple usage:
; (cog-filter-map 'ConceptNode display (list (cog-new-node 'ConceptNode "hello")))
; 
; See also: cgw-filter-atom-type, which does the same thing, but for wires.
;
; XXX TODO This is not really a map, because of the #f behaviour. This
; should be fixed someday ...
;
(define (cog-filter-map atom-type proc atom-list) 
	(define rc #f)
	(cond 
		((null? atom-list) #f)
		((eq? (cog-type (car atom-list)) atom-type) 
			(set! rc (proc (car atom-list))) 
			(if (eq? #f rc) 
				(cog-filter-map atom-type proc (cdr atom-list))
				rc
			)
		) 
		(else (cog-filter-map atom-type proc (cdr atom-list))
		)
	)
)

; cog-filter -- return a list of atoms of given type.
;
; Given a list of atoms, return a list of atoms that are of 'atom-type'
(define (cog-filter atom-type atom-list) 
	(define (is-type? atom) (eq? atom-type (cog-type atom)))
	(filter is-type? atom-list)
)

; cog-filter-incoming -- filter atoms of given type from incoming set.
;
; Given an atom, return a list of atoms from its incoming set that 
; are of type 'atom-type'
(define (cog-filter-incoming atom-type atom)
	(cog-filter atom-type (cog-incoming-set atom))
)

; cog-filter-outgoing -- filter atoms of given type from outgoing set.
;
; Given an atom, return a list of atoms from its outgoing set that 
; are of type 'atom-type'
(define (cog-filter-outgoing atom-type atom)
	(cog-filter atom-type (cog-outgoing-set atom))
)

; -----------------------------------------------------------------------
; cog-chase-link -- Return other atom of a link conecting two atoms.
;
; cog-chase-link link-type endpoint-type anchor
;
; Starting at the atom 'anchor', chase its incoming links of
; 'link-type', and return a list of all of the atoms of type 
; 'node-type' in those links. For example, given:
;
;    SomeLink
;        GivenNode "a"
;        WantedNode  "p"
;
;    SomeLink
;        GivenNode "a"
;        WantedNode  "q"
;
; then this method will return the two WantedNodes's given the
; GivenNode, and the link-type 'SomeLink.
;
; It is presumed that 'anchor' points to some atom (typically a node),
; and that it has many links in its incoming set. So, loop over all of
; the links of 'link-type' in this set. They presumably link to all 
; sorts of things. Find all of the things that are of 'endpoint-type'.
; Return a list of all of these.
;
; See also: cgw-follow-link, which does the same thing, but for wires.
;
(define (cog-chase-link link-type endpoint-type anchor)
	(let ((lst '()))
		(define (mklist inst)
			(set! lst (cons inst lst))
			#f
		)
		(cog-map-chase-link link-type endpoint-type mklist anchor)
		lst
	)
)

; -----------------------------------------------------------------------
; cog-map-chase-link -- Invoke proc on atom connected through type.
;
; Similar to cog-chase-link, but invokes 'proc' on the wanted atom.
; Starting at the atom 'anchor', chase its incoming links of
; 'link-type', and call proceedure 'proc' on all of the atoms of
; type 'node-type' in those links. For example, given:
;
;    SomeLink
;        GivenNode "a"
;        WantedNode  "p"
;
;    SomeLink
;        GivenNode "a"
;        WantedNode  "q"
;
; then 'proc' will be called twice, with each of the WantedNodes's
; as the argument. These wanted nodes were found by following the
; link type 'SomeLink, starting at GivenNode.
;
; It is presumed that 'anchor' points to some atom (typically a node),
; and that it has many links in its incoming set. So, loop over all of
; the links of 'link-type' in this set. They presumably link to all 
; sorts of things. Find all of the things that are of 'endpoint-type'.
; Return a list of all of these.
;
(define (cog-map-chase-link link-type endpoint-type proc anchor)
	(define (get-endpoint w)
		; cog-filter-map returns the return value from proc, we pass it on
		; in turn, so make sure this is last statement
		(cog-filter-map endpoint-type proc (cog-outgoing-set w))
	)
	; cog-filter-map returns the return value from proc, we pass it on
	; in turn, so make sure this is last statement
	(if (null? anchor)
		'()
		(cog-filter-map link-type get-endpoint (cog-incoming-set anchor))
	)
)

; -----------------------------------------------------------------------
; cog-map-chase-links -- Invoke proc on atom connected through type.
;
; Same as cog-chase-link, except that the anchor may be a single atom,
; or it may be a list.
(define (cog-map-chase-links link-type endpoint-type proc anchor)
	(if (list? anchor)
		(map 
			(lambda (one-of)
				(cog-map-chase-links link-type endpoint-type proc one-of)
			)
		anchor)
		(cog-map-chase-link link-type endpoint-type proc anchor)
	)
)

; -----------------------------------------------------------------------
; cog-map-chase-links-chk -- Invoke proc on atom connected through type.
;
; Same as cog-chase-links, except that the type of the anchor is
; checked, and an exception thrown if its the wrong type.
(define (cog-map-chase-links-chk link-type endpoint-type proc anchor anchor-type)
	(if (list? anchor)
		; If we are here, then anchor is a list. Recurse.
		(map 
			(lambda (one-of)
				(cog-map-chase-links link-type endpoint-type proc one-of)
			)
		anchor)
		; If we are here, then its a singleton. Verify type.
		(if (eq? (cog-type anchor) anchor-type)
			(cog-map-chase-link link-type endpoint-type proc anchor)
			(throw 'wrong-atom-type 'cog-map-chase-links
				"Error: expecting atom:" anchor)
		)
	)
)

; -----------------------------------------------------------------------
; cog-map-chase-link-dbg -- debugging version of cog-map-chase-link
;
; cog-map-chase-link-dbg link-type endpoint-type dbg-lmsg dbg-emsg proc anchor
;
; Chase 'link-type' to 'endpoint-type' and apply proc to what is found there.
;
; It is presumed that 'anchor' points to some atom (typically a node),
; and that it has many links in its incoming set. So, loop over all of
; the links of 'link-type' in this set. They presumably link to all 
; sorts of things. Find all of the things that are of 'endpoint-type'
; and then call 'proc' on each of these endpoints. Optionally, print
; some debugging msgs.
;
; The link-chasing halts if proc returns any value other than #f.
; Returns the last value returned by proc, i.e. #f, or the value that
; halted the iteration.
;
; Example usage:
; (cog-map-chase-link-dbg 'ReferenceLink 'WordNode '() '() proc word-inst)
; Given a 'word-inst', this will chase all ReferenceLink's to all 
; WordNode's, and then will call 'proc' on these WordNodes.
;
(define (cog-map-chase-link-dbg link-type endpoint-type dbg-lmsg dbg-emsg proc anchor)
	(define (get-endpoint w)
		(if (not (eq? '() dbg-emsg)) (display dbg-emsg))
		; cog-filter-map returns the return value from proc, we pass it on
		; in turn, so make sure this is last statement
		(cog-filter-map endpoint-type proc (cog-outgoing-set w))
	)
	(if (not (eq? '() dbg-lmsg)) (display dbg-lmsg))
	; cog-filter-map returns the return value from proc, we pass it on
	; in turn, so make sure this is last statement
	(if (null? anchor)
		'()
		(cog-filter-map link-type get-endpoint (cog-incoming-set anchor))
	)
)

; -----------------------------------------------------------------------
;
; cog-map-apply-link link-type endpoint-type proc anchor
;
; Similar to cog-map-chase-link, except that the proc is not called
; on the endpoint, but rather on the link leading to the endpoint.
;
(define (cog-map-apply-link link-type endpoint-type proc anchor)
	(define (get-link l)
		(define (apply-link e)
			(proc l)
		)
		(cog-filter-map endpoint-type apply-link (cog-outgoing-set l))
	)
	(cog-filter-map link-type get-link (cog-incoming-set anchor))
)

;
; cog-get-link link-type endpoint-type anchor
;
; Return a list of links, of type 'link-type', which contain some
; atom of type 'endpoint-type', and also specifically contain the 
; atom 'anchor'.
;
; Thus, for example, suppose the atom-space contains a link of the
; form (ReferenceLink (ConcpetNode "asdf") (WordNode "pqrs"))
; Then, the call 
;    (cog-get-link 'ReferenceLink 'ConcpetNode (WordNode "pqrs"))
; will return that link. Note that "endpoint-type" need not occur
; in the first position in the link; it can appear anywhere.
;
(define (cog-get-link link-type endpoint-type anchor)
	(let ((lst '()))
		(define (mklist inst)
			(set! lst (cons inst lst))
			#f
		)
		(cog-map-apply-link link-type endpoint-type mklist anchor)
		lst
	)
)

; ---------------------------------------------------------------------
; cog-get-pred -- Find all EvaluationLinks of given form.
;
; Return a list of predicates, of the given type, that an instance 
; participates in.  That is, given a "predicate" of the form:
;
;    EvaluationLink
;       SomeAtom
;       ListLink
;           AnotherAtom "abc"
;           GivenAtom "def"
;
; then, given the instance 'inst' (in this example, GivenAtom "def")
; and predicate type 'pred-type' 'SomeAtom, then this routine returns
; a list of all of the EvalutaionLink's in which 'inst' appears.
;
(define (cog-get-pred inst pred-type)
	(concatenate!
		(append!
			(map
				(lambda (lnk) (cog-get-link 'EvaluationLink pred-type lnk))
				(append! (cog-filter-incoming 'ListLink inst)) ;; append removes null's
			)
		)
	)
)

; -----------------------------------------------------------------------
; Given a reference structure, return the referenced list entries.
; That is, given a structure of the form
;
;    ReferenceLink
;        SomeAtom
;        ListLink
;           AnotherAtom
;           AnotherAtom
;           ...
;
; Then, given, as input, "SomeAtom", this returns a list of the "OtherAtom"
;
; XXX! Caution/error! This implictly assumes that there is only one 
; such ReferenceLink in the system, total. This is wrong !!!
;
(define (cog-get-reference refptr)
   (let ((lst (cog-chase-link 'ReferenceLink 'ListLink refptr)))
		(if (null? lst)
			'()
   		(cog-outgoing-set (car lst))
		)
	)
)

; ---------------------------------------------------------------------
; filter-hypergraph -- recursively traverse outgoing links of graph.
;
; filter-hypergraph pred? atom-list
;
; Given a list of atoms, and a scheme-predicate pred?, return a
; list of atoms that satisfy the scheme-predicate.  This is not
; a simple srfi-1 'filter', rather, it traverses the hypergraph,
; applying the predicate to the subgraphs.
;
; In the current implementation, the scheme-predicate is assumed to 
; select only for Nodes.  
;
(define (filter-hypergraph pred? atom-list)
	(define (fv atoms lst)
		(cond
			; If its a query word, append it to the list
			((cog-node? atoms)
				(if (pred? atoms)
					(cons atoms lst)
					lst
				)
			)

			; If its a link, scan its outgoing set
			((cog-link? atoms)
				(fv (cog-outgoing-set atoms) lst)
			)

			; If its a list then scan the list
			((pair? atoms)
				(append! 
					(append-map! 
						(lambda (x) (filter-hypergraph pred? x)) atoms) 
					lst
				)
			)
		)
	)
	(fv atom-list '())
)

; --------------------------------------------------------------------
; cartesian-prod -- create Cartesian product from tuple of sets.
;
; This returns the Cartestion product of a tuple of sets by distributing
; the product across the set elements. Returned is a list of tuples, where
; the elements of the tuple are elements from the corresponding sets.
;
; Let the input tuple be [s_1, s_2, ..., s_m] where each s_k is a set of
; one or more elements.  Let the cardinality of s_k be n_k.
; Then this returns a list of n_1 * n_2 *... * n_m tuples.
; where the projection of the coordinate k is an element of s_k.
; That is, let p_k be the k'th cordinate projection, so that 
; p_k [a_1, a_2, ... , a_m] = a_k
;
; Then, if t is a tuple returned by this function, one has that
; p_k(t) is element of s_k for all t and all 1<= k <= m. Every possible
; combination of set elements is returned.
;
; Example usage and output:
;
; guile> (cartesian-prod (list 'p 'q))
; (p q)
; guile> (cartesian-prod (list 'p (list 'x 'y)))
; ((p x) (p y))
; guile> (cartesian-prod (list (list 'p 'q) 'x))
; ((p x) (q x))
; guile> (cartesian-prod (list (list 'p 'q) (list 'x 'y)))
; ((p x) (q x) (p y) (q y))
; guile> (cartesian-prod (list (list 'p 'q) 'a (list 'x 'y)))
; ((p a x) (q a x) (p a y) (q a y))
; guile> (cartesian-prod (list 'a (list 'p 'q) (list 'x 'y)))
; ((a p x) (a q x) (a p y) (a q y))
;
; XXX TODO -- this would almost surely be simpler to implement using
; srfi-1 fold or srfi-1 map, which is really all that it is ... 
;
(define (cartesian-prod tuple-of-lists)

	; distribute example usage:
	; guile> (distribute (list 'p 'q)  (list '1 '2 '3))
	; ((p 1 2 3) (q 1 2 3))
	; guile> (distribute 'p (list '1 '2 '3))
	; (p 1 2 3)
	; guile> (distribute 'p '1)
	; (p 1)
	; guile> (distribute (list 'p 'q) '1)
	; ((p 1) (q 1))
	;
	(define (distribute heads a-tail)
		(define (match-up a-head tale)
			(if (list? tale)
				(cons a-head tale)
				(list a-head tale)
			)
		)
		(if (list? heads)
			(map (lambda (hed) (match-up hed a-tail)) heads)
			(match-up heads a-tail)
		)
	)

	; guile> (all-combos 'p (list 'x 'y))
	; ((p x) (p y))
	; guile> (all-combos 'p (list (list 'a 'b) (list 'y 'z)))
	; ((p a b) (p y z))
	; guile> (all-combos (list 'p 'q) (list 'x 'y))
	; ((p x) (q x) (p y) (q y))
	; guile> (all-combos (list 'p 'q) 'x)
	; ((p x) (q x))
	; guile> (all-combos 'p  'x)
	; (p x)
	;
	(define (all-combos heads tails)
		(if (list? tails)
			(let ((ll (map (lambda (tale) (distribute heads tale)) tails)))
				(if (list? heads)
					(concatenate! ll)
					ll
				)
			)

			; tails is not a list
			(distribute heads tails)
		)
	)

	(cond 
		((null? tuple-of-lists) '())
		((not (list? tuple-of-lists)) tuple-of-lists)
		(else 
			(let ((lc (car tuple-of-lists))
					(rc (cdr tuple-of-lists))
				)
				(if (null? rc)
					lc
					; cartesian-prod always returns tails, so we
					; just need to take all combinations of the
					; head against the returned tails.
					; (This is not a tail-recursive call but I don't care)
					(all-combos lc (cartesian-prod rc))
				)
			)
		)
	)
)

; ---------------------------------------------------------------------
