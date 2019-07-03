;
; OpenCog AttentionBank module
; Copyright (C) 2018 Linas Vepstas <linasvepstas@gmail.com>
;

(define-module (opencog attention-bank))

(use-modules (opencog) (opencog oc-config))

(load-extension (string-append opencog-ext-path-attentionbonk "libattentionbonk") "opencog_attention_init")
; Load the C library that calls the nameserver to load the types.
(load-extension (string-append opencog-ext-path-attention-types "libattention-types") "attention_types_init")

(load "attentionbank/types/attention_types.scm")

; This avoids complaints, when the docs are set, below.
(export
	cog-av cog-set-av! cog-inc-vlti! cog-dec-vlti!
	cog-update-af cog-af-size cog-set-af-size! cog-stimulate
	cog-bind-af
)

;; -----------------------------------------------------
;
(define-public (cog-new-av STI LTI VLTI)
"
 cog-new-av STI LTI VLTI
    Create an AttentionValue with the given STI, LTI and VLTI.
    Unlike atoms, attention values are ephemeral: they are
    automatically garbage-collected when no longer needed.

    Example:
        ; Create a new attention value:
        guile> (cog-new-av 10 20 0)
"
	(cog-new-value 'AttentionValue STI LTI VLTI)
)

(define-public (cog-av? EXP)
"
 cog-av? EXP
    Return #t if EXP is an attention value, else return #f

    Example:
       ; Define a simple attention value
       guile> (define x (cog-new-av 15 25 0))
       guile> (define y (+ 2 2))
       guile> (cog-av? x)
       #t
       guile> (cog-av? y)
       #f
"
	(if (cog-value? EXP) (eq? (cog-type EXP) 'AttentionValue) #f)
)

(define-public (cog-av->alist AV)
"
 cog-av->alist AV
    Convert an attention value AV to an association list (alist).

    Example:
       guile> (define x (cog-new-av 99 88 0))
       guile> (cog-av->alist x)
       ((sti . 99) (lti . 88) (vlti . 0))
"
	(define avl (cog-value->list AV))
	(acons 'sti (car avl)
	(acons 'lti (cadr avl)
	(acons 'vlti (caddr avl) '())))
)

;; -----------------------------------------------------

(set-procedure-property! cog-av 'documentation
"
 cog-av ATOM
    Return the attention value of ATOM.

    Example:
       ; Define a node
       guile> (define x
                 (cog-new-node 'ConceptNode \"abc\"
                    (cog-new-av 11 21 0)))
       guile> (cog-av x)
       (av 11 21 0)
       guile> (cog-av? (cog-av x))
       #t
")

(set-procedure-property! cog-set-av! 'documentation
"
 cog-set-av! ATOM AV
    Set the attention value of ATOM to AV.

    Example:
       ; Define a node
       guile> (define x (cog-new-node 'ConceptNode \"def\"))
       guile> (cog-av x)
       (av 0 0 0)
       guile> (cog-set-av! x (cog-new-av 44 55 1))
       (ConceptNode \"def\" (av 44 55 1))
       guile> (cog-av x)
       (av 44 55 1)
")

(set-procedure-property! cog-inc-vlti! 'documentation
"
 cog-inc-vlti! ATOM
    Increase the vlti of ATOM by 1.

    Example:
       ; Define a node
       guile> (define x
                 (cog-new-node 'ConceptNode \"abc\"
                    (cog-new-av 11 21 0)))
       guile> (cog-inc-vlti! x)
       (ConceptNode \"abc\" (av 11 21 1))
       guile> (cog-av x)
       (av 11 21 1)
       guile> (cog-inc-vlti! x)
       (ConceptNode \"abc\" (av 11 21 2))
       guile> (cog-av x)
       (av 11 21 2)
")

(set-procedure-property! cog-dec-vlti! 'documentation
"
 cog-dec-vlti! ATOM
    Decrease the vlti of ATOM by 1.

    Example:
       ; Define a node
       guile> (define x
                 (cog-new-node 'ConceptNode \"abc\"
                    (cog-new-av 11 21 1)))
       guile> (cog-dec-vlti! x)
       (ConceptNode \"abc\" (av 11 21 0))
       guile> (cog-av x)
       (av 11 21 0)
")

; -----------------------------------------------------------------------
(define-public (cog-set-sti! atom sti)
"
  Returns the atom after setting its sti to the given value.
"
    (let ((av-alist (cog-av->alist (cog-av atom))))
        (cog-set-av! atom
            (av sti (assoc-ref av-alist 'lti) (assoc-ref av-alist 'vlti)))
    )
)

(define-public (cog-set-lti! atom lti)
"
  Returns the atom after setting its lti to the given value.
"
    (let ((av-alist (cog-av->alist (cog-av atom))))
        (cog-set-av! atom
            (av (assoc-ref av-alist 'sti) lti (assoc-ref av-alist 'vlti)))
    )
)

(define-public (cog-set-vlti! atom vlti)
"
  Returns the atom after setting its vlti to the given value.
"
    (let ((av-alist (cog-av->alist (cog-av atom))))
        (cog-set-av! atom
            (av (assoc-ref av-alist 'sti) (assoc-ref av-alist 'lti) vlti))
    )
)

; -----------------------------------------------------------------------
(define-public (cog-av-sti x)
" cog-av-sti -- Return the STI of an atom."
	(cdr (assoc 'sti (cog-av->alist (cog-av x)))))

; -----------------------------------------------------------------------
(define-public (cog-sti-above y z)
"
  cog-sti-above
  Given a threshold 'y' and a list of atoms 'z', returns a list of atoms
  with STI above the threshold
"
	(filter (lambda (x) (> (cog-av-sti x) y)) z))

; -----------------------------------------------------------------------
(define-public (cog-sti-below y z)
"
  cog-sti-below
  Given a threshold 'y' and a list of atoms 'z', returns a list of atoms
  with STI below the threshold
"
	(filter (lambda (x) (< (cog-av-sti x) y)) z))

;; -----------------------------------------------------
;;

(define* (cog-af #:optional (n -1))
"
 cog-af n
    Return the top n atoms in the AttentionalFocus or
    return all atoms in the AF if n is unspecified or is larger
    than the AF size.

    Example:
    guile> (cog-af)
    (ConceptNode \"ArtificialIntelligence\" (av 15752 0 0))
    (ConceptNode \"Databases\" (av 15752 0 0))

    guile> (cog-af 1)
    (ConceptNode \"ArtificialIntelligence\" (av 15752 0 0))
"
	(cog-value->list
		(cog-value (cog-update-af n) (Predicate "AttentionalFocus")))
)

(export cog-af)

; -----------------------------------------------------------------------

(define-public (cog-af-length)
" cog-af-length -- Length of the list of atoms in the attentional focus."
	(length (cog-af))
)

;; -----------------------------------------------------
;;

(set-procedure-property! cog-af-size 'documentation
"
 cog-af-size
    Return the AttentionalFocus size of the AtomSpace (which is
    an integer value).

    Example:

    guile> (cog-af-size)
    100
")

(set-procedure-property! cog-set-af-size! 'documentation
"
 cog-set-af-size! AF Size
    Set the AttentionalFocus Size of the AtomSpace (which is an
    integer value). Returns the new AttentionalFocus size
    (which is an integer value).

    Example:
    guile> (cog-set-af-size! 200)
    200
")

; --------------------------------------------------------------------

(set-procedure-property! cog-bind-af 'documentation
"
 cog-bind-af HANDLE
    Run pattern matcher on HANDLE.  HANDLE must be a BindLink.
    A special-purpose pattern matcher used by the URE.
")

; --------------------------------------------------------------------
