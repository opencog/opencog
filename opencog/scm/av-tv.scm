;
; av-tv.scm
;
; Useful utilities for working with attention values, truth values
; and the attentional allocation system
;
; Utilities provided:
; -- cog-af-length -- Length of list of atoms in the attentional focus
; -- cog-av-sti -- Return the STI of an atom
; -- cog-sti-above -- Filter atoms with STI above a threshold
; -- cog-sti-below -- Filter atoms with STI below a threshold
; -- cog-stv-strength -- SimpleTruthValue strength of an atom
; -- cog-stv-strength-above -- Filter atoms with TV strength above a threshold
; -- cog-stv-strength-below -- Filter atoms with TV strength below a threshold
; -- cog-stv-confidence -- TruthValue confidence of an atom
; -- cog-stv-confidence-above -- Filter atoms with TV confidence above a threshold
; -- cog-stv-confidence-below -- Filter atoms with TV confidence below a threshold
; -- cog-stv-count -- TruthValue count of an atom
; -- cog-stv-count-above -- Filter atoms with TV count above a threshold
; -- cog-stv-count-below -- Filter atoms with TV count below a threshold
; -- cog-stv-positive-filter -- Filter atoms with positive TV strength and count
;
;
; Copyright (c) 2014 Cosmo Harrigan
;

; -----------------------------------------------------------------------
; cog-af-length
; Length of the list of atoms in the attentional focus
(define (cog-af-length) (length (cog-af)))

; -----------------------------------------------------------------------
; cog-av-sti
; Return the STI of an atom
(define (cog-av-sti x) (cdr (assoc 'sti (cog-av->alist (cog-av x)))))

; -----------------------------------------------------------------------
; cog-sti-above
; Given a threshold 'y' and a list of atoms 'z', returns a list of atoms 
; with STI above the threshold
(define (cog-sti-above y z) (filter (lambda (x) (> (cog-av-sti x) y)) z))

; -----------------------------------------------------------------------
; cog-sti-below
; Given a threshold 'y' and a list of atoms 'z', returns a list of atoms
; with STI below the threshold
(define (cog-sti-below y z) (filter (lambda (x) (< (cog-av-sti x) y)) z))

; -----------------------------------------------------------------------
; cog-stv-strength
; Return the truth value strength of an atom
; (Compatible with atoms that have a SimpleTruthValue)
(define (cog-stv-strength x) (cdr (assoc 'mean (cog-tv->alist (cog-tv x)))))

; -----------------------------------------------------------------------
; cog-stv-strength-above
; Given a threshold 'y' and a list of atoms 'z', returns a list of atoms with
; truth value strength above the threshold
; (Compatible with atoms that have a SimpleTruthValue)
(define (cog-stv-strength-above y z) (filter (lambda (x) (> (cog-stv-strength x) y)) z))

; -----------------------------------------------------------------------
; cog-stv-strength-below
; Given a threshold 'y' and a list of atoms 'z', returns a list of atoms with
; truth value strength above the threshold
; (Compatible with atoms that have a SimpleTruthValue)
(define (cog-stv-strength-below y z) (filter (lambda (x) (< (cog-stv-strength x) y)) z))

; -----------------------------------------------------------------------
; cog-stv-confidence
; Return the truth value confidence of an atom
; (Compatible with atoms that have a SimpleTruthValue)
(define (cog-stv-confidence x) (cdr (assoc 'confidence (cog-tv->alist (cog-tv x)))))

; -----------------------------------------------------------------------
; cog-stv-confidence-above
; Given a threshold 'y' and a list of atoms 'z', returns a list of atoms with
; truth value confidence above the threshold
; (Compatible with atoms that have a SimpleTruthValue)
(define (cog-stv-confidence-above y z) (filter (lambda (x) (> (cog-stv-confidence x) y)) z))

; -----------------------------------------------------------------------
; cog-stv-confidence-below
; Given a threshold 'y' and a list of atoms 'z', returns a list of atoms with
; truth value confidence above the threshold
; (Compatible with atoms that have a SimpleTruthValue)
(define (cog-stv-confidence-below y z) (filter (lambda (x) (< (cog-stv-confidence x) y)) z))

; -----------------------------------------------------------------------
; cog-stv-count
; Return the truth value count of an atom
; (Compatible with atoms that have a SimpleTruthValue)
(define (cog-stv-count x) (cdr (assoc 'count (cog-tv->alist (cog-tv x)))))

; -----------------------------------------------------------------------
; cog-stv-count-above
; Given a threshold 'y' and a list of atoms 'z', returns a list of atoms with
; truth value count above the threshold
; (Compatible with atoms that have a SimpleTruthValue)
(define (cog-stv-count-above y z) (filter (lambda (x) (> (cog-stv-count x) y)) z))

; -----------------------------------------------------------------------
; cog-stv-count-below
; Given a threshold 'y' and a list of atoms 'z', returns a list of atoms with
; truth value count above the threshold
; (Compatible with atoms that have a SimpleTruthValue)
(define (cog-stv-count-below y z) (filter (lambda (x) (< (cog-stv-count x) y)) z))

; -----------------------------------------------------------------------
; cog-stv-positive-filter
; Given a list of atoms, returns a list containing the subset that has
; truth value count > 0 and truth value strength > 0
; (Compatible with atoms that have a SimpleTruthValue)
(define (cog-stv-positive-filter x) (cog-stv-strength-above 0 (cog-stv-count-above 0 x)))
