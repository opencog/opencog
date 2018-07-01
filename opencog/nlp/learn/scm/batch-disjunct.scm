;
; batch-disjunct.scm
;
; Batch-compute marginals needed for the symmetric word-MI.
;
; Copyright (c) 2018 Linas Vepstas
;
; ---------------------------------------------------------------------
; OVERVIEW
; --------
; Given a word, and a vector attached to that word, one can compute a
; symmetric mutual-information (MI) value betweem two words. In the
; typical example, the vector consists of counts observed on disjuncts
; associated to that word (although it could be any vector).
;
; This file precomputes the various partial sums (marginals) that occur
; in the definition of the symmetric MI.  It assumes that the vectors
; (i.e. observation counts) are in an SQL database; it fetches those,
; computes the assorted marginals, and writes those marginals back to
; disk. Specifically, the marginals are those needed for the
; `transpose-api`, which is used to compute the symmetric MI.
;
; REVIEW
; ------
; The symmetric-MI is described in the diary. What follows is a quick
; review that motivates the stuff in this file.
;
; One has numbers (observation counts) `N(w,d)` which count the number
; of times that disjuct `d` was observed on word `w`.  These numbers
; define a vector on the word `w`. Given two words `w` and `u`, one
; may define the vector dot-product between them as:
;
;      D(u,w) = sum_d N(u,d) N(w,d)
;
; Note that this product is symmetric: D(u,w) = D(w,u)
; Note that this product underlies the definition of the cosine between
; the two vectors.  The cosine is just
;
;     cos(u,w) = D(u,w) / sqrt (D(u,u) D(w,w))
;
; This product can also be used to define a symmetric mutial
; information between the words:
;
;     MI(u,w) = log_2 D(u,w) D(*,*) / D(u,*) D(w,*)
;
; where, as usual, the star * defines a wild-card sum:
;
;     D(u,*) = D(*,u) = sum_w D(u,w)
;
; Note that the 
xxxxxxx
; ---------------------------------------------------------------------
;
(use-modules (srfi srfi-1))
(use-modules (opencog))
(use-modules (opencog persist))
(use-modules (opencog matrix))

; ---------------------------------------------------------------------

