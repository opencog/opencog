; Links relex-to-logic output with relex-opencog-output
; It is temporary until the r2l rules are moved into the URE
; XXX Huh ???

; Test sentence : "This is a sentence."

; modules needed for call-with-input-file and get-string-all
;  (use-modules (rnrs io ports))
;  (use-modules (ice-9 rdelim))

(define-module (opencog nlp sureal))

(load-extension "libsureal" "opencog_nlp_sureal_init")

(use-modules (srfi srfi-1)   ; needed for delete-duplicates
             (ice-9 threads) ; needed for par-map
             (ice-9 rdelim) (ice-9 regex) (ice-9 receive)
             (opencog)
             (opencog nlp)
             (opencog nlp lg-dict)
             (opencog nlp relex2logic)
)

(load "sureal/surface-realization.scm")
