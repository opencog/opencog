; Links relex-to-logic output with relex-opencog-output
; It is temporary until the r2l rules are moved into the URE
; XXX Huh ???

; Test sentence : "This is a sentence."

; modules needed for call-with-input-file and get-string-all
;  (use-modules (rnrs io ports))
;  (use-modules (ice-9 rdelim))

(define-module (opencog nlp sureal))

(use-modules (opencog oc-config))
(use-modules (opencog nlp))
(use-modules (opencog nlp oc))

(load-extension (string-append opencog-ext-path-sureal "libsureal") "opencog_nlp_sureal_init")

(load "sureal/surface-realization.scm")
