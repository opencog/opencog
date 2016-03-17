; Links relex-to-logic output with relex-opencog-output
; It is temporary until the r2l rules are moved into the URE
; XXX Huh ???

; Test sentence : "This is a sentence."

; modules needed for call-with-input-file and get-string-all
;  (use-modules (rnrs io ports))
;  (use-modules (ice-9 rdelim))

(define-module (opencog nlp sureal))

; We need this to set the LTDL_LIBRARY_PATH
(use-modules (opencog))

(load-extension "libsureal" "opencog_nlp_sureal_init")

(load "sureal/surface-realization.scm")
