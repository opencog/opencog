;
; run-r2l.scm
; Start up and run the Relex2Logic subsystem.
;

; For users who are not aware of readline ...
(use-modules (ice-9 readline))
(activate-readline)

(use-modules (opencog) (opencog nlp) (opencog nlp oc))
(use-modules (opencog nlp relex2logic))

; The demo uses the `nlp-parse` function in the chatbot.
(use-modules (opencog nlp chatbot))
