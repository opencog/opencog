;
; run-r2l.scm
; Start up and run the Relex2Logic subsystem.
;

; For users who are not aware of readline ...
(use-modules (ice-9 readline))
(activate-readline)

; Stuff actually needed to get r2l running...
(use-modules (opencog) (opencog nlp))

; The primary R2L wrapper is actually in the chatbot dir.
(use-modules (opencog nlp chatbot))
(use-modules (opencog nlp relex2logic))
