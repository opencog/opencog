(use-modules (opencog)
             (opencog nlp)
             (opencog nlp relex2logic)
             (opencog nlp chatbot)
             (opencog openpsi))

(load "terms.scm")
(load "translator.scm")
(load "matcher.scm")

; Things we have in the AtomSpace
(chat-concept "eat" (list "eat" "ingest" "binge and purge"))
(chat-rule '((lemma "I") (concept "eat") (lemma "meat")) '(say "Do you really? I am a vegan."))

; Input to the system
(define input "I eat meat")

; --------------------
; Below should be done in chatbot-psi
; Will do the code integration later

; Parse the input
(define input-sent-node (car (nlp-parse input)))

; Connect the SentenceNode to the anchor
(State (Anchor "Currently Processing") input-sent-node)

; --------------------
; Run a psi-step and get the result
; Right now the output of  the "Say"
; action is written to opencog.log
(psi-step)

; TODO: Find a better way for cleaning the state
(State (Anchor "Currently Processing") (Concept "Default State"))
