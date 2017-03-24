(use-modules (opencog)
             (opencog nlp)
             (opencog nlp relex2logic)
             (opencog nlp chatbot)
             (opencog openpsi))

(load "terms.scm")
(load "translator.scm")
(load "matcher.scm")

; --------------------
; Things we have in the AtomSpace

; Define a concept "eat"
(chat-concept "eat" (list "eat" "ingest" "binge and purge"))

; Should be triggered by "I eat meat" and "I ingest meat"
; "I binge and purge meat" does not work yet
(chat-rule '((lemma "I") (concept "eat") (lemma "meat")) '(say "Do you really? I am a vegan."))

; Should be triggered by "dog" and "dogs"
(chat-rule '((lemma "dog")) '(say "I have a cat"))

; --------------------
; Below should be done in chatbot-psi
; Will do the code integration later
; as there are things need to be fixed first
; in the chatbot-psi module

; Parse and connect the input to this anchor
; and then OpenPsi will handle the rest
(define (chat input)
  (let ((sent-node (car (nlp-parse input))))
    (State (Anchor "Currently Processing") sent-node)))

; --------------------
; Test it!

; Start the OpenPsi loop
(psi-run)

; Input to the system
(chat "I eat meat")

; Note:
; At the moment (DefinedPredicate "Say") sends the output
; to the log file, can tail the opencog.log to see the result
; from where guile is started, for now
