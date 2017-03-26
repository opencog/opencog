
; For users who are not aware of readline ...
(use-modules (ice-9 readline))
(activate-readline)

; Stuff actually needed to get the chatbot running...
(use-modules (opencog) (opencog cogserver))

(use-modules (opencog nlp))
(use-modules (opencog nlp chatbot))
(use-modules (opencog nlp chatbot-eva))
(use-modules (opencog eva-behavior))

; Load the Eva personality.
; (load-eva-config)

(start-cogserver "../../lib/opencog-chatbot.conf")

; XXX remove the below when we get a chance.
; Must load the rulebase before running eva; see bug
; https://github.com/opencog/opencog/issues/2021 for details
; XXX fixme -- we should not need to load either relex2logic or
; the rules right here, since the code in this module does not depend
; directly on these.
(use-modules (opencog nlp relex2logic))
; (load-r2l-rulebase)
;

; XXX temp hack to run in module context, for debugging
; (add-to-load-path ".")
; (add-to-load-path "..")
; (load "chatbot-eva.scm")

; Hush the output on load.
*unspecified*
