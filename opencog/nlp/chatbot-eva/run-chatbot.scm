
; For users who are not aware of readline ...
(use-modules (ice-9 readline))
(activate-readline)

; Stuff actually needed to get the chatbot running...
(use-modules (opencog) (opencog cogserver))

(use-modules (opencog nlp))
(use-modules (opencog nlp chatbot-eva))

(start-cogserver "../../lib/opencog-chatbot.conf")

; Load the ROS stubs ...
(system "echo \"py\\n\" | cat - chatbot-eva/atomic-dbg.py |netcat localhost 17004")

; XXX remove the below when we get a chance.
; Must load the rulebase before running eva; see bug
; https://github.com/opencog/opencog/issues/2021 for details
; XXX fixme -- we should not need to load either relex2logic or
; the rules right here, since the code in this module does not depend
; directly on thes.
(use-modules (opencog nlp relex2logic))
(load-r2l-rulebase)
;
; Prime the atomspace. XXX This belongs in self-model.scm, but
; screws up there for some insane reason. This needs to be fixed.
(nlp-parse "I am looking to the left")
(nlp-parse "I am looking to the right")
(nlp-parse "I am looking up")

; Hush the output on load.
*unspecified*
