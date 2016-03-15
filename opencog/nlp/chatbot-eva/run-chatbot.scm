
; For users who are not aware of readline ...
(use-modules (ice-9 readline))
(activate-readline)

; Stuff actually needed to get the chatbot running...
(use-modules (opencog) (opencog cogserver))

(use-modules (opencog nlp))
(use-modules (opencog nlp chatbot-eva))

; Load the robot model, from opencog/ros-behavior-scripting
(use-modules (opencog eva-behavior))

; Load the Eva personality.
; (display %load-path)
; (load-from-path "opencog/eva-behavior/cfg-eva.scm")

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
; Prime the atomspace with content that sureal can use to generate
; sentences. XXX This belongs in self-model.scm, but it screws up
; in there for some insane reason. This needs to be fixed.  FIXME.
(nlp-parse "I am looking to the left")
(nlp-parse "I am looking to the right")
(nlp-parse "I am looking up")
(nlp-parse "I am looking upward")
(nlp-parse "I am looking downward")
(nlp-parse "I am looking leftwards")
(nlp-parse "I am looking rightwards")
(nlp-parse "I am looking forward")

; Hush the output on load.
*unspecified*
