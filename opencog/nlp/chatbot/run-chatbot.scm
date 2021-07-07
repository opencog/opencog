#! /usr/local/bin/guile \
  -L /usr/local/share/opencog/scm --
!#
; For users who are not aware of readline ...
(use-modules (ice-9 readline))
(activate-readline)

; Stuff actually needed to get the chatbot running...
(use-modules (opencog) (opencog cogserver))

(use-modules (opencog nlp) (opencog nlp oc) (opencog nlp chatbot))
(use-modules (opencog nlp relex2logic))

; Unfortunately, due to the design of the URE, the list of rules
; must be public, and cannot be loaded in the relex2logic or chatbot
; modules above. So we work around this here.
; See https://github.com/opencog/opencog/issues/2021
(load-r2l-rulebase)

(start-cogserver "../lib/opencog-chatbot.conf")
