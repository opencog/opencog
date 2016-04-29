; Copyright (C) 2016 OpenCog Foundation
; This is an experimental example of how simple training within an OpenPsi-
; driven dialog system could work.

#!
(use-modules (opencog)
             (opencog nlp)
             (opencog nlp relex2logic)
             (opencog nlp fuzzy)
             (opencog nlp chatbot)
             (opencog exec)
             (opencog openpsi))

(load-r2l-rulebase)
(set! relex-server-host "172.17.0.2")  ; a relex server manhin has set up
!#

;-------------------------------------------------------------------------------
; Steps to run:
; 1. Make sure the above relex-server-host is set correctly
; 2. Load this example in Guile, e.g.
;    (load "../examples/openpsi/simple-train.scm")
; 3. Use (chat) function to talk to it, e.g.
;    (chat "Are you conscious?")
;-------------------------------------------------------------------------------

(load "chat.scm")

; Behavior rules



