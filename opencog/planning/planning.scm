;
; OpenCog Logger module
;
; Copyright (c) 2016 OpenCog Foundation
;

(define-module (opencog planning))

; We need this to set the LTDL_LIBRARY_PATH
(use-modules (opencog))

(load-extension "libplanning" "opencog_planning_init")
