(define-module (opencog nlp ghost)
  #:use-module (opencog)
  #:use-module (opencog nlp)
  #:use-module (opencog nlp relex2logic)
  #:use-module (opencog nlp chatbot)
  #:use-module (opencog openpsi)
  #:use-module (opencog logger)
  #:use-module (opencog exec)
  #:use-module (opencog eva-behavior)
  #:use-module (srfi srfi-1)
  #:use-module (rnrs io ports)
  #:use-module (ice-9 popen)
  #:use-module (ice-9 optargs)
  #:use-module (ice-9 rdelim)
  #:use-module (ice-9 regex)
  #:use-module (ice-9 getopt-long)
  #:use-module (ice-9 eval-string)
  #:use-module (system base lalr)
)

(load "ghost/translator.scm")
(load "ghost/terms.scm")
(load "ghost/matcher.scm")
(load "ghost/cs-parse.scm")
(load "ghost/test.scm")

(define-public (ghost-parse TXT)
  "Parse the TXT, convert them into atomese."
  (test-parse TXT))

(define-public (ghost-parse-file FILE)
  "Parse everything in FILE, convert them into atomese."
  (test-parse-file FILE))

(define-public (ghost TXT)
  "Parse the input TXT using nlp-parse and connect it to the GHOST anchor.
   Should run this with the main OpenPsi loop."
  (State (Anchor "GHOST: Currently Processing") (car (nlp-parse TXT))))
