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
  #:use-module (ice-9 receive)
  #:use-module (system base lalr)
)

;; --------------------
;; Shared things being used in the module

(define-public (ghost-prefix STR) (string-append "GHOST: " STR))
(define (ghost-var-word NUM)
  (Variable (ghost-prefix
    (string-append "variable-word-" (number->string NUM)))))
(define (ghost-var-lemma NUM)
  (Variable (ghost-prefix
    (string-append "variable-lemma-" (number->string NUM)))))
(define (ghost-uvar STR)
  (Variable (ghost-prefix (string-append "user-variable-" STR))))
(define ghost-anchor (Anchor (ghost-prefix "Currently Processing")))
(define ghost-no-constant (Anchor (ghost-prefix "No constant terms")))
(define ghost-word-seq (Predicate (ghost-prefix "Word Sequence")))
(define ghost-lemma-seq (Predicate (ghost-prefix "Lemma Sequence")))
(define ghost-topic (Anchor (ghost-prefix "Current Topic")))

; Define the logger for GHOST
(define ghost-logger (cog-new-logger))
(cog-logger-set-component! ghost-logger "Ghost")
(cog-logger-set-level! ghost-logger "info")
(cog-logger-set-stdout! ghost-logger #t)
(define-public (ghost-get-logger)
  "Returns the logger for ghost."
  ghost-logger
)

; When set, all the rules created will be under this topic
(define rule-topic '())

; A list of shared goals for all the rules under the same topic file
(define shared-goals '())

; Keep a record of the variables, if any, found in the pattern of a rule
(define pat-vars '())

; Keep a record of the value assigned to the user variables that authors defined
(define uvars '())

; Keep a record of the lemmas we have seen, and it serves as a cache as well
(define lemma-alist '())

; Keep a record of the most recent outputs generated
(define ghost-result '())

;; --------------------
;; The required files

(load "ghost/utils.scm")
(load "ghost/functions.scm")
(load "ghost/translator.scm")
(load "ghost/terms.scm")
(load "ghost/matcher.scm")
(load "ghost/cs-parse.scm")
(load "ghost/test.scm")

;; --------------------
;; To parse rules and interact with GHOST

(define-public (ghost-parse TXT)
  "Parse the TXT, convert them into atomese."
  (test-parse TXT))

(define-public (ghost-parse-file FILE)
  "Parse everything in the topic FILE, and convert them into atomese."
  (test-parse-file FILE))

(define-public (ghost TXT)
  "Parse the input TXT using nlp-parse and connect it to the GHOST anchor.
   Should run this with the main OpenPsi loop."
  (define sent (car (nlp-parse TXT)))
  (generate-word-seqs sent)
  (State ghost-anchor sent))
