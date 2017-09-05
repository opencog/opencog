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
(define ghost-word-set (Predicate (ghost-prefix "Word Set")))
(define ghost-lemma-seq (Predicate (ghost-prefix "Lemma Sequence")))
(define ghost-lemma-set (Predicate (ghost-prefix "Lemma Set")))

; Define the logger for GHOST
(define ghost-logger (cog-new-logger))
(cog-logger-set-component! ghost-logger "Ghost")
(cog-logger-set-level! ghost-logger "info")
(cog-logger-set-stdout! ghost-logger #t)
(define-public (ghost-get-logger)
  "Returns the logger for ghost."
  ghost-logger
)

; For features that are not currently supported
(define (feature-not-supported NAME VAL)
  "Notify the user that a particular feature is not currently supported."
  (cog-logger-error ghost-logger "Feature not supported: \"~a ~a\"" NAME VAL))

; Keep a record of the variables, if any, found in the pattern of a rule
(define pat-vars '())

; Keep a record of the groundings of variables that authors defined
(define var-grd-words '())
(define var-grd-lemmas '())

; Keep a record of the value assigned to the user variables that authors defined
(define uvars '())

; Keep a record of the lemmas we have seen, and it serves as a cache as well
(define lemma-alist '())

; Indicate whether to get the lemma of a word from the RelEx server (default),
; or from the WordNet CLI (for unit test only)
(define test-get-lemma #f)

;; --------------------
;; The required files

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
  "Parse everything in FILE, convert them into atomese."
  (test-parse-file FILE))

(define-public (ghost TXT)
  "Parse the input TXT using nlp-parse and connect it to the GHOST anchor.
   Should run this with the main OpenPsi loop."
  (State ghost-anchor (car (nlp-parse TXT))))
