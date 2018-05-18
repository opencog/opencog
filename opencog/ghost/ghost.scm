(define-module (opencog ghost)
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
  #:use-module (system base lalr))

; Temporarily used during transitioning. The aim is to make life easier for
; developers who work with atomspace before opencog/atomspace/pull/1664 while
; waiting for opencog/opencog/issues/3107 to resolve.
(if (resolve-module '(opencog attention-bank) #:ensure #f)
  (use-modules (opencog attention-bank)))

;; --------------------
;; Shared things being used in the module

(define-public (ghost-prefix . STR)
"
  Append a GHOST prefix to STR, usually for naming a node
  to make it clearer for a person to review/debug.
"
  (string-concatenate (append (list "GHOST ") STR)))

; ----------
(define (ghost-uvar STR)
"
  Define a VariableNode for a user variable.
"
  (Variable (ghost-prefix "user-variable-" STR)))

; ----------
; A component in OpenPsi
(define ghost-component (psi-component "GHOST"))

(define-public (ghost-get-component)
"
  Return the GHOST component.
"
  ghost-component)

; ----------
; Define the logger for GHOST
(define ghost-logger (cog-new-logger))

; Default configuration for the GHOST logger
(cog-logger-set-component! ghost-logger "GHOST")
(cog-logger-set-level! ghost-logger "info")
(cog-logger-set-stdout! ghost-logger #t)

(define-public (ghost-get-logger)
"
  Return the logger for GHOST.
"
  ghost-logger)

; ----------
; Various anchors, predicates, values etc that will be used

(define ghost-curr-proc (Anchor (ghost-prefix "Currently Processing")))
(define ghost-curr-topic (Anchor (ghost-prefix "Current Topic")))
(define ghost-last-executed (Anchor (ghost-prefix "Last Executed")))
(define ghost-no-constant (Anchor (ghost-prefix "No constant terms")))
(define ghost-rule-executed (Predicate (ghost-prefix "Rule Executed")))
(define ghost-word-seq (Predicate (ghost-prefix "Word Sequence")))
(define ghost-lemma-seq (Predicate (ghost-prefix "Lemma Sequence")))
(define ghost-topic (Concept (ghost-prefix "Topic")))
(define ghost-topic-feature (Predicate (ghost-prefix "Topic Feature")))
(define ghost-rule-type (Predicate (ghost-prefix "Rule Type")))
(define ghost-next-responder (Predicate (ghost-prefix "Next Responder")))
(define ghost-next-rejoinder (Predicate (ghost-prefix "Next Rejoinder")))
(define strval-rejoinder (StringValue "rejoinder"))
(define strval-responder (StringValue "responder"))
(define strval-random-gambit (StringValue "random gambit"))
(define strval-gambit (StringValue "gambit"))

;; --------------------
(define-public (ghost-word-seq-pred)
"
  Returns the Predicate that represent word sequences.
"
  ghost-word-seq)

;; --------------------
;; For rule parsing

; When set, all the rules created will be under this topic
(define rule-topic '())

; The initial urge of goals
(define initial-urges '())

; A list of top level goals that will be shared with all the rules
; defined under it
(define top-lv-goals '())

; Whether the rules defined under a top level goal is ordered
(define is-rule-seq #f)

; How many rules we've seen under a particular top level goal
(define goal-rule-cnt 0)

; A list of local variables exist in the pattern of a rule,
; during rule parsing & creation
(define pat-vars '())

; A list of all the labels of the rules we have seen
(define rule-label-list '())

; An association list that contains all the terms needed to create
; the actual rules
; The key of this list is the labels of the rules
(define rule-alist '())

; A list to keep track of what rules hierarchy
; Will be used when dealing with rejoinders
(define rule-hierarchy '())

;; --------------------
;; For rule matching

; Keep a record of the lemmas we have seen, and it serves as a cache as well
(define lemma-alist '())

; Keep a record of the most recent outputs after executing the action
; of a GHOST rule
(define ghost-result '())

; Storing values assigned to the user variables for referencing later
(define uvars '())

; The weights of various parameters used in the action selector
; For experimental purpose
(define strength-weight 1)
(define context-weight 1)
(define sti-weight 1)
(define urge-weight 1)
(define responder-sti-boost 1)
(define rejoinder-sti-boost 10)

;; --------------------
;; For monitoring the status
(define num-rules-found 0)
(define num-rules-evaluated 0)
(define num-rules-satisfied 0)

;; --------------------
;; Load the required files

(load "ghost/test.scm")
(load "ghost/utils.scm")
(load "ghost/terms.scm")
(load "ghost/translator.scm")
(load "ghost/matcher.scm")
(load "ghost/cs-parse.scm")
(load "ghost/stimulation.scm")

;; --------------------
;; Key used to create a singly linked-list to record the sequence of
;; input sentences.
(define sent-input-seq (Anchor "Sentence Input Sequence"))
(define next-sent-key (Predicate "next-sentence"))
(define tail-sent-key (Predicate "last-inputed-sentence"))

;; --------------------
(define (tail-input-sent)
"
  Returns nil or the SentenceNode for the last inputed sentence.
"
  (cog-value sent-input-seq tail-sent-key))

;; --------------------
(define (append-to-sent-seq sent)
"
  append-to-sent-seq SENT

  Returns the SentenceNode SENT after appending it to the sentence
  input sequence.
"
  ; The assumption is that there is only one thread that is adding
  ; to the sequence.
  (let ((tail-sent (tail-input-sent)))
    (if (null? tail-sent)
      (begin
        (cog-set-value! sent-input-seq tail-sent-key sent)
        (cog-set-value! sent-input-seq next-sent-key sent))
      (begin
        (cog-set-value! sent-input-seq tail-sent-key sent)
        (cog-set-value! tail-sent next-sent-key sent))
    )
  ))

;; --------------------
(define (next-input-sent sent)
"
  next-input-sent SENT

  Returns nil or the input SentenceNode that followed SENT SentenceNode.
"
  (cog-value sent next-sent-key))

;; --------------------
;; To parse rules and interact with GHOST, the main interfaces

(define-public (ghost-parse TXT)
"
  Parse the TXT, convert them into atomese.
"
  (test-parse TXT)
  (process-rule-stack)
)

; ----------
(define-public (ghost-parse-file FILE)
"
  Parse everything in the topic FILE, and convert them into atomese.
"
  (test-parse-file FILE)
  (process-rule-stack)
)

; ----------
(define-public (ghost TXT)
"
  Parse the input TXT using nlp-parse and connect it to the GHOST anchor.
  Should run this with the main OpenPsi loop.
"
  (define sent (car (nlp-parse TXT)))
  (generate-word-seqs sent)
  (append-to-sent-seq sent)
  (State ghost-curr-proc sent)
  sent)

; ----------
(define-public (ghost-run)
"
  Start the psi-loop for GHOST.
"
  (psi-run ghost-component))

; ----------
(define-public (ghost-halt)
"
  Halt the psi-loop for GHOST.
"
  (psi-halt ghost-component))
