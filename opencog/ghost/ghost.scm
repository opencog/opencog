(define-module (opencog ghost)
  #:use-module (opencog)
  #:use-module (opencog nlp)
  #:use-module (opencog nlp oc)
  #:use-module (opencog nlp relex2logic)
  #:use-module (opencog nlp chatbot)
  #:use-module (opencog openpsi)
  #:use-module (opencog logger)
  #:use-module (opencog exec)
  #:use-module (opencog ghost procedures)
  #:use-module (opencog attention-bank)
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
(define ghost-last-executed (Anchor (ghost-prefix "Last Executed")))
(define ghost-rule-executed (Predicate (ghost-prefix "Rule Executed")))
(define ghost-time-last-executed (Predicate (ghost-prefix "Time Last Executed")))
(define ghost-word-seq (Predicate (ghost-prefix "Word Sequence")))
(define ghost-word-original (Predicate (ghost-prefix "Word Original")))
(define ghost-rule-type (Predicate (ghost-prefix "Rule Type")))
(define ghost-next-reactive-rule (Predicate (ghost-prefix "Next Reactive Rule")))
(define ghost-next-rejoinder (Predicate (ghost-prefix "Next Rejoinder")))
(define ghost-rej-seq-num (Predicate (ghost-prefix "Rejoinder Sequence Number")))
(define ghost-context-specificity (Predicate (ghost-prefix "Context Specificity")))
(define strval-rejoinder (StringValue "rejoinder"))
(define strval-reactive-rule (StringValue "reactive-rule"))
(define strval-proactive-rule (StringValue "proactive-rule"))

;; --------------------
(define-public (ghost-word-seq-pred)
"
  Returns the Predicate that represent word sequences.
"
  ghost-word-seq)

;; --------------------
;; For rule parsing

; The initial urge of goals
(define initial-urges '())

; The default urge is 0
(define default-urge 0)

; A list of top level goals that will be shared with all the rules
; defined under it
(define top-lv-goals '())

; A list of rule level goals that will only be associated with the
; rule following it
(define rule-lv-goals '())

; When set, all the rules created under it will be linked to these concepts,
; until a new top level goal is defined
(define top-lv-link-concepts '())

; When set, the rule created under it will be linked to these concepts
(define rule-lv-link-concepts '())

; Whether the rules defined under a top level goal is ordered
(define is-rule-seq? #f)

; How many rules we've seen under a particular top level goal
(define goal-rule-cnt 0)

; A list of local variables exist in the pattern of a rule,
; during rule parsing & creation
(define pat-vars '())

; A list of features for a rule, will be used during instantiation
(define rule-features '())

; A list of all the labels of the rules we have seen
(define rule-label-list '())

; An association list of the types (reactive rules, rejoinders etc)
; of the rules
(define rule-type-alist '())

; An association list for storing the default rule contexts and actions
(define global-default-rule-alist '())

; An association list that contains all the terms needed to create
; the actual rules
; The key of this list is the labels of the rules
(define rule-alist '())

; A list to keep track of the rule hierarchy
; Will be used when dealing with rejoinders
(define rule-hierarchy '())

; Keep a record of the goals associated with the rule that has
; just been instantiated, for dealing with rule ordering
(define goals-of-prev-rule '())

; To clear the above states
(define (clear-parsing-states)
  (set! initial-urges '())
  (set! default-urge 0)
  (set! top-lv-goals '())
  (set! rule-lv-goals '())
  (set! top-lv-link-concepts '())
  (set! rule-lv-link-concepts '())
  (set! is-rule-seq? #f)
  (set! goal-rule-cnt 0)
  (set! pat-vars '())
  (set! rule-features '())
  (set! rule-label-list '())
  (set! rule-type-alist '())
  (set! global-default-rule-alist '())
  (set! rule-alist '())
  (set! rule-hierarchy '())
  (set! goals-of-prev-rule '())
)

;; --------------------
;; For rule matching

; The buffer for input being sent to GHOST,
; and the other one is to record the one that has been processed
; They are needed to sync with the action selection cycle
(define ghost-buffer "")
(define ghost-processed "")

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
(define reactive-rule-sti-boost 1)
(define rejoinder-sti-boost 10)
(define refractory-period 1)
(define specificity-based-action-selection #t)

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
    (if (nil? tail-sent)
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
(define (process-ghost-buffer)
  (if (and (not (equal? ghost-buffer ghost-processed))
           (cog-atom? ghost-buffer)
           (equal? (cog-type ghost-buffer) 'SentenceNode))
    (begin
      (set! ghost-processed ghost-buffer)
      (generate-word-seq ghost-buffer)
      (append-to-sent-seq ghost-buffer)
      (State ghost-curr-proc ghost-buffer)))
)

;; --------------------
;; To parse rules and interact with GHOST, the main interfaces

(define-public (ghost-parse TXT)
"
  ghost-parse TXT

  Parse the TXT, convert them into atomese.
"
  (cs-parse TXT)
  (process-rule-stack)
)

; ----------
(define-public (ghost-parse-file FILE)
"
  ghost-parse-file FILE

  Parse everything in the FILE, and convert them into atomese.
"
  (cs-parse-file FILE)
  (process-rule-stack)
)

; ----------
(define-public (ghost-parse-files . FILES)
"
  ghost-parse-files . FILES

  Parse everything in the FILES, and convert them into atomese.
"
  (for-each (lambda (f) (cs-parse-file f)) FILES)
  (process-rule-stack)
)

; ----------
(define-public (ghost TXT)
"
  ghost TXT

  Parse the input TXT using nlp-parse and link it to the GHOST anchor.
  Should run this with ECAN and OpenPsi.

  Run (ghost-get-result) to get the output generated for the input, if any.
"
  (ghost-take-sentence-node (car (nlp-parse (string-trim TXT))))
)

; ----------
(define-public (ghost-take-sentence-node SENTENCE-NODE)
"
  ghost-take-sentence-node SENTENCE-NODE

  Take the SENTENCE-NODE as input and link it to the GHOST anchor.
"
  (set! ghost-result '())
  (set! ghost-buffer SENTENCE-NODE)
  ghost-buffer
)

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
