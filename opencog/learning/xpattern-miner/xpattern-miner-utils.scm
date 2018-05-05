;;
;; xpattern-miner-utils.scm
;;
;;;; Commentary:
;;
;; Handy utilities for working with the ure pattern miner. In
;; particular to configure the rule engine.
;;
;; Utilities include:
;;
;; If you add more utilities don't forget to add them in the
;; export-xpattern-miner-utils function.
;;
;;;; Code:
;; Copyright (c) 2018, OpenCog Foundation
;;

(use-modules (opencog))
(use-modules (opencog exec))
(use-modules (opencog rule-engine))
(use-modules (srfi srfi-1))

(define (top)
"
  Insert the top abstraction in the current atomspace

  (Lambda (Variable \"$X\") (Variable \"$X\"))
"
  (let ((top-arg (Variable "$top-arg")))
    (Lambda top-arg top-arg)))

(define (random-texts-cpt)
"
  Create a random Concept node for adding text members
"
  (random-node 'ConceptNode 16 "texts-"))

(define (random-miner-rbs-cpt)
"
  Create a random Concept node for defining a pattern miner rule base
"
  (random-node 'Concept 16 "pattern-miner-rbs-"))

(define (fill-texts-cpt texts-cpt texts)
"
  For each element text of texts create

  MemberLink
    text
    texts-cpt

  texts can be a Scheme list or an Atomese List or Set.

  Once all memberships have been added to the current atomspace,
  it returns texts-cpt.
"
  (let* ((texts-lst (if (and (cog-atom? texts)
                             (or (eq? (cog-type? texts) 'ListLink)
                                 (eq? (cog-type? texts) 'SetLink)))
                        (cog-outgoing-set texts)
                        texts))
         (mk-member (lambda (text) (Member text texts-cpt))))
    (for-each mk-member texts-lst))
  texts-cpt)

(define (configure-rules pm-rbs)
  ;; Load and associate rules to pm-rbs
  (let* ((rule-path "opencog/xpattern-miner/rules/")
         (base-rule-files (list "shallow-abstraction.scm"
                                "specialization.scm"))
         (mk-full-path (lambda (rf) (string-append rule-path rf)))
         (rule-files (map mk-full-path base-rule-files)))
    (for-each load-from-path rule-files)
    (let* ((rules (list ;; TODO somehow shallow-abstraction-rule-name
                        ;; and specialization-rule-name are not
                        ;; accessible thom here after loading
                        ;; shallow-abstraction.scm and
                        ;; specialization.scm. For that reason we use
                        ;; their Atomese definitions instead. This
                        ;; might be a guile bug.
                        (DefinedSchemaNode "shallow-abstraction-rule")
                        (DefinedSchemaNode "specialization-rule"))))
      (ure-add-rules pm-rbs rules))))

(define* (configure-miner pm-rbs #:key (maxiter -1))
"
  Given a Concept node representing a rule based system for the
  pattern miner. Automatically configure it with the appropriate
  rules and parameters.

  Usage: (configure-miner pm-rbs #:maxiter mi)

  pm-rbs: Concept node of the rule-based system to configure

  mi: [optional] Maximum number of iterations of the rule-engine
"
  ;; Load and associate rules to pm-rbs
  (configure-rules pm-rbs)

  ;; Set parameters
  (ure-set-maximum-iterations maxiter)
  (ure-set-fc-retry-sources #f)
)

(define (minsup-eval pattern texts ms)
"
  Construct

  Evaluation
    Predicate \"minsup\"
    List
      pattern
      texts
      ms
"
  (Evaluation
     (Predicate "minsup")
     (List
        pattern
        texts
        ms)))

(define (minsup-eval-true pattern texts ms)
"
  Like minsup-eval and add (stv 1 1) on the EvaluationLink
"
  (cog-set-tv! (minsup-eval pattern texts ms) (stv 1 1)))

(define* (cog-mine texts ms #:key (maxiter -1) (initpat (top)))
"
  Mine patterns in texts with minimum support ms, optionally
  using maxiter iterations and starting from the initial pattern initpat.

  Usage: (cog-mine texts ms #:maxiter mi #:initpat ip)

  texts: Collection of texts to mine. It can be given in 3 forms

         1. Scheme list of atoms

            (list t1 ... tn)

         2. Atomese list or set of atoms

            (List t1 ... tn)
            (Set t1 ... tn)

         3. A concept with all the texts in it

            (Concept texts-name)

            such that

            (Member
              t1
              (Concept texts-name))
            ...
            (Member
              tn
              (Concept texts-name))

  ms: Minimum support. All pattern with frequency below ms are
      discarded

  mi: [optional] Maximum number of iterations allocated

  ip: [optional] Initial pattern to start the search from. All mined
      pattern will be specializations of this pattern.

  Under the hood it will create a rule base and a query for the rule
  engine, configure it according to the user's options and run it.
  Everything takes place in a child atomspace. After the job is done
  it will remove the child atomspace after having copied the solution
  set in the parent atomspace.

  Pattern mining is a computationally demanding. There are three
  ways to improve performances at this time.

  1. Set ms as high as possible. The higher the minium support the
     more pruning will take place in search tree. That is because
     specializations cannot have more support than their parent
     abstraction.

  2. If it takes too long to copmlete, it means the search tree is
     too large to explorer entirely. Set mi to a positive value to
     halt the exploration after a certain number of iterations of
     the rule engine.

  3. If you have any idea of the kind of patterns you are looking
     for, you can provide an initial pattern, ip. All mined patterns
     will be specialized from that pattern. This can considerably
     reduce the search space as only a subtree of the whole search
     tree is considered.
"
  (let* (;; Create a temporary child atomspace for the URE         
         (tmp-as (cog-new-atomspace (cog-get-atomspace)))
         (parent-as (cog-set-atomspace! tmp-as))
         ;; Construct text concept if necessary
         (create-texts-cpt? (not (and (cog-atom? texts)
                                      (eq? texts-type 'ConceptNode))))
         (texts-cpt (if create-texts-cpt?
                        ;; Construct a temporary concept containing
                        ;; the texts
                        (fill-texts-cpt (random-texts-cpt) texts)
                        ;; Otherwise texts is already a concept
                        texts))
         ;; Check that the initial pattern has enough support
         (es (enough-support? ip texts-cpt ms)))
    (if (not es)
        ;; The initial pattern doesn't have enough support, thus the
        ;; solution set is empty        
        (and ;; Use and to sequence statements
             (cog-set-atomspace! parent-as)
             ;; TODO: delete tmp-as if possible
             (Set))
        ;; The initial pattern has enough support, let's configure the
        ;; rule engine and run the pattern mining query
        (let* ((source (minsup-eval-true ip texts-cpt ms))
               (miner-rbs (random-pm-rbs-cpt)))
          (configure-miner miner-rbs #:maxiter maxiter)
          (let* ((results (cog-fc pm-rbs source))
                 (results-lst (cog-outgoing-set results)))
            (cog-set-atomspace! parent-as)
            ;; TODO: delete tmp-as but without deleting its atoms, if
            ;; possible
            (Set results-lst))))))

(define (export-xpattern-miner-utils)
  (export
    top
    random-texts-cpt
    random-miner-rbs-cpt
    fill-texts-cpt
    configure-rules
    configure-miner
    minsup-eval
    minsup-eval-true
    cog-mine
  )
)
