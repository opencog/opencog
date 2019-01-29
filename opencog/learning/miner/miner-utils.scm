;;
;; miner-utils.scm
;;
;;;; Commentary:
;;
;; Handy utilities for working with the ure pattern miner. In
;; particular to configure the rule engine.
;;
;; Utilities include:
;;
;; If you add more utilities don't forget to add them in the
;; export-miner-utils function.
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
  (random-node 'ConceptNode 16 "pattern-miner-rbs-"))

(define (fill-texts-cpt texts-cpt texts)
"
  For each element text of texts create

  MemberLink
    text
    texts-cpt

  texts can be
  1. a Scheme list
  2. an Atomese List or Set
  3. an AtomSpace

  Once all memberships have been added to the current atomspace,
  texts-cpt is returned.
"
  (define (is-List-Set a)
    (and (cog-atom? texts)
         (or (eq? (cog-type? texts) 'ListLink)
             (eq? (cog-type? texts) 'SetLink))))
  (let* ((texts-lst (cond ;; Scheme list
                          ((list? texts) texts)
                          ;; Atomese List or Set
                          ((is-List-Set texts) (cog-outgoing-set texts))
                          ;; AtomSpace
                          ;; TODO: bug!!! should use the given atomspace
                          ((cog-atomspace? texts) (cog-get-atoms 'Atom #t))))
         (mk-member (lambda (text) (Member text texts-cpt))))
    (for-each mk-member texts-lst))
  texts-cpt)

(define (mk-full-rule-path brf)
  (let ((rule-path "opencog/learning/miner/rules/"))
    (string-append rule-path brf)))

(define (configure-mandatory-rules pm-rbs)
  ;; Load and associate mandatory rules to pm-rbs
  (let* ((base-rule-files (list "shallow-specialization.scm"))
         (rule-files (map mk-full-rule-path base-rule-files))
         (rules (list ;; TODO somehow shallow-specialization-rule-name
                      ;; are not accessible thom here after loading
                      ;; shallow-specialization.scm. For that reason
                      ;; we use its Atomese definition instead. This
                      ;; might be a guile bug.
                      (DefinedSchema "shallow-specialization-rule"))))
    (for-each load-from-path rule-files)
    (ure-add-rules pm-rbs rules)))

(define (false-tv? tv)
  (equal? tv (stv 0 1)))

(define* (configure-optional-rules pm-rbs
                                   #:key
                                   (incremental-expansion (stv 0 1))
                                   (max-conjuncts -1))
  ;; Load conjunction-expansion and associate to pm-rbs
  (if (not (or (false-tv? incremental-expansion) (= max-conjuncts 1)))
      (let* ((ie-tv (if (equal? #t) (stv 0.01 0.5) incremental-expansion))
             (rule-pathfile (mk-full-rule-path "conjunction-expansion.scm"))
             (namify (lambda (i)
                       (string-concatenate
                        (cons "conjunction-expansion-"
                              (if (<= i 0)
                                  (list "rule")
                                  (list (number->string i) "ary-rule"))))))
             (rulify (lambda (i)
                       (list (DefinedSchemaNode (namify i)) ie-tv)))
             (max-conjuncts? (lambda (x) (<= max-conjuncts x)))
             (add1 (lambda (x) (+ x 1)))
             (1-to-max-conjuncts (unfold max-conjuncts? identity add1 1))
             (rules (if (<= max-conjuncts 0)
                        ;; No maximum conjuncts
                        (list (rulify 0))
                        ;; At most max-conjuncts conjuncts
                        (map rulify 1-to-max-conjuncts))))
        (load-from-path rule-pathfile)
        (ure-add-rules pm-rbs rules))))

(define* (configure-rules pm-rbs
                          #:key
                          (incremental-expansion (stv 0 1))
                          (max-conjuncts 3))
  (configure-mandatory-rules pm-rbs)
  (configure-optional-rules pm-rbs
                            #:incremental-expansion incremental-expansion
                            #:max-conjuncts max-conjuncts))

(define* (configure-miner pm-rbs
                          #:key
                          (maximum-iterations 1000)
                          (complexity-penalty 1)
                          (incremental-expansion (stv 0 1))
                          (max-conjuncts 3))
"
  Given a Concept node representing a rule based system for the
  pattern miner. Automatically configure it with the appropriate
  rules and parameters.

  Usage: (configure-miner pm-rbs
                          #:maximum-iterations mi
                          #:complexity-penalty cp
                          #:incremental-expansion tv
                          #:max-conjuncts mc)

  pm-rbs: Concept node of the rule-based system to configure

  mi: [optional, default=1000] Maximum number of iterations allocated.
      If negative then the pattern miner keeps running till all patterns
      have been exhausted (not recommended unless you know what you're doing).

  cp: [optional, default=1] Complexity penalty parameter passed to the forward
      chainer. It controls breadth vs depth search. A high value means more
      breadth. A value of 0 means a equilibrium between breadth and depth.
      A negative value means more depth. Possible range is (-inf, +inf)
      but it's rarely necessary in practice to go outside of [-10, 10].

  tv: [optional, default=(stv 0 1)] Truth value of a rule to expand existing
      conjunctions of patterns. It will only expand conjunctions with enough
      support with patterns with enough support. Alternatively the user can
      provide #t instead of a truth value. In that case a default true value
      will be selected.

  mc: [optional, default=3] In case tv is set to a positive strength and
      confidence, and thus incremental conjunction expansion is enabled, that
      option allows to limit the number of conjuncts to mc. If negative then
      the number of conjuncts can grow unlimited (not recommended unless you
      know what you're doing). As of now mc can not be set above 9 (which
      should be more than enough).
"
  ;; Load and associate rules to pm-rbs
  (configure-rules pm-rbs
                   #:incremental-expansion incremental-expansion
                   #:max-conjuncts max-conjuncts)

  ;; Set parameters
  (ure-set-maximum-iterations pm-rbs maximum-iterations)
  (ure-set-complexity-penalty pm-rbs complexity-penalty)

  ;; If there is no incremental expansion then each rule is
  ;; deterministic, thus no need to retry exhausted sources
  (ure-set-fc-retry-exhausted-sources pm-rbs (not (false-tv? incremental-expansion)))
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
        (if (number? ms) (Number ms) ms))))

(define (minsup-eval-true pattern texts ms)
"
  Like minsup-eval and add (stv 1 1) on the EvaluationLink
"
  (cog-set-tv! (minsup-eval pattern texts ms) (stv 1 1)))


(define (get-members C)
"
  Given a concept node C, return all its members
"
  (let* ((member-links (cog-filter 'MemberLink (cog-incoming-set C)))
         (member-of-C (lambda (x) (equal? C (gdr x))))
         (members (map gar (filter member-of-C member-links))))
    members))

(define (get-cardinality C)
"
  Giveb a concept node C, return its number of members
"
  (length (get-members C)))

(define (size-ge texts ms)
  (let* ((result (>= (get-cardinality texts) (cog-number ms))))
    (bool->tv result)))

(define (texts->atomspace texts)
"
  Create an atomspace with all members of concept texts in it.
"
  (let* ((members (get-members texts))
         (texts-as (cog-new-atomspace)))
    (cog-cp members texts-as)
    texts-as))

(define (pattern->bindlink pattern)
"
  Turn a pattern into a BindLink for subsequent pattern
  matching texts.
"
  (if (= (cog-arity pattern) 2)
      ;; With variable declaration
      (let* ((vardecl (gar pattern))
             (body (gdr pattern)))
        (Bind vardecl body body)) ; to deal with unordered links
      ;; Without variable declaration
      (let* ((body (gar pattern)))
        (Bind body body)))) ; to deal with unordered links

(define (fetch-patterns texts ms)
"
  Fetch all patterns with enough support, thus found in the following
  hypergraphs

  Evaluation (stv 1 1)
    Predicate \"minsup\"
    List
      <pattern>
      texts
      ms
"
  (let* ((patvar (Variable "$patvar"))
         (target (minsup-eval patvar texts ms))
         (vardecl (TypedVariable patvar (Type "LambdaLink")))
         (precond (absolutely-true-eval target))
         (gl (Get vardecl (And target precond))))
    (cog-execute! gl)))

(define* (conjunct-pattern nconj)
"
  Create a pattern of nconj conjunctions.

  For instance (conjunct-pattern 3), creates

  (Lambda
    (VariableList
      (Variable \"$X-1\")
      (Variable \"$X-2\")
      (Variable \"$X-3\"))
    (And
      (Variable \"$X-1\")
      (Variable \"$X-2\")
      (Variable \"$X-3\")))
"
  (let* ((vars (gen-variables "$X" nconj))
         (var-lst (VariableList vars))
         (var-conj (And vars)))
    (Lambda var-lst var-conj)))

(define* (cog-miner . args)
  (display ("The command you are looking for is cog-mine.")))

(define* (cog-mine texts
                   #:key
                   (minsup 10)
                   (initpat (top))
                   (maximum-iterations 1000)
                   (complexity-penalty 1)
                   (incremental-expansion (stv 0 1))
                   (max-conjuncts 3)
                   (surprisingness "I-Surprisingness"))
"
  Mine patterns in texts (text trees, a.k.a. grounded hypergraphs) with minimum
  support ms, optionally using mi iterations and starting from the initial
  pattern initpat.

  Usage: (cog-mine texts
                   #:minsup ms
                   #:initpat ip
                   #:maximum-iterations mi
                   #:complexity-penalty cp
                   #:incremental-expansion tv
                   #:max-conjuncts mc
                   #:surprisingness su)

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

  ms: [optional, default=10] Minimum support. All patterns with frequency below
      ms are discarded. Can be a Scheme number or an Atomese number node.

  ip: [optional, default=(top)] Initial pattern to start the search from.
      All mined patterns will be specializations of this pattern.

  mi: [optional, default=1000] Maximum number of iterations allocated.
      If negative then the pattern miner keeps running till all patterns
      have been exhausted (not recommended unless you know what you're doing).

  cp: [optional, default=1] Complexity penalty parameter passed to the forward
      chainer. It controls breadth vs depth search. A high value means more
      breadth. A value of 0 means an equilibrium between breadth and depth.
      A negative value means more depth. Possible range is (-inf, +inf)
      but it's rarely necessary in practice to go outside of [-10, 10].

  tv: [optional, default=(stv 0 1)] Truth value of a rule to expand existing
      conjunctions of patterns. It will only expand conjunctions with enough
      support with patterns with enough support. Alternatively the user can
      provide #t instead of a truth value. In that case a default true value
      will be selected.

  mc: [optional, default=3] In case tv is set to a positive strength and
      confidence, and thus incremental conjunction expansion is enabled, that
      option allows to limit the number of conjuncts to mc. If negative then
      the number of conjuncts can grow unlimited (not recommended unless you
      know what you're doing). As of now mc can not be set above 9 (which
      should be more than enough).

  su: [optional, default=\"I-Surprisingness\"] After running the pattern miner,
      patterns can be ranked according to some surprisingness measure. Currently,
      only \"I-Surprisingness\" is implemented.

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

  2. If it takes too long to complete, it means the search tree is
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
         (parent-as (cog-push-atomspace))
         (texts-concept? (and (cog-atom? texts)
                              (eq? (cog-type texts) 'ConceptNode)))
         (texts-cpt (if (not texts-concept?)
                        ;; Construct a temporary concept containing
                        ;; the texts
                        (fill-texts-cpt (random-texts-cpt) texts)
                        ;; Otherwise texts is already a concept
                        texts))
         (ms-nn (if (number? minsup) (Number minsup) minsup))
         ;; Check that the initial pattern has enough support
         (es (cog-enough-support? initpat texts-cpt ms-nn)))
    (if (not es)
        ;; The initial pattern doesn't have enough support, thus the
        ;; solution set is empty.
        (begin (cog-pop-atomspace) (Set))

        ;; The initial pattern has enough support, let's configure the
        ;; rule engine and run the pattern mining query
        (let* ((source (minsup-eval-true initpat texts-cpt minsup))
               (miner-rbs (random-miner-rbs-cpt)))
          (configure-miner miner-rbs
                           #:maximum-iterations maximum-iterations
                           #:complexity-penalty complexity-penalty
                           #:incremental-expansion incremental-expansion
                           #:max-conjuncts max-conjuncts)
          (let* (;; Run the pattern miner in a forward way
                 (results (cog-fc miner-rbs source))
                 ;; Fetch all relevant results
                 (patterns (fetch-patterns texts-cpt minsup))
                 (patterns-lst (cog-outgoing-set patterns)))
            (cog-pop-atomspace)
            ;; TODO: copy atoms from temp atomspace to main atomspace.
            (Set patterns-lst))))))

(define (export-miner-utils)
  (export
    top
    random-texts-cpt
    random-miner-rbs-cpt
    fill-texts-cpt
    configure-mandatory-rules
    configure-optional-rules
    configure-rules
    configure-miner
    minsup-eval
    minsup-eval-true
    get-members
    get-cardinality
    fetch-patterns
    conjunct-pattern
    cog-miner
    cog-mine
  )
)
