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

(define (iota-plus-one x)
"
  Like iota but goes from 1 to x included instead of going
  from 0 to x excluded.
"
  (map (lambda (x) (+ x 1)) (iota x)))

(define (greater-tv-strength x y)
"
  Return #t if the tv strength of x is greater than that of y
"
  (> (cog-mean x) (cog-mean y)))

(define (desc-sort-by-tv-strength l)
"
  Given a link of atoms, sort these atom in descending order according
  to their tv strengths.
"
  (List (sort (cog-outgoing-set l) greater-tv-strength)))

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

(define (random-surprisingness-rbs-cpt)
"
  Create a random Concept node for defining a rule base for surprisingness
"
  (random-node 'ConceptNode 16 "surprisingness-rbs-"))

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
  ;; Maybe remove, nothing is mandatory anymore
  *unspecified*)

(define (configure-shallow-specialization-rule pm-rbs unary)
  ;; Load and associate mandatory rules to pm-rbs
  (let* ((base-rule-file "shallow-specialization.scm")
         (rule-file (mk-full-rule-path base-rule-file))
         (rule (if unary
                   (DefinedSchema "shallow-specialization-unary-rule")
                   (DefinedSchema "shallow-specialization-rule"))))
    (load-from-path rule-file)
    (ure-add-rule pm-rbs rule)))

(define (false-tv? tv)
  (equal? tv (stv 0 1)))

(define* (configure-optional-rules pm-rbs
                                   #:key
                                   (incremental-expansion (stv 0 1))
                                   (max-conjuncts -1))
  (define enable-incremental-expansion (not (or (false-tv? incremental-expansion)
                                                (= max-conjuncts 1))))

  ;; Load shallow specialization, either unary, if
  ;; incremental-expansion is enabled, or not
  (configure-shallow-specialization-rule pm-rbs enable-incremental-expansion)

  ;; Load conjunction-expansion and associate to pm-rbs
  (if enable-incremental-expansion
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
             (rules (if (<= max-conjuncts 0)
                        ;; No maximum conjuncts
                        (list (rulify 0))
                        ;; At most max-conjuncts conjuncts
                        (map rulify (iota-plus-one (- max-conjuncts 1))))))
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

(define* (configure-isurp isurp-rbs mode max-conjuncts)
  ;; Load I-Surprisingess rules
  (let* ((base-rule-file "i-surprisingness.scm")
         (rule-pathfile (mk-full-rule-path base-rule-file))
         (rule-fule (mk-full-rule-path base-rule-file))
         (mk-rule-name (lambda (i) (string-append (symbol->string mode) "-"
                                                  (number->string i)
                                                  "ary-rule")))
         (mk-rule-alias (lambda (i) (DefinedSchema (mk-rule-name i))))
         (rules (map mk-rule-alias (cdr (iota-plus-one max-conjuncts)))))
    (load-from-path rule-pathfile)
    (ure-add-rules isurp-rbs rules)))

(define pattern-var
  (Variable "$pattern"))

(define (isurp-target texts-cpt)
  (isurp-eval pattern-var texts-cpt))

(define (isurp-vardecl)
  (TypedVariable pattern-var (Type "LambdaLink")))

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

(define (isurp-eval pattern texts)
"
  Construct

  Evaluation
    Predicate \"isurp\"
    List
      pattern
      texts
"
  (Evaluation
    (Predicate "isurp")
    (List
      pattern
      texts)))

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
                   (surprisingness 'isurp))
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

  su: [optional, default='isurp] After running the pattern miner,
      patterns can be ranked according to some surprisingness measure.
      The following surported modes are:

      'isurp-old:  Verbatim port of Shujing I-Surprisingness.

      'nisurp-old: Verbatim port of Shujing nornalized I-Surprisingness.

      'isurp:      New implementation of I-Surprisingness that takes
                   linkage into account.

      'nisurp:     New implementation of normalized I-Surprisingness
                   that takeslinkage into account.

  Under the hood it will create a rule base and a query for the rule
  engine, configure it according to the user's options and run it.
  Everything takes place in a child atomspace. After the job is done
  it will remove the child atomspace after copying the solution set
  in the parent atomspace.

  Pattern mining is computationally demanding. There are three ways
  to improve performances at this time.

  1. Set ms as high as possible. The higher the minium support the
     more pruning will take place in search tree. That is because
     specializations cannot have more support than their parent
     abstraction.

  2. If it takes too long to complete, it means the search tree is
     too large to explore entirely. Lower the number of iterations
     of the rule engine, mi, to halt the exploration earlier.

  3. If you have any idea of the kind of patterns you are looking
     for, you can provide an initial pattern, ip. All mined patterns
     will be specialized from that pattern. This can considerably
     reduce the search space as only a subtree of the whole search
     tree is considered.

  4. If your pattern is a conjunction of multiple clauses, you can
     enable incremental conjunction expansion, see the
     #:incremental-expansion option.
"
  (let* (;; Create a temporary child atomspace for the URE
         (tmp-as (cog-new-atomspace (cog-atomspace)))
         (parent-as (cog-set-atomspace! tmp-as))
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
        (begin (cog-set-atomspace! parent-as)
               ;; TODO: delete tmp-as
               (Set))

        ;; The initial pattern has enough support, let's configure the
        ;; rule engine and run the pattern mining query
        (let* (;; Configure pattern miner forward chainer
               (source (minsup-eval-true initpat texts-cpt minsup))
               (miner-rbs (random-miner-rbs-cpt))
               (cfg-m (configure-miner miner-rbs
                                       #:maximum-iterations maximum-iterations
                                       #:complexity-penalty complexity-penalty
                                       #:incremental-expansion incremental-expansion
                                       #:max-conjuncts max-conjuncts))

               ;; Run pattern miner in a forward way
               (results (cog-fc miner-rbs source))
               ;; Fetch all relevant results
               (patterns (fetch-patterns texts-cpt minsup))
               (patterns-lst (cog-outgoing-set patterns)))

          (if (equal? surprisingness 'none)

              ;; No surprisingness, simple return the pattern list
              (begin
                (cog-set-atomspace! parent-as)
                ;; TODO: delete tmp-as but without deleting its atoms
                patterns-lst)

              ;; Run surprisingness
              (let*
                  ;; Configure surprisingness backward chainer
                  ((isurp-rbs (random-surprisingness-rbs-cpt))
                   (target (isurp-target texts-cpt))
                   (vardecl (isurp-vardecl))
                   (cfg-s (configure-isurp isurp-rbs surprisingness max-conjuncts))

                   ;; Run surprisingness in a backward way
                   (isurp-results (cog-bc isurp-rbs target #:vardecl vardecl)))
                (cog-set-atomspace! parent-as)
                ;; TODO: delete tmp-as but without deleting its atoms
                (desc-sort-by-tv-strength isurp-results)))))))

(define (export-miner-utils)
  (export
    iota-plus-one
    top
    random-texts-cpt
    random-miner-rbs-cpt
    fill-texts-cpt
    configure-mandatory-rules
    configure-optional-rules
    configure-rules
    configure-isurp
    configure-miner
    minsup-eval
    minsup-eval-true
    isurp-eval
    get-members
    get-cardinality
    fetch-patterns
    conjunct-pattern
    cog-miner
    cog-mine
  )
)
