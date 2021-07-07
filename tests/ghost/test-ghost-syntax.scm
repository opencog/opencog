; --------------------------------------------------
; Setup for the test
(use-modules (opencog test-runner))

(opencog-test-runner)

(define ghost-syntax-utest "GhostSyntaxUTest")

; --------------------------------------------------
; Setup for GHOST
(use-modules
  (opencog)
  (opencog nlp)
  (opencog nlp oc)
  (opencog nlp relex2logic)
  (opencog nlp chatbot)
  (opencog openpsi)
  (opencog ghost)
  (opencog ghost procedures)
  (opencog python))

; Set the address for relex server
(set-relex-server-host)

; Helpers for this test
(define (get-result input)
  (if (nil? input)
    (string)
    (string-join (map cog-name (test-ghost input)))))

(define (get-rules-from-label label)
  (map gar (filter
    (lambda (x)
      (and (psi-rule? (gar x))
           (any (lambda (p) (string=? "alias" (cog-name p)))
             (cog-chase-link 'EvaluationLink 'PredicateNode x))))
    (cog-incoming-by-type (Concept label) 'ListLink))))

; --------------------------------------------------
; Test
(test-begin ghost-syntax-utest)

; --- Lemma --- ;
(define ghost-lemma "GHOST lemma")

(ghost-parse "r: (apple) test lemma")
(ghost-parse "r: (where do) test lemma - do")

(test-equal ghost-lemma "test lemma" (get-result "apple"))
(test-equal ghost-lemma "test lemma" (get-result "apples"))
(test-equal ghost-lemma "test lemma - do" (get-result "where does Alice work"))
(test-equal ghost-lemma "test lemma - do" (get-result "where did you go"))

; --- Literal --- ;
(define ghost-literal "GHOST literal")

(ghost-parse "r: (oranges) test literal")
(ghost-parse "r: (let's) test literal - apostrophe")
(ghost-parse "r: (I'm Good) test literal - capital letter")
(ghost-parse "r: (mr. and MRS. Smith) test literal - personal title")
(ghost-parse "r: (9 a.m.) test literal - time 1")
(ghost-parse "r: (5pm) test literal - time 2")
(ghost-parse "r: (\"wanna drink\") test literal - phrase")

(test-equal ghost-literal (string) (get-result "orange"))
(test-equal ghost-literal "test literal" (get-result "oranges"))
(test-equal ghost-literal "test literal - apostrophe" (get-result "let's"))
(test-equal ghost-literal "test literal - capital letter" (get-result "I'm good"))
(test-equal ghost-literal "test literal - personal title" (get-result "Mr. and Mrs. Smith"))
(test-equal ghost-literal "test literal - time 1" (get-result "9a.m."))
(test-equal ghost-literal "test literal - time 1" (get-result "9 a.m."))
(test-equal ghost-literal "test literal - time 2" (get-result "5pm"))
(test-equal ghost-literal "test literal - time 2" (get-result "5 pm"))
(test-equal ghost-literal "test literal - phrase" (get-result "wanna drink"))
(test-equal ghost-literal (string) (get-result "wanna go drink"))

; --- Concept --- ;
(define ghost-concept "GHOST concept")

(ghost-parse "concept: ~eat (eat ingests \"binge and purge\" ~swallow (break a leg))")
(ghost-parse "concept: ~swallow (gulp)")
(ghost-parse "r: (~eat) test concept")

(test-equal ghost-concept "test concept" (get-result "eat"))
(test-equal ghost-concept "test concept" (get-result "eats"))
(test-equal ghost-concept (string) (get-result "ingest"))
(test-equal ghost-concept "test concept" (get-result "ingests"))
(test-equal ghost-concept (string) (get-result "binges and purge"))
(test-equal ghost-concept "test concept" (get-result "binge and purge"))
(test-equal ghost-concept "test concept" (get-result "gulp"))
(test-equal ghost-concept "test concept" (get-result "break a leg"))
(test-equal ghost-concept "test concept" (get-result "broke a leg"))

; --- Choice --- ;
(define ghost-choice "GHOST choice")

(ghost-parse "concept: ~mobile (move)")
(ghost-parse "r: ([read jumped \"dance around\" ~mobile (know about that)]) test choice")

(test-equal ghost-choice "test choice" (get-result "read"))
(test-equal ghost-choice "test choice" (get-result "jumped"))
(test-equal ghost-choice "test choice" (get-result "dance around"))
(test-equal ghost-choice "test choice" (get-result "move"))
(test-equal ghost-choice "test choice" (get-result "know about that"))

; --- Optional --- ;
(define ghost-optional "GHOST optional")

(ghost-parse "concept: ~re (really)")
(ghost-parse "r: (it's {so ~re} amazing) test optional")

(test-equal ghost-optional "test optional" (get-result "it's amazing"))
(test-equal ghost-optional "test optional" (get-result "it's so amazing"))
(test-equal ghost-optional "test optional" (get-result "it's really amazing"))
(test-equal ghost-optional (string) (get-result "it's not amazing"))

; --- Wildcard --- ;
(define ghost-wildcard "GHOST wildcard")

(ghost-parse "r: (it's * there) test wildcard")
(ghost-parse "r: (it's *1 great) test wildcard - exact")
(ghost-parse "r: (it's *~3 bad) test wildcard - range")

(test-equal ghost-wildcard "test wildcard" (get-result "it's there"))
(test-equal ghost-wildcard "test wildcard" (get-result "it's not there"))
(test-equal ghost-wildcard "test wildcard - exact" (get-result "it's so great"))
(test-equal ghost-wildcard (string) (get-result "it's not so great"))
(test-equal ghost-wildcard (string) (get-result "it's great"))
(test-equal ghost-wildcard "test wildcard - range" (get-result "it's bad"))
(test-equal ghost-wildcard "test wildcard - range" (get-result "it's not bad"))
(test-equal ghost-wildcard "test wildcard - range" (get-result "it's in my opinion bad"))
(test-equal ghost-wildcard (string) (get-result "it's in my opinion not bad"))

; --- Variable --- ;
(define ghost-variable "GHOST variale")

(ghost-parse "r: (go _*) test variable - '_0")
(ghost-parse "r: (ride _*1) test variable - _0")
(ghost-parse "r: (finish _watch people) test variable - '_0")
(ghost-parse "r: (I saw _*) test variable - _0 '_0")

(test-equal ghost-variable "test variable - to those stores" (get-result "go to those stores"))
(test-equal ghost-variable "test variable - bike" (get-result "ride bikes"))
(test-equal ghost-variable "test variable - watching" (get-result "finished watching people"))
(test-equal ghost-variable (string) (get-result "finished firing people"))
(test-equal ghost-variable "test variable - Alice Alice" (get-result "I saw Alice"))

; --- User Variable --- ;
(define ghost-user-variable "GHOST user variable")
(define-public (findkiller) (List (Word "Bob")))

(ghost-parse "$who = Jay")
(ghost-parse "$age = 2")
(ghost-parse "r: (me name be _*) $name='_0 test user variable - '_0")
(ghost-parse "r: (I know you $name==John) test user variable 1")
(ghost-parse "r: (I know $name) test user variable 2")
(ghost-parse "r: (user variable test) test user variable - $who $age")
(ghost-parse "r: (who be the killer) $killer=^findkiller() test user variable - $killer")

(test-equal ghost-user-variable "test user variable - John" (get-result "my name is John"))
(test-equal ghost-user-variable "test user variable 1" (get-result "I know you"))
(test-equal ghost-user-variable "test user variable 2" (get-result "I know John"))
(test-equal ghost-user-variable "test user variable - Jay 2" (get-result "user variable test"))
(test-equal ghost-user-variable "test user variable - Bob" (get-result "who is the killer"))

; --- Nagation --- ;
(define ghost-negation "GHOST negation")

(ghost-parse "r: (!close the shop stays) test negation - lemma")
(ghost-parse "r: (!'driving you are) test negation - literal")
(ghost-parse "r: (!~re the coffee) test negation - concept")
(ghost-parse "r: (![not don't] tennis) test negation - choice")

(test-equal ghost-negation "test negation - lemma" (get-result "the shop stays open"))
(test-equal ghost-negation (string) (get-result "the shop stays close"))
(test-equal ghost-negation "test negation - literal" (get-result "you are diving"))
(test-equal ghost-negation (string) (get-result "you are driving"))
(test-equal ghost-negation "test negation - concept" (get-result "the coffee is hot"))
(test-equal ghost-negation (string) (get-result "the coffee is really hot"))
(test-equal ghost-negation "test negation - choice" (get-result "play tennis"))
(test-equal ghost-negation (string) (get-result "don't play tennis"))
(test-equal ghost-negation (string) (get-result "do not play tennis"))

; --- Sentence Boundary --- ;
(define ghost-sentence-boundary "GHOST sentence boundary")

(ghost-parse "r: (< tea) test sentence boundary - start")
(ghost-parse "r: (tree >) test sentence boundary - end")
(ghost-parse "r: (< green wood >) test sentence boundary - both")

(test-equal ghost-sentence-boundary "test sentence boundary - start" (get-result "tea is an aromatic beverage"))
(test-equal ghost-sentence-boundary (string) (get-result "that cup of tea"))
(test-equal ghost-sentence-boundary "test sentence boundary - end" (get-result "visit the tree"))
(test-equal ghost-sentence-boundary (string) (get-result "visit the tree down the street"))
(test-equal ghost-sentence-boundary "test sentence boundary - both" (get-result "green wood"))
(test-equal ghost-sentence-boundary (string) (get-result "magical green wood"))
(test-equal ghost-sentence-boundary (string) (get-result "green wood paper"))
(test-equal ghost-sentence-boundary (string) (get-result "organic green wood table"))

; --- Function --- ;
(define ghost-function "GHOST function")

(define-public (get-weather-info) (List (Word "sunny")))
(define-public (check-name name) (if (string=? "Bob" (cog-name name)) (stv 1 1) (stv 0 1)))
(ghost-parse "r: (tell me the weather) test function - ^get-weather-info")
(ghost-parse "r: (what is the weather) test function - ^scm_get-weather-info")
(ghost-parse "r: (who am I ^check-name(Bob)) test function - predicate")
(ghost-parse "r: (who are you ^scm_check-name(Bob)) test function - scm predicate")


(python-eval "
from opencog.type_constructors import *
from opencog.scheme_wrapper import scheme_eval_as
from opencog.cogserver_type_constructors import *
# from opencog.nlp_types import *

atomspace = scheme_eval_as('(cog-atomspace)')
set_type_ctor_atomspace(atomspace)

def WordNode(node_name, tv=None):
    return atomspace.add_node(types.WordNode, node_name, tv)

def get_weather_info():
    return ListLink(WordNode('cloudy'))

def check_name(word):
    return TruthValue(1, 1) if word.name == 'Bob' else TruthValue(0, 0)
")

(ghost-parse "r: (what was the weather yesterday) test function - ^py_get_weather_info")
(ghost-parse "r: (who were you ^py_check_name(Bob)) test function - py predicate")

(test-equal ghost-function "test function - sunny" (get-result "tell me the weather"))
(test-equal ghost-function "test function - sunny" (get-result "what is the weather"))
(test-equal ghost-function "test function - cloudy" (get-result "what was the weather yesterday"))
(test-equal ghost-function "test function - predicate" (get-result "who am I"))
(test-equal ghost-function "test function - scm predicate" (get-result "who are you"))
(test-equal ghost-function "test function - py predicate" (get-result "who were you"))

; --- Comparison --- ;
(define ghost-comparison "GHOST comparison")

(ghost-parse "$name=Bob")
(ghost-parse "$num=3")
(define who "Jay")
(define-public (tell-who) (List (Word who)))
(ghost-parse "r: (_* is standing there '_0==$name) test comparison 1")
(ghost-parse "r: (since _* years ago '_0>$num) test comparison 2")
(ghost-parse "r: (_* is a lucky number '_0<=$num) test comparison 3")
(ghost-parse "r: (tell me who ^tell-who()==\"Jay\") test comparison 4")

(test-equal ghost-comparison "test comparison 1" (get-result "Bob is standing there"))
(test-equal ghost-comparison (string) (get-result "Alice is standing there"))
(test-equal ghost-comparison "test comparison 2" (get-result "since 10 years ago"))
(test-equal ghost-comparison (string) (get-result "since 3 years ago"))
(test-equal ghost-comparison "test comparison 3" (get-result "2 is a lucky number"))
(test-equal ghost-comparison "test comparison 3" (get-result "3 is a lucky number"))
(test-equal ghost-comparison (string) (get-result "4 is a lucky number"))
(test-equal ghost-comparison "test comparison 4" (get-result "tell me who"))
(set! who "Bob")
(test-equal ghost-comparison (string) (get-result "tell me who"))

; --- Unordered Match --- ;
(define ghost-unordered "GHOST unordered")

(ghost-parse "r: (<< banana cherry kiwi >>) test unordered")

(test-equal ghost-unordered "test unordered" (get-result "banana cherry kiwi"))
(test-equal ghost-unordered "test unordered" (get-result "banana kiwi cherry"))
(test-equal ghost-unordered "test unordered" (get-result "cherry banana kiwi"))
(test-equal ghost-unordered "test unordered" (get-result "kiwi banana cherry"))
(test-equal ghost-unordered "test unordered" (get-result "cherry kiwi banana"))
(test-equal ghost-unordered "test unordered" (get-result "kiwi cherry banana"))

; --- Action Choice --- ;
(define ghost-action-choice "GHOST action choice")

(ghost-parse "r: (action one) [AC1][AC2][AC3]")
(ghost-parse "r: (action two) [W1][W2] and [W3][W4]")

(define result1 (get-result "action one"))
(define result2 (get-result "action two"))
(test-assert ghost-action-choice
  (or (string=? "AC1" result1)
      (string=? "AC2" result1)
      (string=? "AC3" result1)))
(test-assert ghost-action-choice
  (or (string=? "W1 and W3" result2)
      (string=? "W1 and W4" result2)
      (string=? "W2 and W3" result2)
      (string=? "W2 and W4" result2)))

; --- Question --- ;
(define ghost-question "GHOST question")

(ghost-parse "?: (done) test question")

(test-equal ghost-question "test question" (get-result "is it done yet?"))
(test-equal ghost-question (string) (get-result "it's done"))

; --- Rejoinder --- ;
(define ghost-rejoinder "GHOST rejoinder")

(ghost-parse "
  r: (parent) test rejoinder
    j1: (level one a) test rejoinder j1a
      j2: (level two) test rejoinder j2
    j1: (level one b) test rejoinder j1b
")

(test-equal ghost-rejoinder "test rejoinder" (get-result "parent"))
(test-equal ghost-rejoinder "test rejoinder j1a" (get-result "level one a"))
(test-equal ghost-rejoinder (string) (get-result "level one b"))
(test-equal ghost-rejoinder "test rejoinder j2" (get-result "level two"))
(test-equal ghost-rejoinder "test rejoinder" (get-result "parent"))
(test-equal ghost-rejoinder "test rejoinder j1b" (get-result "level one b"))
(test-equal ghost-rejoinder (string) (get-result "level two"))

; --- Label & Reuse --- ;
(define ghost-label-reuse "GHOST label-reuse")

(ghost-parse "r: R0 (-) test reuse 1")
(ghost-parse "r: (reuse zero) ^reuse(R0)")
(ghost-parse "r: R1 (reuse one) test reuse 2
                 j1: RJ1 (reuse rejoinder) test reuse rej")
(ghost-parse "r: R2 (reuse two) ^reuse(RJ1)")
(ghost-parse "r: (reuse three) ^reuse(R0) ^reuse(R1)")

(test-equal ghost-label-reuse "test reuse 1" (get-result "reuse zero"))
(test-equal ghost-label-reuse "test reuse rej" (get-result "reuse two"))
(test-equal ghost-label-reuse "test reuse 1 test reuse 2" (get-result "reuse three"))
(test-equal ghost-label-reuse "test reuse rej" (get-result "reuse rejoinder"))

; --- System Function --- ;
(define ghost-sys-func "GHOST system functions")

(ghost-parse "r: NR (normal rule) test system functions")
(ghost-parse "r: KR (keep rule) test system functions - keep ^keep()
                j1: (rej rule) test system functions - set-used rej")
(ghost-parse "r: (set used rule) test system functions - set-used ^set_used(KR)")

(test-equal ghost-sys-func "test system functions" (get-result "normal rule"))
(test-assert ghost-sys-func (= 0 (cog-mean (car (ghost-get-rule "NR")))))
(test-equal ghost-sys-func "test system functions - keep" (get-result "keep rule"))
(test-assert ghost-sys-func (< 0 (cog-mean (car (ghost-get-rule "KR")))))
(test-equal ghost-sys-func "test system functions - set-used" (get-result "set used rule"))
(test-assert ghost-sys-func (= 0 (cog-mean (car (ghost-get-rule "KR")))))
(test-equal ghost-sys-func "test system functions - set-used rej" (get-result "rej rule"))

; --- Goal --- ;
(use-modules (srfi srfi-1))
(define ghost-goals "GHOST goal assignment")

(ghost-parse "
  goal: (stay_alive=1)
    r: G1 (-) goal one

    #goal: (novelty=0.6 please_user=0.4)
    r: G2 (-) goal two

    #goal: (stay_alive=0.5)
    r: G3 (-) goal three
")

(define rules-g1 (get-rules-from-label "G1"))
(define rules-g2 (get-rules-from-label "G2"))
(define rules-g3 (get-rules-from-label "G3"))

; Check if the goals have been assigned corrected
(test-assert ghost-goals
  (lset= equal?
    (list (Concept "stay_alive"))
    (map psi-get-goal rules-g1)))
(test-assert ghost-goals
  (lset= equal?
    (list (Concept "stay_alive") (Concept "please_user") (Concept "novelty"))
    (map psi-get-goal rules-g2)))
(test-assert ghost-goals
  (lset= equal?
    (list (Concept "stay_alive"))
    (map psi-get-goal (get-rules-from-label "G3"))))

; Check the strengths of the rules as well
(test-assert ghost-goals
  (lset= equal? (list 1.0) (map cog-mean rules-g1)))
(test-assert ghost-goals
  (lset= equal? (list 1.0 0.6 0.4) (map cog-mean rules-g2)))
(test-assert ghost-goals
  (lset= equal? (list 0.5) (map cog-mean rules-g3)))

; --- Link Concept --- ;
(define ghost-link-concept "GHOST link concept")

(ghost-parse "
  link-concept: (madagascar, golden bean, cocoa)

    #link-concept: (single origin)
    r: LC (-) link concept
")

(define concepts
  (list
    ; The "GHOST" concept will be linked to all of the
    ; GHOST rules by default
    (Concept "GHOST")
    ; These are manually linked from above
    (Concept "madagascar")
    (Concept "golden bean")
    (Concept "cocoa")
    (Concept "single origin")))

(test-assert ghost-link-concept
  (lset= equal? concepts
    (cog-chase-link 'MemberLink 'ConceptNode (car (ghost-get-rule "LC")))))

; End of the test
(test-end ghost-syntax-utest)
