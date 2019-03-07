; --------------------------------------------------
; Setup for the test
(use-modules (opencog test-runner))

(opencog-test-runner)

(define ghost-utest "GhostUTest")

; --------------------------------------------------
; Setup for GHOST
(use-modules
  (opencog)
  (opencog nlp)
  (opencog nlp relex2logic)
  (opencog nlp chatbot)
  (opencog ghost)
  (opencog ghost procedures))

; Set the address for relex server
(set-relex-server-host)

; Helper for this test
(define (get-result input)
  (if (null? input)
    (string)
    (string-join (map cog-name (test-ghost input)))))

; --------------------------------------------------
; Test
(test-begin ghost-utest)

; --- Lemma --- ;
(define ghost-lemma "GHOST lemma")

(ghost-parse "r: (apple) test lemma")

(test-equal ghost-lemma "test lemma" (get-result "apple"))
(test-equal ghost-lemma "test lemma" (get-result "apples"))

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

(test-equal ghost-variable "test variable - to those stores" (get-result "go to those stores"))
(test-equal ghost-variable "test variable - bike" (get-result "ride bikes"))
(test-equal ghost-variable "test variable - watching" (get-result "finished watching people"))
(test-equal ghost-variable (string) (get-result "finished firing people"))

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

(test-equal ghost-user-variable "test user variable - john" (get-result "my name is john"))
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
(ghost-parse "r: (who am I ^check-name(Bob)) test function - predicate")
(ghost-parse "r: (^face-seen()) test function - predicate only")

(test-equal ghost-function "test function - sunny" (get-result "tell me the weather"))
(test-equal ghost-function "test function - predicate" (get-result "who am I"))

; End of the test
(test-end ghost-utest)
