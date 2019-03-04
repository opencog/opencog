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

; End of the test
(test-end ghost-utest)
