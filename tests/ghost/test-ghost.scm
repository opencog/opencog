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
(ghost-parse "r: (apple) test lemma")

(test-equal "GHOST lemma" "test lemma" (get-result "apple"))
(test-equal "GHOST lemma" "test lemma" (get-result "apples"))

; --- Literal --- ;
(ghost-parse "r: (oranges) test literal")
(ghost-parse "r: (let's) test literal - apostrophe")
(ghost-parse "r: (I'm Good) test literal - capital letter")
(ghost-parse "r: (mr. and MRS. Smith) test literal - personal title")
(ghost-parse "r: (9 a.m.) test literal - time 1")
(ghost-parse "r: (5pm) test literal - time 2")
(ghost-parse "r: (\"wanna drink\") test literal - phrase")

(test-equal "GHOST literal" (string) (get-result "orange"))
(test-equal "GHOST literal" "test literal" (get-result "oranges"))
(test-equal "GHOST literal" "test literal - apostrophe" (get-result "let's"))
(test-equal "GHOST literal" "test literal - capital letter" (get-result "I'm good"))
(test-equal "GHOST literal" "test literal - personal title" (get-result "Mr. and Mrs. Smith"))
(test-equal "GHOST literal" "test literal - time 1" (get-result "9 a.m."))
(test-equal "GHOST literal" "test literal - time 2" (get-result "5pm"))
(test-equal "GHOST literal" "test literal - time 2" (get-result "5 pm"))
(test-equal "GHOST literal" "test literal - phrase" (get-result "wanna drink"))
(test-equal "GHOST literal" (string) (get-result "wanna go drink"))

; --- Concept --- ;
(ghost-parse "concept: ~eat (eat ingests \"binge and purge\" ~swallow)")
(ghost-parse "concept: ~swallow (gulp)")
(ghost-parse "r: (~eat) test concept")

(test-equal "GHOST concept" "test concept" (get-result "eat"))
(test-equal "GHOST concept" "test concept" (get-result "eats"))
(test-equal "GHOST concept" (string) (get-result "ingest"))
(test-equal "GHOST concept" "test concept" (get-result "ingests"))
(test-equal "GHOST concept" "test concept" (get-result "binge and purge"))
(test-equal "GHOST concept" "test concept" (get-result "gulp"))

; End of the test
(test-end ghost-utest)
