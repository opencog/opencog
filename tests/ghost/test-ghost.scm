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

(test-equal "GHOST literal" "" (get-result "orange"))
(test-equal "GHOST literal" "test literal" (get-result "oranges"))
(test-equal "GHOST literal" "test literal - apostrophe" (get-result "let's"))
(test-equal "GHOST literal" "test literal - capital letter" (get-result "I'm good"))

; End of the test
(test-end ghost-utest)
