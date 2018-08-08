;unit test for fuzzy-matcher based QA system

(define (TestFuzzy-QA)
	(define prt)
	(set! prt (check "What does Tom eat?"))
	(set! prt (string-append prt "\n"))
	(set! prt (string-append prt (check "what do you like for lunch?")))
	(set! prt (string-append prt "\n"))
	(set! prt (string-append prt (check "what is your name?")))
	(set! prt (string-append prt "\n"))
	(set! prt (string-append prt (check "what are you planing to do?")))
	(set! prt (string-append prt "\n"))
	(set! prt (string-append prt (check "when are you coming?")))
	(set! prt (string-append prt "\n"))
	(set! prt (string-append prt (check "who is your best friend?")))
	(set! prt (string-append prt "\n"))
	(set! prt (string-append prt (check "where are you from?")))
	(set! prt (string-append prt "\n"))
	(set! prt (string-append prt (check "which color do you like most?")))
	(set! prt (string-append prt "\n"))
	(clear) prt)        


(define (check sent)
	(cond
	((equal? (check_query_type (car (nlp-parse sent))) "InterrogativeSpeechAct")
	(string-append "correctly identify type of : "sent))
	((not(equal? (check_query_type (car (nlp-parse sent))) "InterrogativeSpeechAct"))
	(string-append "Failed to identify type of: "sent))
))


