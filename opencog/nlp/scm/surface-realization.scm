;
; Links relex-to-logic output with relex-opencog-output
; It is temporary until the r2l rules are moved into the URE

; Test sentence : "This is a sentence."

; modules needed for call-with-input-file and get-string-all
;  (use-modules (rnrs io ports))
;  (use-modules (ice-9 rdelim))

(use-modules (ice-9 rdelim))
(use-modules (ice-9 regex))
; ---------------------------------------------------------------------
; Patterns used for spliting the string passed from relex server
(define pattern1 "; ##### END OF A PARSE #####")
(define pattern2 "; ##### START OF R2L #####")

; ---------------------------------------------------------------------
; Splits a string into substring delimited by the a pattern and returns a list
; of the substrings.
; TODO: make tail recursive, for efficency
(define (split-string a-pattern a-string)
	(let ((a-match (string-match a-pattern a-string)))
		(if (not a-match)
			(list a-string)
			(append (list (match:prefix a-match))
				(split-string a-pattern (match:suffix a-match)))
		)
	)
)

; ---------------------------------------------------------------------
; returns the name of a ParseNode if the string has a ParseNode entry.
; returns "sentence@dbce9f0d-8b8a-4ea7-a8c8-d69abf05f810_parse_1" from the string
; "84590fb2e46c (ParseNode \"sentence@dbce9f0d-8b8a-4ea7-a8c8-d69abf05f810_parse_1\""
(define (parse-str a-string)
	(substring (match:substring
		(string-match "ParseNode \"sentence@[[:alnum:]_-]*" a-string)) 11)
)

; ---------------------------------------------------------------------
; Evaluate the string and returns a list with SetLink of the initial r2l rule application.
; (ReferenceLink 
; 		(InterpretationNode "sentence@1d220-c7ace2f_parse_2_interpretation_$X")
; 		(SetLink
; 			different links that are a result of r2l rule-functions being applied
;		)
;
; A ReferenceLink is used instead of InterpretationLink so as to differentiate
; the final word-sense-disambiguated , anaphore and cataphor resolved version from
; the initial r2l output. The final version will have structure that is detailed
; @ http://wiki.opencog.org/w/Linguistic_Interpretation.
; Having the output from this function should help during garbage-removal from the
; atomspace as well as the generation of the final set of interpretations for the
; the sentence.
(define (set-link a-string)
	(define z-list (map (lambda (x) (split-string pattern2 x)) 
									(split-string pattern1 a-string)))

	(define (eval-list a-list)
		(if (= 1 (length a-list))
			(begin (eval-string (list-ref a-list 0)) '())
	;		(map-in-order eval-string a-list)
			(begin (eval-string (list-ref a-list 0))
			(let ((parse-name (parse-str (list-ref a-list 0))))
				(ReferenceLink 
				(InterpretationNode (string-append parse-name "_interpretation_$X"))
				(SetLink
					(map-in-order eval-string
								(filter (lambda (x)(not (string=? "" x)))
								(split-string "\n" (list-ref a-list 1)))))
				)
			))
		)
	)

	(par-map eval-list z-list)
)


; ---------------------------------------------------------------------
; A helper function 
(define (set-interpret port)
	(let ((string-read (get-string-all port)))
		(if (eof-object? string-read)
			#f
			;(filter (lambda (x) (not (string=? x "")))  (string-split string-read #\=))
			(set-link string-read)
		)
	)
)

; ---------------------------------------------------------------------
; A copy of relex-parse funtion modified for relex-to-logic(r2l) purposes.
(define (r2l plain-txt)
	(define (do-sock-io sent-txt)
		(let ((s (socket PF_INET SOCK_STREAM 0)) (str ""))
			(connect s AF_INET (inet-pton AF_INET relex-server-host) relex-server-port)

			(display sent-txt s)
			(display "\n" s)
			(system (string-join (list "echo \"Info: send to parser: " sent-txt "\"")))
			(set-interpret s)
			(system (string-join (list "echo Info: close socket to parser" )))
			(close-port s)
		)
	)

	(if (string=? plain-txt "")
		(display "Please enter a valid sentence.")
		(do-sock-io plain-txt)
	)
)




