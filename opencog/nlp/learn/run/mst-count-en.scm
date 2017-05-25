;
; mst-count-en.scm
;
; Run the cogserver, needed for the language-learning disjunct
; counting pipeline. Starts the cogserver, opens the database,
; loads the database (whcih can take an hour or more!)
;
(use-modules (opencog) (opencog cogserver))
(use-modules (opencog persist) (opencog persist-sql))
(use-modules (opencog nlp) (opencog nlp learn))

; Start the cogserver.
; Edit the below, setting it to the desired langauge.
; This has almost no effect, other than to set the cogserver
; port-number and the prompt-style.
(start-cogserver "opencog-en.conf")

; Open the database.
; Edit the below, setting the database name, user and password.
(sql-open "postgres:///en_pairs_tone_mst?user=ubuntu&password=asdf")

; Load up the words
(display "Fetch all words from database. This may take several minutes.\n")
(fetch-all-words)

; Load up the word-pairs -- this can take over half an hour!
(display "Fetch all word-pairs. This may over half-an-hour!\n")

(define (fetch-any-pairs)
	(define any-pair-pred (LinkGrammarRelationshipNode "ANY"))
	(define start-time (current-time))
	(fetch-incoming-set any-pair-pred)
	(format #t "Elapsed time to load ANY-link pairs: ~A secs\n"
		(- (current-time) start-time))
)

(fetch-any-pairs)

; Print the sql stats
(sql-stats)

; Clear the sql cache and the stats counters
(sql-clear-cache)
(sql-clear-stats)
