;
; export-disjuncts.scm
;
; Export disjuncts from the atomspace into a dattabase that can be
; used by the Link-Grammar parser.
;
; Copyright (c) 2015 Rohit Shinde
; Copyright (c) 2017 Linas Vepstas
;
; ---------------------------------------------------------------------
; OVERVIEW
; --------
; After a collection of disjuncts has been observed by the MST pipeline,
; the can be exported to the link Grammar parser, where they can be used
; to parse sentences.
;
; Currently an hack job.
; What's hacky here is that no word-classes (clusters) are used.
; Needs the guile-dbi interfaces, in order to write the SQL files.
;
; Example usage:
; (export-all-csets "dict.db" "EN_us")
;
; Then, in bash:
; cp -pr /usr/local/share/link-grammar/demo-sql ./my-place
; cp dict.db ./my-place
; link-parser ./my-place
; ---------------------------------------------------------------------

(use-modules (srfi srfi-1))
(use-modules (dbi dbi))  ; The guile-dbi interface to SQLite3
(use-modules (opencog))
(use-modules (opencog matrix))
(use-modules (opencog sheaf))

; ---------------------------------------------------------------------
; Return a caching version of AFUNC. Here, AFUNC is a function that
; takes a single atom as an argument, and returns some object
; associated with that atom.
;
; This returns a function that returns the same values that AFUNC would
; return, for the same argument; but if a cached value is available,
; then return just that.  In order for the cache to be valid, the AFUNC
; must be side-effect-free.
;
(define (make-afunc-cache AFUNC)

	; Define the local hash table we will use.
	(define cache (make-hash-table))

	; Guile needs help computing the hash of an atom.
	(define (atom-hash ATOM SZ) (modulo (cog-handle ATOM) SZ))
	(define (atom-assoc ATOM ALIST)
		(find (lambda (pr) (equal? ATOM (car pr))) ALIST))

	(lambda (ITEM)
		(define val (hashx-ref atom-hash atom-assoc cache ITEM))
		(if val val
			(let ((fv (AFUNC ITEM)))
				(hashx-set! atom-hash atom-assoc cache ITEM fv)
				fv)))
)

; ---------------------------------------------------------------------
; Convert an integer into a string of letters. Useful for creating
; link-names.  This prepends the letter "T" to all names, so that
; all MST link-names start with this letter.
; Example:  0 --> TA, 1 --> TB
(define (number->tag num)

	; Convert number to a list of letters.
	(define (number->letters num)
		(define letters "ABCDEFGHIJKLMNOPQRSTUVWXYZ")
		(unfold-right negative?
			(lambda (i) (string-ref letters (remainder i 26)))
			(lambda (i) (- (quotient i 26) 1))
			num))

	(list->string (cons #\T (number->letters num)))
)

;  ---------------------------------------------------------------------
;
; Given a word-pair atom, return a synthetic link name
; The link names are issued in serial order, first-come, first-served.
;
(define get-cnr-name
	(let ((cnt 0))

		; Notice that the lambda does not actually depend on the
		; word-pair. It just issues a new string.  The function
		; cache is what is able to detect and re-emit a previously
		; issued link name.
		(make-afunc-cache
			(lambda (WORD-PAIR)
				(set! cnt (+ cnt 1))
				(number->tag cnt))))
)

;  ---------------------------------------------------------------------

(define cnr-to-left (ConnectorDir "-"))

; Link Grammar expects connectors to be structured in the order of:
;   near- & far- & near+ & far+
; whereas the sections we compute from MST are in the form of
;   far- & near- & near+ & far+
; Thus, for the leftwards-connectors, we have to reverse the order.
(define (cset-to-lg-dj SECTION)
"
  cset-to-lg-dj - SECTION should be a SectionLink
  Return a link-grammar compatible disjunct string.
"
	; The germ of the section (the word)
	(define germ (gar SECTION))

	; Get a link-name identifying this word-pair.
	(define (connector-to-lg-link CONNECTOR)
		(define cnr (gar CONNECTOR))
		(define dir (gdr CONNECTOR))

		(if (equal? dir cnr-to-left)
			(get-cnr-name (ListLink cnr germ))
			(get-cnr-name (ListLink germ cnr))
		)
	)

	; Get a connector, by concatenating the link name with the direction.
	(define (connector-to-lg-cnr CONNECTOR)
		(string-append
			(connector-to-lg-link CONNECTOR)
			(cog-name (gdr CONNECTOR))))

	; Link Grammar expects: near- & far- & near+ & far+
	(define (strappend CONNECTOR dj)
		(define cnr (connector-to-lg-cnr CONNECTOR))
		(if (equal? (gdr CONNECTOR) cnr-to-left)
			(string-append cnr " & " dj)
			(string-append dj " & " cnr)))

	; Create a single string of the connectors, in order.
	; The connectors in SECTION are in the order as noted above:
	;   far- & near- & near+ & far+
	(fold
		(lambda (CNR dj) (if dj (strappend CNR dj)
				(connector-to-lg-cnr CNR)))
		#f
		(cog-outgoing-set (gdr SECTION)))
)

;  ---------------------------------------------------------------------

; Create a function that can store connector-sets to a database.
;
; DB-NAME is the databse name to write to.
; LOCALE is the locale to use; e.g EN_us or ZH_cn
; COST-FN is a function that assigns a link-parser cost to each disjunct.
;
; This returns a function that will write sections to the database.
; That is, this returns (lambda (SECTION) ...) so that, when you call
; it, that section will be saved to the database. Calling with #f closes
; the database.
;
; Example usage:
; (make-database "dict.db" "EN_us" ...)
;
(define (make-database DB-NAME LOCALE COST-FN)
	(let ((db-obj (dbi-open "sqlite3" DB-NAME))
			(cnt 0)
			(nprt 0)
			(secs (current-time))
		)

		; Escape quotes -- replace single quotes by two successive
		; single-quotes. Example: (escquote "fo'sis'a'blort" 0)
		(define (escquote STR BEG)
			(define pos (string-index STR (lambda (C) (equal? C #\')) BEG))
			(if pos
				(escquote
					(string-replace STR "''" pos pos 1 2)
					(+ pos 2))
				STR))

		; Add data to the database
		(define (add-section SECTION)
			; The germ of the section (the word)
			(define germ-str (cog-name (gar SECTION)))
			(define dj-str (cset-to-lg-dj SECTION))

			; Oh no!!! Need to fix LEFT-WLL!
			(if (string=? germ-str "###LEFT-WALL###")
				(set! germ-str "LEFT-WALL"))

			(set! nprt (+ nprt 1))
			(if (equal? 0 (remainder nprt 5000))
				(begin
					(format #t "~D Will insert ~A: ~A; in ~D secs\n"
						nprt germ-str dj-str (- (current-time) secs))
					(set! secs (current-time))
					(dbi-query db-obj "END TRANSACTION;")
					(dbi-query db-obj "BEGIN TRANSACTION;")
				))

			(set! germ-str (escquote germ-str 0))

			; Insert the word
			(set! cnt (+ cnt 1))
			(dbi-query db-obj (format #f
				"INSERT INTO Morphemes VALUES ('~A', '~A.~D', '(~A.~D)');"
				germ-str germ-str cnt germ-str cnt))

			(if (not (equal? 0 (car (dbi-get_status db-obj))))
				(throw 'fail-insert 'make-database
					(cdr (dbi-get_status db-obj))))

			; Insert the disjunct, assigning a cost according
			; to the float-ppoint value returned by teh function
			(dbi-query db-obj (format #f
				"INSERT INTO Disjuncts VALUES ('(~A.~D)', '~A', ~F);"
				germ-str cnt dj-str (COST-FN SECTION)))

			(if (not (equal? 0 (car (dbi-get_status db-obj))))
				(throw 'fail-insert 'make-database
					(cdr (dbi-get_status db-obj))))
		)

		; Write to disk, and close the database.
		(define (shutdown)
			(format #t "Finished inserting ~D records\n" nprt)
			(dbi-query db-obj "END TRANSACTION;")
			(dbi-close db-obj)
		)

		; Create the tables for words and disjuncts.
		; Refer to the Link Grammar documentation to see a
		; description of this table format. Specifically,
		; take a look at `dict.sql`.
		(dbi-query db-obj (string-append
			"CREATE TABLE Morphemes ( "
			"morpheme TEXT NOT NULL, "
			"subscript TEXT UNIQUE NOT NULL, "
			"classname TEXT NOT NULL);" ))

		(if (not (equal? 0 (car (dbi-get_status db-obj))))
			(throw 'fail-create 'make-database
				(cdr (dbi-get_status db-obj))))

		(dbi-query db-obj
			"CREATE INDEX morph_idx ON Morphemes(morpheme);")

		(dbi-query db-obj (string-append
			"CREATE TABLE Disjuncts ("
			"classname TEXT NOT NULL, "
			"disjunct TEXT NOT NULL, "
			"cost REAL );"))

		(dbi-query db-obj
			"CREATE INDEX class_idx ON Disjuncts(classname);")

		(dbi-query db-obj (string-append
			"INSERT INTO Morphemes VALUES ("
			"'<dictionary-version-number>', "
			"'<dictionary-version-number>', "
			"'<dictionary-version-number>');"))

		(dbi-query db-obj (string-append
			"INSERT INTO Disjuncts VALUES ("
			"'<dictionary-version-number>', 'V5v4v0+', 0.0);"))

		(dbi-query db-obj (string-append
			"INSERT INTO Morphemes VALUES ("
			"'<dictionary-locale>', "
			"'<dictionary-locale>', "
			"'<dictionary-locale>');"))

		(dbi-query db-obj (string-append
			"INSERT INTO Disjuncts VALUES ("
			"'<dictionary-locale>', '"
			(string-map (lambda (c) (if (equal? c #\_) #\4 c)) LOCALE)
			"+', 0.0);"))

		; The UNKNOWN-WORD device is needed to make wild-card searches
		; work (when dict debugging). The XXXBOGUS+ will not link to
		; anything. `({@T-} & {@T+})` would almost work, except for two
		; reasons: all connectors are upper-case, and the SQL backend
		; does not support optional-braces {} and multi-connectors @.
		(dbi-query db-obj (string-append
			"INSERT INTO Morphemes VALUES ("
			"'UNKNOWN-WORD', "
			"'UNKNOWN-WORD', "
			"'UNKNOWN-WORD');"))

		(dbi-query db-obj (string-append
			"INSERT INTO Disjuncts VALUES ("
			"'UNKNOWN-WORD', 'XXXBOGUS+', 0.0);"))

		(dbi-query db-obj "PRAGMA synchronous = OFF;")
		(dbi-query db-obj "PRAGMA journal_mode = MEMORY;")
		(dbi-query db-obj "BEGIN TRANSACTION;")

		; Return function that adds data to the database
		; If SECTION if #f, the database is closed.
		(lambda (SECTION)
			(if SECTION
				(add-section SECTION)
				(shutdown))
		))
)

;  ---------------------------------------------------------------------

(define-public (export-csets CSETS DB-NAME LOCALE)
"
  export-csets CSETS DB-NAME LOCALE

  Write connector sets to a Link Grammar-compatible sqlite3 file.
  CSETS is a matrix containing the connector sets to be written.
  DB-NAME is the databse name to write to.
  LOCALE is the locale to use; e.g EN_us or ZH_cn

  Note that link-grammar expects the database file to be called
  \"dict.db\", always!

  Example usage:
     (define pca (make-pseudo-cset-api))
     (define fca (add-subtotal-filter pca 50 50 10 #f))
     (export-csets fca \"dict.db\" \"EN_us\")
"
	; Create the object that knows where the disuncts are in the
	; atomspace. Create the object that knows how to get the MI
	; of a word-disjunct pair.
	(define psa (add-pair-stars CSETS))
	(define mi-source (add-pair-freq-api psa))
	(define looper (add-loop-api psa))

	; Use the MI between word and disjunct as the link-grammar cost
	; LG treats high-cost as "bad", we treat high-MI as "good" so revese
	; the sign.
	(define (cost-fn SECTION)
		(- (mi-source 'pair-fmi SECTION)))

	; Create the SQLite3 database.
	(define sectioner (make-database DB-NAME LOCALE cost-fn))

	(define cnt 0)
	(define (cntr x) (set! cnt (+ cnt 1)))
	(looper 'for-each-pair cntr)
	(format #t "Store ~D csets\n" cnt)

	; Dump all the connector sets into the database
	(looper 'for-each-pair sectioner)

	; Close the database
	(sectioner #f)
)

;  ---------------------------------------------------------------------
