scm
;
; file-process.scm
; Work through a bunch of relex files that are in the compact-file-format,
; and import these into opencog. Then run processing on each one, as needed.
;
(define input-filedir "/home2/linas/src/novamente/data/enwiki/enwiki-20080524/parsed/A")
(define done-filedir "/home2/linas/src/novamente/data/enwiki/enwiki-20080524/coged/A")

(define cff-to-opencog-exe "/home/linas/src/novamente/src/relex-bzr/src/perl/cff-to-opencog.pl")

(use-modules (ice-9 rdelim))
(use-modules (ice-9 popen))

; ---------------------------------------------------------------------
; Read in data from RelEx "compact file format" file, convert it to
; opencog format (using the perl script "cff-to-opencog.pl"), and
; then load it into opencog.
;
(define (load-cff-data filename)
	(exec-scm-from-cmd 
		(string-join (list "cat \"" filename "\" | " cff-to-opencog-exe) "")
	)
)

; ---------------------------------------------------------------------
; Delete atoms that belong to particular sentence instances; we don't
; want to log up the server with grunge text.
;
(define (delete-sentences)
	(let ((n 0))
	; (define (delit atom) (set! n (+ n 1)) #f)
	; (define (delit atom) (cog-delete-recursive atom) #f)
	(define (delit atom) (cog-delete-recursive atom) (set! n (+ n 1)) #f)

	; (define (delone atom) (cog-delete atom) #f)
	(define (delone atom) (cog-delete atom) (set! n (+ n 1)) #f)

	; Part of Speech links are used in the word-sense 
	; database, so cannot delete these.
	; (cog-map-type delone 'PartOfSpeechLink)
	(cog-map-type delone 'LemmaLink)

	; Can't delete inheritanceLink, its used to mark wsd completed... 
	; (cog-map-type delone 'InheritanceLink)
	(cog-map-type delone 'ParseLink)
	(cog-map-type delone 'EvaluationLink)
	(cog-map-type delone 'ReferenceLink)
	(cog-map-type delone 'ListLink)
	(cog-map-type delone 'CosenseLink)
	(cog-map-type delone 'SimilarityLink)

	(cog-map-type delit 'DocumentNode)
	(cog-map-type delit 'SentenceNode)
	(cog-map-type delit 'ParseNode)
	(cog-map-type delit 'WordInstanceNode)
(system (string-join (list "echo deleted: " (number->string n) )))
	)
)

; ---------------------------------------------------------------------
; Process all the files in the input dir, and move them to donedir
; The input files are assumed to be RelEx "compact-file format" files.
; These are converted into opencog data, and are loaded into opencog.
; At this point wsd should run automatically; this script then triggers
; disjunct processing manually. When this completes, the next file is
; processed ... 
;
(define (process-data input-dir done-dir)

	(define (process-file filename)
		(let ((fullname (string-join (list input-dir filename) "/"))
		      (donename (string-join (list done-dir filename) "/"))
			)
			(system (string-join (list "echo start article: \"" filename "\"")))
			(system "date")
			(load-cff-data fullname)
			(system (string-join (list "echo done cff: \"" filename "\"")))
			(system "date")
			(cog-ad-hoc "do-wsd")
			(system (string-join (list "echo done wsd: \"" filename "\"")))
			(system "date")
			(ldj-process)
			(system (string-join (list "echo done ldj: \"" filename "\"")))
			(system "date")
			(system (string-join (list "mv \"" fullname "\" \"" donename "\"") ""))
			(delete-sentences)
			(system (string-join (list "echo done delete: \"" filename "\"")))
			(system "date")
		)
	)
	
	(for-each process-file
		(list-files input-dir)
	)
)

(define (doit) (process-data input-filedir done-filedir))

.
exit

