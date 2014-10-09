; Notes: random-string, random-node-name and choose-var-name can be moved to utilities.scm

; -----------------------------------------------------------------------
; Defines a default simple truth value
; (Note: Without assigning this value, the default stv is (stv 1.0 0.0);
; PLN doesn't work on these as it requires links to have non-zero confidence
; values. Furthermore, some rules (e.g. AbductionRule) require a strength value
; smaller than 1.0. as -- conceptually -- this makes the Node equivalent to
; the entire universe.)
; Defines a default simple truth value for links
(define df-link-stv (stv .99 .99))

; Defines a default simple truth value for nodes
(define df-node-stv (stv .001 .99))

;------------------------------------------------------------------------
; Returns a random string of length 'str-length'.
(define (random-string str-length) 
	(define alphanumeric "abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789")
	(define str "")
	(while (> str-length 0)
		(set! str (string-append str (string (string-ref alphanumeric (random (string-length alphanumeric))))))
		(set! str-length (- str-length 1))
	)
	str
)

; -----------------------------------------------------------------------
; Return #t if there is a node of type node-type with a name "node-name".
(define (check-name? node-name node-type)
        (not (null? (cog-node node-type node-name)))
)

; -----------------------------------------------------------------------
; Creates a possible name 'node-name' of length 'random-length' for a node
; of type 'node-type'. The 'node-name' is not used with any other node
; of type 'node-type'. Prepend 'prepend-text' to the front.
(define (random-node-name node-type random-length prepend-text)
	(define node-name (random-string random-length))
	(define prepend-length (string-length prepend-text))
	(if (> prepend-length 0)
		(set! node-name (string-append prepend-text node-name))
	)
	(while (check-name? node-name node-type)
		(if (> prepend-length 0)
			(set! node-name (string-append prepend-text (random-string random-length)))
			(set! node-name (random-string random-length))
		)
	)
	node-name
)

; -----------------------------------------------------------------------
; Creates name for VariableNodes after checking whether the name is being
; used by other VariableNode.
(define (choose-var-name) (random-node-name 'VariableNode 36 "$"))

; -----------------------------------------------------------------------
; Check if the lemma of a WordInstanceNode 'word-inst' is 'word'.
(define (check-lemma? word word-inst)
	(define lemma (word-inst-get-lemma word-inst))
	(if (null? lemma)
		#f
		(string=? word (cog-name lemma))
	)
)

; -----------------------------------------------------------------------
; Returns a list of WordInstanceNodes from 'parse-node', which have a LemmaLink with a
; WordNode named 'word'.
(define (get-word-inst-nodes word parse-node)
	(define word-inst-list (parse-get-words-in-order parse-node))
	(append-map (lambda (a-predicate a-word-inst) (if a-predicate (list a-word-inst) '()))
		(map (lambda (a-word-inst) (check-lemma? word a-word-inst)) word-inst-list)
		word-inst-list
	)
)

; -----------------------------------------------------------------------
; Gets the occurence count of the a word node in a parse
(define (get-word-inst-index word-inst)
	(define parse-node (car (cog-chase-link 'WordInstanceLink 'ParseNode word-inst)))
	(define word (cog-name (word-inst-get-lemma word-inst)))
	(+ 1 (list-index (lambda (a-node) (equal? word-inst a-node)) (get-word-inst-nodes word parse-node)))
)

; -----------------------------------------------------------------------
; Returns the word-instance name when its word-lemma, word-index and parse-node is inputed.
; It also checks whether an atom name is a word-instance name for the given parse-node and
; word lemma and returns the word-instance name.
(define (get-instance-name word word-index parse-node)
	(cond	((number? word-index)
			(cog-name (list-ref (get-word-inst-nodes word parse-node) (- word-index 1)))
		)
		; in case word-index is a string, word-index is already the name we want to use
		((and (string? word-index) (check-name? word-index 'WordInstanceNode))
			word-index
		)
	)
)


; =======================================================================
; Actual helper functions for creating OpenCog atoms
; =======================================================================
; The atoms added to the atomspace , by the helper functions below must be
; returned in a list or should be a single atom so as to simplify
; post-processing and NLG tasks.

; -----------------------------------------------------------------------
; SV and related rules
; -----------------------------------------------------------------------
; Examples: "Socrates is a man", "Cats are animals", "Trees are plants"
(define (be-inheritance-rule subj_concept subj_instance obj_concept obj_instance)
	(list (InheritanceLink (ConceptNode subj_instance df-node-stv) (ConceptNode subj_concept df-node-stv) df-link-stv)
	(InheritanceLink (ConceptNode obj_instance df-node-stv) (ConceptNode obj_concept df-node-stv) df-link-stv)
	(InheritanceLink (ConceptNode subj_instance df-node-stv) (ConceptNode obj_instance df-node-stv) df-link-stv)
	)
)

(define (SVIO-rule subj_concept  subj_instance  verb  verb_instance  obj_concept  obj_instance iobj_concept iobj_instance)
	(list (ImplicationLink (PredicateNode verb_instance df-node-stv) (PredicateNode verb df-node-stv) df-link-stv)
	(InheritanceLink (ConceptNode subj_instance df-node-stv) (ConceptNode subj_concept df-node-stv) df-link-stv)
	(InheritanceLink (ConceptNode obj_instance df-node-stv) (ConceptNode obj_concept df-node-stv) df-link-stv)
	(InheritanceLink (ConceptNode iobj_instance df-node-stv) (ConceptNode iobj_concept df-node-stv) df-link-stv)
	(EvaluationLink df-link-stv
        (PredicateNode verb_instance df-node-stv)
        (ListLink df-link-stv
            (ConceptNode subj_instance df-node-stv)
            (ConceptNode obj_instance df-node-stv)
            (ConceptNode iobj_instance df-node-stv)
        )
	))
)

(define (SVO-rule subj_concept  subj_instance  verb  verb_instance  obj_concept  obj_instance)
	(list (ImplicationLink (PredicateNode verb_instance df-node-stv) (PredicateNode verb df-node-stv) df-link-stv)
	(InheritanceLink (ConceptNode subj_instance df-node-stv) (ConceptNode subj_concept df-node-stv) df-link-stv)
	(InheritanceLink (ConceptNode obj_instance df-node-stv) (ConceptNode obj_concept df-node-stv) df-link-stv)
	(EvaluationLink df-link-stv
		(PredicateNode verb_instance df-node-stv)
		(ListLink df-link-stv
			(ConceptNode subj_instance df-node-stv)
			(ConceptNode obj_instance df-node-stv)
		)
	))
)

(define (SV-rule subj_concept  subj_instance  verb  verb_instance)
	(list (ImplicationLink (PredicateNode verb_instance df-node-stv) (PredicateNode verb df-node-stv) df-link-stv)
	(InheritanceLink (ConceptNode subj_instance df-node-stv) (ConceptNode subj_concept df-node-stv) df-link-stv)
	(EvaluationLink df-link-stv
		(PredicateNode verb_instance df-node-stv)
		(ListLink df-link-stv
			(ConceptNode subj_instance df-node-stv)
		)
	))
)

(define (SVP-rule subj  subj_instance  predicative  predicative_instance)
	(list (InheritanceLink (ConceptNode predicative_instance df-node-stv) (ConceptNode predicative df-node-stv) df-link-stv)
	(InheritanceLink (ConceptNode subj_instance df-node-stv) (ConceptNode subj df-node-stv) df-link-stv)
	(InheritanceLink (ConceptNode subj_instance df-node-stv) (ConceptNode predicative_instance df-node-stv) df-link-stv)
	)
)

(define (to-be-rule verb verb_ins adj adj_ins subj subj_ins)
	(list (ImplicationLink (PredicateNode verb_ins df-node-stv) (PredicateNode verb df-node-stv) df-link-stv)
	(InheritanceLink (ConceptNode subj_ins df-node-stv) (ConceptNode subj df-node-stv) df-link-stv)
	(InheritanceLink (ConceptNode adj_ins df-node-stv) (ConceptNode adj df-node-stv) df-link-stv)
	(EvaluationLink df-link-stv
		(PredicateNode verb_ins df-node-stv)
		(ListLink df-link-stv
			(InheritanceLink (ConceptNode subj_ins df-link-stv) (ConceptNode adj_ins df-node-stv) df-link-stv)
		)
	))
)


; -----------------------------------------------------------------------
; Adjective and adverb rules
; -----------------------------------------------------------------------
(define (amod-rule concept instance adj adj_instance)
	(list (InheritanceLink  (ConceptNode adj_instance df-node-stv) (ConceptNode adj df-node-stv) df-link-stv)
	(InheritanceLink  (ConceptNode instance df-node-stv) (ConceptNode adj_instance df-node-stv) df-link-stv)
	(InheritanceLink  (ConceptNode instance df-node-stv) (ConceptNode concept df-node-stv) df-link-stv)
	)
)

(define (advmod-rule verb instance adv adv_instance)
	(list (InheritanceLink  (ConceptNode adv_instance df-node-stv) (ConceptNode adv df-node-stv) df-link-stv)
	(InheritanceLink  (SatisfyingSetLink (PredicateNode instance df-node-stv) df-link-stv) (ConceptNode adv_instance df-node-stv) df-link-stv)
	(ImplicationLink  (PredicateNode instance df-node-stv) (PredicateNode verb df-node-stv) df-link-stv)
	)
)


; -----------------------------------------------------------------------
; unary rules
; -----------------------------------------------------------------------
(define (entity-rule word word_instance) 
	(list (InheritanceLink (SpecificEntityNode word_instance df-node-stv) (ConceptNode word df-node-stv) df-link-stv))
)

(define (gender-rule word word_instance gender_type)
	(define concept_node (ConceptNode word df-node-stv)) ; This rule is not used.
	(cond ((string=? gender_type "feminine")
		(list (InheritanceLink (SpecificEntityNode word_instance df-node-stv) (ConceptNode "woman" df-node-stv) df-link-stv)
		(InheritanceLink (SpecificEntityNode word_instance df-node-stv) (ConceptNode word df-node-stv) df-link-stv)
	    (InheritanceLink (SpecificEntityNode word_instance df-node-stv) (ConceptNode "person" df-node-stv) df-link-stv)
		))
		((string=? gender_type "masculine")
		(list (InheritanceLink (SpecificEntityNode word_instance df-node-stv) (ConceptNode "man" df-node-stv) df-link-stv)
		(InheritanceLink (SpecificEntityNode word_instance df-node-stv) (ConceptNode word df-node-stv) df-link-stv)
	    (InheritanceLink (SpecificEntityNode word_instance df-node-stv) (ConceptNode "person" df-node-stv) df-link-stv)
		))
	)
)

(define (tense-rule verb instance tense)
	(list (ImplicationLink (PredicateNode instance df-node-stv) (PredicateNode verb df-node-stv) df-link-stv)
	(InheritanceLink (PredicateNode instance df-node-stv) (ConceptNode tense df-node-stv) df-link-stv)
	)
)

(define (det-rule concept instance var_name determiner)
	(cond ((or (string=? determiner "those") (string=? determiner "these"))
		(list (ImplicationLink df-link-stv
			(MemberLink (VariableNode var_name df-node-stv) (ConceptNode instance df-node-stv) df-link-stv)
			(InheritanceLink (VariableNode var_name df-node-stv) (ConceptNode concept df-node-stv) df-link-stv)))
		)
		((or (string=? determiner "this") (string=? determiner "that"))
			(list (InheritanceLink (VariableNode var_name df-node-stv) (ConceptNode concept df-node-stv) df-link-stv))
		)
	)
)

(define (negative-rule verb instance)
	(list (ImplicationLink (PredicateNode instance df-node-stv) (NotLink (PredicateNode verb df-node-stv) df-link-stv) df-link-stv))
)

(define (definite-rule word word_instance)
    (list (InheritanceLink (ConceptNode word_instance df-node-stv) (ConceptNode word df-node-stv) df-link-stv)
    (EvaluationLink df-link-stv
        (PredicateNode "definite" df-node-stv)
        (ListLink df-link-stv
	        (ConceptNode word_instance df-node-stv)
        )
    ))
)

; Example: "Maybe she eats lunch.", "Perhaps she is nice."
(define (maybe-rule word word_instance)
	(list
		(ImplicationLink (PredicateNode word_instance df-node-stv) (PredicateNode word df-node-stv) df-link-stv)
		(EvaluationLink df-link-stv
			(PredicateNode "maybemarker" df-node-stv)
			(ListLink df-link-stv
				(PredicateNode word_instance df-node-stv)
			)
		)
	)
)


; -----------------------------------------------------------------------
; misc rules
; -----------------------------------------------------------------------
(define (number-rule noun noun_instance num num_instance)
	(define noun_ins_concept (ConceptNode noun_instance df-node-stv))
	(list (InheritanceLink noun_ins_concept (ConceptNode noun df-node-stv) df-link-stv)
	(InheritanceLink (NumberNode num_instance df-node-stv) (NumberNode num df-node-stv) df-link-stv)
	(QuantityLink (ConceptNode noun_instance df-node-stv) (NumberNode num_instance df-node-stv) df-link-stv)
	)
)

(define (about-rule verb verb_instance  noun noun_instance)
	(list (ImplicationLink (PredicateNode verb_instance df-node-stv) (PredicateNode verb df-node-stv) df-link-stv)
	(InheritanceLink (ConceptNode noun_instance df-node-stv) (ConceptNode noun df-node-stv) df-link-stv)
	(EvaluationLink df-link-stv
		(PredicateNode "about" df-node-stv)
		(ListLink df-link-stv
			(PredicateNode verb_instance df-node-stv)
			(ConceptNode noun_instance df-node-stv)
		)
	))
) 

(define (nn-rule n1 n1_instance n2 n2_instance)
	(list (InheritanceLink (ConceptNode n1_instance df-node-stv) (ConceptNode n1 df-node-stv) df-link-stv)
	(InheritanceLink (ConceptNode n2_instance df-node-stv) (ConceptNode n2 df-node-stv) df-link-stv)
	(InheritanceLink (ConceptNode n1_instance df-node-stv) (ConceptNode n2_instance df-node-stv) df-link-stv)
	)
)

(define (possessive-rule noun noun_instance word word_instance)
	(list (InheritanceLink (ConceptNode noun_instance df-node-stv) (ConceptNode noun df-node-stv) df-link-stv)
	(InheritanceLink (ConceptNode word_instance df-node-stv) (ConceptNode word df-node-stv) df-link-stv)
	(EvaluationLink df-link-stv
		(PredicateNode "possession" df-node-stv)
		(ListLink df-link-stv
			(ConceptNode noun_instance df-node-stv)
			(ConceptNode word_instance df-node-stv)
		)
	))
)


; -----------------------------------------------------------------------
; to-do rules
; -----------------------------------------------------------------------
; Example: "She wants to help John."
(define (to-do-rule-1 v1 v1_instance v2 v2_instance s s_instance o o_instance)
	(list (InheritanceLink (ConceptNode s_instance df-node-stv) (ConceptNode s df-node-stv) df-link-stv)
	(InheritanceLink (ConceptNode o_instance df-node-stv) (ConceptNode o df-node-stv) df-link-stv)
	(ImplicationLink (PredicateNode v1_instance df-node-stv) (PredicateNode v1 df-node-stv) df-link-stv)
	(ImplicationLink (PredicateNode v2_instance df-node-stv) (PredicateNode v2 df-node-stv) df-link-stv)
	(EvaluationLink df-link-stv
		(PredicateNode v1_instance df-node-stv)
		(ListLink df-link-stv
			(ConceptNode s_instance df-node-stv)
			(EvaluationLink df-link-stv
				(PredicateNode v2_instance df-node-stv)
				(ListLink df-link-stv
					(ConceptNode s_instance df-node-stv)
					(ConceptNode o_instance df-node-stv)
				)
			)
		)
	))
)

(define (to-do-rule-2 v1 v1_instance v2 v2_instance s1 s1_instance s2 s2_instance o o_instance) 
	(list (InheritanceLink (ConceptNode s1_instance df-node-stv) (ConceptNode s1 df-node-stv) df-link-stv)
	(InheritanceLink (ConceptNode s2_instance df-node-stv) (ConceptNode s2 df-node-stv) df-link-stv)
	(InheritanceLink (ConceptNode o_instance df-node-stv) (ConceptNode o df-node-stv) df-link-stv)
	(ImplicationLink (PredicateNode v1_instance df-node-stv) (PredicateNode v1 df-node-stv) df-link-stv)
	(ImplicationLink (PredicateNode v2_instance df-node-stv) (PredicateNode v2 df-node-stv) df-link-stv)
	(EvaluationLink df-link-stv
		(PredicateNode v1_instance df-node-stv)
		(ListLink df-link-stv
			(ConceptNode s1_instance df-node-stv)
			(EvaluationLink df-link-stv
				(PredicateNode v2_instance df-node-stv)
				(ListLink df-link-stv
					(ConceptNode s2_instance df-node-stv)
					(ConceptNode o_instance df-node-stv)
				)
			)
		)
	))
)

; Example: "She is nice to help with the project."
(define (to-do-rule-3 v1 v1_instance v2 v2_instance v3 v3_instance)
    (list (InheritanceLink (ConceptNode v1_instance df-node-stv) (ConceptNode v1 df-node-stv) df-link-stv)
    (ImplicationLink (PredicateNode v2_instance df-node-stv) (PredicateNode v2 df-node-stv) df-link-stv)
    (InheritanceLink (ConceptNode v3_instance df-node-stv) (ConceptNode v3 df-node-stv) df-link-stv)
    (EvaluationLink df-link-stv
        (PredicateNode v2_instance df-node-stv)
        (ListLink df-link-stv ; does this ListLink make sense here? (by sebastianruder)
                (InheritanceLink (ConceptNode v3_instance df-node-stv) (ConceptNode v1_instance df-node-stv) df-link-stv)
        )
    ))
)

; Example: "She must be able to sing." ; v1 = sing , v2 = she
(define (to-do-rule-4 v1 v1_instance v2 v2_instance) 
    (list (InheritanceLink (ConceptNode v2_instance) (ConceptNode v2))
    (ImplicationLink (PredicateNode v1_instance) (PredicateNode v1))
    (EvaluationLink
        (PredicateNode "able_to")
        (ListLink
            (ConceptNode v2_instance)
            (PredicateNode v1_instance)
        )
    ))
)

; Example: "She wants to sing."; verb1 = want, verb2 = sing, subj = she
(define (to-do-rule-5 verb1 verb1_instance verb2 verb2_instance subj subj_instance)
    (list (InheritanceLink (ConceptNode subj_instance) (ConceptNode subj))
    (ImplicationLink (PredicateNode verb1_instance) (PredicateNode verb1))
    (ImplicationLink (PredicateNode verb2_instance) (PredicateNode verb2))
    (EvaluationLink
        (PredicateNode verb1_instance)
        (ListLink
            (ConceptNode subj_instance)
            (PredicateNode verb2_instance)
        )
    ))
)

;--------------------------------------------------------------------------------
; Relex2Logic question rule-helpers
;
;--------------------------------------------------------------------------------------------------------------------
; Rules for yes / no questions with main-verb to-be (except phrasal 'be+prep' or 'be' with no object)
; and either nominal, prepositional, or predicate adjective complements.
;--------------------------------------------------------------------------------------------------------------------
;
; Example: "Are you the one?"
;
(define (obj-ynQ-rule subj_concept subj_instance obj_concept obj_instance)
	(list (InheritanceLink (ConceptNode subj_instance df-node-stv) (ConceptNode subj_concept df-node-stv) df-link-stv)
	(InheritanceLink (ConceptNode obj_instance df-node-stv) (ConceptNode obj_concept df-node-stv) df-link-stv)
	(EvaluationLink (PredicateNode "Truth Value")
		(ListLink (InheritanceLink (ConceptNode subj_instance df-node-stv)(ConceptNode obj_instance df-node-stv) df-link-stv))
		(VariableNode "$var1"))
))
;
; Example: "Are you mad?" 
;
(define (predadj-ynQ-rule subj_concept subj_instance predicative_concept predicative_instance)
	(list (InheritanceLink (ConceptNode subj_instance df-node-stv) (ConceptNode subj_concept df-node-stv) df-link-stv)
	(InheritanceLink (PredicateNode predicative_instance df-node-stv) (ConceptNode predicative_concept df-node-stv) df-link-stv)
	(EvaluationLink (PredicateNode "Truth Value")
		(EvaluationLink df-link-stv (PredicateNode predicative_instance df-node-stv) 
			(ListLink df-link-stv (ConceptNode subj_instance df-node-stv)))
			(VariableNode "$var1"))
))
;
; Example: "Is the book under the table?"
;
; NB: sentences with prepositional phrases as the predicate-complements of the subject are treated by Relex in a variety
; of ways, including the prepobj relation and also treating the prep as a simple predicate; those sentences that do not
; call the rule below, call the SV-ynq rule.
;
(define (prepobj-ynQ-rule subj_concept subj_instance predprep_concept predprep_instance obj_concept obj_instance)
	(list (InheritanceLink (ConceptNode subj_instance df-node-stv) (ConceptNode subj_concept df-node-stv) df-link-stv)
	(InheritanceLink (ConceptNode obj_instance df-node-stv) (ConceptNode obj_concept df-node-stv) df-link-stv)
	(InheritanceLink (PredicateNode predprep_instance df-node-stv) (ConceptNode predprep_concept df-node-stv) df-link-stv)
	(EvaluationLink (PredicateNode "Truth Value")
		(EvaluationLink df-link-stv (PredicateNode predprep_instance df-node-stv)
			(ListLink df-link-stv (ConceptNode subj_instance df-node-stv)(ConceptNode obj_instance df-node-stv)))
			(VariableNode "$var1"))
))
;
;-------------------------------------------------------------------------------------------------------
; Rules for yes/no questions with any main-verb plus auxilliary verbs
; and SV,SVO, or SVIO
;
; NB: It seems a bit obnoxious to have SV, SVO, and SVIO question rules for 
; not only these, but also each Q-word as follows below, but since the declarative
; sentence r2l rules are not take-apartable it seems necessary; the only way 
; to avoid it, I think, would be to have separate rules for assigning subjects,
; objects, and iobj's and then call each of them from the rules for SV, SVO, SVIO,
; or to do questions as post-processing and call the declarative rules.
;-------------------------------------------------------------------------------------------------------
;
; Examples: "Have you slept?", "Will you sleep?", "Did you sleep?"
; 
(define (SV-ynQ-rule subj_concept subj_instance verb verb_instance)
	(list (ImplicationLink (PredicateNode verb_instance df-node-stv) (PredicateNode verb df-node-stv) df-link-stv)
	(InheritanceLink (ConceptNode subj_instance df-node-stv) (ConceptNode subj_concept df-node-stv) df-link-stv)
	(EvaluationLink df-link-stv (PredicateNode "Truth-Value" df-node-stv)
		(EvaluationLink df-link-stv (PredicateNode verb_instance df-node-stv)
			(ListLink df-link-stv (ConceptNode subj_instance df-node-stv)))
		(VariableNode "$qVar" df-node-stv))
))
;
; Example: "Did you eat the leftover baba-ganoush?"
;
; NB: 	This rule applies incorrectly to many SVIO sentences where the IO is interpreted by relex as 
;	an extra prepositional phrase rather than an IO.
;
(define (SVO-ynQ-rule subj_concept subj_instance verb verb_instance obj_concept obj_instance)
	(list (ImplicationLink (PredicateNode verb_instance df-node-stv) (PredicateNode verb df-node-stv) df-link-stv)
	(InheritanceLink (ConceptNode subj_instance df-node-stv) (ConceptNode subj_concept df-node-stv) df-link-stv)
	(InheritanceLink (ConceptNode obj_instance df-node-stv) (ConceptNode obj_concept df-node-stv) df-link-stv)
	(EvaluationLink df-link-stv (PredicateNode "Truth-Value" df-node-stv)
		(EvaluationLink df-link-stv (PredicateNode verb_instance df-node-stv)
			(ListLink df-link-stv 
				(ConceptNode subj_instance df-node-stv)
				(ConceptNode obj_instance df-node-stv)))
		(VariableNode "$qVar" df-node-stv))
))
;
; Example: "Did you give her the money?"
;
(define (SVIO-ynQ-rule subj_concept subj_instance verb verb_instance obj_concept obj_instance iobj_concept iobj_instance)
	(list (ImplicationLink (PredicateNode verb_instance df-node-stv) (PredicateNode verb df-node-stv) df-link-stv)
	(InheritanceLink (ConceptNode subj_instance df-node-stv) (ConceptNode subj_concept df-node-stv) df-link-stv)
	(InheritanceLink (ConceptNode obj_instance df-node-stv) (ConceptNode obj_concept df-node-stv) df-link-stv)
	(InheritanceLink (ConceptNode iobj_instance df-node-stv) (ConceptNode iobj_concept df-node-stv) df-link-stv)
	(EvaluationLink df-link-stv (PredicateNode "Truth-Value" df-node-stv)
		(EvaluationLink df-link-stv (PredicateNode verb_instance df-node-stv)
			(ListLink df-link-stv			
				(ConceptNode subj_instance df-node-stv)
				(ConceptNode obj_instance df-node-stv)
				(ConceptNode iobj_instance df-node-stv)))
		(VariableNode "$qVar" df-node-stv))
))
;
;------------------------------------------------------------------------------------------
; who/what subject questions (who and what may be treated identically as far as I can tell).
;------------------------------------------------------------------------------------------
;
; Examples: "Who farted?", "What happened?"
;
(define (whowhatsubj-SV-Q-rule verb verb_instance)
	(list (ImplicationLink (PredicateNode verb_instance df-node-stv) (PredicateNode verb df-node-stv) df-link-stv)
	(EvaluationLink df-link-stv
		(PredicateNode verb_instance df-node-stv)
		(ListLink df-link-stv
			(VariableNode "_$qVar" df-node-stv)))
))
;
; Examples: "What killed him?", "Who ate the pizza?"
;
(define (whowhatsubj-SVO-Q-rule verb verb_instance obj_concept obj_instance)
	(list (ImplicationLink (PredicateNode verb_instance df-node-stv) (PredicateNode verb df-node-stv) df-link-stv)
	(InheritanceLink (ConceptNode obj_instance df-node-stv) (ConceptNode obj_concept df-node-stv) df-link-stv)
	(EvaluationLink df-link-stv
		(PredicateNode verb_instance df-node-stv)
		(ListLink df-link-stv
			(VariableNode "_$qVar" df-node-stv)
			(ConceptNode obj_instance df-node-stv)))
))
;
; Examples: "What gave you that idea?", "Who told you that?"
;
(define (whowhatsubj-SVIO-Q-rule verb verb_instance obj_concept obj_instance iobj_concept iobj_instance)
	(list (ImplicationLink (PredicateNode verb_instance df-node-stv) (PredicateNode verb df-node-stv) df-link-stv)
	(InheritanceLink (ConceptNode obj_instance df-node-stv) (ConceptNode obj_concept df-node-stv) df-link-stv)
	(InheritanceLink (ConceptNode iobj_instance df-node-stv) (ConceptNode iobj_concept df-node-stv) df-link-stv)
	(EvaluationLink df-link-stv
		(PredicateNode verb_instance df-node-stv)
		(ListLink df-link-stv
			(VariableNode "_$qVar" df-node-stv)
			(ConceptNode obj_instance df-node-stv)
			(ConceptNode iobj_instance df-node-stv)))
))
;
; Examples: "What is for dinner?", Who's on first?"
;
(define (whowhatpsubj-Q-rule prep prep_instance obj_concept obj_instance)
	(list (ImplicationLink (PredicateNode prep_instance df-node-stv) (PredicateNode prep df-node-stv) df-link-stv)
	(InheritanceLink (ConceptNode obj_instance df-node-stv) (ConceptNode obj_concept df-node-stv) df-link-stv)
	(EvaluationLink df-link-stv
		(PredicateNode prep_instance df-node-stv)
		(ListLink df-link-stv
			(VariableNode "_$qVar" df-node-stv)
			(ConceptNode obj_instance df-node-stv)))
))
;
; Example: "Who are you?"
;
(define (whocop-Q-rule subj_concept subj_instance)
	(list (InheritanceLink (ConceptNode subj_instance df-node-stv) (ConceptNode subj_concept df-node-stv) df-link-stv)
	(InheritanceLink (ConceptNode subj_instance df-node-stv) (VariableNode "_$qVar" df-node-stv))
))
;
;----------------------------------------------------------------------------------------
; who/what object and who/what indirect object questions
;----------------------------------------------------------------------------------------
;
; Examples: "Who do you love?", "What do you think?"
;
(define (whowhatobj-Q-rule subj_concept subj_instance verb verb_instance)
	(list (ImplicationLink (PredicateNode verb_instance df-node-stv) (PredicateNode verb df-node-stv) df-link-stv)
	(InheritanceLink (ConceptNode subj_instance df-node-stv) (ConceptNode subj_concept df-node-stv) df-link-stv)
	(EvaluationLink df-link-stv
		(PredicateNode verb_instance df-node-stv)
		(ListLink df-link-stv
			(ConceptNode subj_instance df-node-stv)
			(VariableNode "_$qVar")))		
))
;
; Examples: "To whom did you sell the children?"
;
(define (whowhatiobj-Q-rule subj_concept subj_instance verb verb_instance obj_concept obj_instance)
	(list (ImplicationLink (PredicateNode verb_instance df-node-stv) (PredicateNode verb df-node-stv) df-link-stv)
	(InheritanceLink (ConceptNode subj_instance df-node-stv) (ConceptNode subj_concept df-node-stv) df-link-stv)
	(InheritanceLink (ConceptNode obj_instance df-node-stv) (ConceptNode obj_concept df-node-stv) df-link-stv)
	(EvaluationLink df-link-stv
		(PredicateNode verb_instance df-node-stv)
		(ListLink df-link-stv
			(ConceptNode subj_instance df-node-stv)
			(ConceptNode obj_instance df-node-stv)
			(VariableNode "_$qVar")))
))
;
;-------------------------------------------------------------------------------------------------------
; The following rules for where, when, why, and how (of manner) seem highly redundant; they are exactly
; the same three rules (SV, SVO, SVIO) for each, except for the rule name and the name of the questioned
; predicate (AtTime, AtPlace, Because, and InManner).  One could in theory call all of them "Condition"
; and just use three rules in total . . . 
;-------------------------------------------------------------------------------------------------------
; WHERE QUESTIONS
;-------------------------------------------------------------------------------------------------------
;
; Example: "Where do you live?"
;
(define (where-SVQ-rule subj_concept subj_instance verb verb_instance)
	(list (ImplicationLink (PredicateNode verb_instance df-node-stv) (PredicateNode verb df-node-stv) df-link-stv)
	(InheritanceLink (ConceptNode subj_instance df-node-stv) (ConceptNode subj_concept df-node-stv) df-link-stv)
	(EvaluationLink df-link-stv 
		(PredicateNode "AtPlace" df-node-stv)
			(ListLink df-link-stv	
				(VariableNode "$qVar")
				(EvaluationLink df-link-stv
					(PredicateNode verb_instance df-node-stv)
					(ListLink (ConceptNode subj_instance df-node-stv) df-link-stv)))
)))
;
; "Where did you eat dinner?", "Where can I buy a fedora?"
;
(define (where-SVOQ-rule subj_concept subj_instance verb verb_instance obj_concept obj_instance)
	(list (ImplicationLink (PredicateNode verb_instance df-node-stv) (PredicateNode verb df-node-stv) df-link-stv)
	(InheritanceLink (ConceptNode subj_instance df-node-stv) (ConceptNode subj_concept df-node-stv) df-link-stv)
	(InheritanceLink (ConceptNode obj_instance df-node-stv) (ConceptNode obj_concept df-node-stv) df-link-stv)
	(EvaluationLink df-link-stv 
		(PredicateNode "AtPlace" df-node-stv)
			(ListLink df-link-stv	
				(VariableNode "$qVar")
				(EvaluationLink df-link-stv
					(PredicateNode verb_instance df-node-stv)
						(ListLink df-link-stv
							(ConceptNode subj_instance df-node-stv)
							(ConceptNode obj_instance df-node-stv)))))
))
;
; "Where did you give him the message?" 
;
; NB: need to compose to-do rules for sentences like "Where did she tell him to meet her?" (to-do) if we want to use such sentences . . .
;
(define (where-SVIOQ-rule subj_concept subj_instance verb verb_instance obj_concept obj_instance iobj_concept iobj_instance)
	(list (ImplicationLink (PredicateNode verb_instance df-node-stv) (PredicateNode verb df-node-stv) df-link-stv)
	(InheritanceLink (ConceptNode subj_instance df-node-stv) (ConceptNode subj_concept df-node-stv) df-link-stv)
	(InheritanceLink (ConceptNode obj_instance df-node-stv) (ConceptNode obj_concept df-node-stv) df-link-stv)
	(InheritanceLink (ConceptNode iobj_instance df-node-stv) (ConceptNode iobj_concept df-node-stv) df-link-stv)
	(EvaluationLink df-link-stv 
		(PredicateNode "AtPlace" df-node-stv)
			(ListLink df-link-stv	
				(VariableNode "$qVar")
				(EvaluationLink df-link-stv
					(PredicateNode verb_instance df-node-stv)
						(ListLink df-link-stv
							(ConceptNode subj_instance df-node-stv)
							(ConceptNode obj_instance df-node-stv)
							(ConceptNode iobj_instance df-node-stv)))))
))
;
; Examples: "Where is the party?", "Where are we?"
; 
(define (wherecop-Q-rule subj_concept subj_instance)
	(list (InheritanceLink (ConceptNode subj_instance df-node-stv) (ConceptNode subj_concept df-node-stv) df-link-stv)
	(EvaluationLink df-link-stv 
		(PredicateNode "AtPlace" df-node-stv)
			(ListLink df-link-stv	
				(VariableNode "$qVar")
				(ConceptNode subj_instance df-node-stv)))
))
;
;--------------------------------------------------------------------------------------------------------
; WHEN QUESTIONS
;--------------------------------------------------------------------------------------------------------
; 
; Example: "When did jazz die?"
;
(define (when-SVQ-rule subj_concept subj_instance verb verb_instance)
	(list (ImplicationLink (PredicateNode verb_instance df-node-stv) (PredicateNode verb df-node-stv) df-link-stv)
	(InheritanceLink (ConceptNode subj_instance df-node-stv) (ConceptNode subj_concept df-node-stv) df-link-stv)
	(AtTimeLink df-link-stv
		(VariableNode "$qVar")
		(EvaluationLink df-link-stv
			(PredicateNode verb_instance df-node-stv)
				(ListLink (ConceptNode subj_instance df-node-stv) df-ink-stv))
)))
;
; Example: "When did you bake the cake?" 
;
(define (when-SVOQ-rule subj_concept subj_instance verb verb_instance obj_concept obj_instance)
	(list (ImplicationLink (PredicateNode verb_instance df-node-stv) (PredicateNode verb df-node-stv) df-link-stv)
	(InheritanceLink (ConceptNode subj_instance df-node-stv) (ConceptNode subj_concept df-node-stv) df-link-stv)
	(InheritanceLink (ConceptNode obj_instance df-node-stv) (ConceptNode obj_concept df-node-stv) df-link-stv)
	(AtTimeLink df-link-stv
		(VariableNode "$qVar")
		(EvaluationLink df-link-stv
			(PredicateNode verb_instance df-node-stv)
				(ListLink df-link-stv
					(ConceptNode subj_instance df-node-stv)
					(ConceptNode obj_instance df-node-stv))))
))
; 
; Example: "When did you give him the money?"
;
(define (when-SVIOQ-rule subj_concept subj_instance verb verb_instance obj_concept obj_instance iobj_concept iobj_instance)
	(list (ImplicationLink (PredicateNode verb_instance df-node-stv) (PredicateNode verb df-node-stv) df-link-stv)
	(InheritanceLink (ConceptNode subj_instance df-node-stv) (ConceptNode subj_concept df-node-stv) df-link-stv)
	(InheritanceLink (ConceptNode obj_instance df-node-stv) (ConceptNode obj_concept df-node-stv) df-link-stv)
	(InheritanceLink (ConceptNode iobj_instance df-node-stv) (ConceptNode iobj_concept df-node-stv) df-link-stv)
	(AtTimeLink df-link-stv 
		(VariableNode "$qVar")
		(EvaluationLink df-link-stv
			(PredicateNode verb_instance df-node-stv)
				(ListLink df-link-stv
					(ConceptNode subj_instance df-node-stv)
					(ConceptNode obj_instance df-node-stv)
					(ConceptNode iobj_instance df-node-stv))))
))
;
; Example "When is the party?"
;
(define (whencop-Q-rule subj_concept subj_instance)
	(list (InheritanceLink (ConceptNode subj_instance df-node-stv) (ConceptNode subj_concept df-node-stv) df-link-stv)
	(AtTimeLink df-link-stv
		(VariableNode "$qVar")
		(ConceptNode subj_instance df-node-stv))
))
;
;----------------------------------------------------------------------------------------------------------
; Why questions
;----------------------------------------------------------------------------------------------------------
;
; Example: "Why do you live?"
;
(define (why-SVQ-rule subj_concept subj_instance verb verb_instance)
	(list (ImplicationLink (PredicateNode verb_instance df-node-stv) (PredicateNode verb df-node-stv) df-link-stv)
	(InheritanceLink (ConceptNode subj_instance df-node-stv) (ConceptNode subj_concept df-node-stv) df-link-stv)
	(EvaluationLink df-link-stv 
		(PredicateNode "Because" df-node-stv)
			(ListLink df-link-stv	
				(VariableNode "$qVar")
				(EvaluationLink df-link-stv
					(PredicateNode verb_instance df-node-stv)
					(ListLink (ConceptNode subj_instance df-node-stv) df-link-stv)))
)))
;
; Example: "Why do you like terrible music?" 
;
(define (why-SVOQ-rule subj_concept subj_instance verb verb_instance obj_concept obj_instance)
	(list (ImplicationLink (PredicateNode verb_instance df-node-stv) (PredicateNode verb df-node-stv) df-link-stv)
	(InheritanceLink (ConceptNode subj_instance df-node-stv) (ConceptNode subj_concept df-node-stv) df-link-stv)
	(InheritanceLink (ConceptNode obj_instance df-node-stv) (ConceptNode obj_concept df-node-stv) df-link-stv)
	(EvaluationLink df-link-stv 
		(PredicateNode "Because" df-node-stv)
			(ListLink df-link-stv	
				(VariableNode "$qVar")
				(EvaluationLink df-link-stv
					(PredicateNode verb_instance df-node-stv)
						(ListLink df-link-stv
							(ConceptNode subj_instance df-node-stv)
							(ConceptNode obj_instance df-node-stv)))))
))
;
; Example: "Why did you give him the money?"
;
(define (why-SVIOQ-rule subj_concept subj_instance verb verb_instance obj_concept obj_instance iobj_concept iobj_instance)
	(list (ImplicationLink (PredicateNode verb_instance df-node-stv) (PredicateNode verb df-node-stv) df-link-stv)
	(InheritanceLink (ConceptNode subj_instance df-node-stv) (ConceptNode subj_concept df-node-stv) df-link-stv)
	(InheritanceLink (ConceptNode obj_instance df-node-stv) (ConceptNode obj_concept df-node-stv) df-link-stv)
	(InheritanceLink (ConceptNode iobj_instance df-node-stv) (ConceptNode iobj_concept df-node-stv) df-link-stv)
	(EvaluationLink df-link-stv 
		(PredicateNode "Because" df-node-stv)
			(ListLink df-link-stv	
				(VariableNode "$qVar")
				(EvaluationLink df-link-stv
					(PredicateNode verb_instance df-node-stv)
						(ListLink df-link-stv
							(ConceptNode subj_instance df-node-stv)
							(ConceptNode obj_instance df-node-stv)
							(ConceptNode iobj_instance df-node-stv)))))
))
;
; Example: "Why are you so stupid?"
;
(define (whypredadj-Q-rule subj_concept subj_instance predicative_concept predicative_instance)
	(list (InheritanceLink (ConceptNode subj_instance df-node-stv) (ConceptNode subj_concept df-node-stv) df-link-stv)
	(InheritanceLink (ConceptNode predicative_instance df-node-stv) (ConceptNode predicative_concept df-node-stv) df-link-stv)
	(EvaluationLink df-link-stv 
		(PredicateNode "Because" df-node-stv)
			(ListLink df-link-stv	
				(VariableNode "$qVar")
				(EvaluationLink df-link-stv (PredicateNode predicative_instance df-node-stv) 
					(ListLink df-link-stv (ConceptNode subj_instance df-node-stv)))))
))
;
;----------------------------------------------------------------------------------------------------------
; How adverbial (manner) questions
;----------------------------------------------------------------------------------------------------------
; 
; Example: "How did you sleep?"
;
(define (how-SVQ-rule subj_concept subj_instance verb verb_instance)
	(list (ImplicationLink (PredicateNode verb_instance df-node-stv) (PredicateNode verb df-node-stv) df-link-stv)
	(InheritanceLink (ConceptNode subj_instance df-node-stv) (ConceptNode subj_concept df-node-stv) df-link-stv)
	(EvaluationLink df-link-stv 
		(PredicateNode "InManner" df-node-stv)
			(ListLink df-link-stv	
				(VariableNode "$qVar")
				(EvaluationLink df-link-stv
					(PredicateNode verb_instance df-node-stv)
					(ListLink (ConceptNode subj_instance df-node-stv) df-link-stv)))
)))
;
; Example: "How did you like the movie?"
;
(define (how-SVOQ-rule subj_concept subj_instance verb verb_instance obj_concept obj_instance)
	(list (ImplicationLink (PredicateNode verb_instance df-node-stv) (PredicateNode verb df-node-stv) df-link-stv)
	(InheritanceLink (ConceptNode subj_instance df-node-stv) (ConceptNode subj_concept df-node-stv) df-link-stv)
	(InheritanceLink (ConceptNode obj_instance df-node-stv) (ConceptNode obj_concept df-node-stv) df-link-stv)
	(EvaluationLink df-link-stv 
		(PredicateNode "InManner" df-node-stv)
			(ListLink df-link-stv	
				(VariableNode "$qVar")
				(EvaluationLink df-link-stv
					(PredicateNode verb_instance df-node-stv)
						(ListLink df-link-stv
							(ConceptNode subj_instance df-node-stv)
							(ConceptNode obj_instance df-node-stv)))))
))
; 
; Example: "How did you send him the message?"
;
(define (how-SVIOQ-rule subj_concept subj_instance verb verb_instance obj_concept obj_instance iobj_concept iobj_instance)
	(list (ImplicationLink (PredicateNode verb_instance df-node-stv) (PredicateNode verb df-node-stv) df-link-stv)
	(InheritanceLink (ConceptNode subj_instance df-node-stv) (ConceptNode subj_concept df-node-stv) df-link-stv)
	(InheritanceLink (ConceptNode obj_instance df-node-stv) (ConceptNode obj_concept df-node-stv) df-link-stv)
	(InheritanceLink (ConceptNode iobj_instance df-node-stv) (ConceptNode iobj_concept df-node-stv) df-link-stv)
	(EvaluationLink df-link-stv 
		(PredicateNode "InManner" df-node-stv)
			(ListLink df-link-stv	
				(VariableNode "$qVar")
				(EvaluationLink df-link-stv
					(PredicateNode verb_instance df-node-stv)
						(ListLink df-link-stv
							(ConceptNode subj_instance df-node-stv)
							(ConceptNode obj_instance df-node-stv)
							(ConceptNode iobj_instance df-node-stv)))))
))
;---------------------------------------------------------------------------------------------
; Predicative How
;---------------------------------------------------------------------------------------------
; 
; Examples: "How was the party?" "How is your food?")
;
(define (howpredadj-Q-rule subj_concept subj_instance)
	(list (InheritanceLink (ConceptNode subj_instance df-node-stv) (ConceptNode subj_concept df-node-stv) df-link-stv)
	(InheritanceLink (ConceptNode subj_instance df-node-stv) (VariableNode "$qVar"))
))
;
;----------------------------------------------------------------------------------------------------------------	
; How of quantity and degree questions
;----------------------------------------------------------------------------------------------------------------
;
; Example: "How much money does it cost?", "How many books have you read?"
;
; NB: doesn't call rule if there is no noun after "how much" because relex doesn't give it a quantity dependency relation in that case
;
(define (howquantQ-rule concept instance)
	(list (InheritanceLink (ConceptNode instance df-node-stv) (ConceptNode concept df-node-stv) df-link-stv)
	(QuantityLink df-link-stv	
		(VariableNode "_$qVar")
		(ConceptNode instance df-node-stv))
))
;
; Example: "How fast does it go?"
;
(define (howdegQ-rule concept instance)
	(list (InheritanceLink (ConceptNode instance df-node-stv) (ConceptNode concept df-node-stv) df-link-stv)
	(EvaluationLink	df-link-stv
		(PredicateNode "DegreeLink" df-node-stv)
			(ListLink df-link-stv
				(VariableNode "_$qVar")
				(ConceptNode instance df-node-stv)))
))
;
;-----------------------------------------------------------------------------------------------
; CHOICE-TYPE QUESTIONS
;-----------------------------------------------------------------------------------------------
;
; Example: "Which girl do you like?"
;
(define (whichobjQ-rule obj_concept obj_instance verb verb_instance subj_concept subj_instance)
	(list (ImplicationLink (PredicateNode verb_instance df-node-stv) (PredicateNode verb df-node-stv) df-link-stv)
	(InheritanceLink (ConceptNode subj_instance df-node-stv) (ConceptNode subj_concept df-node-stv) df-link-stv)
	(InheritanceLink (ConceptNode obj_instance df-node-stv) (ConceptNode obj_concept df-node-stv) df-link-stv)
	(InheritanceLink (VariableNode "$qVar") (ConceptNode obj_instance df-node-stv) df-link-stv)
	(SatisfyingSetLink df-link-stv
		(VariableNode "$qVar") 	
		(EvaluationLink df-link-stv (PredicateNode verb_instance df-node-stv)
			(ListlInk df-link-stv
				(ConceptNode subj_instance df-node-stv)
				(VariableNode "$qVar" df-node-stv))))
))
;
; Example: "Which girl likes you?"
;
;need to write SV, SVIO versions of these 
;
;
(define (whichsubjQ-rule subj_concept subj_instance verb verb_instance obj_concept obj_instance)			
	(list (ImplicationLink (PredicateNode verb_instance df-node-stv) (PredicateNode verb df-node-stv) df-link-stv)
	(InheritanceLink (ConceptNode subj_instance df-node-stv) (ConceptNode subj_concept df-node-stv) df-link-stv)
	(ImplicationLink (PredicateNode obj_instance df-node-stv) (PredicateNode obj_concept df-node-stv) df-link-stv)
	(InheritanceLink (VariableNode "$qVar") (ConceptNode subj_instance df-node-stv) df-link-stv)
	(SatisfyingSetLink df-link-stv
		(variableNode "$qVar")
		(EvaluationLink df-link-stv (PredicateNode verb_instance df-node-stv)
			(ListlInk df-link-stv
			(VariableNode "$qVar" df-node-stv)
			(PredicateNode obj_instance df-node-stv))))
))
;
; Example: "Which girl is crazy?"
;
(define (whichpredadjQ-rule subj_concept subj_instance pred_concept pred_instance)
	(list (InheritanceLink (ConceptNode subj_instance df-node-stv) (ConceptNode subj_concept df-node-stv) df-link-stv)
	(ImplicationLink (PredicateNode pred_instance df-node-stv) (PredicateNode pred_concept df-node-stv) df-link-stv)	
	(InheritanceLink (VariableNode "$qVar") (ConceptNode subj_instance df-node-stv)  df-link-stv)
	(InheritanceLink (VariableNode "$qVar") (PredicateNode pred_instance df-node-stv) df-link-stv)
))
;
; -----------------------------------------------------------------------
; all rules
; -----------------------------------------------------------------------
; Create a "allmarker" to handle all forms of "all" sentences, such as
; "All Canadians are right-handed."      -> (all-rule "Canadians@333")
; "All my writings are bad."             -> (all-rule "writings@333")
; "All Canadians give their dogs a hug." -> (all-rule "Canadians@333")
; "All Canadians write."                 -> (all-rule "Canadians@333")
; "All right-handed Canadians write."    -> (all-rule "Canadians@333")
(define (all-rule noun_instance)
	(list
		(EvaluationLink
			(PredicateNode "allmarker")
			(ListLink
				(ConceptNode noun_instance)
			)
		)
	)
)


; -----------------------------------------------------------------------
; passive verb rules
; -----------------------------------------------------------------------
; Example: "The books were written by Charles Dickens."
(define (passive-rule1 verb verb_instance obj obj_instance passive_obj passive_obj_instance)
    (list (ImplicationLink (PredicateNode verb_instance df-node-stv) (PredicateNode verb df-node-stv) df-link-stv)
    (InheritanceLink (ConceptNode obj_instance df-node-stv) (ConceptNode obj df-node-stv) df-link-stv)
    (InheritanceLink (ConceptNode passive_obj_instance df-node-stv) (ConceptNode passive_obj df-node-stv) df-link-stv)
    (EvaluationLink df-link-stv
            (PredicateNode verb_instance df-node-stv)
            (ListLink df-link-stv
                    (ConceptNode passive_obj_instance df-node-stv)
                    (ConceptNode obj_instance df-node-stv)
            )
    ))
)

; Example: "The books are published."
(define (passive-rule2 verb verb_instance obj obj_instance)
    (list (ImplicationLink (PredicateNode verb_instance df-node-stv) (PredicateNode verb df-node-stv) df-link-stv)
    (InheritanceLink (ConceptNode obj_instance df-node-stv) (ConceptNode obj df-node-stv) df-link-stv)
    (EvaluationLink df-link-stv
            (PredicateNode verb_instance df-node-stv)
            (ListLink df-link-stv
                    (VariableNode "$x" df-node-stv)
                    (ConceptNode obj_instance df-node-stv)
            )
    ))
)


; -----------------------------------------------------------------------
; and rules
; -----------------------------------------------------------------------
; Example:"I did my homework, and I went to school."
(define (and-rule1 var1 var1_instance var2 var2_instance)
	(list (ImplicationLink (PredicateNode var1_instance) (PredicateNode var1))
	(ImplicationLink (PredicateNode var2_instance) (PredicateNode var2))
	(EvaluationLink
        (PredicateNode "andmarker")
		(ListLink
			(PredicateNode var1_instance)
			(PredicateNode var2_instance)
		)
	))
)

; Example:"John and Madison eat the cake."
(define (and-rule2 var1 var1_instance var2 var2_instance)
	(list (InheritanceLink (ConceptNode var1_instance) (ConceptNode var1))
	(InheritanceLink (ConceptNode var2_instance) (ConceptNode var2))
        (EvaluationLink
        (PredicateNode "andmarker")
		(ListLink
			(ConceptNode var1_instance)
			(ConceptNode var2_instance)
		)
	))
)


; -----------------------------------------------------------------------
; that rule for creating thatmarker
; -----------------------------------------------------------------------
; Example: "I think that dogs can fly.", "He is glad that she won.", "He ran so quickly that he flew."
; A that-rule for "object clause", "content clause", "complement clause", etc, but not "adjective clause"
(define (that-rule main main_instance main_pos sub sub_instance sub_pos)
	; get the corresponding node
	(define main-node
		(if (string=? main_pos "verb")
			(PredicateNode main_instance df-node-stv)
			(ConceptNode main_instance df-node-stv)
		)
	)
	(define sub-node
		(if (string=? sub_pos "verb")
			(PredicateNode sub_instance df-node-stv)
			(ConceptNode sub_instance df-node-stv)
		)
	)

	; create the link between instance and word just in case
	(define main-link
		(if (string=? main_pos "verb")
			(ImplicationLink (PredicateNode main_instance df-node-stv) (PredicateNode main df-node-stv) df-link-stv)
			(InheritanceLink (ConceptNode main_instance df-node-stv) (ConceptNode main df-node-stv) df-link-stv)
		)
	)
	(define sub-link
		(if (string=? sub_pos "verb")
			(ImplicationLink (PredicateNode sub_instance df-node-stv) (PredicateNode sub df-node-stv) df-link-stv)
			(InheritanceLink (ConceptNode sub_instance df-node-stv) (ConceptNode sub df-node-stv) df-link-stv)
		)
	)

	(list
		main-link
		sub-link
		(EvaluationLink df-link-stv
			(PredicateNode "that" df-node-stv)
			(ListLink df-link-stv
				main-node
				sub-node
			)
		)
	)
)

; -----------------------------------------------------------------------
; time rules to create time relations
; -----------------------------------------------------------------------
; second argument of before() and after() can be a verb, adjective, pronoun
; or noun; rule can be invoked both with a before or after relation; it is
; checked which node needs to be created; discussion can be viewed here:
; https://github.com/opencog/opencog/pull/993
; Examples: "She went home before I left", "I went after him", "He sleeps
; before he is tired"
(define (before-after-rule $x_instance $y_instance $y_pos $before_or_after)
    (define y-node
        (if (string=? $y_pos "verb")
            (PredicateNode $y_instance df-node-stv)
            (ConceptNode $y_instance df-node-stv)
        )
    )
    (list
        (EvaluationLink df-link-stv
            (PredicateNode $before_or_after df-node-stv)
            (ListLink df-link-stv (PredicateNode $x_instance df-node-stv) y-node)
        )
    )
)

; Examples: "I had dinner at 6 pm", "I went to sleep at 1 am"
(define (time-rule $hour $period $v_instance)
    (define time-node
        (if (string=? $period "am")
            (TimeNode $hour df-node-stv)
            (TimeNode (number->string (+ (string->number $hour) 12)) df-node-stv)
        )
    )
    (list (AtTimeLink time-node (PredicateNode $v_instance df-node-stv) df-link-stv))
)


; -----------------------------------------------------------------------
; functions without R2L rule, not working, unneeded, etc
; -----------------------------------------------------------------------
(define (comparative-rule w1 w1_instance w2 w2_instance adj adj_instance)
	(list (InheritanceLink (ConceptNode adj_instance df-node-stv) (ConceptNode adj df-node-stv) df-link-stv)
	(InheritanceLink (ConceptNode w1_instance df-node-stv) (ConceptNode w1 df-node-stv) df-link-stv)
	(InheritanceLink (ConceptNode w2_instance df-node-stv) (ConceptNode w2 df-node-stv) df-link-stv)
	(TruthValueGreaterThanLink df-link-stv
		(InheritanceLink (ConceptNode w1_instance df-node-stv) (ConceptNode adj_instance df-node-stv) df-link-stv)
		(InheritanceLink (ConceptNode w2_instance df-node-stv) (ConceptNode adj_instance df-node-stv) df-link-stv)
	))
)

(define (on-rule w1 w1_instance w2 w2_instance)
	(list (InheritanceLink (ConceptNode w1_instance df-node-stv) (ConceptNode w1 df-node-stv) df-link-stv)
	(InheritanceLink (ConceptNode w2_instance df-node-stv) (ConceptNode w2 df-node-stv) df-link-stv)
	(EvaluationLink df-link-stv
		(PredicateNode "on" df-node-stv)
		(ListLink df-link-stv
			(ConceptNode w1_instance df-node-stv)
			(ConceptNode w2_instance df-node-stv)
		)
	))
)


;
;
;(define (which-rule antecedent  antecedent_instance  verb  verb_instance)
;	(list (InheritanceLink (ConceptNode antecedent_instance df-node-stv) (ConceptNode antecedent df-node-stv) df-link-stv)
;	(ImplicationLink (PredicateNode verb_instance df-node-stv) (PredicateNode verb df-node-stv) df-link-stv)
;        (EvaluationLink df-link-stv
;		(PredicateNode "whichmarker" df-node-stv)
;		(ListLink df-link-stv
;			(ConceptNode antecedent_instance df-node-stv)
;			(PredicateNode verb_instance df-node-stv)
;		)
;	))
;)

