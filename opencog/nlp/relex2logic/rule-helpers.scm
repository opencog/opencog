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

; =========================================================================================
; Actual helper functions for creating OpenCog atoms
; =========================================================================================
; The atoms added to the atomspace , by the helper functions below must be
; returned in a list or should be a single atom so as to simplify
; post-processing and NLG tasks.
;
;
;
;
;
; =========================================================================================
; Predicate-Argument templates
; Major revision October 2014 -- conditional substitution of query variables replaces separate query-rules
;==========================================================================================
;
; Subject-Copula-Object
;
; Declarative Examples: "Socrates is a man", "Cats are animals", "Trees are plants"
; Question Examples: 	"What is Socrates?" (object query) "Who is the teacher?" (object query) "Who is a man?" (subject query (rare))
;
; (*actually relex processes all of these questions as subject-queries, which reverses the logic of inheritance they should have, hence
; the rule below has the "object" inheriting the variable in order to compensate*since there is no way syntactic way to distinguish subject from 
; object queries for these sentences, correcting the logic of the rare true subject queries will have to be left to general intelligence*)
;
(define (be-inheritance-rule subj_concept subj_instance obj_concept obj_instance)
	(cond ((string=? subj_concept "_$qVar")
			(list 
				(InheritanceLink (ConceptNode obj_instance df-node-stv) (ConceptNode obj_concept df-node-stv) df-link-stv)
				(InheritanceLink (ConceptNode obj_instance df-node-stv) (VariableNode "_$qVar" df-node-stv))
			)
		)	
		((string=? obj_concept "_$qVar")
			(list 
				(InheritanceLink (ConceptNode subj_instance df-node-stv) (ConceptNode subj_concept df-node-stv) df-link-stv)
				(InheritanceLink (ConceptNode subj_instance df-node-stv) (VariableNode "_$qVar" df-node-stv))
			)
		)
	(else (list 
		(InheritanceLink (ConceptNode subj_instance df-node-stv) (ConceptNode subj_concept df-node-stv) df-link-stv)
		(InheritanceLink (ConceptNode obj_instance df-node-stv) (ConceptNode obj_concept df-node-stv) df-link-stv)
		(InheritanceLink (ConceptNode subj_instance df-node-stv) (ConceptNode obj_instance df-node-stv) df-link-stv)
		)
	)
	)
)
;--------------------------------------------------------------------------------------------
;
; SVIO
;
; Declarative: 			"Bill gave Mary the pox." / "Bill sold his children to the gypsies."
; Questioned subject: 		"Who told you that bullshit?" "Who told that story to the police?," "What gives you that idea?" "What gave that idea to the police?"
; Questioned object: 		"What did you tell the fuzz?" "What did you give to Mary?" "Who did you give the slavers?" "Who did you sell to the slavers?"
; Questioned indirect object: 	"To whom did you sell the children?" "To what do we owe the pleasure?" "Who did you sell the children to?"
;
;	NB: The first two iobj q-types work, although the relex output for the second one relies on a to(), instead of iobj(); the third one still doesn't work
;	because the relex output for it is a disaster and requires LG changes that I haven't gotten to work yet (10-22-14 AN)
;
(define (SVIO-rule subj_concept  subj_instance  verb  verb_instance  obj_concept  obj_instance iobj_concept iobj_instance)
	(cond ((string=? subj_concept "_$qVar")
		(list 	(ImplicationLink (PredicateNode verb_instance df-node-stv) (PredicateNode verb df-node-stv) df-link-stv)
			(InheritanceLink (ConceptNode obj_instance df-node-stv) (ConceptNode obj_concept df-node-stv) df-link-stv)
			(InheritanceLink (ConceptNode iobj_instance df-node-stv) (ConceptNode iobj_concept df-node-stv) df-link-stv)
			(EvaluationLink df-link-stv
				(PredicateNode verb_instance df-node-stv)
				(ListLink df-link-stv
					(VariableNode "_$qVar" df-node-stv)
					(ConceptNode obj_instance df-node-stv)
					(ConceptNode iobj_instance df-node-stv)
				)
			)
		))
		((string=? obj_concept "_$qVar")
		(list 	(ImplicationLink (PredicateNode verb_instance df-node-stv) (PredicateNode verb df-node-stv) df-link-stv)
			(InheritanceLink (ConceptNode subj_instance df-node-stv) (ConceptNode subj_concept df-node-stv) df-link-stv)
			(InheritanceLink (ConceptNode iobj_instance df-node-stv) (ConceptNode iobj_concept df-node-stv) df-link-stv)
			(EvaluationLink df-link-stv
				(PredicateNode verb_instance df-node-stv)
				(ListLink df-link-stv
					(ConceptNode subj_instance df-node-stv)
					(VariableNode "_$qVar" df-node-stv)
					(ConceptNode iobj_instance df-node-stv)
				)
			)
		))
		((string=? iobj_concept "_$qVar")
		(list 	(ImplicationLink (PredicateNode verb_instance df-node-stv) (PredicateNode verb df-node-stv) df-link-stv)
			(InheritanceLink (ConceptNode subj_instance df-node-stv) (ConceptNode subj_concept df-node-stv) df-link-stv)
			(InheritanceLink (ConceptNode obj_instance df-node-stv) (ConceptNode obj_concept df-node-stv) df-link-stv)
			(EvaluationLink df-link-stv
				(PredicateNode verb_instance df-node-stv)
				(ListLink df-link-stv
					(ConceptNode subj_instance df-node-stv)
					(ConceptNode obj_instance df-node-stv)
					(VariableNode "_$qVar" df-node-stv)))
		))
	(else (list 	(ImplicationLink (PredicateNode verb_instance df-node-stv) (PredicateNode verb df-node-stv) df-link-stv)
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
			)
		)
	))
)
;-----------------------------------------------------------------------------------------------------------------------------
;
; SVO
;
; Declarative: 			"Computers can bite me."
; Prepositional:		"The book is on the table."
; Subject query: 		"What bothers you?" "Who programmed you?" "What is on the table?"
; Object query: 		"What did you say?" "Who do you love?"
; Prepositional subject query: 	"What is for dinner?", Who's on first?"
;
(define (SVO-rule subj_concept subj_instance verb verb_instance obj_concept obj_instance)
	(cond ((string=? subj_concept "_$qVar")
		(list	(ImplicationLink (PredicateNode verb_instance df-node-stv) (PredicateNode verb df-node-stv) df-link-stv)
			(InheritanceLink (ConceptNode obj_instance df-node-stv) (ConceptNode obj_concept df-node-stv) df-link-stv)
			(EvaluationLink df-link-stv
				(PredicateNode verb_instance df-node-stv)
				(ListLink df-link-stv
					(VariableNode "_$qVar" df-node-stv)
					(ConceptNode obj_instance df-node-stv)
				)
			)
		))
		((string=? obj_concept "_$qVar")
		(list	(ImplicationLink (PredicateNode verb_instance df-node-stv) (PredicateNode verb df-node-stv) df-link-stv)
			(InheritanceLink (ConceptNode subj_instance df-node-stv) (ConceptNode subj_concept df-node-stv) df-link-stv)
			(EvaluationLink df-link-stv
				(PredicateNode verb_instance df-node-stv)
				(ListLink df-link-stv
					(ConceptNode subj_instance df-node-stv)
					(VariableNode "_$qVar" df-node-stv)
				)
			)
		))
	(else (list	(ImplicationLink (PredicateNode verb_instance df-node-stv) (PredicateNode verb df-node-stv) df-link-stv)
			(InheritanceLink (ConceptNode subj_instance df-node-stv) (ConceptNode subj_concept df-node-stv) df-link-stv)
			(InheritanceLink (ConceptNode obj_instance df-node-stv) (ConceptNode obj_concept df-node-stv) df-link-stv)
			(EvaluationLink df-link-stv
				(PredicateNode verb_instance df-node-stv)
				(ListLink df-link-stv
					(ConceptNode subj_instance df-node-stv)
					(ConceptNode obj_instance df-node-stv)
				)
			)
		)
	))
)
;----------------------------------------------------------------------------------------------------------------------------
;
; SV
;
; Declarative verb:			"Computers suck."
; Declarative predicate adjective:	"I am happy."
; Subject query:			"Who farted?" "What is happening?" "Who is correct?" "What is right?"
; Verb query:				"What are you doing?"
;
(define (SV-rule subj_concept subj_instance verb verb_instance)
	(cond ((string=? subj_concept "_$qVar")
		(list	(ImplicationLink (PredicateNode verb_instance df-node-stv) (PredicateNode verb df-node-stv) df-link-stv)
			(EvaluationLink df-link-stv
				(PredicateNode verb_instance df-node-stv)
				(ListLink df-link-stv
					(VariableNode "_$qVar" df-node-stv)
				)
			)
		))
		((string=? verb "_$qVar")
		(list	(InheritanceLink (ConceptNode subj_instance df-node-stv) (ConceptNode subj_concept df-node-stv) df-link-stv)
			(EvaluationLink df-link-stv
				(VariableNode "_$qVar" df-node-stv)
				(ListLink df-link-stv
					(ConceptNode subj_instance df-node-stv)
				)
			)
		))
	(else (list	(ImplicationLink (PredicateNode verb_instance df-node-stv) (PredicateNode verb df-node-stv) df-link-stv)
			(InheritanceLink (ConceptNode subj_instance df-node-stv) (ConceptNode subj_concept df-node-stv) df-link-stv)
			(EvaluationLink df-link-stv
				(PredicateNode verb_instance df-node-stv)
				(ListLink df-link-stv
					(ConceptNode subj_instance df-node-stv)
				)
			)
		)
	))
)
;----------------------------------------------------------------------------------------------------------------------------
;
; "To-be" rule
; Declarative: "He seems happy." "He seems to be happy."
;
; NB: This rule is not getting called enough; with the verb "appears" it only gets called on the second parse. With the verb "looks" not at all.
;
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
;--------------------------------------------------------------------------------------------------------------------
; Yes / no question rules
;--------------------------------------------------------------------------------------------------------------------
;
; Copula example: "Are you the one?"
;
(define (cop-ynQ-rule subj_concept subj_instance obj_concept obj_instance)
	(list 	(InheritanceLink (ConceptNode subj_instance df-node-stv) (ConceptNode subj_concept df-node-stv) df-link-stv)
		(InheritanceLink (ConceptNode obj_instance df-node-stv) (ConceptNode obj_concept df-node-stv) df-link-stv)
		(EvaluationLink (PredicateNode "Truth Value")
			(ListLink (InheritanceLink (ConceptNode subj_instance df-node-stv)(ConceptNode obj_instance df-node-stv) df-link-stv))
			(VariableNode "$var1")
		)
	)
)
;
; Predicate Adjective example: 			"Are you mad?" 
; Predicate prepositional phrase example: 	"Is the book under the table?"
; to-be example:				"Does he seem mad?"
; to-do1 example:				"Does she want to help us?"
; to-do2 example:				"Does she want you to help us?" (questions appropriately but the original rule assigns the wrong subject to verb2*)
; to-do3 example:				"Was she good enough to help?" (questions appropriately but the original rule is a logical disaster -- rewrite*)
; to-do4 example:				"Must she be able to sing?" (*ditto*)
; to-do5 example:				"Does she want to sing?"
; SV examples: 					"Have you slept?", "Will you sleep?", "Did you sleep?"
; SVO example: 					"Did you eat the leftover baba-ganoush?"
; SVIO examples: 				"Did you give her the money?", "Did you give the money to her?"
;
; NB: this rule also allows the system to handle all intonation-only versions of these questions (i.e. "The book is under the table?" etc.)
;
(define (pred-ynQ-rule predicate_concept predicate_instance)
	(list	(ImplicationLink (PredicateNode predicate_instance df-node-stv) (PredicateNode predicate_concept df-node-stv) df-link-stv)
		(EvaluationLink (PredicateNode "Truth Value")
			(EvaluationLink df-link-stv (PredicateNode predicate_instance df-node-stv))
			(VariableNode "$var1")
		)
	)
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
	(define concept_node (ConceptNode word df-node-stv))
	(cond ((string=? gender_type "feminine")
		(list (InheritanceLink (SpecificEntityNode word_instance df-node-stv) (ConceptNode "female" df-node-stv) df-link-stv)
		(InheritanceLink (SpecificEntityNode word_instance df-node-stv) (ConceptNode word df-node-stv) df-link-stv)
		))
		((string=? gender_type "masculine")
		(list (InheritanceLink (SpecificEntityNode word_instance df-node-stv) (ConceptNode "male" df-node-stv) df-link-stv)
		(InheritanceLink (SpecificEntityNode word_instance df-node-stv) (ConceptNode word df-node-stv) df-link-stv)
		))
        ((string=? gender_type "person")
		(list (InheritanceLink (SpecificEntityNode word_instance df-node-stv) (ConceptNode "unknown_gender" df-node-stv) df-link-stv)
		(InheritanceLink (SpecificEntityNode word_instance df-node-stv) (ConceptNode word df-node-stv) df-link-stv)
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
;
; Example: "She wants to help John."
;
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
;
; Example: "She wants you to help us." -- assigns wrong subject in second clause
;
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
;
; Example: "She is nice to help with the project." -- the scheme output for this rule is a logical mess
;
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
;
; Example: "She must be able to sing." ; v1 = sing , v2 = she
;
; What about "She must need to sing?" "She must want to sing?" -- why is must being treated as a main-verb rather than an auxiliary?
;
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
;
; Example: "She wants to sing."; verb1 = want, verb2 = sing, subj = she
;
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
;-------------------------------------------------------------------------------------------------------
;
; Where, When, Why, and How (of manner) -- the first rule of each of these groups handles questions with
; any of the templates SV, SVO, SVIO, SP, to-be, to-do, prep etc. (see the Y/N question rules for a complete list) 
;
;-------------------------------------------------------------------------------------------------------
; WHERE QUESTIONS
;-------------------------------------------------------------------------------------------------------
;
; Examples: "Where do you live?","Where did you eat dinner?" etc.
;
(define (where-rule verb verb_instance)
	(list (ImplicationLink (PredicateNode verb_instance df-node-stv) (PredicateNode verb df-node-stv) df-link-stv)
	(EvaluationLink df-link-stv 
		(PredicateNode "AtPlace" df-node-stv)
			(ListLink df-link-stv	
				(VariableNode "$qVar")
				(PredicateNode verb_instance df-node-stv)
			)
	)
	)
)
;
; Examples: "Where is the party?", "Where will she be happy?" etc.
; 
(define (wherecop-Q-rule subj_concept subj_instance)
	(list (InheritanceLink (ConceptNode subj_instance df-node-stv) (ConceptNode subj_concept df-node-stv) df-link-stv)
	(EvaluationLink df-link-stv 
		(PredicateNode "AtPlace" df-node-stv)
			(ListLink df-link-stv	
				(VariableNode "$qVar")
				(ConceptNode subj_instance df-node-stv)
			)
	)
	)
)
;
;--------------------------------------------------------------------------------------------------------
; WHEN QUESTIONS
;--------------------------------------------------------------------------------------------------------
; 
; Example: "When did jazz die?","When did you bake the cake?", "When did you give him the money?" etc.
;
(define (when-rule verb verb_instance)
	(list (ImplicationLink (PredicateNode verb_instance df-node-stv) (PredicateNode verb df-node-stv) df-link-stv)
	(AtTimeLink df-link-stv
		(VariableNode "$qVar")
		(PredicateNode verb_instance df-node-stv)
	)
	)
)
;
; Example "When is the party?" etc.
;
(define (whencop-Q-rule subj_concept subj_instance)
	(list (InheritanceLink (ConceptNode subj_instance df-node-stv) (ConceptNode subj_concept df-node-stv) df-link-stv)
	(AtTimeLink df-link-stv
		(VariableNode "$qVar")
		(ConceptNode subj_instance df-node-stv)
	)
	)
)
;
;----------------------------------------------------------------------------------------------------------
; Why questions
;----------------------------------------------------------------------------------------------------------
;
; Example: "Why do you live?","Why do you like terrible music?
;
(define (why-rule verb verb_instance)
	(list (ImplicationLink (PredicateNode verb_instance df-node-stv) (PredicateNode verb df-node-stv) df-link-stv)
	(EvaluationLink df-link-stv 
		(PredicateNode "Because" df-node-stv)
			(ListLink df-link-stv	
				(VariableNode "$qVar")
				(PredicateNode verb_instance df-node-stv)
			)
	)
	)
)
;
; Example "Why are you such a fool?" etc.
;
(define (whencop-Q-rule subj_concept subj_instance)
	(list (InheritanceLink (ConceptNode subj_instance df-node-stv) (ConceptNode subj_concept df-node-stv) df-link-stv)
	(EvaluationLink df-link-stv 
		(PredicateNode "Because" df-node-stv)
			(ListLink df-link-stv	
				(VariableNode "$qVar")
				(ConceptNode subj_instance df-node-stv)
			)
	)
))
;----------------------------------------------------------------------------------------------------------
; How adverbial (manner) questions
;----------------------------------------------------------------------------------------------------------
; 
; Example: "How did you sleep?" etc.
;
(define (how-rule verb verb_instance)
	(list (ImplicationLink (PredicateNode verb_instance df-node-stv) (PredicateNode verb df-node-stv) df-link-stv)
	(EvaluationLink df-link-stv 
		(PredicateNode "InManner" df-node-stv)
			(ListLink df-link-stv	
				(VariableNode "$qVar")
				(PredicateNode verb_instance df-node-stv)
			)
	)
	)
)
;---------------------------------------------------------------------------------------------
; Predicative How
;---------------------------------------------------------------------------------------------
; 
; Examples: "How was the party?" "How is your food?")
;
(define (howpredadj-Q-rule subj_concept subj_instance)
	(list (InheritanceLink (ConceptNode subj_instance df-node-stv) (ConceptNode subj_concept df-node-stv) df-link-stv)
	(InheritanceLink (ConceptNode subj_instance df-node-stv) (VariableNode "$qVar"))
	)
)
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
		(ConceptNode instance df-node-stv)
	)
	)
)
;
; Example: "How fast does it go?"
;
(define (howdegQ-rule concept instance)
	(list (InheritanceLink (ConceptNode instance df-node-stv) (ConceptNode concept df-node-stv) df-link-stv)
	(EvaluationLink	df-link-stv
		(PredicateNode "DegreeLink" df-node-stv)
			(ListLink df-link-stv
				(VariableNode "_$qVar")
				(ConceptNode instance df-node-stv)
			)
	)
	)
)
;
;-----------------------------------------------------------------------------------------------
; CHOICE-TYPE QUESTIONS 
;
; fixed a bug in the semantic algorithms so that these don't get _quantity() relations and now
; the r2l rules are not calling, for no reason that I can discern.
;
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
				(ListLink df-link-stv
					(ConceptNode subj_instance df-node-stv)
					(VariableNode "$qVar" df-node-stv)
				)
			)
		)
	)
)
;
; Example: "Which girl likes you?"
;
; May need to write SV, SVIO, prep, to-be versions of these if they cannot be fused with the sentence templates
;
;
(define (whichsubjQ-rule subj_concept subj_instance verb verb_instance obj_concept obj_instance)			
	(list (ImplicationLink (PredicateNode verb_instance df-node-stv) (PredicateNode verb df-node-stv) df-link-stv)
		(InheritanceLink (ConceptNode subj_instance df-node-stv) (ConceptNode subj_concept df-node-stv) df-link-stv)
		(ImplicationLink (PredicateNode obj_instance df-node-stv) (PredicateNode obj_concept df-node-stv) df-link-stv)
		(InheritanceLink (VariableNode "$qVar") (ConceptNode subj_instance df-node-stv) df-link-stv)
		(SatisfyingSetLink df-link-stv
			(VariableNode "$qVar")
			(EvaluationLink df-link-stv (PredicateNode verb_instance df-node-stv)
				(ListLink df-link-stv
					(VariableNode "$qVar" df-node-stv)
					(PredicateNode obj_instance df-node-stv)
				)
			)
		)
	)
)
;
; Example: "Which girl is crazy?"
;
(define (whichpredadjQ-rule subj_concept subj_instance pred_concept pred_instance)
	(list (InheritanceLink (ConceptNode subj_instance df-node-stv) (ConceptNode subj_concept df-node-stv) df-link-stv)
		(ImplicationLink (PredicateNode pred_instance df-node-stv) (PredicateNode pred_concept df-node-stv) df-link-stv)	
		(InheritanceLink (VariableNode "$qVar") (ConceptNode subj_instance df-node-stv)  df-link-stv)
		(SatisfyingSetLink df-link-stv
			(VariableNode "$qVar")
			(EvaluationLink df-link-stv (PredicateNode pred_instance df-node-stv)
				(ListLink df-link-stv
					(VariableNode "$qVar" df-node-stv)
				)
			)
		)
	)
)
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
;  conjunction rules
; -----------------------------------------------------------------------
;coordinating conjunction - And, but, for, nor, or, so, and yet
;Example:"I did my homework, and I went to school."
;       "John and Madison eat the cake."
;       " Joan is poor  but  happy." 

(define (and-rule var1 var1_instance var2 var2_instance pos)
    (cond [(equal? pos "verb") 
	(list (ImplicationLink (PredicateNode var1_instance) (PredicateNode var1))
	(ImplicationLink (PredicateNode var2_instance) (PredicateNode var2))
        (EvaluationLink
        (PredicateNode "and")
		(ListLink
			(PredicateNode var1_instance)
			(PredicateNode var2_instance)
		)
	))]
	[else 
    (list (InheritanceLink (ConceptNode var1_instance) (ConceptNode var1))
	(InheritanceLink (ConceptNode var2_instance) (ConceptNode var2))
        (EvaluationLink
        (PredicateNode "and")
		(ListLink
			(ConceptNode var1_instance)
			(ConceptNode var2_instance)
		)
	))])
)

(define (but-rule var1 var1_instance var2 var2_instance pos)
    (cond [(equal? pos "verb") 
	(list (ImplicationLink (PredicateNode var1_instance) (PredicateNode var1))
	(ImplicationLink (PredicateNode var2_instance) (PredicateNode var2))
        (EvaluationLink
        (PredicateNode "but")
		(ListLink
			(PredicateNode var1_instance)
			(PredicateNode var2_instance)
		)
	))]
	[else 
    (list (InheritanceLink (ConceptNode var1_instance) (ConceptNode var1))
	(InheritanceLink (ConceptNode var2_instance) (ConceptNode var2))
        (EvaluationLink
        (PredicateNode "but")
		(ListLink
			(ConceptNode var1_instance)
			(ConceptNode var2_instance)
		)
	))])
)

(define (or-rule var1 var1_instance var2 var2_instance pos)
    (cond [(equal? pos "verb") 
	(list (ImplicationLink (PredicateNode var1_instance) (PredicateNode var1))
	(ImplicationLink (PredicateNode var2_instance) (PredicateNode var2))
        (EvaluationLink
        (PredicateNode "or")
		(ListLink
			(PredicateNode var1_instance)
			(PredicateNode var2_instance)
		)
	))]
	[else 
    (list (InheritanceLink (ConceptNode var1_instance) (ConceptNode var1))
	(InheritanceLink (ConceptNode var2_instance) (ConceptNode var2))
        (EvaluationLink
        (PredicateNode "or")
		(ListLink
			(ConceptNode var1_instance)
			(ConceptNode var2_instance)
		)
	))])
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
;-----------------------------------------------------------------------------------------------------------------------------
; Functional Question rules replaced by conditional statements and templates in October 2014
;-----------------------------------------------------------------------------------------------------------------------------
;
; NB: sentences with prepositional phrases as the predicate-complements of the subject are treated by Relex in a variety
; of ways, including the prepobj relation and also treating the prep as a simple predicate; those sentences that do not
; call the rule below, call the SV-ynq rule.
;
;(define (prepobj-ynQ-rule subj_concept subj_instance predprep_concept predprep_instance obj_concept obj_instance)
;	(list (InheritanceLink (ConceptNode subj_instance df-node-stv) (ConceptNode subj_concept df-node-stv) df-link-stv)
;	(InheritanceLink (ConceptNode obj_instance df-node-stv) (ConceptNode obj_concept df-node-stv) df-link-stv)
;	(ImplicationLink (PredicateNode predprep_instance df-node-stv) (PredicateNode predprep_concept df-node-stv) df-link-stv)
;	(EvaluationLink (PredicateNode "Truth Value")
;		(EvaluationLink df-link-stv (PredicateNode predprep_instance df-node-stv)
;			(ListLink df-link-stv (ConceptNode subj_instance df-node-stv)(ConceptNode obj_instance df-node-stv)))
;			(VariableNode "$var1"))
;))
;
;-------------------------------------------------------------------------------------------------------
; 
;
;(define (SV-ynQ-rule subj_concept subj_instance verb verb_instance)
;	(list (ImplicationLink (PredicateNode verb_instance df-node-stv) (PredicateNode verb df-node-stv) df-link-stv)
;	(InheritanceLink (ConceptNode subj_instance df-node-stv) (ConceptNode subj_concept df-node-stv) df-link-stv)
;	(EvaluationLink df-link-stv (PredicateNode "Truth-Value" df-node-stv)
;		(EvaluationLink df-link-stv (PredicateNode verb_instance df-node-stv)
;			(ListLink df-link-stv (ConceptNode subj_instance df-node-stv)))
;		(VariableNode "$qVar" df-node-stv))
;))
;

;
; NB: 	This rule applies incorrectly to many SVIO sentences where the IO is interpreted by relex as 
;	an extra prepositional phrase rather than an IO.
;
;(define (SVO-ynQ-rule subj_concept subj_instance verb verb_instance obj_concept obj_instance)
;	(list (ImplicationLink (PredicateNode verb_instance df-node-stv) (PredicateNode verb df-node-stv) df-link-stv)
;	(InheritanceLink (ConceptNode subj_instance df-node-stv) (ConceptNode subj_concept df-node-stv) df-link-stv)
;	(InheritanceLink (ConceptNode obj_instance df-node-stv) (ConceptNode obj_concept df-node-stv) df-link-stv)
;	(EvaluationLink df-link-stv (PredicateNode "Truth-Value" df-node-stv)
;		(EvaluationLink df-link-stv (PredicateNode verb_instance df-node-stv)
;			(ListLink df-link-stv 
;				(ConceptNode subj_instance df-node-stv)
;				(ConceptNode obj_instance df-node-stv)))
;		(VariableNode "$qVar" df-node-stv))
;))
;

;
;(define (SVIO-ynQ-rule subj_concept subj_instance verb verb_instance obj_concept obj_instance iobj_concept iobj_instance)
;	(list (ImplicationLink (PredicateNode verb_instance df-node-stv) (PredicateNode verb df-node-stv) df-link-stv)
;	(InheritanceLink (ConceptNode subj_instance df-node-stv) (ConceptNode subj_concept df-node-stv) df-link-stv)
;	(InheritanceLink (ConceptNode obj_instance df-node-stv) (ConceptNode obj_concept df-node-stv) df-link-stv)
;	(InheritanceLink (ConceptNode iobj_instance df-node-stv) (ConceptNode iobj_concept df-node-stv) df-link-stv)
;	(EvaluationLink df-link-stv (PredicateNode "Truth-Value" df-node-stv)
;		(EvaluationLink df-link-stv (PredicateNode verb_instance df-node-stv)
;			(ListLink df-link-stv			
;				(ConceptNode subj_instance df-node-stv)
;				(ConceptNode obj_instance df-node-stv)
;				(ConceptNode iobj_instance df-node-stv)))
;		(VariableNode "$qVar" df-node-stv))
;))
;
;------------------------------------------------------------------------------------------
; who/what subject questions (who and what may be treated identically as far as I can tell).
;------------------------------------------------------------------------------------------
;
; Examples: "Who farted?", "What happened?"
;
;(define (whowhatsubj-SV-Q-rule verb verb_instance)
;	(list (ImplicationLink (PredicateNode verb_instance df-node-stv) (PredicateNode verb df-node-stv) df-link-stv)
;	(EvaluationLink df-link-stv
;		(PredicateNode verb_instance df-node-stv)
;		(ListLink df-link-stv
;			(VariableNode "_$qVar" df-node-stv)))
;))
;
; Examples: "What killed him?", "Who ate the pizza?"
;
;(define (whowhatsubj-SVO-Q-rule verb verb_instance obj_concept obj_instance)
;	(list (ImplicationLink (PredicateNode verb_instance df-node-stv) (PredicateNode verb df-node-stv) df-link-stv)
;	(InheritanceLink (ConceptNode obj_instance df-node-stv) (ConceptNode obj_concept df-node-stv) df-link-stv)
;	(EvaluationLink df-link-stv
;		(PredicateNode verb_instance df-node-stv)
;		(ListLink df-link-stv
;			(VariableNode "_$qVar" df-node-stv)
;			(ConceptNode obj_instance df-node-stv)))
;))
;
; Examples: "What gave you that idea?", "Who told you that?"
;
;(define (whowhatsubj-SVIO-Q-rule verb verb_instance obj_concept obj_instance iobj_concept iobj_instance)
;	(list (ImplicationLink (PredicateNode verb_instance df-node-stv) (PredicateNode verb df-node-stv) df-link-stv)
;	(InheritanceLink (ConceptNode obj_instance df-node-stv) (ConceptNode obj_concept df-node-stv) df-link-stv)
;	(InheritanceLink (ConceptNode iobj_instance df-node-stv) (ConceptNode iobj_concept df-node-stv) df-link-stv)
;	(EvaluationLink df-link-stv
;		(PredicateNode verb_instance df-node-stv)
;		(ListLink df-link-stv
;			(VariableNode "_$qVar" df-node-stv)
;			(ConceptNode obj_instance df-node-stv)
;			(ConceptNode iobj_instance df-node-stv)))
;))
;
; Examples: "What is for dinner?", Who's on first?"
;
;(define (whowhatpsubj-Q-rule prep prep_instance obj_concept obj_instance)
;	(list (ImplicationLink (PredicateNode prep_instance df-node-stv) (PredicateNode prep df-node-stv) df-link-stv)
;	(InheritanceLink (ConceptNode obj_instance df-node-stv) (ConceptNode obj_concept df-node-stv) df-link-stv)
;	(EvaluationLink df-link-stv
;		(PredicateNode prep_instance df-node-stv)
;		(ListLink df-link-stv
;			(VariableNode "_$qVar" df-node-stv)
;			(ConceptNode obj_instance df-node-stv)))
;))
;
; Example: "Who are you?"
;
;(define (whocop-Q-rule subj_concept subj_instance)
;	(list (InheritanceLink (ConceptNode subj_instance df-node-stv) (ConceptNode subj_concept df-node-stv) df-link-stv)
;	(InheritanceLink (ConceptNode subj_instance df-node-stv) (VariableNode "_$qVar" df-node-stv))
;))
;
;----------------------------------------------------------------------------------------
; who/what object and who/what indirect object questions
;----------------------------------------------------------------------------------------
;
; Examples: "Who do you love?", "What do you think?"
;
;(define (whowhatobj-Q-rule subj_concept subj_instance verb verb_instance)
;	(list (ImplicationLink (PredicateNode verb_instance df-node-stv) (PredicateNode verb df-node-stv) df-link-stv)
;	(InheritanceLink (ConceptNode subj_instance df-node-stv) (ConceptNode subj_concept df-node-stv) df-link-stv)
;	(EvaluationLink df-link-stv
;		(PredicateNode verb_instance df-node-stv)
;		(ListLink df-link-stv
;			(ConceptNode subj_instance df-node-stv)
;			(VariableNode "_$qVar")))		
;))
;
; Examples: "To whom did you sell the children?"
;
;(define (whowhatiobj-Q-rule subj_concept subj_instance verb verb_instance obj_concept obj_instance)
;	(list (ImplicationLink (PredicateNode verb_instance df-node-stv) (PredicateNode verb df-node-stv) df-link-stv)
;	(InheritanceLink (ConceptNode subj_instance df-node-stv) (ConceptNode subj_concept df-node-stv) df-link-stv)
;	(InheritanceLink (ConceptNode obj_instance df-node-stv) (ConceptNode obj_concept df-node-stv) df-link-stv)
;	(EvaluationLink df-link-stv
;		(PredicateNode verb_instance df-node-stv)
;		(ListLink df-link-stv
;			(ConceptNode subj_instance df-node-stv)
;			(ConceptNode obj_instance df-node-stv)
;			(VariableNode "_$qVar")))
;))
;
;----------------------------------------------------------------------------------------------------------
;
; Example: 
;
;(define (when-SVOQ-rule subj_concept subj_instance verb verb_instance obj_concept obj_instance)
;	(list (ImplicationLink (PredicateNode verb_instance df-node-stv) (PredicateNode verb df-node-stv) df-link-stv)
;	(InheritanceLink (ConceptNode subj_instance df-node-stv) (ConceptNode subj_concept df-node-stv) df-link-stv)
;	(InheritanceLink (ConceptNode obj_instance df-node-stv) (ConceptNode obj_concept df-node-stv) df-link-stv)
;	(AtTimeLink df-link-stv
;		(VariableNode "$qVar")
;		(EvaluationLink df-link-stv
;			(PredicateNode verb_instance df-node-stv)
;				(ListLink df-link-stv
;					(ConceptNode subj_instance df-node-stv)
;					(ConceptNode obj_instance df-node-stv))))
;))
; 
; Example: 
;
;(define (when-SVIOQ-rule subj_concept subj_instance verb verb_instance obj_concept obj_instance iobj_concept iobj_instance)
;	(list (ImplicationLink (PredicateNode verb_instance df-node-stv) (PredicateNode verb df-node-stv) df-link-stv)
;	(InheritanceLink (ConceptNode subj_instance df-node-stv) (ConceptNode subj_concept df-node-stv) df-link-stv)
;	(InheritanceLink (ConceptNode obj_instance df-node-stv) (ConceptNode obj_concept df-node-stv) df-link-stv)
;	(InheritanceLink (ConceptNode iobj_instance df-node-stv) (ConceptNode iobj_concept df-node-stv) df-link-stv)
;	(AtTimeLink df-link-stv 
;		(VariableNode "$qVar")
;		(EvaluationLink df-link-stv
;			(PredicateNode verb_instance df-node-stv)
;				(ListLink df-link-stv
;					(ConceptNode subj_instance df-node-stv)
;					(ConceptNode obj_instance df-node-stv)
;					(ConceptNode iobj_instance df-node-stv))))
;))
;------------------------------------------------------------------------------------------------------------
;
; Example: " 
;
;(define (why-SVOQ-rule subj_concept subj_instance verb verb_instance obj_concept obj_instance)
;	(list (ImplicationLink (PredicateNode verb_instance df-node-stv) (PredicateNode verb df-node-stv) df-link-stv)
;	(InheritanceLink (ConceptNode subj_instance df-node-stv) (ConceptNode subj_concept df-node-stv) df-link-stv)
;	(InheritanceLink (ConceptNode obj_instance df-node-stv) (ConceptNode obj_concept df-node-stv) df-link-stv)
;	(EvaluationLink df-link-stv 
;		(PredicateNode "Because" df-node-stv)
;			(ListLink df-link-stv	
;				(VariableNode "$qVar")
;				(EvaluationLink df-link-stv
;					(PredicateNode verb_instance df-node-stv)
;						(ListLink df-link-stv
;							(ConceptNode subj_instance df-node-stv)
;							(ConceptNode obj_instance df-node-stv)))))
;))
;
; Example: "Why did you give him the money?"
;
;(define (why-SVIOQ-rule subj_concept subj_instance verb verb_instance obj_concept obj_instance iobj_concept iobj_instance)
;	(list (ImplicationLink (PredicateNode verb_instance df-node-stv) (PredicateNode verb df-node-stv) df-link-stv)
;	(InheritanceLink (ConceptNode subj_instance df-node-stv) (ConceptNode subj_concept df-node-stv) df-link-stv)
;	(InheritanceLink (ConceptNode obj_instance df-node-stv) (ConceptNode obj_concept df-node-stv) df-link-stv)
;	(InheritanceLink (ConceptNode iobj_instance df-node-stv) (ConceptNode iobj_concept df-node-stv) df-link-stv)
;	(EvaluationLink df-link-stv 
;		(PredicateNode "Because" df-node-stv)
;			(ListLink df-link-stv	
;				(VariableNode "$qVar")
;				(EvaluationLink df-link-stv
;					(PredicateNode verb_instance df-node-stv)
;						(ListLink df-link-stv
;							(ConceptNode subj_instance df-node-stv)
;							(ConceptNode obj_instance df-node-stv)
;							(ConceptNode iobj_instance df-node-stv)))))
;))
;
; Example: "Why are you so stupid?"
;
;(define (why-Q-rule subj_concept subj_instance predicative_concept predicative_instance)
;	(list (InheritanceLink (ConceptNode subj_instance df-node-stv) (ConceptNode subj_concept df-node-stv) df-link-stv)
;	(InheritanceLink (ConceptNode predicative_instance df-node-stv) (ConceptNode predicative_concept df-node-stv) df-link-stv)
;	(EvaluationLink df-link-stv 
;		(PredicateNode "Because" df-node-stv)
;			(ListLink df-link-stv	
;				(VariableNode "$qVar")
;				(EvaluationLink df-link-stv (PredicateNode predicative_instance df-node-stv) 
;					(ListLink df-link-stv (ConceptNode subj_instance df-node-stv)))))
;))
;
;------------------------------------------------------------------------------------------------------------
;
; Example: "How did you like the movie?"
;
;(define (how-SVOQ-rule subj_concept subj_instance verb verb_instance obj_concept obj_instance)
;	(list (ImplicationLink (PredicateNode verb_instance df-node-stv) (PredicateNode verb df-node-stv) df-link-stv)
;	(InheritanceLink (ConceptNode subj_instance df-node-stv) (ConceptNode subj_concept df-node-stv) df-link-stv)
;	(InheritanceLink (ConceptNode obj_instance df-node-stv) (ConceptNode obj_concept df-node-stv) df-link-stv)
;	(EvaluationLink df-link-stv 
;		(PredicateNode "InManner" df-node-stv)
;			(ListLink df-link-stv	
;				(VariableNode "$qVar")
;				(EvaluationLink df-link-stv
;					(PredicateNode verb_instance df-node-stv)
;						(ListLink df-link-stv
;							(ConceptNode subj_instance df-node-stv)
;							(ConceptNode obj_instance df-node-stv)))))
;))
; 
; Example: "How did you send him the message?"
;
;(define (how-SVIOQ-rule subj_concept subj_instance verb verb_instance obj_concept obj_instance iobj_concept iobj_instance)
;	(list (ImplicationLink (PredicateNode verb_instance df-node-stv) (PredicateNode verb df-node-stv) df-link-stv)
;	(InheritanceLink (ConceptNode subj_instance df-node-stv) (ConceptNode subj_concept df-node-stv) df-link-stv)
;	(InheritanceLink (ConceptNode obj_instance df-node-stv) (ConceptNode obj_concept df-node-stv) df-link-stv)
;	(InheritanceLink (ConceptNode iobj_instance df-node-stv) (ConceptNode iobj_concept df-node-stv) df-link-stv)
;	(EvaluationLink df-link-stv 
;		(PredicateNode "InManner" df-node-stv)
;			(ListLink df-link-stv	
;				(VariableNode "$qVar")
;				(EvaluationLink df-link-stv
;					(PredicateNode verb_instance df-node-stv)
;						(ListLink df-link-stv
;							(ConceptNode subj_instance df-node-stv)
;							(ConceptNode obj_instance df-node-stv)
;							(ConceptNode iobj_instance df-node-stv)))))
;))
;----------------------------------------------------------------------------------------------------------------------------
;(define (SVP-rule subj  subj_instance  predicative  predicative_instance)
;	(list (InheritanceLink (ConceptNode predicative_instance df-node-stv) (ConceptNode predicative df-node-stv) df-link-stv)
;	(InheritanceLink (ConceptNode subj_instance df-node-stv) (ConceptNode subj df-node-stv) df-link-stv)
;	(InheritanceLink (ConceptNode subj_instance df-node-stv) (ConceptNode predicative_instance df-node-stv) df-link-stv)
;	)
;
