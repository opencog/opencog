; Notes: random-string, random-node-name and choose-var-name can be moved to utilities.scm

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
; -----------------------------------------------------------------------
; Connect WordInstanceNode With ConceptNode
(define (r2l-wordinst-concept inst-name)
	(ReferenceLink
		(ConceptNode inst-name)
		(WordInstanceNode inst-name)
	)
)
; -----------------------------------------------------------------------
; Connect WordInstanceNode With PredicateNode
(define (r2l-wordinst-Predicate inst-name)
	(ReferenceLink
		(PredicateNode inst-name)
		(WordInstanceNode inst-name)
	)
)
; =========================================================================================
; Actual helper functions for creating OpenCog atoms
; =========================================================================================
; The atoms added to the atomspace , by the helper functions below must be
; returned in a list or should be a single atom so as to simplify
; post-processing and NLG tasks.
;==========================================================================================
;
; Tag Interpretation Nodes with Speech Act Types
; (interrogative means question-word questions, while truthquery means yes/no questions)
;
; NB: Imperatives are not perfect -- (1) I've got relex to add an implicit subject "you" to imperatives,
; 	but the "instance" still has to be the Left-Wall -- the Left-Wall inherits the concept "you";
;	the problem is that if you replace the left-wall with "you" entirely it totally de-rails relex.
;	(2) Imperatives with subjects, i.e. "You tell me!" are interpreted as declaratives.
;
(define (imperative-rule interpretation_index)
	(list (InheritanceLink interpretation_index (ConceptNode "ImperativeSpeechAct"))
	)
)
(define (interrogative-rule interpretation_index)
	(list (InheritanceLink interpretation_index (ConceptNode "InterrogativeSpeechAct"))
	)
)
(define (truth-query-rule interpretation_index)
	(list (InheritanceLink interpretation_index (ConceptNode "TruthQuerySpeechAct"))
	)
)
(define (declarative-rule interpretation_index)
	(list (InheritanceLink interpretation_index (ConceptNode "DeclarativeSpeechAct"))
	)
)
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
			(let ((var_name (choose-var-name)))
				(list
				(InheritanceLink (ConceptNode obj_instance) (ConceptNode obj_concept))
				(InheritanceLink (ConceptNode obj_instance) (VariableNode var_name))
				)
			)
		)	
		((string=? obj_concept "_$qVar")
			(let ((var_name (choose-var-name)))
				(list
				(InheritanceLink (ConceptNode subj_instance) (ConceptNode subj_concept))
				(InheritanceLink (ConceptNode subj_instance) (VariableNode var_name))
				)
			)
		)
	(else (list 
		(InheritanceLink (ConceptNode subj_instance) (ConceptNode subj_concept))
		(InheritanceLink (ConceptNode obj_instance) (ConceptNode obj_concept))
		(InheritanceLink (ConceptNode subj_instance) (ConceptNode obj_instance))
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
;	NB: The first two iobj q-types work, although the relex output for the second one relies on a to(), instead of iobj()
;	the third one still doesn't work because the relex output for it is a disaster and requires LG changes that I haven't gotten to work yet (10-22-14 AN)
;
(define (SVIO-rule subj_concept  subj_instance  verb  verb_instance  obj_concept  obj_instance iobj_concept iobj_instance)
	(cond ((string=? subj_concept "_$qVar")
		(let ((var_name (choose-var-name)))
			(list
				(ImplicationLink (PredicateNode verb_instance) (PredicateNode verb))
				(InheritanceLink (ConceptNode obj_instance) (ConceptNode obj_concept))
				(InheritanceLink (ConceptNode iobj_instance) (ConceptNode iobj_concept))
				(EvaluationLink
					(PredicateNode verb_instance)
					(ListLink
						(VariableNode var_name)
						(ConceptNode obj_instance)
						(ConceptNode iobj_instance)
					)
				)
			)
		))
		((string=? obj_concept "_$qVar")
			(let ((var_name (choose-var-name)))
				(list
					(ImplicationLink (PredicateNode verb_instance) (PredicateNode verb))
					(InheritanceLink (ConceptNode subj_instance) (ConceptNode subj_concept))
					(InheritanceLink (ConceptNode iobj_instance) (ConceptNode iobj_concept))
					(EvaluationLink
						(PredicateNode verb_instance)
						(ListLink
							(ConceptNode subj_instance)
							(VariableNode var_name)
							(ConceptNode iobj_instance)
						)
					)
				)
			)
		)
		((string=? iobj_concept "_$qVar")
			(let ((var_name (choose-var-name)))
				(list
					(ImplicationLink (PredicateNode verb_instance) (PredicateNode verb))
					(InheritanceLink (ConceptNode subj_instance) (ConceptNode subj_concept))
					(InheritanceLink (ConceptNode obj_instance) (ConceptNode obj_concept))
					(EvaluationLink
						(PredicateNode verb_instance)
						(ListLink
							(ConceptNode subj_instance)
							(ConceptNode obj_instance)
							(VariableNode var_name)
						)
					)
				)
			)
		)
		(else (list 	(ImplicationLink (PredicateNode verb_instance) (PredicateNode verb))
				(InheritanceLink (ConceptNode subj_instance) (ConceptNode subj_concept))
				(InheritanceLink (ConceptNode obj_instance) (ConceptNode obj_concept))
				(InheritanceLink (ConceptNode iobj_instance) (ConceptNode iobj_concept))
					(EvaluationLink
        					(PredicateNode verb_instance)
        					(ListLink
            						(ConceptNode subj_instance)
            						(ConceptNode obj_instance)
            						(ConceptNode iobj_instance)
						)
					)
			)
		)
	)
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
		(let ((var_name (choose-var-name)))
			(list
				(ImplicationLink (PredicateNode verb_instance) (PredicateNode verb))
				(InheritanceLink (ConceptNode obj_instance) (ConceptNode obj_concept))
				(EvaluationLink
					(PredicateNode verb_instance)
					(ListLink
						(VariableNode var_name)
						(ConceptNode obj_instance)
					)
				)
			)
		))
		((string=? obj_concept "_$qVar")
			(let ((var_name (choose-var-name)))
				(list
					(ImplicationLink (PredicateNode verb_instance) (PredicateNode verb))
					(InheritanceLink (ConceptNode subj_instance) (ConceptNode subj_concept))
					(EvaluationLink
						(PredicateNode verb_instance)
						(ListLink
							(ConceptNode subj_instance)
							(VariableNode var_name)
						)
					)
				)
			)
		)
	(else (list	(ImplicationLink (PredicateNode verb_instance) (PredicateNode verb))
			(InheritanceLink (ConceptNode subj_instance) (ConceptNode subj_concept))
			(InheritanceLink (ConceptNode obj_instance) (ConceptNode obj_concept))
			(EvaluationLink
				(PredicateNode verb_instance)
				(ListLink
					(ConceptNode subj_instance)
					(ConceptNode obj_instance)
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
		(let ((var_name (choose-var-name)))
			(list
				(ImplicationLink (PredicateNode verb_instance) (PredicateNode verb))
				(EvaluationLink
					(PredicateNode verb_instance)
					(ListLink
						(VariableNode var_name)
					)
				)
			)
		))
		((string=? verb "_$qVar")
			(let ((var_name (choose-var-name)))
				(list
					(InheritanceLink (ConceptNode subj_instance) (ConceptNode subj_concept))
					(EvaluationLink
						(PredicateNode var_name)
						(ListLink
							(ConceptNode subj_instance)
						)
					)
				)
			)
		)
		(else (list
			(r2l-wordinst-concept subj_instance)
			(r2l-wordinst-Predicate verb_instance)
			(ImplicationLink (PredicateNode verb_instance) (PredicateNode verb))
			(InheritanceLink (ConceptNode subj_instance) (ConceptNode subj_concept))
			(EvaluationLink
				(PredicateNode verb_instance)
				(ListLink
					(ConceptNode subj_instance)
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
	(list (ImplicationLink (PredicateNode verb_ins) (PredicateNode verb))
	(InheritanceLink (ConceptNode subj_ins) (ConceptNode subj))
	(InheritanceLink (ConceptNode adj_ins) (ConceptNode adj))
	(EvaluationLink
		(PredicateNode verb_ins)
		(ListLink
			(InheritanceLink (ConceptNode subj_ins) (ConceptNode adj_ins))
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
	(let ((var_name (choose-var-name)))
		(list
			(InheritanceLink (ConceptNode subj_instance) (ConceptNode subj_concept))
			(InheritanceLink (ConceptNode obj_instance) (ConceptNode obj_concept))
			(EvaluationLink (PredicateNode "Truth Value")
				(ListLink (InheritanceLink (ConceptNode subj_instance)(ConceptNode obj_instance)))
				(VariableNode var_name)
			)
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
	(let ((var_name (choose-var-name)))
		(list
			(ImplicationLink (PredicateNode predicate_instance) (PredicateNode predicate_concept))
			(EvaluationLink (PredicateNode "Truth Value")
				(EvaluationLink (PredicateNode predicate_instance))
				(VariableNode var_name)
			)
		)
	)
)
; -----------------------------------------------------------------------
; Adjective and adverb rules
; (advmod is also called for adverbials, such as some prepositional phrases)
;
; Prepositional adverbial example: "The man sang with a powerful voice."
; Prepositional adjectival example: "The man with a powerful voice sang."
;
; -----------------------------------------------------------------------
(define (amod-rule concept instance adj adj_instance)
	(list (InheritanceLink  (ConceptNode adj_instance) (ConceptNode adj))
	(InheritanceLink  (ConceptNode instance) (ConceptNode concept))
	(InheritanceLink  (ConceptNode instance) (ConceptNode adj_instance)))
)

(define (advmod-rule verb instance adv adv_instance)
	(list (InheritanceLink  (ConceptNode adv_instance) (ConceptNode adv))
	(ImplicationLink  (PredicateNode instance) (PredicateNode verb))
	(InheritanceLink  (SatisfyingSetLink (PredicateNode instance)) (ConceptNode adv_instance))
	)
)

(define (adverbialpp-rule verb instance prep  prep_instance noun noun_instance)
    (list (InheritanceLink  (ConceptNode noun_instance) (ConceptNode noun))
    (ImplicationLink  (PredicateNode instance) (PredicateNode verb))
    (ImplicationLink  (PredicateNode prep_instance) (PredicateNode prep))
    (EvaluationLink
        (PredicateNode prep_instance)
        (ListLink
            (PredicateNode instance)
            (ConceptNode noun_instance))  
	))
)
;-----------------------------------------------------------------------
; prepositional phrase rule
;-----------------------------------------------------------------------
(define (pp-rule prep_concept prep_instance noun_concept noun_instance)
	(list (ImplicationLink (PredicateNode prep_instance) (PredicateNode prep_concept))
	(InheritanceLink (ConceptNode noun_instance) (ConceptNode noun_concept))
	(EvaluationLink
		(PredicateNode prep_instance)
		(ListLink (ConceptNode noun_instance))
	))
)

; -----------------------------------------------------------------------
; unary rules
; -----------------------------------------------------------------------
(define (entity-rule word word_instance) 
	(list (InheritanceLink (SpecificEntityNode word_instance) (ConceptNode word)))
)

(define (gender-rule word word_instance gender_type)
	(define concept_node (ConceptNode word))
	(cond ((string=? gender_type "feminine")
		(list (InheritanceLink (SpecificEntityNode word_instance) (ConceptNode "female"))
		(InheritanceLink (SpecificEntityNode word_instance) (ConceptNode word))
		))
	((string=? gender_type "masculine")
		(list (InheritanceLink (SpecificEntityNode word_instance) (ConceptNode "male"))
		(InheritanceLink (SpecificEntityNode word_instance) (ConceptNode word))
		))
        ((string=? gender_type "person")
		(list (InheritanceLink (SpecificEntityNode word_instance) (ConceptNode "unknown_gender"))
		(InheritanceLink (SpecificEntityNode word_instance) (ConceptNode word))
		))
	)
)

(define (tense-rule verb instance tense)
	(list (ImplicationLink (PredicateNode instance) (PredicateNode verb))
	(InheritanceLink (PredicateNode instance) (ConceptNode tense))
	)
)

(define (quantity-rule noun_concept noun_instance quantifier_concept quantifier_instance)
	(list (InheritanceLink (ConceptNode noun_instance) (ConceptNode noun_concept))
	(InheritanceLink (ConceptNode quantifier_instance) (ConceptNode quantifier_concept))
	(QuantityLink (ConceptNode noun_instance)(ConceptNode quantifier_instance))
	)
)	

;-----------------------------------------------------------------------------
; Determiner-question-word questions, e.g.
; "At what time . . . ", "For what reason . . .", "In what way . . .", "To what degree . . . ", "At what location . . ."
;-----------------------------------------------------------------------------
(define (q-det-rule noun_concept noun_instance verb verb_instance qtype)
	(list (InheritanceLink (VariableNode "$qVar") (ConceptNode noun_concept))
	(InheritanceLink (ConceptNode noun_instance) (VariableNode "$qVar"))
	(ImplicationLink (PredicateNode verb_instance) (PredicateNode verb))
	(cond ((string=? qtype "when")
		(AtTimeLink
			(VariableNode "$qVar")
			(PredicateNode verb_instance)
		))
	((string=? qtype "where")
		(EvaluationLink 
			(PredicateNode "AtPlace")
			(ListLink	
				(VariableNode "$qVar")
				(PredicateNode verb_instance)
			)
		)
	)
	((string=? qtype "why")
		(EvaluationLink 
			(PredicateNode "Because")
			(ListLink	
				(VariableNode "$qVar")
				(PredicateNode verb_instance)
			)
		)
	)
	((string=? qtype "how")
		(EvaluationLink 
			(PredicateNode "InManner")
			(ListLink	
				(VariableNode "$qVar")
				(PredicateNode verb_instance)
			)
		)
	)
	((string=? qtype "how_much")
		(EvaluationLink
			(PredicateNode "Degree")
			(ListLink
				(VariableNode "$qVar")
				(PredicateNode verb_instance)
			)
		)
	)
	))
)
(define (det-rule concept instance var_name determiner)
	(cond ((or (string=? determiner "those") (string=? determiner "these"))
		(list (ImplicationLink
			(MemberLink (VariableNode var_name) (ConceptNode instance))
			(InheritanceLink (VariableNode var_name) (ConceptNode concept))))
		)
		((or (string=? determiner "this") (string=? determiner "that"))
		(list (InheritanceLink (VariableNode var_name) (ConceptNode concept)))
		)
	)
)
(define (negative-rule verb instance)
	(list (ImplicationLink (PredicateNode instance) (NotLink (PredicateNode verb))))
)

(define (definite-rule word word_instance)
    (list (InheritanceLink (ConceptNode word_instance) (ConceptNode word))
    (EvaluationLink
        (PredicateNode "definite")
        (ListLink
	        (ConceptNode word_instance)
        )
    ))
)

; Example: "Maybe she eats lunch.", "Perhaps she is nice."
(define (maybe-rule word word_instance)
	(list
		(ImplicationLink (PredicateNode word_instance) (PredicateNode word))
		(EvaluationLink
			(PredicateNode "maybemarker")
			(ListLink
				(PredicateNode word_instance)
			)
		)
	)
)

; -----------------------------------------------------------------------
; misc rules
; -----------------------------------------------------------------------
(define (number-rule noun noun_instance num num_instance)
	(define noun_ins_concept (ConceptNode noun_instance))
	(list (InheritanceLink noun_ins_concept (ConceptNode noun))
	(InheritanceLink (NumberNode num_instance) (NumberNode num))
	(QuantityLink (ConceptNode noun_instance) (NumberNode num_instance))
	)
)

(define (about-rule verb verb_instance  noun noun_instance)
	(list (ImplicationLink (PredicateNode verb_instance) (PredicateNode verb))
	(InheritanceLink (ConceptNode noun_instance) (ConceptNode noun))
	(EvaluationLink
		(PredicateNode "about")
		(ListLink
			(PredicateNode verb_instance)
			(ConceptNode noun_instance)
		)
	))
) 

(define (nn-rule n1 n1_instance n2 n2_instance)
	(list (InheritanceLink (ConceptNode n1_instance) (ConceptNode n1))
	(InheritanceLink (ConceptNode n2_instance) (ConceptNode n2))
	(InheritanceLink (ConceptNode n1_instance) (ConceptNode n2_instance))
	)
)

(define (possessive-rule noun noun_instance word word_instance)
	(list (InheritanceLink (ConceptNode noun_instance) (ConceptNode noun))
	(InheritanceLink (ConceptNode word_instance) (ConceptNode word))
	(r2l-wordinst-concept noun_instance)
	(r2l-wordinst-concept word_instance)
	(EvaluationLink
		(PredicateNode "possession")
		(ListLink
			(ConceptNode noun_instance)
			(ConceptNode word_instance)
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
	(list (InheritanceLink (ConceptNode s_instance) (ConceptNode s))
	(InheritanceLink (ConceptNode o_instance) (ConceptNode o))
	(ImplicationLink (PredicateNode v1_instance) (PredicateNode v1))
	(ImplicationLink (PredicateNode v2_instance) (PredicateNode v2))
	(r2l-wordinst-concept s_instance)
	(r2l-wordinst-concept o_instance)
	(r2l-wordinst-Predicate v1_instance)
	(r2l-wordinst-Predicate v2_instance)
	(EvaluationLink
		(PredicateNode v1_instance)
		(ListLink
			(ConceptNode s_instance)
			(EvaluationLink
				(PredicateNode v2_instance)
				(ListLink
					(ConceptNode s_instance)
					(ConceptNode o_instance)
				)
			)
		)
	))
)
;
; Example: "She wants you to help us." -- assigns wrong subject in second clause
;
(define (to-do-rule-2 v1 v1_instance v2 v2_instance s1 s1_instance s2 s2_instance o o_instance) 
	(list (InheritanceLink (ConceptNode s1_instance) (ConceptNode s1))
	(InheritanceLink (ConceptNode s2_instance) (ConceptNode s2))
	(InheritanceLink (ConceptNode o_instance) (ConceptNode o))
	(ImplicationLink (PredicateNode v1_instance) (PredicateNode v1))
	(ImplicationLink (PredicateNode v2_instance) (PredicateNode v2))
	(r2l-wordinst-concept s1_instance)
	(r2l-wordinst-concept s2_instance)
	(r2l-wordinst-concept o_instance)
	(r2l-wordinst-Predicate v1_instance)
	(r2l-wordinst-Predicate v2_instance)
	(EvaluationLink
		(PredicateNode v1_instance)
		(ListLink
			(ConceptNode s1_instance)
			(EvaluationLink
				(PredicateNode v2_instance)
				(ListLink
					(ConceptNode s2_instance)
					(ConceptNode o_instance)
				)
			)
		)
	))
)
;
; Example: "She is nice to help with the project." -- the scheme output for this rule is a logical mess
;
(define (to-do-rule-3 v1 v1_instance v2 v2_instance v3 v3_instance)
    (list (InheritanceLink (ConceptNode v1_instance) (ConceptNode v1))
    (ImplicationLink (PredicateNode v2_instance) (PredicateNode v2))
    (InheritanceLink (ConceptNode v3_instance) (ConceptNode v3))
	(r2l-wordinst-concept v1_instance)
	(r2l-wordinst-concept v3_instance)
	(r2l-wordinst-Predicate v2_instance)
    (EvaluationLink
        (PredicateNode v2_instance)
        (ListLink ; does this ListLink make sense here? (by sebastianruder)
                (InheritanceLink (ConceptNode v3_instance) (ConceptNode v1_instance))
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
	(r2l-wordinst-concept v2_instance)
	(r2l-wordinst-Predicate v1_instance)
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
	(r2l-wordinst-concept subj_instance)
	(r2l-wordinst-Predicate verb1_instance)
	(r2l-wordinst-Predicate verb2_instance)
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
	(let ((var_name (choose-var-name)))
		(list
			(ImplicationLink (PredicateNode verb_instance) (PredicateNode verb))
			(r2l-wordinst-Predicate verb_instance)
			(EvaluationLink 
				(PredicateNode "AtPlace")
				(ListLink	
					(VariableNode var_name)
					(PredicateNode verb_instance)
				)
			)
		)
	)
)
;
; Examples: "Where is the party?", "Where will she be happy?" etc.
; 
(define (wherecop-Q-rule subj_concept subj_instance)
	(let ((var_name (choose-var-name)))
		(list
			(InheritanceLink (ConceptNode subj_instance) (ConceptNode subj_concept))
			(r2l-wordinst-concept subj_instance)
			(EvaluationLink 
				(PredicateNode "AtPlace")
				(ListLink	
					(VariableNode var_name)
					(ConceptNode subj_instance)
				)
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
	(let ((var_name (choose-var-name)))
		(list
			(ImplicationLink (PredicateNode verb_instance) (PredicateNode verb))
			(r2l-wordinst-Predicate verb_instance)
			(AtTimeLink
				(VariableNode var_name)
				(PredicateNode verb_instance)
			)	
		)
	)
)
;
; Example "When is the party?" etc.
;
(define (whencop-Q-rule subj_concept subj_instance)
	(let ((var_name (choose-var-name)))
		(list 	
			(InheritanceLink (ConceptNode subj_instance) (ConceptNode subj_concept))
			(r2l-wordinst-concept subj_instance)
			(AtTimeLink
				(VariableNode var_name)
				(ConceptNode subj_instance)
			)
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
	(let ((var_name (choose-var-name)))
		(list
			(ImplicationLink (PredicateNode verb_instance) (PredicateNode verb))
			(EvaluationLink 
				(PredicateNode "Because")
				(ListLink	
					(VariableNode var_name)
					(PredicateNode verb_instance)
				)
			)
		)
	)
)
;
; Example "Why are you such a fool?" etc.
;
(define (whycop-Q-rule subj_concept subj_instance)
	(let ((var_name (choose-var-name)))
		(list
			(InheritanceLink (ConceptNode subj_instance) (ConceptNode subj_concept))
			(EvaluationLink 
				(PredicateNode "Because")
				(ListLink	
					(VariableNode var_name)
					(ConceptNode subj_instance)
				)
			)
		)
	)
)
;----------------------------------------------------------------------------------------------------------
; How adverbial (manner) questions
;----------------------------------------------------------------------------------------------------------
; 
; Example: "How did you sleep?" etc.
;
(define (how-rule verb verb_instance)
	(let ((var_name (choose-var-name)))
		(list
			(ImplicationLink (PredicateNode verb_instance) (PredicateNode verb))
			(EvaluationLink 
				(PredicateNode "InManner")
				(ListLink	
					(VariableNode var_name)
					(PredicateNode verb_instance)
				)
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
	(let ((var_name (choose-var-name)))
		(list 	
			(InheritanceLink (ConceptNode subj_instance) (ConceptNode subj_concept))
			(InheritanceLink (ConceptNode subj_instance) (VariableNode var_name))
		)
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
	(let ((var_name (choose-var-name)))
		(list
			(InheritanceLink (ConceptNode instance) (ConceptNode concept))
			(QuantityLink	
				(ConceptNode instance)
				(VariableNode var_name)
			)
		)
	)
)
;
; Example: "How fast does it go?"
;
(define (howdegQ-rule concept instance)
	(let ((var_name (choose-var-name)))
		(list
			(InheritanceLink (ConceptNode instance) (ConceptNode concept))
			(EvaluationLink
				(PredicateNode "DegreeLink")
				(ListLink
					(VariableNode var_name)
					(ConceptNode instance)
				)
			)
		)
	)
)
;
;-----------------------------------------------------------------------------------------------
; CHOICE-TYPE QUESTIONS (also known as question-determiners) -- what and which
;
; NB: I don't think these rules could be made to plug into the setence templates because of the
; SatisfyingSet logic . . .
;
; Therefore still need to write SV, prep, to-do, and to-be versions of these if we want that functionality -- 
; or do them without the satisfying set logic, so they can just plug into the templates . . .
;
;-----------------------------------------------------------------------------------------------
;
; Example: "Which girl do you like?" "What book are you reading?"
;
(define (whichobjQ-rule obj_concept obj_instance verb verb_instance subj_concept subj_instance)
	(let ((var_name (choose-var-name)))
		(list
			(ImplicationLink (PredicateNode verb_instance) (PredicateNode verb))
			(InheritanceLink (ConceptNode subj_instance) (ConceptNode subj_concept))
			(InheritanceLink (ConceptNode obj_instance) (ConceptNode obj_concept))
			(InheritanceLink (VariableNode var_name) (ConceptNode obj_instance))
			(SatisfyingSetLink
				(VariableNode var_name) 	
				(EvaluationLink (PredicateNode verb_instance)
					(ListLink
						(ConceptNode subj_instance)
						(VariableNode var_name)
					)
				)
			)
		)
	)
)
;
; Example: "Which girl likes you?" "What fool said that?"
;
(define (whichsubjSVOQ-rule subj_concept subj_instance verb verb_instance obj_concept obj_instance)
	(let ((var_name (choose-var-name)))
		(list
			(ImplicationLink (PredicateNode verb_instance) (PredicateNode verb))
			(InheritanceLink (ConceptNode subj_instance) (ConceptNode subj_concept))
			(ImplicationLink (PredicateNode obj_instance) (PredicateNode obj_concept))
			(InheritanceLink (VariableNode var_name) (ConceptNode subj_instance))
			(SatisfyingSetLink
				(VariableNode var_name)
				(EvaluationLink (PredicateNode verb_instance)
					(ListLink
						(VariableNode var_name)
						(ConceptNode obj_instance)
					)
				)
			)
		)
	)
)
;
; Example: "To which address did you send the email?"
;
(define (whichiobjQ-rule subj_concept subj_instance verb verb_instance obj_concept obj_instance iobj_concept iobj_instance)
	(let ((var_name (choose-var-name)))
		(list
			(ImplicationLink (PredicateNode verb_instance) (PredicateNode verb))
			(InheritanceLink (ConceptNode subj_instance) (ConceptNode subj_concept))
			(InheritanceLink (ConceptNode obj_instance) (ConceptNode obj_concept))
			(InheritanceLink (ConceptNode iobj_instance) (ConceptNode iobj_concept))
			(InheritanceLink (VariableNode var_name) (ConceptNode iobj_instance))
			(SatisfyingSetLink
				(VariableNode var_name)
				(EvaluationLink (PredicateNode verb_instance)
					(ListLink
						(ConceptNode subj_instance)
						(ConceptNode obj_instance)
						(VariableNode var_name)
					)
				)
			)
		)
	)
)
;
; Example: "Which girl is crazy?"
;
(define (whichpredadjQ-rule subj_concept subj_instance pred_concept pred_instance)
	(let ((var_name (choose-var-name)))
		(list
			(InheritanceLink (ConceptNode subj_instance) (ConceptNode subj_concept))
			(ImplicationLink (PredicateNode pred_instance) (PredicateNode pred_concept))	
			(InheritanceLink (VariableNode var_name) (ConceptNode subj_instance) )
			(SatisfyingSetLink
				; (ScopeLink ;; not yet supported !? Huhh ??
					(VariableNode var_name)
					(EvaluationLink
						(PredicateNode pred_instance)
						(ListLink
							(VariableNode var_name))
					)
				; )
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
    (list (ImplicationLink (PredicateNode verb_instance) (PredicateNode verb))
    (InheritanceLink (ConceptNode obj_instance) (ConceptNode obj))
    (InheritanceLink (ConceptNode passive_obj_instance) (ConceptNode passive_obj))
    (EvaluationLink
            (PredicateNode verb_instance)
            (ListLink
                    (ConceptNode passive_obj_instance)
                    (ConceptNode obj_instance)
            )
    ))
)

; Example: "The books are published."
(define (passive-rule2 verb verb_instance obj obj_instance)
	(let ((var_name (choose-var-name)))
		(list
			(ImplicationLink (PredicateNode verb_instance) (PredicateNode verb))
			(InheritanceLink (ConceptNode obj_instance) (ConceptNode obj))
			(EvaluationLink
				(PredicateNode verb_instance)
					(ListLink
						(VariableNode var_name)
						(ConceptNode obj_instance)
					)
			)
		)
	)
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

;-----------------------------------------------------------------------
; complement clauses
;-----------------------------------------------------------------------
(define (complement-rule comp_concept comp_instance pred_concept pred_instance)
	(list 
		(ImplicationLink (PredicateNode comp_instance) (PredicateNode comp_concept))
		(ImplicationLink (PredicateNode pred_instance) (PredicateNode pred_concept))
		(EvaluationLink
			(PredicateNode comp_instance)
			(ListLink
				(PredicateNode pred_instance)
			)
		)
	)
)

(define (compmod-rule comp_concept comp_instance pred_concept pred_instance)
	(list 
		(ImplicationLink (PredicateNode comp_instance) (PredicateNode comp_concept))
		(ImplicationLink (PredicateNode pred_instance) (PredicateNode pred_concept))
		(EvaluationLink
			(PredicateNode "InManner")
			(ListLink
				(PredicateNode pred_instance)
				(ConceptNode comp_instance)
			)
		)
	)
)

(define (because-rule comp_concept comp_instance pred_concept pred_instance)
	(list 
		(ImplicationLink (PredicateNode comp_instance) (PredicateNode comp_concept))
		(ImplicationLink (PredicateNode pred_instance) (PredicateNode pred_concept))
		(EvaluationLink
			(PredicateNode "Because")
			(ListLink
				(PredicateNode pred_instance)
				(ConceptNode comp_instance)
			)
		)
	)
)

(define (attime-rule comp_concept comp_instance pred_concept pred_instance)
	(list 
		(ImplicationLink (PredicateNode comp_instance) (PredicateNode comp_concept))
		(ImplicationLink (PredicateNode pred_instance) (PredicateNode pred_concept))
		(AtTimeLink)
			(PredicateNode pred_instance)
			(ConceptNode comp_instance)
	)
)

(define (rep-rule comp_concept comp_instance pred_concept pred_instance)
	(list 
		(ImplicationLink (PredicateNode comp_instance) (PredicateNode comp_concept))
		(ImplicationLink (PredicateNode pred_instance) (PredicateNode pred_concept))
		(EvaluationLink
			(PredicateNode pred_instance)
			(ListLink
				(ConceptNode comp_instance)
			)
		)
	)
)

; -----------------------------------------------------------------------
; that rule for creating thatmarker
; -----------------------------------------------------------------------
; Example: "I think that dogs can fly.", "He is glad that she won.", "He ran so quickly that he flew."
; A that-rule for "object clause", "content clause", "complement clause", etc, but not "adjective clause"
(define (that-rule main main_instance sub sub_instance)
	(list
		(ImplicationLink (PredicateNode main_instance) (PredicateNode main))
		(ImplicationLink (PredicateNode sub_instance) (PredicateNode sub))
		(EvaluationLink
			(PredicateNode "that")
			(ListLink
				(PredicateNode main_instance)
				(PredicateNode sub_instance)
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
        (if (or (string=? $y_pos "verb") (string=? $y_pos "adj"))
            (PredicateNode $y_instance)
            (ConceptNode $y_instance)
        )
    )
    (list
        (EvaluationLink
            (PredicateNode $before_or_after)
            (ListLink (PredicateNode $x_instance) y-node)
        )
    )
)

; Examples: "I had dinner at 6 pm", "I went to sleep at 1 am"
(define (time-rule $hour $period $v_instance)
    (define time-node
        (if (string=? $period "am")
            (TimeNode $hour)
            (TimeNode (number->string (+ (string->number $hour) 12)))
        )
    )
    (list (AtTimeLink time-node (PredicateNode $v_instance)))
)


; -----------------------------------------------------------------------
; functions without R2L rule, not working, unneeded, etc
; -----------------------------------------------------------------------
(define (comparative-rule w1 w1_instance w2 w2_instance adj adj_instance)
	(list (InheritanceLink (ConceptNode adj_instance) (ConceptNode adj))
	(InheritanceLink (ConceptNode w1_instance) (ConceptNode w1))
	(InheritanceLink (ConceptNode w2_instance) (ConceptNode w2))
	(TruthValueGreaterThanLink
		(InheritanceLink (ConceptNode w1_instance) (ConceptNode adj_instance))
		(InheritanceLink (ConceptNode w2_instance) (ConceptNode adj_instance))
	))
)

(define (on-rule w1 w1_instance w2 w2_instance)
	(list (InheritanceLink (ConceptNode w1_instance) (ConceptNode w1))
	(InheritanceLink (ConceptNode w2_instance) (ConceptNode w2))
	(EvaluationLink
		(PredicateNode "on")
		(ListLink
			(ConceptNode w1_instance)
			(ConceptNode w2_instance)
		)
	))
)


;
;
;(define (which-rule antecedent  antecedent_instance  verb  verb_instance)
;	(list (InheritanceLink (ConceptNode antecedent_instance) (ConceptNode antecedent))
;	(ImplicationLink (PredicateNode verb_instance) (PredicateNode verb))
;        (EvaluationLink
;		(PredicateNode "whichmarker")
;		(ListLink
;			(ConceptNode antecedent_instance)
;			(PredicateNode verb_instance)
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
;	(list (InheritanceLink (ConceptNode subj_instance) (ConceptNode subj_concept))
;	(InheritanceLink (ConceptNode obj_instance) (ConceptNode obj_concept))
;	(ImplicationLink (PredicateNode predprep_instance) (PredicateNode predprep_concept))
;	(EvaluationLink (PredicateNode "Truth Value")
;		(EvaluationLink (PredicateNode predprep_instance)
;			(ListLink (ConceptNode subj_instance)(ConceptNode obj_instance)))
;			(VariableNode "$var1"))
;))
;
;-------------------------------------------------------------------------------------------------------
; 
;
;(define (SV-ynQ-rule subj_concept subj_instance verb verb_instance)
;	(list (ImplicationLink (PredicateNode verb_instance) (PredicateNode verb))
;	(InheritanceLink (ConceptNode subj_instance) (ConceptNode subj_concept))
;	(EvaluationLink (PredicateNode "Truth-Value")
;		(EvaluationLink (PredicateNode verb_instance)
;			(ListLink (ConceptNode subj_instance)))
;		(VariableNode "$qVar"))
;))
;

;
; NB: 	This rule applies incorrectly to many SVIO sentences where the IO is interpreted by relex as 
;	an extra prepositional phrase rather than an IO.
;
;(define (SVO-ynQ-rule subj_concept subj_instance verb verb_instance obj_concept obj_instance)
;	(list (ImplicationLink (PredicateNode verb_instance) (PredicateNode verb))
;	(InheritanceLink (ConceptNode subj_instance) (ConceptNode subj_concept))
;	(InheritanceLink (ConceptNode obj_instance) (ConceptNode obj_concept))
;	(EvaluationLink (PredicateNode "Truth-Value")
;		(EvaluationLink (PredicateNode verb_instance)
;			(ListLink 
;				(ConceptNode subj_instance)
;				(ConceptNode obj_instance)))
;		(VariableNode "$qVar"))
;))
;

;
;(define (SVIO-ynQ-rule subj_concept subj_instance verb verb_instance obj_concept obj_instance iobj_concept iobj_instance)
;	(list (ImplicationLink (PredicateNode verb_instance) (PredicateNode verb))
;	(InheritanceLink (ConceptNode subj_instance) (ConceptNode subj_concept))
;	(InheritanceLink (ConceptNode obj_instance) (ConceptNode obj_concept))
;	(InheritanceLink (ConceptNode iobj_instance) (ConceptNode iobj_concept))
;	(EvaluationLink (PredicateNode "Truth-Value")
;		(EvaluationLink (PredicateNode verb_instance)
;			(ListLink			
;				(ConceptNode subj_instance)
;				(ConceptNode obj_instance)
;				(ConceptNode iobj_instance)))
;		(VariableNode "$qVar"))
;))
;
;------------------------------------------------------------------------------------------
; who/what subject questions (who and what may be treated identically as far as I can tell).
;------------------------------------------------------------------------------------------
;
; Examples: "Who farted?", "What happened?"
;
;(define (whowhatsubj-SV-Q-rule verb verb_instance)
;	(list (ImplicationLink (PredicateNode verb_instance) (PredicateNode verb))
;	(EvaluationLink
;		(PredicateNode verb_instance)
;		(ListLink
;			(VariableNode "_$qVar")))
;))
;
; Examples: "What killed him?", "Who ate the pizza?"
;
;(define (whowhatsubj-SVO-Q-rule verb verb_instance obj_concept obj_instance)
;	(list (ImplicationLink (PredicateNode verb_instance) (PredicateNode verb))
;	(InheritanceLink (ConceptNode obj_instance) (ConceptNode obj_concept))
;	(EvaluationLink
;		(PredicateNode verb_instance)
;		(ListLink
;			(VariableNode "_$qVar")
;			(ConceptNode obj_instance)))
;))
;
; Examples: "What gave you that idea?", "Who told you that?"
;
;(define (whowhatsubj-SVIO-Q-rule verb verb_instance obj_concept obj_instance iobj_concept iobj_instance)
;	(list (ImplicationLink (PredicateNode verb_instance) (PredicateNode verb))
;	(InheritanceLink (ConceptNode obj_instance) (ConceptNode obj_concept))
;	(InheritanceLink (ConceptNode iobj_instance) (ConceptNode iobj_concept))
;	(EvaluationLink
;		(PredicateNode verb_instance)
;		(ListLink
;			(VariableNode "_$qVar")
;			(ConceptNode obj_instance)
;			(ConceptNode iobj_instance)))
;))
;
; Examples: "What is for dinner?", Who's on first?"
;
;(define (whowhatpsubj-Q-rule prep prep_instance obj_concept obj_instance)
;	(list (ImplicationLink (PredicateNode prep_instance) (PredicateNode prep))
;	(InheritanceLink (ConceptNode obj_instance) (ConceptNode obj_concept))
;	(EvaluationLink
;		(PredicateNode prep_instance)
;		(ListLink
;			(VariableNode "_$qVar")
;			(ConceptNode obj_instance)))
;))
;
; Example: "Who are you?"
;
;(define (whocop-Q-rule subj_concept subj_instance)
;	(list (InheritanceLink (ConceptNode subj_instance) (ConceptNode subj_concept))
;	(InheritanceLink (ConceptNode subj_instance) (VariableNode "_$qVar"))
;))
;
;----------------------------------------------------------------------------------------
; who/what object and who/what indirect object questions
;----------------------------------------------------------------------------------------
;
; Examples: "Who do you love?", "What do you think?"
;
;(define (whowhatobj-Q-rule subj_concept subj_instance verb verb_instance)
;	(list (ImplicationLink (PredicateNode verb_instance) (PredicateNode verb))
;	(InheritanceLink (ConceptNode subj_instance) (ConceptNode subj_concept))
;	(EvaluationLink
;		(PredicateNode verb_instance)
;		(ListLink
;			(ConceptNode subj_instance)
;			(VariableNode "_$qVar")))		
;))
;
; Examples: "To whom did you sell the children?"
;
;(define (whowhatiobj-Q-rule subj_concept subj_instance verb verb_instance obj_concept obj_instance)
;	(list (ImplicationLink (PredicateNode verb_instance) (PredicateNode verb))
;	(InheritanceLink (ConceptNode subj_instance) (ConceptNode subj_concept))
;	(InheritanceLink (ConceptNode obj_instance) (ConceptNode obj_concept))
;	(EvaluationLink
;		(PredicateNode verb_instance)
;		(ListLink
;			(ConceptNode subj_instance)
;			(ConceptNode obj_instance)
;			(VariableNode "_$qVar")))
;))
;
;----------------------------------------------------------------------------------------------------------
;
; Example: 
;
;(define (when-SVOQ-rule subj_concept subj_instance verb verb_instance obj_concept obj_instance)
;	(list (ImplicationLink (PredicateNode verb_instance) (PredicateNode verb))
;	(InheritanceLink (ConceptNode subj_instance) (ConceptNode subj_concept))
;	(InheritanceLink (ConceptNode obj_instance) (ConceptNode obj_concept))
;	(AtTimeLink
;		(VariableNode "$qVar")
;		(EvaluationLink
;			(PredicateNode verb_instance)
;				(ListLink
;					(ConceptNode subj_instance)
;					(ConceptNode obj_instance))))
;))
; 
; Example: 
;
;(define (when-SVIOQ-rule subj_concept subj_instance verb verb_instance obj_concept obj_instance iobj_concept iobj_instance)
;	(list (ImplicationLink (PredicateNode verb_instance) (PredicateNode verb))
;	(InheritanceLink (ConceptNode subj_instance) (ConceptNode subj_concept))
;	(InheritanceLink (ConceptNode obj_instance) (ConceptNode obj_concept))
;	(InheritanceLink (ConceptNode iobj_instance) (ConceptNode iobj_concept))
;	(AtTimeLink 
;		(VariableNode "$qVar")
;		(EvaluationLink
;			(PredicateNode verb_instance)
;				(ListLink
;					(ConceptNode subj_instance)
;					(ConceptNode obj_instance)
;					(ConceptNode iobj_instance))))
;))
;------------------------------------------------------------------------------------------------------------
;
; Example: " 
;
;(define (why-SVOQ-rule subj_concept subj_instance verb verb_instance obj_concept obj_instance)
;	(list (ImplicationLink (PredicateNode verb_instance) (PredicateNode verb))
;	(InheritanceLink (ConceptNode subj_instance) (ConceptNode subj_concept))
;	(InheritanceLink (ConceptNode obj_instance) (ConceptNode obj_concept))
;	(EvaluationLink 
;		(PredicateNode "Because")
;			(ListLink	
;				(VariableNode "$qVar")
;				(EvaluationLink
;					(PredicateNode verb_instance)
;						(ListLink
;							(ConceptNode subj_instance)
;							(ConceptNode obj_instance)))))
;))
;
; Example: "Why did you give him the money?"
;
;(define (why-SVIOQ-rule subj_concept subj_instance verb verb_instance obj_concept obj_instance iobj_concept iobj_instance)
;	(list (ImplicationLink (PredicateNode verb_instance) (PredicateNode verb))
;	(InheritanceLink (ConceptNode subj_instance) (ConceptNode subj_concept))
;	(InheritanceLink (ConceptNode obj_instance) (ConceptNode obj_concept))
;	(InheritanceLink (ConceptNode iobj_instance) (ConceptNode iobj_concept))
;	(EvaluationLink 
;		(PredicateNode "Because")
;			(ListLink	
;				(VariableNode "$qVar")
;				(EvaluationLink
;					(PredicateNode verb_instance)
;						(ListLink
;							(ConceptNode subj_instance)
;							(ConceptNode obj_instance)
;							(ConceptNode iobj_instance)))))
;))
;
; Example: "Why are you so stupid?"
;
;(define (why-Q-rule subj_concept subj_instance predicative_concept predicative_instance)
;	(list (InheritanceLink (ConceptNode subj_instance) (ConceptNode subj_concept))
;	(InheritanceLink (ConceptNode predicative_instance) (ConceptNode predicative_concept))
;	(EvaluationLink 
;		(PredicateNode "Because")
;			(ListLink	
;				(VariableNode "$qVar")
;				(EvaluationLink (PredicateNode predicative_instance) 
;					(ListLink (ConceptNode subj_instance)))))
;))
;
;------------------------------------------------------------------------------------------------------------
;
; Example: "How did you like the movie?"
;
;(define (how-SVOQ-rule subj_concept subj_instance verb verb_instance obj_concept obj_instance)
;	(list (ImplicationLink (PredicateNode verb_instance) (PredicateNode verb))
;	(InheritanceLink (ConceptNode subj_instance) (ConceptNode subj_concept))
;	(InheritanceLink (ConceptNode obj_instance) (ConceptNode obj_concept))
;	(EvaluationLink 
;		(PredicateNode "InManner")
;			(ListLink	
;				(VariableNode "$qVar")
;				(EvaluationLink
;					(PredicateNode verb_instance)
;						(ListLink
;							(ConceptNode subj_instance)
;							(ConceptNode obj_instance)))))
;))
; 
; Example: "How did you send him the message?"
;
;(define (how-SVIOQ-rule subj_concept subj_instance verb verb_instance obj_concept obj_instance iobj_concept iobj_instance)
;	(list (ImplicationLink (PredicateNode verb_instance) (PredicateNode verb))
;	(InheritanceLink (ConceptNode subj_instance) (ConceptNode subj_concept))
;	(InheritanceLink (ConceptNode obj_instance) (ConceptNode obj_concept))
;	(InheritanceLink (ConceptNode iobj_instance) (ConceptNode iobj_concept))
;	(EvaluationLink 
;		(PredicateNode "InManner")
;			(ListLink	
;				(VariableNode "$qVar")
;				(EvaluationLink
;					(PredicateNode verb_instance)
;						(ListLink
;							(ConceptNode subj_instance)
;							(ConceptNode obj_instance)
;							(ConceptNode iobj_instance)))))
;))
;----------------------------------------------------------------------------------------------------------------------------
;(define (SVP-rule subj  subj_instance  predicative  predicative_instance)
;	(list (InheritanceLink (ConceptNode predicative_instance) (ConceptNode predicative))
;	(InheritanceLink (ConceptNode subj_instance) (ConceptNode subj))
;	(InheritanceLink (ConceptNode subj_instance) (ConceptNode predicative_instance))
;	)
;
