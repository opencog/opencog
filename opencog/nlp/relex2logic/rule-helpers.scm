(use-modules (srfi srfi-1))
(use-modules (opencog))

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
; Returns a list of WordInstanceNodes from 'parse-node', which have
; a LemmaLink with a WordNode named 'word'.
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
; Returns the word-instance name when its word-lemma, word-index and
; parse-node is inputed. It also checks whether an atom name is a
; word-instance name for the given parse-node and word lemma and
; returns the word-instance name.
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
; TODO: do the same for non-instanced node
(define (r2l-wordinst-concept inst-name)
	(ReferenceLink
		(ConceptNode inst-name)
		(WordInstanceNode inst-name)
	)
)
; -----------------------------------------------------------------------
; Connect WordInstanceNode With PredicateNode
; TODO: do the same for non-instanced node
(define (r2l-wordinst-predicate inst-name)
	(ReferenceLink
		(PredicateNode inst-name)
		(WordInstanceNode inst-name)
	)
)
; ======================================================
; Actual helper functions for creating OpenCog atoms
; ======================================================
; The atoms added to the atomspace (by the helper functions below) must
; be returned in a list or should be a single atom so as to simplify
; post-processing and NLG tasks.
;=======================================================
;
;
; ======================================================
; Predicate-Argument templates
; Conditional substitution of query variables replaces separate query-rules
;=======================================================
;
; Subject-Copula-Object
;
; Declarative Examples: "Socrates is a man", "Cats are animals",
;                       "Trees are plants"
;
; Question Examples: "What is Socrates?" (object query) "Who is the
; teacher?" (object query) "Who is a man?" (subject query (rare))
;
; Relex processes all of these questions as subject-queries, which
; reverses the logic of inheritance they should have, hence the
; rule below has the "object" inheriting the variable in order to
; compensate, since there is no way syntactic way to distinguish
; subject from object queries for these sentences, correcting the
; logic of the rare true subject queries will have to be left to
; general intelligence.
;
(define-public (be-inheritance-rule subj_lemma subj_inst obj_lemma obj_inst)
	(define subj_concept (cog-name subj_lemma))
	(define subj_instance (cog-name subj_inst))
	(define obj_concept (cog-name obj_lemma))
	(define obj_instance (cog-name obj_inst))

	(cond
		((string=? subj_concept "_$qVar")
			(let ((var_name (choose-var-name)))
				(ListLink
					(r2l-wordinst-concept obj_instance)
					(Inheritance (Concept obj_instance) (Concept obj_concept))
					(Inheritance (Concept obj_instance) (Variable var_name))
				)
			)
		)
		((string=? obj_concept "_$qVar")
			(let ((var_name (choose-var-name)))
				(ListLink
					(r2l-wordinst-concept subj_instance)
					(Inheritance (Concept subj_instance) (Concept subj_concept))
					(Inheritance (Concept subj_instance) (Variable var_name))
				)
			)
		)
		(else
			(ListLink
				(r2l-wordinst-concept subj_instance)
				(r2l-wordinst-concept obj_instance)
				(Inheritance (Concept subj_instance) (Concept subj_concept))
				(Inheritance (Concept obj_instance) (Concept obj_concept))
				(Inheritance (Concept subj_instance) (Concept obj_instance))
			)
		)
	)
)
;------------------------------------------------
;
; SVIO
;
; Declarative:          "Bill gave Mary the pox."
;                       "Bill sold his children to the gypsies."
; Questioned subject: 	"Who told you that bullshit?"
;                       "Who told that story to the police?"
;                       "What gives you that idea?"
;                       "What gave that idea to the police?"
; Questioned object: 	"What did you tell the fuzz?"
;                       "What did you give to Mary?"
;                       "Who did you give the slavers?"
;                       "Who did you sell to the slavers?"
; Questioned indirect object: 	"To whom did you sell the children?"
;                       "To what do we owe the pleasure?"
;                       "Who did you sell the children to?"
;
; NB: The first two iobj q-types work, although the relex output for
; the second one relies on a to(), instead of iobj() the third one
; still doesn't work because the relex output for it is a disaster
; and requires LG changes that I haven't gotten to work yet (10-22-14 AN)
;
(define-public (SVIO-rule subj_lemma subj_inst
         verb_lemma verb_inst
         obj_lemma obj_inst
         iobj_lemma iobj_inst)
	(define subj_concept (cog-name subj_lemma))
	(define subj_instance (cog-name subj_inst))
	(define verb (cog-name verb_lemma))
	(define verb_instance (cog-name verb_inst))
	(define obj_concept (cog-name obj_lemma))
	(define obj_instance (cog-name obj_inst))
	(define iobj_concept (cog-name iobj_lemma))
	(define iobj_instance (cog-name iobj_inst))

	(cond ((string=? subj_concept "_$qVar")
		(let ((var_name (choose-var-name)))
			(ListLink
				(ImplicationLink (PredicateNode verb_instance) (PredicateNode verb))
				(InheritanceLink (ConceptNode obj_instance) (ConceptNode obj_concept))
				(InheritanceLink (ConceptNode iobj_instance) (ConceptNode iobj_concept))
				(r2l-wordinst-predicate verb_instance)
				(r2l-wordinst-concept obj_instance)
				(r2l-wordinst-concept iobj_instance)
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
				(ListLink
					(ImplicationLink (PredicateNode verb_instance) (PredicateNode verb))
					(InheritanceLink (ConceptNode subj_instance) (ConceptNode subj_concept))
					(InheritanceLink (ConceptNode iobj_instance) (ConceptNode iobj_concept))
					(r2l-wordinst-predicate verb_instance)
					(r2l-wordinst-concept subj_instance)
					(r2l-wordinst-concept iobj_instance)
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
				(ListLink
					(ImplicationLink (PredicateNode verb_instance) (PredicateNode verb))
					(InheritanceLink (ConceptNode subj_instance) (ConceptNode subj_concept))
					(InheritanceLink (ConceptNode obj_instance) (ConceptNode obj_concept))
					(r2l-wordinst-predicate verb_instance)
					(r2l-wordinst-concept subj_instance)
					(r2l-wordinst-concept obj_instance)
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
		(else
			(ListLink
				(ImplicationLink (PredicateNode verb_instance) (PredicateNode verb))
				(InheritanceLink (ConceptNode subj_instance) (ConceptNode subj_concept))
				(InheritanceLink (ConceptNode obj_instance) (ConceptNode obj_concept))
				(InheritanceLink (ConceptNode iobj_instance) (ConceptNode iobj_concept))
				(r2l-wordinst-predicate verb_instance)
				(r2l-wordinst-concept subj_instance)
				(r2l-wordinst-concept iobj_instance)
				(r2l-wordinst-concept obj_instance)
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
;---------------------------------------------------------------
;
; SVO
;
; Declarative: 		"Computers can bite me."
; Prepositional:		"The book is on the table."
; Subject query: 		"What bothers you?" "Who programmed you?" "What is on the table?"
; Object query: 		"What did you say?" "Who do you love?"
; Prepositional subject query: 	"What is for dinner?", Who's on first?"
;
(define-public (SVO-rule subj_lemma subj_inst verb_lemma verb_inst obj_lemma obj_inst)
	(define subj_concept (cog-name subj_lemma))
	(define subj_instance (cog-name subj_inst))
	(define verb (cog-name verb_lemma))
	(define verb_instance (cog-name verb_inst))
	(define obj_concept (cog-name obj_lemma))
	(define obj_instance (cog-name obj_inst))

	(cond ((string=? subj_concept "_$qVar")
		(let ((var_name (choose-var-name)))
			(ListLink
				(ImplicationLink (PredicateNode verb_instance) (PredicateNode verb))
				(InheritanceLink (ConceptNode obj_instance) (ConceptNode obj_concept))
				(r2l-wordinst-predicate verb_instance)
				(r2l-wordinst-concept obj_instance)
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
				(ListLink
					(ImplicationLink (PredicateNode verb_instance) (PredicateNode verb))
					(InheritanceLink (ConceptNode subj_instance) (ConceptNode subj_concept))
					(r2l-wordinst-predicate verb_instance)
					(r2l-wordinst-concept subj_instance)
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
		(else
			(ListLink
				(ImplicationLink
					(PredicateNode verb_instance) (PredicateNode verb))
				(InheritanceLink
					(ConceptNode subj_instance) (ConceptNode subj_concept))
				(InheritanceLink
					(ConceptNode obj_instance) (ConceptNode obj_concept))
				(r2l-wordinst-predicate verb_instance)
				(r2l-wordinst-concept subj_instance)
				(r2l-wordinst-concept obj_instance)
				(EvaluationLink
					(PredicateNode verb_instance)
					(ListLink
						(ConceptNode subj_instance)
						(ConceptNode obj_instance)))
			)
	))
)
;--------------------------------------------------------------
;
; SV
;
; Declarative verb:      "Computers suck."
; Declarative predicative adjective:   "I am happy."
; Subject query:         "Who farted?" "What is happening?"
;                        "Who is correct?" "What is right?"
; Verb query:            "What are you doing?"
;
(define-public (SV-rule subj-lemma subj-inst verb-lemma verb-inst)
	(define subj_concept  (cog-name subj-lemma))
	(define subj_instance (cog-name subj-inst))
	(define verb          (cog-name verb-lemma))
	(define verb_instance (cog-name verb-inst))

	(cond
		((string=? subj_concept "_$qVar")
			(let ((var_name (choose-var-name)))
				(ListLink
					(Implication (Predicate verb_instance) (Predicate verb))
					(r2l-wordinst-predicate verb_instance)
					(EvaluationLink
						(PredicateNode verb_instance)
						(ListLink (VariableNode var_name)))
				)
		))
		((string=? verb "_$qVar")
			(let ((var_name (choose-var-name)))
				(ListLink
					(Inheritance (Concept subj_instance) (Concept subj_concept))
					(r2l-wordinst-concept subj_instance)
					(EvaluationLink
						(VariableNode var_name)
						(ListLink (ConceptNode subj_instance)))
				)
		))
		(else
			(ListLink
				(r2l-wordinst-concept subj_instance)
				(r2l-wordinst-predicate verb_instance)
				(Implication (Predicate verb_instance) (Predicate verb))
				(Inheritance (Concept subj_instance) (Concept subj_concept))
				(EvaluationLink
					(PredicateNode verb_instance)
					(ListLink (ConceptNode subj_instance)))
			)
		)
	)
)
;------------------------------------------------------------------------
;
; "To-be" rule
; Declarative: "He seems happy." "He seems to be happy."
;
; NB: This rule is not getting called enough; with the verb "appears"
; it only gets called on the second parse. With the verb "looks" not
; at all.
;
(define-public (to-be-rule verb_lemma verb_inst adj_lemma adj_inst subj_lemma subj_inst)
	(define verb (cog-name verb_lemma))
	(define verb_ins (cog-name verb_inst))
	(define adj (cog-name adj_lemma))
	(define adj_ins (cog-name adj_inst))
	(define subj (cog-name subj_lemma))
	(define subj_ins (cog-name subj_inst))
	(ListLink
		(ImplicationLink (PredicateNode verb_ins) (PredicateNode verb))
		(InheritanceLink (ConceptNode subj_ins) (ConceptNode subj))
		(InheritanceLink (ConceptNode adj_ins) (ConceptNode adj))
		(r2l-wordinst-concept subj_ins)
		(r2l-wordinst-predicate verb_ins)
		(r2l-wordinst-concept adj_ins)
		(EvaluationLink
			(PredicateNode verb_ins)
			(ListLink
				(InheritanceLink (ConceptNode subj_ins) (ConceptNode adj_ins))
			)
		))
)
;----------------------------------------------------------------
; Yes / no question rules
;----------------------------------------------------------------
;
; Copula example: "Are you the one?"
;
(define-public (cop-ynQ-rule subj_concept subj_instance obj_concept obj_instance)
	(let ((var_name (choose-var-name)))
		(ListLink
			(InheritanceLink (ConceptNode subj_instance) (ConceptNode subj_concept))
			(InheritanceLink (ConceptNode obj_instance) (ConceptNode obj_concept))
			(r2l-wordinst-concept subj_instance)
			(r2l-wordinst-concept obj_instance)
			(EvaluationLink (DefinedLinguisticPredicateNode "Truth Value")
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
; SV examples: 				"Have you slept?", "Will you sleep?", "Did you sleep?"
; SVO example: 				"Did you eat the leftover baba-ganoush?"
; SVIO examples: 				"Did you give her the money?", "Did you give the money to her?"
;
; NB: this rule also allows the system to handle all intonation-only versions of these questions (i.e. "The book is under the table?" etc.)
;
(define-public (pred-ynQ-rule predicate_concept predicate_instance)
	(let ((var_name (choose-var-name)))
		(ListLink
			(ImplicationLink (PredicateNode predicate_instance) (PredicateNode predicate_concept))
			(r2l-wordinst-predicate predicate_instance)
			(EvaluationLink (DefinedLinguisticPredicateNode "Truth Value")
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
(define-public (amod-rule concept instance adj adj_instance)
	(ListLink
		(InheritanceLink  (ConceptNode adj_instance) (ConceptNode adj))
		(r2l-wordinst-concept adj_instance)
		(r2l-wordinst-concept instance)
		(InheritanceLink  (ConceptNode instance) (ConceptNode concept))
		(InheritanceLink  (ConceptNode instance) (ConceptNode adj_instance))
))

(define-public (advmod-rule verb-node instance-node adv-node adv-instance-node)
	(define verb (cog-name verb-node))
	(define instance (cog-name instance-node))
	(define adv (cog-name adv-node))
	(define adv_instance (cog-name adv-instance-node))
	(ListLink
		(InheritanceLink  (ConceptNode adv_instance) (ConceptNode adv))
			(ImplicationLink  (PredicateNode instance) (PredicateNode verb))
			(r2l-wordinst-predicate instance)
			(r2l-wordinst-concept adv_instance)
			(InheritanceLink  (SatisfyingSetLink (PredicateNode instance)) (ConceptNode adv_instance))
	)
)

(define-public (adverbialpp-rule verb instance prep  prep_instance noun noun_instance)
    (ListLink
    (InheritanceLink  (ConceptNode noun_instance) (ConceptNode noun))
    (ImplicationLink  (PredicateNode instance) (PredicateNode verb))
    (ImplicationLink  (PredicateNode prep_instance) (PredicateNode prep))
    (r2l-wordinst-predicate instance)
    (r2l-wordinst-concept noun_instance)
    (r2l-wordinst-predicate prep_instance)
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
(define-public (pp-rule prep_concept prep_instance noun_concept noun_instance)
	(ListLink
		(ImplicationLink (PredicateNode prep_instance) (PredicateNode prep_concept))
		(InheritanceLink (ConceptNode noun_instance) (ConceptNode noun_concept))
		(r2l-wordinst-predicate prep_instance)
		(r2l-wordinst-concept noun_instance)
		(EvaluationLink
			(PredicateNode prep_instance)
			(ListLink (ConceptNode noun_instance))
		))
)

; -----------------------------------------------------------------------
; unary rules
; -----------------------------------------------------------------------
; XXX this rule is not used anywhere!
(define-public (entity-rule word word_instance)
	(ListLink
		(InheritanceLink (SpecificEntityNode word_instance) (ConceptNode word)))
)

; FIXME: this is bad because in SV, SVO type rules the same word is
; ConceptNode instead
(define-public (gender-rule lemma word_inst gender)
	(define word (cog-name lemma))
	(define word_instance (cog-name word_inst))
	(define gender_type (cog-name gender))

	(define concept_node (ConceptNode word))

	(cond
		((string=? gender_type "feminine")
			(ListLink
				(Inheritance
					(SpecificEntityNode word_instance) (DefinedLinguisticConceptNode "female"))
				(Inheritance
					(SpecificEntityNode word_instance) (Concept word))
		))
		((string=? gender_type "masculine")
			(ListLink
				(Inheritance
					(SpecificEntityNode word_instance) (DefinedLinguisticConceptNode "male"))
				(Inheritance
					(SpecificEntityNode word_instance) (Concept word))
		))
		((string=? gender_type "person")
			(ListLink
				(Inheritance
					(SpecificEntityNode word_instance) (DefinedLinguisticConceptNode "unknown_gender"))
				(InheritanceLink
					(SpecificEntityNode word_instance) (Concept word))
		))
	)
)

(define-public (tense-rule lemma inst tns)
	(define verb (cog-name lemma))
	(define instance (cog-name inst))
	(define tense (cog-name tns))

	(ListLink
		(Implication (Predicate instance) (Predicate verb))
		(r2l-wordinst-predicate instance)
		(Inheritance (Predicate instance) (DefinedLinguisticConceptNode tense))
	)
)

(define-public (quantity-rule noun_concept noun_instance quantifier_concept quantifier_instance)
	(ListLink
		(InheritanceLink (ConceptNode noun_instance) (ConceptNode noun_concept))
		(InheritanceLink (ConceptNode quantifier_instance) (ConceptNode quantifier_concept))
		(r2l-wordinst-concept noun_instance)
		(r2l-wordinst-concept quantifier_instance)
		(QuantityLink (ConceptNode noun_instance)(ConceptNode quantifier_instance))
	)
)

;-----------------------------------------------------------------------------
; Determiner-question-word questions, e.g.
; "At what time . . . ", "For what reason . . .", "In what way . . .",
; "To what degree . . . ", "At what location . . ."
;-----------------------------------------------------------------------------
(define-public (q-det-rule noun_concept noun_instance verb verb_instance qtype)
	(ListLink (InheritanceLink (VariableNode "$qVar") (ConceptNode noun_concept))
	(InheritanceLink (ConceptNode noun_instance) (VariableNode "$qVar"))
	(ImplicationLink (PredicateNode verb_instance) (PredicateNode verb))
	(r2l-wordinst-predicate verb_instance)
	(cond ((string=? qtype "when")
		(AtTimeLink
			(PredicateNode verb_instance)
			(VariableNode "$qVar")
		))
	((string=? qtype "where")
		(EvaluationLink
			(DefinedLinguisticPredicateNode "AtPlace")
			(ListLink
				(VariableNode "$qVar")
				(PredicateNode verb_instance)
			)
		)
	)
	((string=? qtype "why")
		(EvaluationLink
			(DefinedLinguisticPredicateNode "Because")
			(ListLink
				(VariableNode "$qVar")
				(PredicateNode verb_instance)
			)
		)
	)
	((string=? qtype "how")
		(EvaluationLink
			(DefinedLinguisticPredicateNode "InManner")
			(ListLink
				(VariableNode "$qVar")
				(PredicateNode verb_instance)
			)
		)
	)
	((string=? qtype "how_much")
		(EvaluationLink
			(DefinedLinguisticPredicateNode "Degree")
			(ListLink
				(VariableNode "$qVar")
				(PredicateNode verb_instance)
			)
		)
	)
	))
)

(define-public (det-rule noun_lemma noun_inst det_lemma)
	(define concept (cog-name noun_lemma))
	(define instance (cog-name noun_inst))
	(define determiner (cog-name det_lemma))
	(define var_name (choose-var-name))
	(cond
		((or (string=? determiner "those") (string=? determiner "these"))
			(ListLink
				;; XXX FIXME: right now, this says ImplicationScopeLink
				;; But I think the intended meaning is a for-all link:
				;; (ForAllLink (VariableNode var_name) (ImplicationLink ...))
				;; Right?
				(ImplicationScopeLink
					(MemberLink (VariableNode var_name) (ConceptNode instance))
					(InheritanceLink (VariableNode var_name) (ConceptNode concept))))
			(r2l-wordinst-concept instance)
		)
		((or (string=? determiner "this") (string=? determiner "that"))
			(ListLink
				(r2l-wordinst-concept instance)
				(InheritanceLink (VariableNode var_name) (ConceptNode concept)))
		)

		; XXX Just to avoid getting the `#<Invalid handle>` error
		; TODO Need to add more to the list (e.g. "the") to cover all cases
		; or update the below to generate something reasonable
		(else (ListLink))
	)
)

(define-public (negative-rule lemma verb-inst)
	(define verb (cog-name lemma))
	(define instance (cog-name verb-inst))
	(ListLink
		(r2l-wordinst-predicate instance)
		(ImplicationLink (PredicateNode instance) (NotLink (PredicateNode verb))))
)

(define-public (definite-rule lemma word-inst)
	(define word (cog-name lemma))
	(define word_instance (cog-name word-inst))

	(ListLink
		(Inheritance (Concept word_instance) (Concept word))
		(r2l-wordinst-concept word_instance)
		(Evaluation
			(DefinedLinguisticPredicateNode "definite")
			(ListLink (Concept word_instance)))
	)
)

; Example: "Maybe she eats lunch.", "Perhaps she is nice."
(define-public (maybe-rule word-node word-instance-node)
	(define word (cog-name word-node))
	(define word_instance (cog-name word-instance-node))
	(ListLink
		(ImplicationLink (PredicateNode word_instance) (PredicateNode word))
		(r2l-wordinst-predicate word_instance)
		(EvaluationLink
			(DefinedLinguisticPredicateNode "maybemarker")
			(ListLink
				(PredicateNode word_instance)
			)
		)
	)
)

; -----------------------------------------------------------------------
; misc rules
; -----------------------------------------------------------------------
; XXX this rule is not used anywhere!
(define-public (number-rule noun noun_instance num num_instance)
	(define noun_ins_concept (ConceptNode noun_instance))
	(ListLink
		(InheritanceLink noun_ins_concept (ConceptNode noun))
		(InheritanceLink (NumberNode num_instance) (NumberNode num))
		(QuantityLink (ConceptNode noun_instance) (NumberNode num_instance))
	)
)

; XXX this rule is not used anywhere!
(define-public (about-rule verb verb_instance  noun noun_instance)
	(ListLink
		(ImplicationLink (PredicateNode verb_instance) (PredicateNode verb))
		(InheritanceLink (ConceptNode noun_instance) (ConceptNode noun))
		(EvaluationLink
			(DefinedLinguisticPredicateNode "about")
			(ListLink
				(PredicateNode verb_instance)
				(ConceptNode noun_instance)
			)
		))
)

(define-public (nn-rule n1-lemma n1-inst n2-lemma n2-inst)
	(define n1 (cog-name n1-lemma))
	(define n1_instance (cog-name n1-inst ))
	(define n2 (cog-name n2-lemma))
	(define n2_instance (cog-name n2-inst ))
; XXX FIXME these two are not returned ???
	(r2l-wordinst-concept n1_instance)
	(r2l-wordinst-concept n2_instance)
	(ListLink
		(InheritanceLink (ConceptNode n1_instance) (ConceptNode n1))
		(InheritanceLink (ConceptNode n2_instance) (ConceptNode n2))
		(InheritanceLink (ConceptNode n1_instance) (ConceptNode n2_instance))
	)
)

(define-public (possessive-rule noun noun_instance word word_instance)
	(ListLink
		(InheritanceLink (ConceptNode noun_instance) (ConceptNode noun))
		(InheritanceLink (ConceptNode word_instance) (ConceptNode word))
		(r2l-wordinst-concept noun_instance)
		(r2l-wordinst-concept word_instance)
		(EvaluationLink
			(DefinedLinguisticPredicateNode "possession")
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
(define-public (to-do-rule-1
                verb1_lemma verb1_inst
                verb2_lemma verb2_inst
                subj_lemma subj_inst
                obj_lemma obj_inst)
	(define v1 (cog-name verb1_lemma))
	(define v1_instance (cog-name verb1_inst))
	(define v2 (cog-name verb2_lemma))
	(define v2_instance (cog-name verb2_inst))
	(define s (cog-name subj_lemma))
	(define s_instance (cog-name subj_inst))
	(define o (cog-name obj_lemma))
	(define o_instance (cog-name obj_inst))
	(ListLink
		(InheritanceLink (ConceptNode s_instance) (ConceptNode s))
		(InheritanceLink (ConceptNode o_instance) (ConceptNode o))
		(ImplicationLink (PredicateNode v1_instance) (PredicateNode v1))
		(ImplicationLink (PredicateNode v2_instance) (PredicateNode v2))
		(r2l-wordinst-concept s_instance)
		(r2l-wordinst-concept o_instance)
		(r2l-wordinst-predicate v1_instance)
		(r2l-wordinst-predicate v2_instance)
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
; Example: "She wants you to help us." -- assigns wrong subject in
; second clause. XXX FIXME
;
(define-public (to-do-rule-2
                verb1_lemma verb1_inst
                verb2_lemma verb2_inst
                subj1_lemma subj1_inst
                subj2_lemma subj2_inst
                obj_lemma obj_inst)
	(define v1 (cog-name verb1_lemma))
	(define v1_instance (cog-name verb1_inst))
	(define v2 (cog-name verb2_lemma))
	(define v2_instance (cog-name verb2_inst))
	(define s1 (cog-name subj1_lemma))
	(define s1_instance (cog-name subj1_inst))
	(define s2 (cog-name subj2_lemma))
	(define s2_instance (cog-name subj2_inst))
	(define o (cog-name obj_lemma))
	(define o_instance (cog-name obj_inst))
	(ListLink
		(InheritanceLink (ConceptNode s1_instance) (ConceptNode s1))
		(InheritanceLink (ConceptNode s2_instance) (ConceptNode s2))
		(InheritanceLink (ConceptNode o_instance) (ConceptNode o))
		(ImplicationLink (PredicateNode v1_instance) (PredicateNode v1))
		(ImplicationLink (PredicateNode v2_instance) (PredicateNode v2))
		(r2l-wordinst-concept s1_instance)
		(r2l-wordinst-concept s2_instance)
		(r2l-wordinst-concept o_instance)
		(r2l-wordinst-predicate v1_instance)
		(r2l-wordinst-predicate v2_instance)
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
; Example: "She is nice to help with the project." -- the scheme output
; for this rule is a logical mess
;
(define-public (to-do-rule-3 adj_lemma adj_inst verb_lemma verb_inst subj_lemma subj_inst)
	(define v1 (cog-name adj_lemma))
	(define v1_instance (cog-name adj_inst))
	(define v2 (cog-name verb_lemma))
	(define v2_instance (cog-name verb_inst))
	(define v3 (cog-name subj_lemma))
	(define v3_instance (cog-name subj_inst))
	(ListLink
		(InheritanceLink (ConceptNode v1_instance) (ConceptNode v1))
		(ImplicationLink (PredicateNode v2_instance) (PredicateNode v2))
		(InheritanceLink (ConceptNode v3_instance) (ConceptNode v3))
		(r2l-wordinst-concept v1_instance)
		(r2l-wordinst-concept v3_instance)
		(r2l-wordinst-predicate v2_instance)
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
; What about "She must need to sing?" "She must want to sing?"
; -- why is must being treated as a main-verb rather than an auxiliary?
;
(define-public (to-do-rule-4 v1 v1_instance v2 v2_instance)
	(ListLink
		(InheritanceLink (ConceptNode v2_instance) (ConceptNode v2))
		(ImplicationLink (PredicateNode v1_instance) (PredicateNode v1))
		(r2l-wordinst-concept v2_instance)
		(r2l-wordinst-predicate v1_instance)
		(EvaluationLink
			(DefinedLinguisticPredicateNode "able_to")
			(ListLink
				(ConceptNode v2_instance)
				(PredicateNode v1_instance)
			)
		))
)
;
; Example: "She wants to sing."; verb1 = want, verb2 = sing, subj = she
;
(define-public (to-do-rule-5
		verb1-node verb1-instance-node
		verb2-node verb2-instance-node
		subj-node subj-instance-node)

	(define verb1 (cog-name verb1-node))
	(define verb1_instance (cog-name verb1-instance-node))
	(define verb2 (cog-name verb2-node))
	(define verb2_instance (cog-name verb2-instance-node))
	(define subj (cog-name subj-node))
	(define subj_instance (cog-name subj-instance-node))
	(ListLink
		(InheritanceLink (ConceptNode subj_instance) (ConceptNode subj))
		(ImplicationLink (PredicateNode verb1_instance) (PredicateNode verb1))
		(ImplicationLink (PredicateNode verb2_instance) (PredicateNode verb2))
		(r2l-wordinst-concept subj_instance)
		(r2l-wordinst-predicate verb1_instance)
		(r2l-wordinst-predicate verb2_instance)
		(EvaluationLink
			(PredicateNode verb1_instance)
			(ListLink
				(ConceptNode subj_instance)
				(PredicateNode verb2_instance)
			)
		))
)
;---------------------------------------------------
;
; Where, When, Why, and How (of manner) -- the first rule of each of these groups handles questions with
; any of the templates SV, SVO, SVIO, SP, to-be, to-do, prep etc. (see the Y/N question rules for a complete list)
;
;---------------------------------------------------
; WHERE QUESTIONS
;---------------------------------------------------
;
; Examples: "Where do you live?","Where did you eat dinner?" etc.
;
(define-public (where-rule verb-node verb-instance-node)
	(let ((var_name (choose-var-name))
			(verb (cog-name verb-node))
			(verb_instance (cog-name verb-instance-node))
		)
		(ListLink
			(ImplicationLink (PredicateNode verb_instance) (PredicateNode verb))
			(r2l-wordinst-predicate verb_instance)
			(EvaluationLink
				(DefinedLinguisticPredicateNode "AtPlace")
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
(define-public (wherecop-Q-rule subj-concept-node subj-instance-node)
	(let ((var_name (choose-var-name))
			(subj_concept (cog-name subj-concept-node))
			(subj_instance (cog-name subj-instance-node))
		)
		(ListLink
			(InheritanceLink (ConceptNode subj_instance) (ConceptNode subj_concept))
			(r2l-wordinst-concept subj_instance)
			(EvaluationLink
				(DefinedLinguisticPredicateNode "AtPlace")
				(ListLink
					(VariableNode var_name)
					(ConceptNode subj_instance)
				)
			)
		)
	)
)
;
;----------------------------------------------------
; WHEN QUESTIONS
;----------------------------------------------------
;
; Example: "When did jazz die?","When did you bake the cake?",
; "When did you give him the money?" etc.
;
(define-public (when-rule lemma verb-inst)
	(let ((verb (cog-name lemma))
			(verb_instance (cog-name verb-inst))
			(var_name (choose-var-name)))
		(ListLink
			(ImplicationLink (PredicateNode verb_instance) (PredicateNode verb))
			(r2l-wordinst-predicate verb_instance)
			(AtTimeLink
				(PredicateNode verb_instance)
				(VariableNode var_name)
			)
		)
	)
)
;
; Example "When is the party?" etc.
;
(define-public (whencop-Q-rule subj-concept-node subj-instance-node)
	(let ((var_name (choose-var-name))
			(subj_concept (cog-name subj-concept-node))
			(subj_instance (cog-name subj-instance-node))
		)
		(ListLink
			(InheritanceLink (ConceptNode subj_instance) (ConceptNode subj_concept))
			(r2l-wordinst-concept subj_instance)
			(AtTimeLink
				(ConceptNode subj_instance)
				(VariableNode var_name)
			)
		)
	)
)
;
;------------------------------------------------------
; Why questions
;------------------------------------------------------
;
; Example: "Why do you live?", "Why do you like terrible music?"
;
(define-public (why-rule lemma verb-inst)
	(let ((verb (cog-name lemma))
			(verb_instance (cog-name verb-inst))
			(var_name (choose-var-name)))
		(ListLink
			(ImplicationLink (PredicateNode verb_instance) (PredicateNode verb))
			(r2l-wordinst-predicate verb_instance)
			(EvaluationLink
				(DefinedLinguisticPredicateNode "Because")
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
(define-public (whycop-Q-rule subj-lemma subj-inst)
	(let ((subj_concept (cog-name subj-lemma))
			(subj_instance (cog-name subj-inst))
			(var_name (choose-var-name)))
		(ListLink
			(InheritanceLink (ConceptNode subj_instance) (ConceptNode subj_concept))
			(r2l-wordinst-concept subj_instance)
			(EvaluationLink
				(DefinedLinguisticPredicateNode "Because")
				(ListLink
					(VariableNode var_name)
					(ConceptNode subj_instance)
				)
			)
		)
	)
)

;------------------------------------------------------
; How adverbial (manner) questions
;------------------------------------------------------
;
; Example: "How did you sleep?" etc.
;
(define-public (how-rule verb verb_instance)
	(let ((var_name (choose-var-name)))
		(ListLink
			(ImplicationLink (PredicateNode verb_instance) (PredicateNode verb))
			(r2l-wordinst-predicate verb_instance)
			(EvaluationLink
				(DefinedLinguisticPredicateNode "InManner")
				(ListLink
					(VariableNode var_name)
					(PredicateNode verb_instance)
				)
			)
		)
	)
)
;-----------------------------------------
; Predicative How
;-----------------------------------------
;
; Examples: "How was the party?" "How is your food?")
;
(define-public (howpredadj-Q-rule subj_concept subj_instance)
	(let ((var_name (choose-var-name)))
		(ListLink
			(InheritanceLink (ConceptNode subj_instance) (ConceptNode subj_concept))
			(r2l-wordinst-concept subj_instance)
			(InheritanceLink (ConceptNode subj_instance) (VariableNode var_name))
		)
	)
)
;
;------------------------------------------------------------
; How of quantity and degree questions
;------------------------------------------------------------
;
; Example: "How much money does it cost?", "How many books have you read?"
;
; NB: doesn't call rule if there is no noun after "how much" because
; relex doesn't give it a quantity dependency relation in that case
;
(define-public (howquantQ-rule concept instance)
	(let ((var_name (choose-var-name)))
		(ListLink
			(InheritanceLink (ConceptNode instance) (ConceptNode concept))
			(r2l-wordinst-concept instance)
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
(define-public (howdegQ-rule concept instance)
	(let ((var_name (choose-var-name)))
		(ListLink
			(InheritanceLink (ConceptNode instance) (ConceptNode concept))
			(r2l-wordinst-concept instance)
			(EvaluationLink
				(DefinedLinguisticPredicateNode "DegreeLink")
				(ListLink
					(VariableNode var_name)
					(ConceptNode instance)
				)
			)
		)
	)
)
;
;-------------------------------------------
; CHOICE-TYPE QUESTIONS (also known as question-determiners)
; -- what and which questions
;
; NB: I don't think these rules could be made to plug into the
; setence templates because of the SatisfyingSet logic . . .
;
; Therefore still need to write SV, prep, to-do, and to-be versions
; of these if we want that functionality -- or do them without the
; satisfying set logic, so they can just plug into the templates . . .
;
;-------------------------------------------
;
; Example: "Which girl do you like?" "What book are you reading?"
;
(define-public (whichobjQ-rule
	subj_lemma subj_inst verb_lemma verb_inst obj_lemma obj_inst)
	(define subj_concept (cog-name subj_lemma))
	(define subj_instance (cog-name subj_inst))
	(define verb (cog-name verb_lemma))
	(define verb_instance (cog-name verb_inst))
	(define obj_concept (cog-name obj_lemma))
	(define obj_instance (cog-name obj_inst))

	(let ((var_name (choose-var-name)))
		(ListLink
			(ImplicationLink (PredicateNode verb_instance) (PredicateNode verb))
			(InheritanceLink (ConceptNode subj_instance) (ConceptNode subj_concept))
			(InheritanceLink (ConceptNode obj_instance) (ConceptNode obj_concept))
			(InheritanceLink (VariableNode var_name) (ConceptNode obj_instance))
			(r2l-wordinst-concept subj_instance)
			(r2l-wordinst-concept obj_instance)
			(r2l-wordinst-predicate verb_instance)
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
(define-public (whichsubjSVOQ-rule
	subj_lemma subj_inst verb_lemma verb_inst obj_lemma obj_inst)
	(define subj_concept (cog-name subj_lemma))
	(define subj_instance (cog-name subj_inst))
	(define verb (cog-name verb_lemma))
	(define verb_instance (cog-name verb_inst))
	(define obj_concept (cog-name obj_lemma))
	(define obj_instance (cog-name obj_inst))

	(let ((var_name (choose-var-name)))
		(ListLink
			(ImplicationLink (PredicateNode verb_instance) (PredicateNode verb))
			(InheritanceLink (ConceptNode subj_instance) (ConceptNode subj_concept))
			(ImplicationLink (PredicateNode obj_instance) (PredicateNode obj_concept))
			(InheritanceLink (VariableNode var_name) (ConceptNode subj_instance))
			(r2l-wordinst-concept subj_instance)
			(r2l-wordinst-concept obj_instance)
			(r2l-wordinst-predicate verb_instance)
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
(define-public (whichiobjQ-rule
	subj_lemma subj_inst verb_lemma verb_inst obj_lemma obj_inst iobj_lemma iobj_inst)
	(define subj_concept (cog-name subj_lemma))
	(define subj_instance (cog-name subj_inst))
	(define verb (cog-name verb_lemma))
	(define verb_instance (cog-name verb_inst))
	(define obj_concept (cog-name obj_lemma))
	(define obj_instance (cog-name obj_inst))
	(define iobj_concept (cog-name iobj_lemma))
	(define iobj_instance (cog-name iobj_inst))

	(let ((var_name (choose-var-name)))
		(ListLink
			(ImplicationLink (PredicateNode verb_instance) (PredicateNode verb))
			(InheritanceLink (ConceptNode subj_instance) (ConceptNode subj_concept))
			(InheritanceLink (ConceptNode obj_instance) (ConceptNode obj_concept))
			(InheritanceLink (ConceptNode iobj_instance) (ConceptNode iobj_concept))
			(InheritanceLink (VariableNode var_name) (ConceptNode iobj_instance))
			(r2l-wordinst-concept subj_instance)
			(r2l-wordinst-concept obj_instance)
			(r2l-wordinst-concept iobj_instance)
			(r2l-wordinst-predicate verb_instance)
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
(define-public (whichpredadjQ-rule
	subj_lemma subj_inst pred_lemma pred_inst)
	(define subj_concept (cog-name subj_lemma))
	(define subj_instance (cog-name subj_inst))
	(define pred_concept (cog-name pred_lemma))
	(define pred_instance (cog-name pred_inst))
	(let ((var_name (choose-var-name)))
		(ListLink
			(InheritanceLink (ConceptNode subj_instance) (ConceptNode subj_concept))
			(ImplicationLink (PredicateNode pred_instance) (PredicateNode pred_concept))
			(InheritanceLink (VariableNode var_name) (ConceptNode subj_instance) )
			(r2l-wordinst-concept subj_instance)
			(r2l-wordinst-predicate pred_instance)
			(SatisfyingSetLink
				(VariableNode var_name)
				(EvaluationLink
					(PredicateNode pred_instance)
					(ListLink
						(VariableNode var_name))
				)
			)
		)
	)
)

(define-public (whichsubjQ-rule
	subj_lemma subj_inst verb_lemma verb_inst obj_lemma obj_inst)
	(define subj_concept (cog-name subj_lemma))
	(define subj_instance (cog-name subj_inst))
	(define verb (cog-name verb_lemma))
	(define verb_instance (cog-name verb_inst))
	(define obj_concept (cog-name obj_lemma))
	(define obj_instance (cog-name obj_inst))
	(throw 'not-implemented)
)

(define-public (whichsubjSVIOQ-rule
	subj_lemma subj_inst verb_lemma verb_inst obj_lemma obj_inst iobj_lemma iobj_inst)
	(define subj_concept (cog-name subj_lemma))
	(define subj_instance (cog-name subj_inst))
	(define verb (cog-name verb_lemma))
	(define verb_instance (cog-name verb_inst))
	(define obj_concept (cog-name obj_lemma))
	(define obj_instance (cog-name obj_inst))
	(define iobj_concept (cog-name iobj_lemma))
	(define iobj_instance (cog-name iobj_inst))
	(throw 'not-implemented)
)

(define-public (whichobjSVIOQ-rule
	subj_lemma subj_inst verb_lemma verb_inst obj_lemma obj_inst iobj_lemma iobj_inst)
	(define subj_concept (cog-name subj_lemma))
	(define subj_instance (cog-name subj_inst))
	(define verb (cog-name verb_lemma))
	(define verb_instance (cog-name verb_inst))
	(define obj_concept (cog-name obj_lemma))
	(define obj_instance (cog-name obj_inst))
	(define iobj_concept (cog-name iobj_lemma))
	(define iobj_instance (cog-name iobj_inst))
	(throw 'not-implemented)
)

(define-public (whichpobjQ-rule
	subj_lemma subj_inst prep_lemma prep_inst pobj_lemma pobj_inst)
	(define subj_concept (cog-name subj_lemma))
	(define subj_instance (cog-name subj_inst))
	(define prep (cog-name prep_lemma))
	(define prep_instance (cog-name prep_inst))
	(define pobj_concept (cog-name pobj_lemma))
	(define pobj_instance (cog-name pobj_inst))
	(throw 'not-implemented)
)

(define-public (whichsubjpobjQ-rule
	subj_lemma subj_inst prep_lemma prep_inst pobj_lemma pobj_inst)
	(define subj_concept (cog-name subj_lemma))
	(define subj_instance (cog-name subj_inst))
	(define prep (cog-name prep_lemma))
	(define prep_instance (cog-name prep_inst))
	(define pobj_concept (cog-name pobj_lemma))
	(define pobj_instance (cog-name pobj_inst))
	(throw 'not-implemented)
)

(define-public (whichsubjSVQ-rule
	subj_lemma subj_inst verb_lemma verb_inst)
	(define subj_concept (cog-name subj_lemma))
	(define subj_instance (cog-name subj_inst))
	(define verb (cog-name verb_lemma))
	(define verb_instance (cog-name verb_inst))
	(throw 'not-implemented)
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
;
; XXX all-rule is not used anywhere ...
(define-public (all-rule noun_instance)
	(ListLink
		(r2l-wordinst-concept noun_instance)
		(EvaluationLink
			(DefinedLinguisticPredicateNode "allmarker")
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
; XXX this rule is not used anywhere ...
(define-public (passive-rule1 verb verb_instance obj obj_instance passive_obj passive_obj_instance)
    (ListLink
	 (ImplicationLink (PredicateNode verb_instance) (PredicateNode verb))
    (InheritanceLink (ConceptNode obj_instance) (ConceptNode obj))
    (InheritanceLink (ConceptNode passive_obj_instance) (ConceptNode passive_obj))
	(r2l-wordinst-concept obj_instance)
	(r2l-wordinst-concept passive_obj_instance)
	(r2l-wordinst-predicate verb_instance)
    (EvaluationLink
            (PredicateNode verb_instance)
            (ListLink
                    (ConceptNode passive_obj_instance)
                    (ConceptNode obj_instance)
            )
    ))
)

; Example: "The books are published."
(define-public (passive-rule2 verb_lemma verb_inst obj_lemma obj_inst)
	(define verb (cog-name verb_lemma))
	(define verb_instance (cog-name verb_inst))
	(define obj (cog-name obj_lemma))
	(define obj_instance (cog-name obj_inst))
	(let ((var_name (choose-var-name)))
		(ListLink
			(ImplicationLink (PredicateNode verb_instance) (PredicateNode verb))
			(InheritanceLink (ConceptNode obj_instance) (ConceptNode obj))
			(r2l-wordinst-concept obj_instance)
			(r2l-wordinst-predicate verb_instance)
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
; coordinating conjunction - And, but, for, nor, or, so, and yet
; Example:"I did my homework, and I went to school."
;         "John and Madison eat the cake."
;         " Joan is poor  but  happy."

(define-public (and-rule var1 var1_instance var2 var2_instance pos)

    (cond
    [(equal? pos "verb")
        (ListLink
        (ImplicationLink (PredicateNode var1_instance) (PredicateNode var1))
        (ImplicationLink (PredicateNode var2_instance) (PredicateNode var2))
        (r2l-wordinst-predicate var1_instance)
        (r2l-wordinst-predicate var2_instance)
        (EvaluationLink
            (DefinedLinguisticPredicateNode "and")
            (ListLink
                (PredicateNode var1_instance)
                (PredicateNode var2_instance)
            )
        ))]
    [else
        (ListLink
        (InheritanceLink (ConceptNode var1_instance) (ConceptNode var1))
        (InheritanceLink (ConceptNode var2_instance) (ConceptNode var2))
        (r2l-wordinst-concept var1_instance)
        (r2l-wordinst-concept var2_instance)
        (EvaluationLink
            (DefinedLinguisticPredicateNode "and")
            (ListLink
               (ConceptNode var1_instance)
               (ConceptNode var2_instance)
            )
       ))])
)

(define-public (but-rule var1 var1_instance var2 var2_instance pos)
    (cond [(equal? pos "verb")
        (ListLink
        (ImplicationLink (PredicateNode var1_instance) (PredicateNode var1))
        (ImplicationLink (PredicateNode var2_instance) (PredicateNode var2))
        (r2l-wordinst-predicate var1_instance)
        (r2l-wordinst-predicate var2_instance)
        (EvaluationLink
            (DefinedLinguisticPredicateNode "but")
            (ListLink
                (PredicateNode var1_instance)
                (PredicateNode var2_instance)
            )
        ))]
    [else
        (ListLink
        (InheritanceLink (ConceptNode var1_instance) (ConceptNode var1))
        (r2l-wordinst-concept var1_instance)
        (r2l-wordinst-concept var2_instance)
        (EvaluationLink
            (DefinedLinguisticPredicateNode "but")
            (ListLink
                (ConceptNode var1_instance)
                (ConceptNode var2_instance)
            )
        ))])
)

(define-public (or-rule var1 var1_instance var2 var2_instance pos)
    (cond [(equal? pos "verb")
        (ListLink
        (ImplicationLink (PredicateNode var1_instance) (PredicateNode var1))
        (ImplicationLink (PredicateNode var2_instance) (PredicateNode var2))
        (r2l-wordinst-predicate var1_instance)
        (r2l-wordinst-predicate var2_instance)
        (EvaluationLink
            (DefinedLinguisticPredicateNode "or")
            (ListLink
                (PredicateNode var1_instance)
                (PredicateNode var2_instance)
            )
        ))]
    [else
        (ListLink
        (InheritanceLink (ConceptNode var1_instance) (ConceptNode var1))
        (InheritanceLink (ConceptNode var2_instance) (ConceptNode var2))
        (r2l-wordinst-concept var1_instance)
        (r2l-wordinst-concept var2_instance)
        (EvaluationLink
            (DefinedLinguisticPredicateNode "or")
            (ListLink
                (ConceptNode var1_instance)
                (ConceptNode var2_instance)
            )
        ))])
)

;-----------------------------------------------------------------------
; complement clauses
;-----------------------------------------------------------------------
(define-public (complement-rule comp_concept comp_instance pred_concept pred_instance)
	(ListLink
		(ImplicationLink (PredicateNode comp_instance) (PredicateNode comp_concept))
		(ImplicationLink (PredicateNode pred_instance) (PredicateNode pred_concept))
		(r2l-wordinst-predicate comp_instance)
		(r2l-wordinst-predicate pred_instance)
		(EvaluationLink
			(PredicateNode comp_instance)
			(ListLink
				(PredicateNode pred_instance)
			)
		)
	)
)

(define-public (compmod-rule comp_concept comp_instance pred_concept pred_instance)
	(ListLink
		(ImplicationLink (PredicateNode comp_instance) (PredicateNode comp_concept))
		(ImplicationLink (PredicateNode pred_instance) (PredicateNode pred_concept))
		(r2l-wordinst-predicate comp_instance)
		(r2l-wordinst-predicate pred_instance)
		(r2l-wordinst-concept comp_instance)
		(EvaluationLink
			(DefinedLinguisticPredicateNode "InManner")
			(ListLink
				(PredicateNode pred_instance)
				(ConceptNode comp_instance)
			)
		)
	)
)

(define-public (because-rule comp_concept comp_instance pred_concept pred_instance)
	(ListLink
		(ImplicationLink (PredicateNode comp_instance) (PredicateNode comp_concept))
		(ImplicationLink (PredicateNode pred_instance) (PredicateNode pred_concept))
		(r2l-wordinst-predicate comp_instance)
		(r2l-wordinst-predicate pred_instance)
		(r2l-wordinst-concept comp_instance)
		(EvaluationLink
			(DefinedLinguisticPredicateNode "Because")
			(ListLink
				(PredicateNode pred_instance)
				(ConceptNode comp_instance)
			)
		)
	)
)

(define-public (attime-rule comp_concept comp_instance pred_concept pred_instance)
	(ListLink
		(ImplicationLink (PredicateNode comp_instance) (PredicateNode comp_concept))
		(ImplicationLink (PredicateNode pred_instance) (PredicateNode pred_concept))
		(r2l-wordinst-predicate comp_instance)
		(r2l-wordinst-predicate pred_instance)
		(r2l-wordinst-concept comp_instance)
		(AtTimeLink
			(PredicateNode pred_instance)
			(ConceptNode comp_instance)
		)
	)
)

(define-public (rep-rule comp_word comp_inst pred_word pred_inst)
	(define comp_concept (cog-name comp_word))
	(define comp_instance (cog-name comp_inst))
	(define pred_concept (cog-name pred_word))
	(define pred_instance (cog-name pred_inst))
	(ListLink
		(ImplicationLink (PredicateNode comp_instance) (PredicateNode comp_concept))
		(ImplicationLink (PredicateNode pred_instance) (PredicateNode pred_concept))
		(r2l-wordinst-predicate comp_instance)
		(r2l-wordinst-predicate pred_instance)
		(r2l-wordinst-concept comp_instance)
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
; Examples: "I think that dogs can fly.", "He is glad that she won.",
; "He ran so quickly that he flew."
; A that-rule for "object clause", "content clause", "complement clause",
; etc, but not "adjective clause"
; XXX that-rule is not used anywhere!
;(define (that-rule main main_instance sub sub_instance)
;	(ListLink
;		(ImplicationLink (PredicateNode main_instance) (PredicateNode main))
;		(ImplicationLink (PredicateNode sub_instance) (PredicateNode sub))
;		(r2l-wordinst-predicate main_instance)
;		(r2l-wordinst-predicate sub_instance)
;		(EvaluationLink
;			(DefinedLinguisticPredicateNode "that")
;			(ListLink
;				(PredicateNode main_instance)
;				(PredicateNode sub_instance)
;			)
;		)
;	)
;)
;
; -----------------------------------------------------------------------
; time rules to create time relations
; -----------------------------------------------------------------------
; second argument of before() and after() can be a verb, adjective, pronoun
; or noun; rule can be invoked both with a before or after relation; it is
; checked which node needs to be created; discussion can be viewed here:
; https://github.com/opencog/opencog/pull/993
; Examples: "She went home before I left", "I went after him", "He sleeps
; before he is tired"
; XXX before-after-rule is not used anywhere!
;(define (before-after-rule $x_instance $y_instance $y_pos $before_or_after)
;    (define y-node
;        (if (or (string=? $y_pos "verb") (string=? $y_pos "adj"))
;            (PredicateNode $y_instance)
;            (ConceptNode $y_instance)
;        )
;    )
;    (ListLink
;        (EvaluationLink
;            (PredicateNode $before_or_after)
;            (ListLink (PredicateNode $x_instance) y-node)
;        )
;    )
;)
;
; -----------------------------------------------------------------------
; Examples: "I had dinner at 6 pm", "I went to sleep at 1 am"
; XXX time-rule is not used anywhere!
;(define (time-rule $hour $period $v_instance)
;    (define time-node
;        (if (string=? $period "am")
;            (TimeNode $hour)
;            (TimeNode (number->string (+ (string->number $hour) 12)))
;        )
;    )
;    (ListLink (AtTimeLink (PredicateNode $v_instance) time-node))
;)
;
;
; -----------------------------------------------------------------------
; functions without R2L rule, not working, unneeded, etc
; -----------------------------------------------------------------------
; XXX FIXME: there is no such thing as a "TruthValueGreaterThanLink",
; so this rule is borken.
;(define (comparative-rule w1 w1_instance w2 w2_instance adj adj_instance)
;	(ListLink
;  (InheritanceLink (ConceptNode adj_instance) (ConceptNode adj))
;	(InheritanceLink (ConceptNode w1_instance) (ConceptNode w1))
;	(InheritanceLink (ConceptNode w2_instance) (ConceptNode w2))
;	(r2l-wordinst-concept adj_instance)
;	(r2l-wordinst-concept w1_instance)
;	(r2l-wordinst-concept w2_instance)
;	(TruthValueGreaterThanLink
;		(InheritanceLink (ConceptNode w1_instance) (ConceptNode adj_instance))
;		(InheritanceLink (ConceptNode w2_instance) (ConceptNode adj_instance))
;	))
;)
;
;---------------------------------------------------------------
; XXX on-rule is not used anywhere!
;(define (on-rule w1 w1_instance w2 w2_instance)
;	(ListLink
;	(InheritanceLink (ConceptNode w1_instance) (ConceptNode w1))
;	(InheritanceLink (ConceptNode w2_instance) (ConceptNode w2))
;	(r2l-wordinst-concept w1_instance)
;	(r2l-wordinst-concept w2_instance)
;	(EvaluationLink
;		(DefinedLinguisticPredicateNode "on")
;		(ListLink
;			(ConceptNode w1_instance)
;			(ConceptNode w2_instance)
;		)
;	))
;)
;
;
;
;
;(define (which-rule antecedent  antecedent_instance  verb  verb_instance)
;	(ListLink
;  (InheritanceLink (ConceptNode antecedent_instance) (ConceptNode antecedent))
;	(ImplicationLink (PredicateNode verb_instance) (PredicateNode verb))
;        (EvaluationLink
;		(PredicateNode "whichmarker")
;		(ListLink
;			(ConceptNode antecedent_instance)
;			(PredicateNode verb_instance)
;		)
;	))
;)
;---------------------------------------------------------------
; Functional Question rules replaced by conditional statements and
; templates in October 2014
;---------------------------------------------------------------
;
; NB: sentences with prepositional phrases as the predicate-complements of the subject are treated by Relex in a variety
; of ways, including the prepobj relation and also treating the prep as a simple predicate; those sentences that do not
; call the rule below, call the SV-ynq rule.
;
;(define (prepobj-ynQ-rule subj_concept subj_instance predprep_concept predprep_instance obj_concept obj_instance)
;	(ListLink
;  (InheritanceLink (ConceptNode subj_instance) (ConceptNode subj_concept))
;	(InheritanceLink (ConceptNode obj_instance) (ConceptNode obj_concept))
;	(ImplicationLink (PredicateNode predprep_instance) (PredicateNode predprep_concept))
;	(EvaluationLink (PredicateNode "Truth Value")
;		(EvaluationLink (PredicateNode predprep_instance)
;			(ListLink (ConceptNode subj_instance)(ConceptNode obj_instance)))
;			(VariableNode "$var1"))
;))
;
;---------------------------------------------------
;
;
;(define (SV-ynQ-rule subj_concept subj_instance verb verb_instance)
;	(ListLink
;  (ImplicationLink (PredicateNode verb_instance) (PredicateNode verb))
;	(InheritanceLink (ConceptNode subj_instance) (ConceptNode subj_concept))
;	(EvaluationLink (PredicateNode "Truth-Value")
;		(EvaluationLink (PredicateNode verb_instance)
;			(ListLink (ConceptNode subj_instance)))
;		(VariableNode "$qVar"))
;))
;

;
; NB:	This rule applies incorrectly to many SVIO sentences where
;  the IO is interpreted by relex as
;	an extra prepositional phrase rather than an IO.
;
;(define (SVO-ynQ-rule subj_concept subj_instance verb verb_instance obj_concept obj_instance)
;	(ListLink (ImplicationLink (PredicateNode verb_instance) (PredicateNode verb))
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
;	(ListLink
;  (ImplicationLink (PredicateNode verb_instance) (PredicateNode verb))
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
;------------------------------------------------------
; who/what subject questions (who and what may be treated identically as far as I can tell).
;------------------------------------------------------
;
; Examples: "Who farted?", "What happened?"
;
;(define (whowhatsubj-SV-Q-rule verb verb_instance)
;	(ListLink
;  (ImplicationLink (PredicateNode verb_instance) (PredicateNode verb))
;	(EvaluationLink
;		(PredicateNode verb_instance)
;		(ListLink
;			(VariableNode "_$qVar")))
;))
;
; Examples: "What killed him?", "Who ate the pizza?"
;
;(define (whowhatsubj-SVO-Q-rule verb verb_instance obj_concept obj_instance)
;	(ListLink
;  (ImplicationLink (PredicateNode verb_instance) (PredicateNode verb))
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
;	(ListLink
;  (ImplicationLink (PredicateNode verb_instance) (PredicateNode verb))
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
;	(ListLink
;  (ImplicationLink (PredicateNode prep_instance) (PredicateNode prep))
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
;	(ListLink
;  (InheritanceLink (ConceptNode subj_instance) (ConceptNode subj_concept))
;	(InheritanceLink (ConceptNode subj_instance) (VariableNode "_$qVar"))
;))
;
;----------------------------------------------------
; who/what object and who/what indirect object questions
;----------------------------------------------------
;
; Examples: "Who do you love?", "What do you think?"
;
;(define (whowhatobj-Q-rule subj_concept subj_instance verb verb_instance)
;	(ListLink
;  (ImplicationLink (PredicateNode verb_instance) (PredicateNode verb))
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
;	(ListLink
;  (ImplicationLink (PredicateNode verb_instance) (PredicateNode verb))
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
;----------------------------------------------------------------------
;
; Example:
;
;(define (when-SVOQ-rule subj_concept subj_instance verb verb_instance obj_concept obj_instance)
;	(ListLink
;  (ImplicationLink (PredicateNode verb_instance) (PredicateNode verb))
;	(InheritanceLink (ConceptNode subj_instance) (ConceptNode subj_concept))
;	(InheritanceLink (ConceptNode obj_instance) (ConceptNode obj_concept))
;	(AtTimeLink
;		(EvaluationLink
;			(PredicateNode verb_instance)
;				(ListLink
;					(ConceptNode subj_instance)
;					(ConceptNode obj_instance)))
;		(VariableNode "$qVar"))
;))
;
; Example:
;
;(define (when-SVIOQ-rule subj_concept subj_instance verb verb_instance obj_concept obj_instance iobj_concept iobj_instance)
;	(ListLink
;  (ImplicationLink (PredicateNode verb_instance) (PredicateNode verb))
;	(InheritanceLink (ConceptNode subj_instance) (ConceptNode subj_concept))
;	(InheritanceLink (ConceptNode obj_instance) (ConceptNode obj_concept))
;	(InheritanceLink (ConceptNode iobj_instance) (ConceptNode iobj_concept))
;	(AtTimeLink
;		(EvaluationLink
;			(PredicateNode verb_instance)
;				(ListLink
;					(ConceptNode subj_instance)
;					(ConceptNode obj_instance)
;					(ConceptNode iobj_instance)))
;		(VariableNode "$qVar"))
;))
;--------------------------------------------------------
;
; Example: "
;
;(define (why-SVOQ-rule subj_concept subj_instance verb verb_instance obj_concept obj_instance)
;	(ListLink
;  (ImplicationLink (PredicateNode verb_instance) (PredicateNode verb))
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
;	(ListLink
;  (ImplicationLink (PredicateNode verb_instance) (PredicateNode verb))
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
;	(ListLink
;  (InheritanceLink (ConceptNode subj_instance) (ConceptNode subj_concept))
;	(InheritanceLink (ConceptNode predicative_instance) (ConceptNode predicative_concept))
;	(EvaluationLink
;		(PredicateNode "Because")
;			(ListLink
;				(VariableNode "$qVar")
;				(EvaluationLink (PredicateNode predicative_instance)
;					(ListLink (ConceptNode subj_instance)))))
;))
;
;--------------------------------------------------------
;
; Example: "How did you like the movie?"
;
;(define (how-SVOQ-rule subj_concept subj_instance verb verb_instance obj_concept obj_instance)
;	(ListLink
;  (ImplicationLink (PredicateNode verb_instance) (PredicateNode verb))
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
;	(ListLink
;  (ImplicationLink (PredicateNode verb_instance) (PredicateNode verb))
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
;--------------------------------------------------------------
;(define (SVP-rule subj  subj_instance  predicative  predicative_instance)
;	(Listlink
;  (InheritanceLink (ConceptNode predicative_instance) (ConceptNode predicative))
;	(InheritanceLink (ConceptNode subj_instance) (ConceptNode subj))
;	(InheritanceLink (ConceptNode subj_instance) (ConceptNode predicative_instance))
;	)
;
