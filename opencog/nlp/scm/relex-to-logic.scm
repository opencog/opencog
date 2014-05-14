; Notes: random-string, random-node-name and choose-var-name can be moved to utilities.scm

; -----------------------------------------------------------------------
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
; Creates a possible name 'node-name' of lenght 'name-length' for a node
; of type 'node-type'. The 'node-name' is not used with any other node
; of type 'node-type'. 
(define (random-node-name node-type name-length)
	(define node-name (random-string name-length))
	(if (equal? node-type 'VariableNode)
		(set! node-name (string-append "$" node-name))
	)
	(while (check-name? node-name node-type)
		(if (equal? node-type 'VariableNode)
			(set! node-name (string-append "$" (random-string name-length)))
			(set! node-name (random-string name-length))
		)
	)
	node-name
)

; -----------------------------------------------------------------------
; Creates name for VariableNodes after checking whether the name is being
; used by other VariableNode.
(define (choose-var-name) (random-node-name 'VariableNode 36))

; -----------------------------------------------------------------------
; Check if the lemma of a WordInstanceNode 'word-inst' is 'word'.
(define (check-lemma? word word-inst)
	(string=? word (cog-name (word-inst-get-lemma word-inst)))
)

; -----------------------------------------------------------------------
; Returns a list of WordInstanceNodes from 'parse-node', which have a LemmaLink with a
; WordNode named 'word'.
(define (get-word-inst-nodes word parse-node)
	(define word-inst-list (parse-get-words parse-node))
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
		((and (string? word-index) (check-name? word-index 'WordInstanceNode))
			(cog-name (list-ref
					(get-word-inst-nodes word parse-node)
					(- (get-word-inst-index (WordInstanceNode word-index)) 1)))
		)
	)
)

; -----------------------------------------------------------------------
(define (amod-rule concept instance adj adj_instance)
	(InheritanceLink  (ConceptNode adj_instance) (ConceptNode adj))
	(InheritanceLink  (ConceptNode instance) (ConceptNode adj_instance))
	(InheritanceLink  (ConceptNode instance) (ConceptNode concept))
)

(define (advmod-rule verb instance adv adv_instance)
	(InheritanceLink  (ConceptNode adv_instance) (ConceptNode adv))
	(InheritanceLink  (PredicateNode instance) (ConceptNode adv_instance))
	(InheritanceLink  (PredicateNode instance) (PredicateNode verb))
)

(define (tense-rule verb instance tense)
	(InheritanceLink (PredicateNode instance) (PredicateNode verb))
	(InheritanceLink (PredicateNode instance) (ConceptNode tense))
)

(define (det-rule concept instance var_name determiner)
	(cond ((or (string=? determiner "those") (string=? determiner "these"))
		(ImplicationLink
			(MemberLink (VariableNode var_name) (ConceptNode instance))
			(InheritanceLink (VariableNode var_name) (ConceptNode concept)))
		)
		((or (string=? determiner "this") (string=? determiner "that"))
			(InheritanceLink (VariableNode var_name) (ConceptNode concept))
		)
	)
)

(define (negative-rule verb instance) 
	(InheritanceLink (PredicateNode instance) (PredicateNode verb))
	(NotLink (PredicateNode instance))
)

(define (possessive-rule noun noun_instance word word_instance)
	(InheritanceLink (ConceptNode noun_instance) (ConceptNode noun))
        (InheritanceLink (ConceptNode word_instance) (ConceptNode word))
	(EvaluationLink
		(PredicateNode "Possession")
		(ListLink
			(ConceptNode noun_instance)
			(ConceptNode word_instance)
		)
	)
)

(define (comparative-rule w1 w1_instance w2 w2_instance adj adj_instance)
	(InheritanceLink (ConceptNode adj_instance) (ConceptNode adj))
	(InheritanceLink (ConceptNode w1_instance) (ConceptNode w1))
	(InheritanceLink (ConceptNode w2_instance) (ConceptNode w2))
	(TruthValueGreaterThanLink
		(InheritanceLink (ConceptNode w1_instance) (ConceptNode adj_instance))
		(InheritanceLink (ConceptNode w2_instance) (ConceptNode adj_instance))
	)
)

(define (number-rule noun noun_instance num num_instance)
	(define noun_ins_concept (ConceptNode noun_instance))
	(InheritanceLink noun_ins_concept (ConceptNode noun))
	(InheritanceLink (NumberNode num_instance) (NumberNode num))
	(QuantityLink (ConceptNode noun_instance) (NumberNode num_instance))
)

(define (on-rule w1 w1_instance w2 w2_instance)
	(InheritanceLink (ConceptNode w1_instance) (ConceptNode w1))
	(InheritanceLink (ConceptNode w2_instance) (ConceptNode w2))
	(EvaluationLink (PredicateNode "On") (ConceptNode w1_instance) (ConceptNode w2_instance))
)

(define (to-do-rule-1 v1 v1_instance v2 v2_instance s s_instance o o_instance)
	(InheritanceLink (ConceptNode s_instance) (ConceptNode s))
	(InheritanceLink (ConceptNode o_instance) (ConceptNode o))
	(InheritanceLink (PredicateNode v1_instance) (PredicateNode v1))
	(InheritanceLink (PredicateNode v2_instance) (PredicateNode v2))
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
	)
)

(define (to-do-rule-2 v1 v1_instance v2 v2_instance s1 s1_instance s2 s2_instance o o_instance) 
	(InheritanceLink (ConceptNode s1_instance) (ConceptNode s1))
	(InheritanceLink (ConceptNode s2_instance) (ConceptNode s2))
	(InheritanceLink (ConceptNode o_instance) (ConceptNode o))
	(InheritanceLink (PredicateNode v1_instance) (PredicateNode v1))
	(InheritanceLink (PredicateNode v2_instance) (PredicateNode v2))
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
	)
)

(define (to-do-rule-3 v1 v1_instance v2 v2_instance v3 v3_instance)
        (InheritanceLink (ConceptNode v1_instance) (ConceptNode v1))
        (InheritanceLink (PredicateNode v2_instance) (PredicateNode v2))
        (InheritanceLink (ConceptNode v3_instance) (ConceptNode v3))
        (EvaluationLink
                (PredicateNode v2_instance)
                (ListLink
                        (InheritanceLink
                                (ConceptNode v3_instance)
                                (ConceptNode v1_instance)
                        )
                )
        )
)

(define (to-be-rule verb verb_ins adj adj_ins subj subj_ins)
	(InheritanceLink (PredicateNode verb_ins) (PredicateNode verb))
	(InheritanceLink (ConceptNode subj_ins) (ConceptNode subj))
	(InheritanceLink (ConceptNode adj_ins) (ConceptNode adj))
	(EvaluationLink
		(PredicateNode verb_ins)
		(InheritanceLink (ConceptNode subj_ins) (ConceptNode adj_ins))
	)
)

(define (all-rule noun  noun_instance)
	(ForAllLink (ConceptNode noun_instance)
		(InheritanceLink (ConceptNode noun_instance) (ConceptNode noun))
	)
)

(define (entity-rule word word_instance) 
	(InheritanceLink (SpecificEntityNode word_instance) (ConceptNode word))
)

(define (gender-rule word word_instance gender_type)
	(define concept_node (ConceptNode word))
	(InheritanceLink (SpecificEntityNode word_instance) (ConceptNode word))
	(InheritanceLink (SpecificEntityNode word_instance) (ConceptNode "person"))
	(cond ((string=? gender_type "feminine")
		(InheritanceLink (SpecificEntityNode word_instance) (ConceptNode "woman"))
		)
		((string=? gender_type "masculine")
		(InheritanceLink (SpecificEntityNode word_instance) (ConceptNode "man"))
		)
	)
)

(define (about-rule verb verb_instance  noun noun_instance)
	(InheritanceLink (PredicateNode verb_instance) (PredicateNode verb))
	(InheritanceLink (ConceptNode noun_instance) (ConceptNode noun))
	(EvaluationLink (PredicateNode "about") (PredicateNode verb_instance) (ConceptNode noun_instance))
) 

(define (nn-rule n1 n1_instance n2 n2_instance)
	(InheritanceLink (ConceptNode n1_instance) (ConceptNode n1))
	(InheritanceLink (ConceptNode n2_instance) (ConceptNode n2))
	(InheritanceLink (ConceptNode n1_instance) (ConceptNode n2_instance))
)

(define (SV-rule subj_concept  subj_instance  verb  verb_instance)
	(InheritanceLink (PredicateNode verb_instance) (PredicateNode verb))
	(InheritanceLink (ConceptNode subj_instance) (ConceptNode subj_concept))
	(EvaluationLink (PredicateNode verb_instance) (ConceptNode subj_instance))
)

(define (SVO-rule subj_concept  subj_instance  verb  verb_instance  obj_concept  obj_instance)
	(InheritanceLink (PredicateNode verb_instance) (PredicateNode verb))
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

(define (SVP-rule subj  subj_instance  predicative  predicative_instance)
	(InheritanceLink (ConceptNode predicative_instance) (ConceptNode predicative))
	(InheritanceLink (ConceptNode subj_instance) (ConceptNode subj))
	(InheritanceLink (ConceptNode subj_instance) (ConceptNode predicative_instance))
)

(define (which-rule antecedent  antecedent_instance  verb  verb_instance)
	(InheritanceLink (ConceptNode antecedent_instance) (ConceptNode antecedent))
	(InheritanceLink (PredicateNode verb_instance) (PredicateNode verb))
        (EvaluationLink
		(PredicateNode "whichmarker")
		(ListLink
			(ConceptNode antecedent_instance)
			(PredicateNode verb_instance)
		)
	)
)

(define (SVIO-rule subj_concept  subj_instance  verb  verb_instance  obj_concept  obj_instance iobj_concept iobj_instance)
	(InheritanceLink (PredicateNode verb_instance) (PredicateNode verb))
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

(define (be-inheritance-rule subj_concept subj_instance obj_concept obj_instance)
    (InheritanceLink (ConceptNode subj_instance) (ConceptNode subj_concept))
	(InheritanceLink (ConceptNode obj_instance) (ConceptNode obj_concept))
    (InheritanceLink (ConceptNode subj_instance) (ConceptNode obj_instance))
)
