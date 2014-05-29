; Notes: random-string, random-node-name and choose-var-name can be moved to utilities.scm

; -----------------------------------------------------------------------
; Defines a default simple truth value
; (Note: Without assigning this value, the default stv is (stv 1.0 0.0);
; PLN doesn't work on these as it requires links to have non-zero confidence
; values. Furthermore, some rules (e.g. AbductionRule) require a strength value
; smaller than 1.0. as -- conceptually -- this makes the Node equivalent to
; the entire universe.)
(define df-stv (stv .9 .9))

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
	(InheritanceLink  (ConceptNode adj_instance df-stv) (ConceptNode adj df-stv) df-stv)
	(InheritanceLink  (ConceptNode instance df-stv) (ConceptNode adj_instance df-stv) df-stv)
	(InheritanceLink  (ConceptNode instance df-stv) (ConceptNode concept df-stv) df-stv)
)

(define (advmod-rule verb instance adv adv_instance)
	(InheritanceLink  (ConceptNode adv_instance df-stv) (ConceptNode adv df-stv) df-stv)
	(InheritanceLink  (SatisfyingSetLink (PredicateNode instance df-stv) df-stv) (ConceptNode adv_instance df-stv) df-stv)
	(InheritanceLink  (PredicateNode instance df-stv) (PredicateNode verb df-stv) df-stv)
)

(define (tense-rule verb instance tense)
	(InheritanceLink (PredicateNode instance df-stv) (PredicateNode verb df-stv) df-stv)
	(InheritanceLink (PredicateNode instance df-stv) (ConceptNode tense df-stv) df-stv)
)

(define (det-rule concept instance var_name determiner)
	(cond ((or (string=? determiner "those") (string=? determiner "these"))
		(ImplicationLink df-stv
			(MemberLink (VariableNode var_name df-stv) (ConceptNode instance df-stv) df-stv)
			(InheritanceLink (VariableNode var_name df-stv) (ConceptNode concept df-stv) df-stv))
		)
		((or (string=? determiner "this") (string=? determiner "that"))
			(InheritanceLink (VariableNode var_name df-stv) (ConceptNode concept df-stv) df-stv)
		)
	)
)

(define (negative-rule verb instance) 
	(InheritanceLink (PredicateNode instance df-stv) (PredicateNode verb df-stv) df-stv)
	(NotLink (PredicateNode instance df-stv) df-stv)
)

(define (possessive-rule noun noun_instance word word_instance)
	(InheritanceLink (ConceptNode noun_instance df-stv) (ConceptNode noun df-stv) df-stv)
        (InheritanceLink (ConceptNode word_instance df-stv) (ConceptNode word df-stv) df-stv)
	(EvaluationLink df-stv
		(PredicateNode "Possession" df-stv)
		(ListLink df-stv
			(ConceptNode noun_instance df-stv)
			(ConceptNode word_instance df-stv)
		)
	)
)

(define (comparative-rule w1 w1_instance w2 w2_instance adj adj_instance)
	(InheritanceLink (ConceptNode adj_instance df-stv) (ConceptNode adj df-stv) df-stv)
	(InheritanceLink (ConceptNode w1_instance df-stv) (ConceptNode w1 df-stv) df-stv)
	(InheritanceLink (ConceptNode w2_instance df-stv) (ConceptNode w2 df-stv) df-stv)
	(TruthValueGreaterThanLink df-stv
		(InheritanceLink (ConceptNode w1_instance df-stv) (ConceptNode adj_instance df-stv) df-stv)
		(InheritanceLink (ConceptNode w2_instance df-stv) (ConceptNode adj_instance df-stv) df-stv)
	)
)

(define (number-rule noun noun_instance num num_instance)
	(define noun_ins_concept (ConceptNode noun_instance df-stv))
	(InheritanceLink noun_ins_concept (ConceptNode noun df-stv) df-stv)
	(InheritanceLink (NumberNode num_instance df-stv) (NumberNode num df-stv) df-stv)
	(QuantityLink (ConceptNode noun_instance df-stv) (NumberNode num_instance df-stv) df-stv)
)

(define (on-rule w1 w1_instance w2 w2_instance)
	(InheritanceLink (ConceptNode w1_instance df-stv) (ConceptNode w1 df-stv) df-stv)
	(InheritanceLink (ConceptNode w2_instance df-stv) (ConceptNode w2 df-stv) df-stv)
	(EvaluationLink (PredicateNode "On" df-stv) (ConceptNode w1_instance df-stv) (ConceptNode w2_instance df-stv) df-stv)
)

(define (to-do-rule-1 v1 v1_instance v2 v2_instance s s_instance o o_instance)
	(InheritanceLink (ConceptNode s_instance df-stv) (ConceptNode s df-stv) df-stv)
	(InheritanceLink (ConceptNode o_instance df-stv) (ConceptNode o df-stv) df-stv)
	(InheritanceLink (PredicateNode v1_instance df-stv) (PredicateNode v1 df-stv) df-stv)
	(InheritanceLink (PredicateNode v2_instance df-stv) (PredicateNode v2 df-stv) df-stv)
	(EvaluationLink df-stv
		(PredicateNode v1_instance df-stv)
		(ListLink df-stv
			(ConceptNode s_instance df-stv)
			(EvaluationLink df-stv
				(PredicateNode v2_instance df-stv)
				(ListLink df-stv
					(ConceptNode s_instance df-stv)
					(ConceptNode o_instance df-stv)
				)
			)
		)
	)
)

(define (to-do-rule-2 v1 v1_instance v2 v2_instance s1 s1_instance s2 s2_instance o o_instance) 
	(InheritanceLink (ConceptNode s1_instance df-stv) (ConceptNode s1 df-stv) df-stv)
	(InheritanceLink (ConceptNode s2_instance df-stv) (ConceptNode s2 df-stv) df-stv)
	(InheritanceLink (ConceptNode o_instance df-stv) (ConceptNode o df-stv) df-stv)
	(InheritanceLink (PredicateNode v1_instance df-stv) (PredicateNode v1 df-stv) df-stv)
	(InheritanceLink (PredicateNode v2_instance df-stv) (PredicateNode v2 df-stv) df-stv)
	(EvaluationLink df-stv
		(PredicateNode v1_instance df-stv)
		(ListLink df-stv
			(ConceptNode s1_instance df-stv)
			(EvaluationLink df-stv
				(PredicateNode v2_instance df-stv)
				(ListLink df-stv
					(ConceptNode s2_instance df-stv)
					(ConceptNode o_instance df-stv)
				)
			)
		)
	)
)

(define (to-do-rule-3 v1 v1_instance v2 v2_instance v3 v3_instance)
        (InheritanceLink (ConceptNode v1_instance df-stv) (ConceptNode v1 df-stv) df-stv)
        (InheritanceLink (PredicateNode v2_instance df-stv) (PredicateNode v2 df-stv) df-stv)
        (InheritanceLink (ConceptNode v3_instance df-stv) (ConceptNode v3 df-stv) df-stv)
        (EvaluationLink df-stv
                (PredicateNode v2_instance df-stv)
                (ListLink df-stv
                        (InheritanceLink df-stv
                                (ConceptNode v3_instance df-stv)
                                (ConceptNode v1_instance df-stv)
                        )
                )
        )
)

(define (to-be-rule verb verb_ins adj adj_ins subj subj_ins)
	(InheritanceLink (PredicateNode verb_ins df-stv) (PredicateNode verb df-stv) df-stv)
	(InheritanceLink (ConceptNode subj_ins df-stv) (ConceptNode subj df-stv) df-stv)
	(InheritanceLink (ConceptNode adj_ins df-stv) (ConceptNode adj df-stv) df-stv)
	(EvaluationLink df-stv
		(PredicateNode verb_ins df-stv)
		(InheritanceLink (ConceptNode subj_ins df-stv) (ConceptNode adj_ins df-stv) df-stv)
	)
)

(define (all-rule noun  noun_instance)
	(ForAllLink (ConceptNode noun_instance df-stv) df-stv
		(InheritanceLink (ConceptNode noun_instance df-stv) (ConceptNode noun df-stv) df-stv)
	)
)

(define (entity-rule word word_instance) 
	(InheritanceLink (SpecificEntityNode word_instance df-stv) (ConceptNode word df-stv) df-stv)
)

(define (gender-rule word word_instance gender_type)
	(define concept_node (ConceptNode word df-stv)) ; This rule is not used.
	(InheritanceLink (SpecificEntityNode word_instance df-stv) (ConceptNode word df-stv) df-stv)
	(InheritanceLink (SpecificEntityNode word_instance df-stv) (ConceptNode "person" df-stv) df-stv)
	(cond ((string=? gender_type "feminine")
		(InheritanceLink (SpecificEntityNode word_instance df-stv) (ConceptNode "woman" df-stv) df-stv)
		)
		((string=? gender_type "masculine")
		(InheritanceLink (SpecificEntityNode word_instance df-stv) (ConceptNode "man" df-stv) df-stv)
		)
	)
)

(define (about-rule verb verb_instance  noun noun_instance)
	(InheritanceLink (PredicateNode verb_instance df-stv) (PredicateNode verb df-stv) df-stv)
	(InheritanceLink (ConceptNode noun_instance df-stv) (ConceptNode noun df-stv) df-stv)
	(EvaluationLink (PredicateNode "about" df-stv) (PredicateNode verb_instance df-stv) (ConceptNode noun_instance df-stv) df-stv)
) 

(define (nn-rule n1 n1_instance n2 n2_instance)
	(InheritanceLink (ConceptNode n1_instance df-stv) (ConceptNode n1 df-stv) df-stv)
	(InheritanceLink (ConceptNode n2_instance df-stv) (ConceptNode n2 df-stv) df-stv)
	(InheritanceLink (ConceptNode n1_instance df-stv) (ConceptNode n2_instance df-stv) df-stv)
)

(define (SV-rule subj_concept  subj_instance  verb  verb_instance)
	(InheritanceLink (PredicateNode verb_instance df-stv) (PredicateNode verb df-stv) df-stv)
	(InheritanceLink (ConceptNode subj_instance df-stv) (ConceptNode subj_concept df-stv) df-stv)
	(EvaluationLink (PredicateNode verb_instance df-stv) (ConceptNode subj_instance df-stv) df-stv)
)

(define (SVO-rule subj_concept  subj_instance  verb  verb_instance  obj_concept  obj_instance)
	(InheritanceLink (PredicateNode verb_instance df-stv) (PredicateNode verb) df-stv)
	(InheritanceLink (ConceptNode subj_instance df-stv) (ConceptNode subj_concept df-stv) df-stv)
	(InheritanceLink (ConceptNode obj_instance df-stv) (ConceptNode obj_concept df-stv) df-stv)
	(EvaluationLink df-stv
		(PredicateNode verb_instance df-stv)
		(ListLink df-stv
			(ConceptNode subj_instance df-stv)
			(ConceptNode obj_instance df-stv)
		)
	)
)

(define (SVP-rule subj  subj_instance  predicative  predicative_instance)
	(InheritanceLink (ConceptNode predicative_instance df-stv) (ConceptNode predicative df-stv) df-stv)
	(InheritanceLink (ConceptNode subj_instance df-stv) (ConceptNode subj df-stv) df-stv)
	(InheritanceLink (ConceptNode subj_instance df-stv) (ConceptNode predicative_instance df-stv) df-stv)
)

(define (which-rule antecedent  antecedent_instance  verb  verb_instance)
	(InheritanceLink (ConceptNode antecedent_instance df-stv) (ConceptNode antecedent df-stv) df-stv)
	(InheritanceLink (PredicateNode verb_instance df-stv) (PredicateNode verb df-stv) df-stv)
        (EvaluationLink df-stv
		(PredicateNode "whichmarker" df-stv)
		(ListLink df-stv
			(ConceptNode antecedent_instance df-stv)
			(PredicateNode verb_instance df-stv)
		)
	)
)

(define (SVIO-rule subj_concept  subj_instance  verb  verb_instance  obj_concept  obj_instance iobj_concept iobj_instance)
	(InheritanceLink (PredicateNode verb_instance df-stv) (PredicateNode verb df-stv) df-stv)
	(InheritanceLink (ConceptNode subj_instance df-stv) (ConceptNode subj_concept df-stv) df-stv)
	(InheritanceLink (ConceptNode obj_instance df-stv) (ConceptNode obj_concept df-stv) df-stv)
	(InheritanceLink (ConceptNode iobj_instance df-stv) (ConceptNode iobj_concept df-stv) df-stv)
	(EvaluationLink df-stv
            (PredicateNode verb_instance df-stv)
        	(ListLink df-stv
			    (ConceptNode subj_instance df-stv)
		    	(ConceptNode obj_instance df-stv)
		    	(ConceptNode iobj_instance df-stv)
        	)
    	)
)

; Examples: "Socrates is a man", "Cats are animals", "Trees are plants"
(define (be-inheritance-rule subj_concept subj_instance obj_concept obj_instance)
	(InheritanceLink (ConceptNode subj_instance df-stv) (ConceptNode subj_concept df-stv) df-stv)
	(InheritanceLink (ConceptNode obj_instance df-stv) (ConceptNode obj_concept df-stv) df-stv)
	(InheritanceLink (ConceptNode subj_instance df-stv) (ConceptNode obj_instance df-stv) df-stv)
)


;Example: "The books were written by Charles Dickens."
(define (passive-rule1 verb verb_instance obj obj_instance passive_obj passive_obj_instance)
        (InheritanceLink (PredicateNode verb_instance df-stv) (PredicateNode verb df-stv) df-stv)
        (InheritanceLink (ConceptNode obj_instance df-stv) (ConceptNode obj df-stv) df-stv)
        (InheritanceLink (ConceptNode passive_obj_instance df-stv) (ConceptNode passive_obj df-stv) df-stv)
        (EvaluationLink df-stv
                (PredicateNode verb_instance df-stv)
                (ListLink df-stv
                        (ConceptNode passive_obj_instance df-stv)
                        (ConceptNode obj_instance df-stv)
                )
      )
)

;Example: "The books are published."
(define (passive-rule2 verb verb_instance obj obj_instance)
        (InheritanceLink (PredicateNode verb_instance df-stv) (PredicateNode verb df-stv) df-stv)
        (InheritanceLink (ConceptNode obj_instance df-stv) (ConceptNode obj df-stv) df-stv)
        (EvaluationLink df-stv
                (PredicateNode verb_instance df-stv)
                (ListLink df-stv
                        (VariableNode "$x" df-stv)
                        (ConceptNode obj_instance df-stv)
                )
        )
)
