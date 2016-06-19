;
; rule-utils.scm
;
; Some generic rule utilities, not limited to r2l.
;
;--------------------------------------------------------------------
;
; Short-hand for declaring a variable.
(define (var-decl var type)
   (TypedVariableLink (VariableNode var) (TypeNode type)))

(define (var-decl-choice var type-a type-b)
   (TypedVariableLink (VariableNode var)
		(TypeChoice (TypeNode type-a) (TypeNode type-b))))

(define (word-in-parse word-inst parse)
"  The WordInstanceNode WORD is in ParseNode PARSE. "
	(WordInstanceLink (VariableNode word-inst) (VariableNode parse)))

(define (interp-of-parse interp parse)
"  The InterpretationNode INTERP is in ParseNode PARSE. "
	(InterpretationLink (VariableNode interp) (VariableNode parse)))

(define (parse-of-sent parse sent)
"  The ParseNode PARSE of SentenceNode SENTENCE. "
	(ParseLink (VariableNode parse) (VariableNode sent)))

(define (word-lemma word-inst lemma)
"  The WordInstanceNode WORD has lemma LEMMA. "
	(LemmaLink (VariableNode word-inst) (VariableNode lemma)))


(define (dependency rel head dep)
"  RelEx dependency relation REL(HEAD, DEP) "
	(EvaluationLink
		(DefinedLinguisticRelationshipNode rel)
		(ListLink (VariableNode head) (VariableNode dep))))

(define (lg-link rel left right)
"  Link Grammar link LEFT-REL-RIGHT "
	(EvaluationLink
		(LinkGrammarRelationshipNode rel)
		(ListLink (VariableNode left) (VariableNode right))))


(define (word-pos word-inst pos)
"  The WordInstanceNode WORD has PartOfSpeechLink POS. "
	(PartOfSpeechLink (VariableNode word-inst)
		(DefinedLinguisticConceptNode pos)))

(define (word-feat word-inst feat)
"  The WordInstanceNode WORD has InheritanceLink FEAT. "
	(InheritanceLink (VariableNode word-inst)
		(DefinedLinguisticConceptNode feat)))

(define (verb-tense verb-inst tense)
"  The WordInstanceNode VERB has Tense TENSE. "
	(TenseLink (VariableNode verb-inst)
		(DefinedLinguisticConceptNode tense)))
