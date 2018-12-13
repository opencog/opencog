; e.g. she runs
(define bind-subj
  (Bind
    (VariableList
      (TypedVariable (Variable "$sent") (Type "SentenceNode"))
      (TypedVariable (Variable "$parse") (Type "ParseNode"))
      (TypedVariable (Variable "$tense") (Type "DefinedLinguisticConceptNode"))
      (TypedVariable (Variable "$word-inst-p") (Type "WordInstanceNode"))
      (TypedVariable (Variable "$word-inst-s") (Type "WordInstanceNode"))
      (TypedVariable (Variable "$pred-inst") (Type "PredicateNode"))
      (TypedVariable (Variable "$cnpt-inst") (Type "ConceptNode"))
      (TypedVariable (Variable "$pred") (Type "PredicateNode"))
      (TypedVariable (Variable "$cnpt") (Type "ConceptNode")))
    (And
      (State (Anchor "GHOST Currently Processing") (Variable "$sent"))
      (Parse (Variable "$parse") (Variable "$sent"))
      (WordInstance (Variable "$word-inst-p") (Variable "$parse"))
      (WordInstance (Variable "$word-inst-s") (Variable "$parse"))
      (Reference (Variable "$pred-inst") (Variable "$word-inst-p"))
      (Reference (Variable "$cnpt-inst") (Variable "$word-inst-s"))
      (Implication (Variable "$pred-inst") (Variable "$pred"))
      (Inheritance (Variable "$cnpt-inst") (Variable "$cnpt"))
      (Tense (Variable "$word-inst-p") (Variable "$tense"))
      (Evaluation
        (DefinedLinguisticRelationship "_subj")
        (List (Variable "$word-inst-p") (Variable "$word-inst-s"))))
    (Set
      (Inheritance
        (InterpretationNode "MicroplanningNewSentence")
        (DefinedLinguisticConcept "InterrogativeSpeechAct"))
      (Inheritance
        (Variable "$pred")
        (ExecutionOutput
          (GroundedSchema "scm: tense-to-inf")
          (List (Variable "$tense"))))
      (Evaluation
        (Variable "$pred")
        (List (Variable "$cnpt-inst"))))))

(define-public (tense-to-inf dlcn)
"
  tense-to-inf DLCN

  DLCN is a DefinedLinguisticConceptNode, denoting the tense of a predicate.

  This function turns present/past tense into present_infinitive/past_infinitive
  tense. It's just for the stochastic question generator.
"
  (define tense (cog-name dlcn))

  ; To turn a statement, e.g. "she runs", into a question, e.g.
  ; "why does she run", requires turning the tense from present/past
  ; to present-infinitive/past-infinitive, and this should be done
  ; before passing it to SuReal.
  (cond
    ((string=? "present" tense) (DefinedLinguisticConcept "present_infinitive"))
    ((string=? "past" tense) (DefinedLinguisticConcept "past_infinitive"))
    (else dlcn))
)
