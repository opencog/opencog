; Facts taken from SimpleWikipedia:


; A scientist is a person who works in science.

; It would be good if there would be a way to only display the atoms
; produced by RelEx2Logic in the AtomSpace and be able to hide all
; the WordInstanceNodes, DefinedLinguisticRelationshipNodes, etc.
; This would not only be valuable for inspecting the output, but also for
; checking without distractions if PLN performs the right deductions.

; Desired output (with some refinements):

(InheritanceLink (stv 1.0 1.0)
    (ConceptNode "scientist")
    (AndLink
        (InheritanceLink
            (VariableNode "$x")
            (ConceptNode "person"))
        (EvaluationLink
            (PredicadeNode "work_in") ; how would 'in' be handled?
            (ListLink
                (VariableNode "$x")
                (ConceptNode "science")))))

; Current RelEx2Logic output:

; be-inheritance-rule
(InheritanceLink (stv 1.000000 1.000000)
  (ConceptNode "scientist@5d709cad-32b0-4d4c-93fc-0fee959d6a94") ; [365]
  (ConceptNode "scientist") ; [366]
) ; [367]

(InheritanceLink (stv 1.000000 1.000000)
  (ConceptNode "person@faa54fae-15dc-4bbc-a714-8449221e1768") ; [368]
  (ConceptNode "person") ; [369]
) ; [370]

(InheritanceLink (stv 1.000000 1.000000)
  (ConceptNode "scientist@5d709cad-32b0-4d4c-93fc-0fee959d6a94") ; [365]
  (ConceptNode "person@faa54fae-15dc-4bbc-a714-8449221e1768") ; [368]
) ; [371]

; tense-rule
(InheritanceLink (stv 1.000000 0.000000)
  (PredicateNode "is@fb91bdf4-a47a-4d67-8732-af8f2a73f294") ; [372]
  (PredicateNode "be") ; [373]
) ; [374]

; The SVO-rule isn't applied for the subordinate clause because,
; currently, a RelEx2Logic rule can only be applied once per sentence.
; As be-inheritance is already applied, this currently prevents SVO from being
; applied to the subordinate clause because they are disjoint.
; This is discussed here: https://github.com/opencog/relex/issues/39 and
; should be fixed to deal with this sentence.


; A scientist tries to understand how our world works.

; Desired output (needs some refinement):

(EvaluationLink (stv 1.0 1.0)
    (PredicadeNode "try to understand") ; how would infinitive constructions be handled?
    (ListLink
        (ConceptNode "scientist")
        (ConceptNode "how our world works"))) ; how are embedded sentences handled?

; There are three to-do-rules. None, however, deal with this constellation:
; _to-do(try, understand)
; _subj(try, scientist)
; _subj(work, world)
; 
; (S (NP A scientist.n) (VP tries.v (S (VP to.r (VP understand.v (SBAR (WHADVP how) (S (NP our world.n) (VP works.v))))))) .)

; Current RelEx2Logic output:

(InheritanceLink (stv 1.000000 0.000000)
  (ConceptNode "world@15badd06-cb2c-4d39-adac-0253d6c74b01") ; [538]
  (ConceptNode "world") ; [539]
) ; [540]

(InheritanceLink (stv 1.000000 0.000000)
  (ConceptNode "our@d06ca837-7ee6-4f4a-a181-3a70916d4b95") ; [542]
  (ConceptNode "us") ; [543]
) ; [544]

(InheritanceLink (stv 1.000000 0.000000)
  (PredicateNode "works@463aae53-ef79-4cd9-8b4e-ee67c7481e40") ; [533]
  (PredicateNode "work") ; [534]
) ; [535]

(InheritanceLink (stv 1.000000 0.000000)
  (PredicateNode "works@463aae53-ef79-4cd9-8b4e-ee67c7481e40") ; [533]
  (ConceptNode "present") ; [536]
) ; [537]

(EvaluationLink (stv 1.000000 0.000000)
  (PredicateNode "Possession") ; [545]
  (ListLink (stv 1.000000 0.000000)
    (ConceptNode "world@15badd06-cb2c-4d39-adac-0253d6c74b01") ; [538]
    (ConceptNode "our@d06ca837-7ee6-4f4a-a181-3a70916d4b95") ; [542]
  ) ; [546]
) ; [547]

(EvaluationLink (stv 1.000000 0.000000)
  (PredicateNode "works@463aae53-ef79-4cd9-8b4e-ee67c7481e40") ; [533]
  (ConceptNode "world@15badd06-cb2c-4d39-adac-0253d6c74b01") ; [538]
) ; [541]

; Same problem as above. 'A scientist tries to understand' is not captured.


; Scientist make observations and ask questions (and do extensive research work in finding the answers to these questions).

; Desired output:

(EvaluationLink (stv 1.0 1.0)
    (PredicadeNode "make")
    (ListLink
        (ConceptNode "scientist")
        (ConceptNode "observations")))

(EvaluationLink (stv 1.0 1.0)
    (PredicadeNode "ask")
    (ListLink
        (ConceptNode "scientist")
        (ConceptNode "questions")))

; Current RelEx2Logic output:

; The tree is parsed correctly by link-grammar: (S (NP Scientists.n) (VP make.v (NP observations.n) and.j-v ask.v (NP questions.n)) .)

; RelEx, however, isn't able to assign the objects correctly:
; _obj(make, make)
; _obj(make, ask)
; conj_and(make, ask)
; _subj(make, scientist)
; _subj(ask, scientist)

; Is this a general issue that RelEx isn't able to handle conjunction yet?


; Scientists work for governments, companies, schools and research institutes.

; (Approximate) desired output:

(EvaluationLink (stv 1.0 1.0)
    (PredicadeNode "work_for")
    (ListLink
        (ConceptNode "scientist")
        (OrLink ; I wonder how RelEx2Logic would be able to realize that these nouns
                ; are connected by OR and not by AND
            (ConceptNode "governments")
            (ConceptNode "companies")
            (ConceptNode "schools")
            (ConceptNode "research institutes"))))

; Current RelEx2Logic output:
 
; Seemingly, RelEx isn't able to handle compound verbs. I imagine this is a known issue? 
 
; for(work, and)
; _subj(work, scientist)
; conj_and(,, institute)
; _nn(institute, research)

; The same applies to the enumeration, even though link-grammar does a good job parsing the sentence:
; (S (NP Scientists.n) (VP work.v (PP for.p governments.n , companies.n , schools.n and.j-n research.n-u institutes.n)) .)


; Some scientists teach at universities (and other places and train people to become scientists).

; (Approximate) desired output:

(EvaluationLink (stv .5 .8)
    (PredicadeNode "teach_at")
    (ListLink
        (ConceptNode "scientist")
        (ConceptNode "universities")))

; Current RelEx2Logic output:

; Rules: SV-rule, tense-rule

; Relations:
; at(teach, university)
; _subj(teach, scientist)
; _quantity(scientist, some)

(EvaluationLink (stv 1.000000 0.000000)
  (PredicateNode "teach@e1ae1613-14b5-4316-9ccc-1f3b40f8d86b") ; [1013]
  (ConceptNode "scientists@dfd9c430-1675-46f5-8c8c-6cfa7de97e61") ; [1018]
) ; [1021]

; Quantifier 'some' should modify stv.
; Rule to make sense of spatial relation needed.


; Scientists often make experiments to find out more about reality.

(EvaluationLink (stv .7 .8)
    (PredicadeNode "make")
    (ListLink
        (ConceptNode "scientist")
        (ConceptNode "experiments")))


; Scientists sometimes may repeat experiments or use control groups.

(EvaluationLink (stv .3 .5)
    (PredicadeNode "repeat")
    (ListLink
        (ConceptNode "scientist")
        (ConceptNode "experiments")))


; Scientific work is also carred out in laboratories.

(InheritanceLink
    (ConceptNode "scientific_work")
    (ConceptNode "science"))


; Scientists are more likely to be atheists than the rest of the general population.

(InheritanceLink (stv .2 .2) ; to estimate this, the likelihood of the general population
                             ; to be atheist has to be known
    (ConceptNode "scientist")
    (ConceptNode "atheist"))


; Scientists that study physics are physicists. 

(InheritanceLink (stv 1.0 1.0)
    (AndLink
        (InheritanceLink
            (VariableNode "$x")
            (ConceptNode "scientist"))
        (EvaluationLink
            (PredicadeNode "study")
            (ListLink
                (VariableNode "$x")
                (ConceptNode "physics"))))
    (ConceptNode "physicist"))


; Scientists write papers.

(EvaluationLink (stv 1.0 1.0)
    (PredicadeNode "write")
    (ListLink
        (ConceptNode "scientist")
        (ConceptNode "papers")))


; A few scientists win a nobel prize.

(EvaluationLink (stv .1 .1)
    (PredicadeNode "win")
    (ListLink
        (ConceptNode "scientist")
        (ConceptNode "nobel_prize")))


; Example sentences for inference:


; Galileo made experiments about free fall.

(EvaluationLink (stv 1.0 1.0)
    (PredicadeNode "make")
    (ListLink
        (ConceptNode "Galileo")
        (ConceptNode "experiments")))

; Infer: Is Galileo a scientist?


; Einstein won a nobel prize.

(EvaluationLink (stv 1.0 1.0)
    (PredicadeNode "win")
    (ListLink
        (ConceptNode "Einstein")
        (ConceptNode "nobel_prize")))

; Infer: Did Einstein make experiments?
