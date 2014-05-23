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


; Scientists often make experiments (to find out more about reality).

; (Approximate) desired output:

(EvaluationLink (stv .7 .8)
    (PredicadeNode "make")
    (ListLink
        (ConceptNode "scientist")
        (ConceptNode "experiments")))

; Current output:
; 
; _obj(make, experiment)
; _advmod(make, often)
; _subj(make, scientist)
; 
; SVO-rule, tense-rule and adv-mod-rule are used.

(InheritanceLink (stv 1.000000 0.000000)
  (PredicateNode "make@d6d5ff22-4fd7-4616-a443-31147a8bdb67") ; [220]
  (PredicateNode "make") ; [221]
) ; [222]

(InheritanceLink (stv 1.000000 0.000000)
  (ConceptNode "scientists@1dac5b75-dadd-4c04-aa63-dc56e63f802c") ; [223]
  (ConceptNode "scientist") ; [224]
) ; [225]

(InheritanceLink (stv 1.000000 0.000000)
  (ConceptNode "experiments@80482db7-def5-4f26-9f6a-1505bb94b7d6") ; [226]
  (ConceptNode "experiment") ; [227]
) ; [228]

(InheritanceLink (stv 1.000000 0.000000)
  (PredicateNode "make@d6d5ff22-4fd7-4616-a443-31147a8bdb67") ; [220]
  (ConceptNode "present") ; [231]
) ; [232]

(InheritanceLink (stv 1.000000 0.000000)
  (ConceptNode "often@bf26a7e3-a467-40bd-922c-fb2d87221fe8") ; [233]
  (ConceptNode "often") ; [234]
) ; [235]

(InheritanceLink (stv 1.000000 0.000000)
  (SatisfyingSetLink (stv 1.000000 0.000000)
    (PredicateNode "make@d6d5ff22-4fd7-4616-a443-31147a8bdb67") ; [220]
  ) ; [236]
  (ConceptNode "often@bf26a7e3-a467-40bd-922c-fb2d87221fe8") ; [233]
) ; [237]

(EvaluationLink (stv 1.000000 1.000000)
  (PredicateNode "make@d6d5ff22-4fd7-4616-a443-31147a8bdb67") ; [220]
  (ListLink (stv 1.000000 0.000000)
    (ConceptNode "scientists@1dac5b75-dadd-4c04-aa63-dc56e63f802c") ; [223]
    (ConceptNode "experiments@80482db7-def5-4f26-9f6a-1505bb94b7d6") ; [226]
  ) ; [229]
) ; [230]

; Often needs to modify stv.


; Scientists sometimes may repeat experiments or use control groups.

; (Approximate) desired output:

(EvaluationLink (stv .3 .5)
    (PredicadeNode "repeat")
    (ListLink
        (ConceptNode "scientist")
        (ConceptNode "experiments")))

; Current output:

; conj_or(repeat, use)
; _obj(repeat, repeat)
; _obj(repeat, use)
; _nn(group, control)
; _obj(or, group)
; _advmod(or, sometimes)
; _subj(or, scientist)
; 
; SVO-rule is falsely applied: SVO-rule "scientist" "or" "group"
; Other applied rules: tense-rule "repeat" "present", advmod-rule "or" "sometimes",
; nn-rule "group" "control"
; 
; Needed: rule using conj_or($v1, $v2)


; Scientific work is also carred out in laboratories.

(InheritanceLink
    (ConceptNode "scientific_work")
    (ConceptNode "science"))


; Scientists are more likely to be atheists than the rest of the general population.

(InheritanceLink (stv .2 .2) ; to estimate this, the likelihood of the general population
                             ; to be atheist has to be known
    (ConceptNode "scientist")
    (ConceptNode "atheist"))

; This is quite difficult. The stv has to be estimated using knowledge of the rest of
; the universe.

; Scientists that study physics are physicists. 

; Desired output:

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

; Current RelEx relations:

bj(be, physicist)
_subj(be, scientist)
_obj(study, physics)
_subj(study, scientist)
that_adj(scientist, study)

; Currently applied R2L rules: be-inheritance-rule "scientist" "physicist",
; tense-rule "be" "present"

; RelEx relations are all in place. Required rules for that-subordinate clause;
; disjoint rules need to be possible for different clauses.


; Scientists write papers.

; Desired output:

(EvaluationLink (stv 1.0 1.0)
    (PredicadeNode "write")
    (ListLink
        (ConceptNode "scientist")
        (ConceptNode "papers")))

; Current output:

(EvaluationLink (stv 1.000000 1.000000)
  (PredicateNode "write@7255c1d6-2cce-47d5-9cc1-80bd3cbf5d2e") ; [586]
  (ListLink (stv 1.000000 0.000000)
    (ConceptNode "scientists@1f0e427c-4772-4a1e-8883-40f229a65d46") ; [589]
    (ConceptNode "papers@53f7eaef-ed9a-4736-9a93-c75d360a2c84") ; [592]
  ) ; [595]
) ; [596] check!


; A few scientists win a nobel prize.

; (Approximate) desired output:

(EvaluationLink (stv .1 .1)
    (PredicadeNode "win")
    (ListLink
        (ConceptNode "scientist")
        (ConceptNode "nobel_prize")))

; Current RelEx relations:

_obj(win, prize)
_subj(win, scientist)
_nn(prize, nobel)
_quantity(scientist, a_few)

; Currently applied R2L rules: SVO-rule "scientist" "win" "prize",
; tense-rule "win" "present", nn-rule "nobel" "prize"

; One rule needed that modifies stvs based on _quantity relation;
; probably best using scoped variables.


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
