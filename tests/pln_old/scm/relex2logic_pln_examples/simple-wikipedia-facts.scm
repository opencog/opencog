; Facts taken from SimpleWikipedia:

; See here for useful discussions concerning the sentences:
; - https://github.com/opencog/opencog/pull/778/files#r12911998

; A scientist is a person who works in science.

; Desired output (with some refinements):

(InheritanceLink (stv 1.0 1.0)
    (ConceptNode "scientist")
    (SatisfyingSetScopeLink
        (VariableNode "$x")
        (AndLink
            (InheritanceLink
                (VariableNode "$x")
                (ConceptNode "person"))
            (EvaluationLink
                (PredicadeNode "work_in") 
                (ListLink
                    (VariableNode "$x")
                    (ConceptNode "science")))))

; Current RelEx2Logic output:

; SV-rule
(EvaluationLink (stv 1.000000 0.000000)
  (PredicateNode "works@b54f5154-d1e5-4ccc-9b24-b6607a7059f5") ; [209]
  (ConceptNode "person@14eeec29-2e60-4a17-93bc-9e330ca11a0a") ; [200]
) ; [213]

; tense-rule
(InheritanceLink (stv 1.000000 0.000000)
  (PredicateNode "is@efbcc00b-4b6f-4d5d-804e-6a8951dc5cb4") ; [204]
  (DefinedLinguisticConceptNode "present") ; [207]
) ; [208]

(InheritanceLink (stv 1.000000 0.000000)
  (PredicateNode "works@b54f5154-d1e5-4ccc-9b24-b6607a7059f5") ; [209]
  (PredicateNode "work") ; [210]
) ; [211]

; tense-rule
(InheritanceLink (stv 1.000000 0.000000)
  (PredicateNode "works@b54f5154-d1e5-4ccc-9b24-b6607a7059f5") ; [209]
  (DefinedLinguisticConceptNode "present") ; [207]
) ; [212]

(InheritanceLink (stv 1.000000 1.000000)
  (ConceptNode "scientist@337d31cb-04e0-41d8-8885-1a4021ea94ab") ; [197]
  (ConceptNode "scientist") ; [198]
) ; [199]

(InheritanceLink (stv 1.000000 1.000000)
  (ConceptNode "person@14eeec29-2e60-4a17-93bc-9e330ca11a0a") ; [200]
  (ConceptNode "person") ; [201]
) ; [202]

; be-inheritance-rule
(InheritanceLink (stv 1.000000 1.000000)
  (ConceptNode "scientist@337d31cb-04e0-41d8-8885-1a4021ea94ab") ; [197]
  (ConceptNode "person@14eeec29-2e60-4a17-93bc-9e330ca11a0a") ; [200]
) ; [203]

; As pointed out by Ruiting, link grammar can handle verbs with particles.
; RelEx should be able to produce PredicateNodes for common verb+particle
; pairs.
; Not feasible because there are so many combinations? (see below: teach_at, etc) 
; Also, verbs + particles and prepositions have to be differentiated.


; A scientist tries to understand how our world works.

; Desired output (needs some refinement):

(EvaluationLink (stv 1.0 1.0)
    (PredicateNode "try")
    (ListLink
        (ConceptNode "scientist")
        (EvaluationLink
            (PredicateNode "understand")
            (ListLink
                (ConceptNode "scientist")
                (SatisfyingSetScopeLink
                    (VariableNode "$x")
                    (EvaluationLink
                        (PredicateNode "how")
                        (ListLink
                            (VariableNode "$x")
                            (EvaluationLink
                                (PredicateNode "work")
                                (ConceptNode "world")))))))))

; Link-grammar sentence parse:

; (S (NP A scientist.n) (VP tries.v (S (VP to.r (VP understand.v (SBAR (WHADVP how) (S (NP our world.n) (VP works.v))))))) .)

; Current RelEx2Logic output:

(ListLink (stv 0.990000 0.990000)
  (ConceptNode "world@cf823ba7-fdca-4674-851f-51988a8de699") ; [375]
  (ConceptNode "our@16e09edd-8f0c-4b93-8b7e-c2a3a4d14c7d") ; [379]
) ; [383]

(EvaluationLink (stv 1.000000 0.000000)
  (PredicateNode "understand@26f7796c-b25d-49d9-92be-5b6642a8640b") ; [392]
  (ConceptNode "scientist@3fc15db2-5bd5-41e2-a9fe-55ca0d38d9a6") ; [389]
) ; [395]

(EvaluationLink (stv 1.000000 0.000000)
  (PredicateNode "tries@8e4f8728-d2b3-4f1d-aa87-45cd9c24e6aa") ; [385]
  (EvaluationLink (stv 1.000000 0.000000)
    (PredicateNode "understand@26f7796c-b25d-49d9-92be-5b6642a8640b") ; [392]
    (ConceptNode "scientist@3fc15db2-5bd5-41e2-a9fe-55ca0d38d9a6") ; [389]
  ) ; [395]
) ; [396]

(EvaluationLink (stv 0.990000 0.990000)
  (PredicateNode "tries@8e4f8728-d2b3-4f1d-aa87-45cd9c24e6aa") ; [385]
  (ConceptNode "scientist@3fc15db2-5bd5-41e2-a9fe-55ca0d38d9a6") ; [389]
) ; [397]

(EvaluationLink (stv 0.990000 0.990000)
  (PredicateNode "works@562a14be-6028-4012-be39-5724003ecafc") ; [370]
  (ConceptNode "world@cf823ba7-fdca-4674-851f-51988a8de699") ; [375]
) ; [378]

(EvaluationLink (stv 0.990000 0.990000)
  (DefinedLinguisticPredicateNode "possession") ; [382]
  (ListLink (stv 0.990000 0.990000)
    (ConceptNode "world@cf823ba7-fdca-4674-851f-51988a8de699") ; [375]
    (ConceptNode "our@16e09edd-8f0c-4b93-8b7e-c2a3a4d14c7d") ; [379]
  ) ; [383]
) ; [384]

(InheritanceLink (stv 0.990000 0.990000)
  (ConceptNode "world@cf823ba7-fdca-4674-851f-51988a8de699") ; [375]
  (ConceptNode "world") ; [376]
) ; [377]

(InheritanceLink (stv 0.990000 0.990000)
  (ConceptNode "our@16e09edd-8f0c-4b93-8b7e-c2a3a4d14c7d") ; [379]
  (ConceptNode "us") ; [380]
) ; [381]

(InheritanceLink (stv 0.990000 0.990000)
  (PredicateNode "tries@8e4f8728-d2b3-4f1d-aa87-45cd9c24e6aa") ; [385]
  (PredicateNode "try") ; [386]
) ; [387]

(InheritanceLink (stv 0.990000 0.990000)
  (PredicateNode "tries@8e4f8728-d2b3-4f1d-aa87-45cd9c24e6aa") ; [385]
  (DefinedLinguisticConceptNode "present") ; [373]
) ; [388]

(InheritanceLink (stv 0.990000 0.990000)
  (ConceptNode "scientist@3fc15db2-5bd5-41e2-a9fe-55ca0d38d9a6") ; [389]
  (ConceptNode "scientist") ; [390]
) ; [391]

(InheritanceLink (stv 1.000000 0.000000)
  (PredicateNode "understand@26f7796c-b25d-49d9-92be-5b6642a8640b") ; [392]
  (PredicateNode "understand") ; [393]
) ; [394]

(InheritanceLink (stv 0.990000 0.990000)
  (PredicateNode "works@562a14be-6028-4012-be39-5724003ecafc") ; [370]
  (PredicateNode "work") ; [371]
) ; [372]

(InheritanceLink (stv 0.990000 0.990000)
  (PredicateNode "works@562a14be-6028-4012-be39-5724003ecafc") ; [370]
  (DefinedLinguisticConceptNode "present") ; [373]
) ; [374]

; Nodes need to be embedded.


; Scientists make observations and ask questions (and do extensive research work in finding the answers to these questions).

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

; RelEx relations:
; bj(make, observation)
; conj_and(make, ask)
; _subj(make, scientist)
; _obj(ask, question)
; _subj(ask, scientist)

; No R2L output is produced for me right now: https://github.com/opencog/relex/issues/46.
; Probably just a bug.


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

; Same as above: Verb + particle need to be handled. 
 
; for(work, and)
; _subj(work, scientist)
; conj_and(,, institute)
; _nn(institute, research)


; Enumeration (conjunction) isn't handled properly in this case yet,
; even though link-grammar does a good job parsing the sentence:
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

(EvaluationLink (stv 0.990000 0.990000)
  (PredicateNode "teach@5ea61496-0b3a-4ddc-b520-5aae837b642b") ; [1005]
  (ConceptNode "scientists@79a3955d-0bf1-4a9e-9f6c-2f6dfe7e0bbc") ; [1010]
) ; [1013]

(InheritanceLink (stv 0.990000 0.990000)
  (PredicateNode "teach@5ea61496-0b3a-4ddc-b520-5aae837b642b") ; [1005]
  (PredicateNode "teach") ; [1006]
) ; [1007]

(InheritanceLink (stv 0.990000 0.990000)
  (PredicateNode "teach@5ea61496-0b3a-4ddc-b520-5aae837b642b") ; [1005]
  (DefinedLinguisticConceptNode "present") ; [1008]
) ; [1009]

(InheritanceLink (stv 0.990000 0.990000)
  (ConceptNode "scientists@79a3955d-0bf1-4a9e-9f6c-2f6dfe7e0bbc") ; [1010]
  (ConceptNode "scientist") ; [1011]
) ; [1012]

; Quantifier 'some' should modify stv.
; How? Concrete probability values for 'some', 'few', 'a lot', etc?
; E.g. 'few' := .2, 'some' := .3, 'a lot' := .7...
; Has this already been discussed?
; Again: Verb+particle/preposition.

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
  (DefinedLinguisticConceptNode "present") ; [231]
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

; Fuzzy adverbs/adjectives (e.g. often) need to modify stv. R2L will deal
; with them separately. So far, only 'maybe' has been implemented (albeit
; without a helper function which, however, is specified here:
; http://wiki.opencog.org/w/RelEx2Logic#Maybe_Rule)
; Could I go forward and start implementing rules for fuzzy adverbs or would
; this be counter-productive or made redundant when temporary markers are
; going to be reduced in the post-processing step?


; Scientists sometimes may repeat experiments or use control groups.

; (Approximate) desired output:

(OrLink
    (EvaluationLink (stv .3 .5)
        (PredicateNode "repeat")
        (ListLink
            (ConceptNode "scientist")
            (ConceptNode "experiments")))
    (EvaluationLink (stv .3 .5)
        (Predicate "use")
        (ListLink
            (ConceptNode "scientist")
            (ConceptNode "control_groups"))))

; Current output:

; nj_or(repeat, use)
; _obj(repeat, experiment)
; _nn(group, control)
; _obj(use, group)
; _obj(or, group)
; _advmod(or, sometimes)
; _subj(or, scientist)

; _subj relations are missing for 'repeat' and 'use': https://github.com/opencog/relex/issues/52 


; Scientific work is also carred out in laboratories.

(ContextLink (stv .8 .8)
    (ConceptNode "Science"
    (EvaluationLink (stv .7 .7)
        (PredicateNode "be_carried_out_in")
        (ListLink
            (ConceptNode "work")
            (ConceptNode "laboratories")))))


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
    (SatisfyingSetScopeLink
        (VariableNode "$x")
        (AndLink
            (InheritanceLink
                (VariableNode "$x")
                (ConceptNode "scientist")
            (EvaluationLink
                (PredicateNode "study")
                (ListLink
                    (VariableNode "$x")
                    (ConceptNode "physics"))))))
    (ConceptNode "physicist"))
        
; Current RelEx relations:

; bj(be, physicist)
; _subj(be, scientist)
; _obj(study, physics)
; _subj(study, scientist)
; that_adj(scientist, study)

(ListLink (stv 0.990000 0.990000)
  (ConceptNode "scientists@18c624d0-c717-4e9b-acd6-d63f99f74ef4") ; [1311]
  (ConceptNode "physics@a2225099-e913-4429-9c7c-9d2fb9be3e03") ; [1326]
) ; [1329]

(EvaluationLink (stv 0.990000 0.990000)
  (PredicateNode "study@68cafe06-3ff2-4b39-8486-763d7a243e40") ; [1323]
  (ListLink (stv 0.990000 0.990000)
    (ConceptNode "scientists@18c624d0-c717-4e9b-acd6-d63f99f74ef4") ; [1311]
    (ConceptNode "physics@a2225099-e913-4429-9c7c-9d2fb9be3e03") ; [1326]
  ) ; [1329]
) ; [1330]

(InheritanceLink (stv 0.990000 0.990000)
  (ConceptNode "physicists@5f8900a3-d388-48ba-8d44-00c09e1d5453") ; [1314]
  (ConceptNode "physicist") ; [1315]
) ; [1316]

(InheritanceLink (stv 0.990000 0.990000)
  (ConceptNode "scientists@18c624d0-c717-4e9b-acd6-d63f99f74ef4") ; [1311]
  (ConceptNode "physicists@5f8900a3-d388-48ba-8d44-00c09e1d5453") ; [1314]
) ; [1317]

(InheritanceLink (stv 0.990000 0.990000)
  (PredicateNode "are@eb91546b-c50f-4574-b2f8-e41a4f776111") ; [1318]
  (PredicateNode "be") ; [1319]
) ; [1320]

(InheritanceLink (stv 0.990000 0.990000)
  (PredicateNode "are@eb91546b-c50f-4574-b2f8-e41a4f776111") ; [1318]
  (DefinedLinguisticConceptNode "present") ; [1321]
) ; [1322]

(InheritanceLink (stv 0.990000 0.990000)
  (PredicateNode "study@68cafe06-3ff2-4b39-8486-763d7a243e40") ; [1323]
  (PredicateNode "study") ; [1324]
) ; [1325]

(InheritanceLink (stv 0.990000 0.990000)
  (ConceptNode "physics@a2225099-e913-4429-9c7c-9d2fb9be3e03") ; [1326]
  (ConceptNode "physics") ; [1327]
) ; [1328]

(InheritanceLink (stv 0.990000 0.990000)
  (PredicateNode "study@68cafe06-3ff2-4b39-8486-763d7a243e40") ; [1323]
  (DefinedLinguisticConceptNode "present") ; [1321]
) ; [1331]

(InheritanceLink (stv 0.990000 0.990000)
  (ConceptNode "scientists@18c624d0-c717-4e9b-acd6-d63f99f74ef4") ; [1311]
  (ConceptNode "scientist") ; [1312]
) ; [1313]

; RelEx relations are all in place. Required rules for that-subordinate clause;


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

; _obj(win, prize)
; _subj(win, scientist)
; _nn(prize, nobel)
; _quantity(scientist, a_few)

; Currently applied R2L rules: SVO-rule "scientist" "win" "prize",
; tense-rule "win" "present", nn-rule "nobel" "prize"

; One rule needed that modifies stvs based on _quantity relation;
; probably best using scoped variables. Also see 'some' above.


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
