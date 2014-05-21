; Facts taken from SimpleWikipedia:


; A scientist is a person who works in science.

; It would be good if there would be a way to only display the atoms
; produced by RelEx2Logic in the AtomSpace and be able to hide all
; the WordInstanceNodes, DefinedLinguisticRelationshipNodes, etc.
; This would not only be valuable for inspecting the output, but also for
; checking without distractions if PLN performs the right deductions.

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

; A scientist tries to understand how our world works.

(EvaluationLink (stv 1.0 1.0)
    (PredicadeNode "try to understand") ; how would infinitive constructions be handled?
    (ListLink
        (ConceptNode "scientist")
        (ConceptNode "how our world works"))) ; how are embedded sentences handled?

; There are three to-do-rules. No-one, however, deals with this constellation:
; _to-do(try, understand)
; _subj(try, scientist)
; _subj(work, world)
; 
; (S (NP A scientist.n) (VP tries.v (S (VP to.r (VP understand.v (SBAR (WHADVP how) (S (NP our world.n) (VP works.v))))))) .)



; Scientist make observations, ask questions and do extensive research work in finding the answers to these questions.

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


; Scientists work for governments, companies, schools and research institutes.

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


; Some scientists teach at universities and other places and train people to become scientists.

(EvaluationLink (stv .5 .8)
    (PredicadeNode "teach_at")
    (ListLink
        (ConceptNode "scientist")
        (ConceptNode "universities")))


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
