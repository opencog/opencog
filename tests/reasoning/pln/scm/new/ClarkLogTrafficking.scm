; Example to proove that Clark is involved in suspicious log
; tracfficking activities

; Alison is an accountant who is also a musician.  Alison is emotional
; in the context of music, but not in the context of accounting.  She
; frequently mentions Canadian place names in the context of music
; (maybe she's a Canadian music fan), but not in the context of
; accounting. Bob is in a similar situation, but he frequently
; mentions Canadian related stuff in both the music and accounting
; contexts. Clark is also in a similar situation, but he frequently
; mentions Canadian related stuff only in the accounting context, not
; the music context. Trivially, Canadian places are associated with
; Canadian people. People who have a lot to do with Canadian people,
; and a lot to do with money, have a chance of being involved in
; suspicious log trafficking activities. Trivially, accounting has to
; do with money.

; persons
(define Alice (ConceptNode "Alice"))
(define Bob (ConceptNode "Bob"))
(define Clark (ConceptNode "Clark"))

; places
(define CanadianPlaceNames (ConceptNode "CanadianPlaceNames"))
(define Canada (ConceptNode "Canada"))

; contexts
(define Music (ConceptNode "Music" (stv .3 .9)))
(define Accounting (ConceptNode "Accounting" (stv .2 .9)))

; other concepts
(define Money (ConceptNode "Money"))
(define CanadianPeople (ConceptNode "CanadianPeople"))
(define LogTrafficking (ConceptNode "LogTrafficking"))

; predicates
(define Mention (PredicateNode "Mention"))
(define Involved (PredicateNode "Involved"))

; variables
(define X (VariableNode "X"))
(define Y (VariableNode "Y"))

; axioms
; 1) In the context of music Alice frequently mentions Canadian place names

(define axiom1 (ContextLink (stv .5 .9)
                            Music
                            (EvaluationLink Mention
                                            (ListLink Alice
                                                      CanadianPlaceNames))))

; 2) In the context of music Bob frequently mentions Canadian place names

(define axiom2 (ContextLink (stv .5 .9)
                            Music
                            (EvaluationLink Mention
                                            (ListLink Bob
                                                      CanadianPlaceNames))))

; 3) In the context of music Clark does not frequently mention
; Canadian place names

(define axiom3 (ContextLink (stv .01 .9)
                            Music
                            (EvaluationLink Mention
                                            (ListLink Clark
                                                      CanadianPlaceNames))))

; 4) In the context of accounting Alice does not frequently mention
; Canadian place names

(define axiom4 (ContextLink (stv .01 .9)
                            Accounting
                            (EvaluationLink Mention
                                            (ListLink Alice
                                                      CanadianPlaceNames))))

; 5) In the context of accounting Bob frequently mentions Canadian place names

(define axiom5 (ContextLink (stv .5 .9)
                            Accounting
                            (EvaluationLink Mention
                                            (ListLink Bob
                                                      CanadianPlaceNames))))

; 6) In the context of accounting Clark frequently mentions Canadian
; place names

(define axiom6 (ContextLink (stv .5 .9)
                            Accounting
                            (EvaluationLink Mention
                                            (ListLink Clark
                                                      CanadianPlaceNames))))

; 7) Accounting is associated with Money

(define axiom7 (InheritanceLink (stv .7 .9)
                                Accounting
                                Money))

; 8) CanadianPlaces is associated with Canada

(define axiom8 (InheritanceLink (stv .8 .9)
                                CanadianPlaceNames
                                Canada))

; 10) If someone X frequently mentions Y then he/she is highly involved with Y

(define axiom9 (AverageLink (stv .9 .8)
                            (ListLink X Y)
                            (ImplicationLink (EvaluationLink Mention
                                                             (ListLink X Y))
                                             (EvaluationLink Involved
                                                             (ListLink X Y)))))

; 11) Canada is associated with Canadian people

(define axiom10 (InheritanceLink (stv .7 .7)
                                 CanadianPeople
                                 Canada))

; 12) Non Canadian People involved with Canadian people and money are
; associated with log trafficking

(define axiom11
  (AverageLink (stv .6 .7)
               (ListLink X)
               (ImplicationLink (AndLink (InheritanceLink
                                          X
                                          (NotLink CanadianPeople))
                                         (InheritanceLink
                                          X
                                          Money)
                                         (EvaluationLink
                                          Involved
                                          (ListLink X CanadianPeople)))
                                (InheritanceLink
                                 X
                                 LogTrafficking))))

; 13) Clark is not Canadian

(define axiom12 (InheritanceLink (stv .99 .9)
                                 Clark
                                 (NotLink CanadianPeople)))
;
; Inference, target theorem
;
; Inheritance <?>
;     Clark
;     LogTrafficking

; 1) Instantiate axiom9, with X=Clark and Y=CanadianPlaceNames
;
; ImplicationLink <.9, .8>
;     EvaluationLink 
;         Mention
;         ListLink 
;             Clark
;             CanadianPlaceNames
;     EvaluationLink
;         Involved
;         ListLink
;             Clark
;             CanadianPlaceNames

(define step1 (AverageInstantiationRule axiom9 Clark CanadianPlaceNames))

; 2) the following steps are necessary so that all TVs are correctly
; contextualized in the Accounting context, it uses
; ContextFreeToSensitiveRule
;
; ContextLink <.9, .48>
;     Accounting
;     ImplicationLink
;         EvaluationLink 
;             Mention
;             ListLink 
;                 Clark
;                 CanadianPlaceNames
;         EvaluationLink
;             Involved
;             ListLink
;                 Clark
;                 CanadianPlaceNames

(define step2 (ContextFreeToSensitiveRule Accounting step1))

; 2.2) Contextualize in Accounting, note that this step should not be
; necessary since the TV of the atom to contextualize is DEFAULT_TV
; anyway. But without this, for the moment, it crashes because getting
; there is no VersionedTV associated with that context
;
; ContextLink
;     Accounting
;     EvaluationLink 
;         Involved
;         ListLink 
;             Clark
;             CanadianPlaceNames

(define step2.1 (ContextFreeToSensitiveRule Accounting (gadr step1)))

; 3) Using Modus Ponens in the context of Accounting (it is possible
; due to step2 and step2.1) with step1 as the implication and axiom6
; as the antecedent. (gadr axiom6) is used to take the contextualized
; axiom directly rather than the contextLink.
; 
; Context <0.45, 0.48>
;     Accounting
;     EvaluationLink
;         Involved
;         ListLink
;             Clark
;             CanadianPlaceNames

(define step3 (ModusPonensRule step1 (gadr axiom6) Accounting))

step1
