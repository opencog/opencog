; Example to proove that Clark is involved in suspicious log
; tracfficking activities.

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
(define CanadianPlaceNames (ConceptNode "CanadianPlaceNames" (stv .1 .8)))

; contexts
(define Music (ConceptNode "Music" (stv .3 .9)))
(define Accounting (ConceptNode "Accounting" (stv .2 .9)))
(define Money (ConceptNode "Money" (stv .2 .9)))

; other concepts
(define CanadianPeople (ConceptNode "CanadianPeople" (stv .25 .8)))
(define LogTrafficking (ConceptNode "LogTrafficking"))

; predicates
(define Mention (PredicateNode "Mention"))
(define Involved (PredicateNode "Involved"))

; variables
(define X (VariableNode "X"))
(define Y (VariableNode "Y"))

;;;;;;;;;;;;
;; axioms ;;
;;;;;;;;;;;;
; 1) In the context of music Alice frequently mentions Canadian place names
;
; ContextLink <.5, .9>
;     Music
;     EvaluationLink
;         Mention
;         ListLink
;             Alice
;             CanadianPlaceNames

(define axiom1 (ContextLink (stv .5 .9)
                            Music
                            (EvaluationLink Mention
                                            (ListLink Alice
                                                      CanadianPlaceNames))))

; 2) In the context of music Bob frequently mentions Canadian place names
;
; ContextLink <.5, .9>
;     Music
;     EvaluationLink
;         Mention
;         ListLink
;             Bob
;             CanadianPlaceNames

(define axiom2 (ContextLink (stv .5 .9)
                            Music
                            (EvaluationLink Mention
                                            (ListLink Bob
                                                      CanadianPlaceNames))))

; 3) In the context of music Clark does not frequently mention
; Canadian place names
;
; ContextLink <.01, .9>
;     Music
;     EvaluationLink
;         Mention
;         ListLink
;             Clark
;             CanadianPlaceNames

(define axiom3 (ContextLink (stv .01 .9)
                            Music
                            (EvaluationLink Mention
                                            (ListLink Clark
                                                      CanadianPlaceNames))))

; 4) In the context of accounting Alice does not frequently mention
; Canadian place names
;
; ContextLink <.01, .9>
;     Accounting
;     EvaluationLink
;         Mention
;         ListLink
;             Alice
;             CanadianPlaceNames

(define axiom4 (ContextLink (stv .01 .9)
                            Accounting
                            (EvaluationLink Mention
                                            (ListLink Alice
                                                      CanadianPlaceNames))))

; 5) In the context of accounting Bob frequently mentions Canadian place names
;
; ContextLink <.5, .9>
;     Accounting
;     EvaluationLink
;         Mention
;         ListLink
;             Bob
;             CanadianPlaceNames

(define axiom5 (ContextLink (stv .5 .9)
                            Accounting
                            (EvaluationLink Mention
                                            (ListLink Bob
                                                      CanadianPlaceNames))))

; 6) In the context of accounting Clark frequently mentions Canadian
; place names
;
; ContextLink <.5, .9>
;     Accounting
;     EvaluationLink
;         Mention
;         ListLink
;             Clark
;             CanadianPlaceNames

(define axiom6 (ContextLink (stv .5 .9)
                            Accounting
                            (EvaluationLink Mention
                                            (ListLink Clark
                                                      CanadianPlaceNames))))

; 7) Accounting is associated with Money
;
; ExtensionalInheritance <.7, .9>
;     Money
;     Accounting

(define axiom7 (SubsetLink (stv .7 .9)
                           Money
                           Accounting))

; 8) CanadianPeople is associated with CanadianPlaceNames
;
; Inheritance <.8, .9>
;     CanadianPeople
;     CanadianPlaceNames

(define axiom8 (InheritanceLink (stv .7 .9)
                                CanadianPeople
                                CanadianPlaceNames))

; 9) If X frequently mentions Y then he/she is highly involved with Y
;
; AverageLink <.9, .8>
;     ListLink
;     X
;     Y
;     ImplicationLink
;         EvaluationLink
;             Mention
;             ListLink
;                 X
;                 Y
;         EvaluationLink
;             Involved
;             ListLink
;                 X
;                 Y

(define axiom9 (AverageLink (stv .9 .9)
                            (ListLink X Y)
                            (ImplicationLink (EvaluationLink Mention
                                                             (ListLink X Y))
                                             (EvaluationLink Involved
                                                             (ListLink X Y)))))

; 10) Non Canadian People involved with Canadian people in the context
; of Money have a chance of being associated with log trafficking
; activities
;
; AverageLink <0.6, 0.8>
;     ListLink
;         X
;     ImplicationLink
;         AndLink
;             SubsetLink
;                 X
;                 NotLink
;                     CanadianPeople
;             ContextLink
;                 Money
;                 EvaluationLink
;                     Involved
;                     ListLink
;                         X
;                         CanadianPeople
;         InheritanceLink
;             X
;             LogTrafficking

(define axiom10
  (AverageLink (stv .6 .8)
               (ListLink X)
               (ImplicationLink (AndLink (InheritanceLink
                                          X
                                          (NotLink CanadianPeople))
                                         (ContextLink
                                          Money
                                          (EvaluationLink
                                           Involved
                                           (ListLink X CanadianPeople))))
                                (InheritanceLink
                                 X
                                 LogTrafficking))))

; 11) Clark is not Canadian
;
; SubsetLink <0.9, 0.9>
;     Clark
;     NotLink
;         CanadianPeople

(define axiom11 (SubsetLink (stv .9 .9)
                            Clark
                            (NotLink CanadianPeople)))
;
; Inference, target theorem
;
; InheritanceLink <?>
;     Clark
;     LogTrafficking

; 1) Instantiate axiom9, with X=Clark and Y=CanadianPlaceNames
;
; ImplicationLink <.9, .9>
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
; ContextLink <.9, .54>
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

; 2.1) Contextualize in Accounting, note that this step should not be
; necessary since the TV of the atom to contextualize is DEFAULT_TV
; anyway. But without this, for the moment, it crashes because there
; is no VersionedTV associated with that context
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
; ContextLink <0.45, 0.48>
;     Accounting
;     EvaluationLink
;         Involved
;         ListLink
;             Clark
;             CanadianPlaceNames

(define step3 (ModusPonensRule step1 (gadr axiom6) Accounting))

; 3.1) that step is here because for the moment the ContextLink is not
; updated when the VersionedTV is changed

(define step3.1 (ContextLink (stv 0.45 0.48) Accounting
                             (EvaluationLink Involved
                                             (ListLink Clark
                                                       CanadianPlaceNames))))

; 4) Decontextualize the result of step3
;
; SubsetLink <0.45, 0.48>
;     Accounting
;     SatisfyingSetLink
;         EvaluationLink
;             Involved
;             ListLink
;                 Clark
;                 CanadianPlaceNames

(define step4 (DecontextualizerRule (ContextLink Accounting step3)))

; 5) Apply SubsetDeductionRule on axiom7 and the result of step4
;
; SubsetLink <0.45, 0.48>
;     Money
;     SatisfyingSetLink
;         EvaluationLink
;             Involved
;             ListLink
;                 Clark
;                 CanadianPlaceNames

(define step5 (SubsetDeductionRule axiom7 step4))

; 6) Contextualize step5 using ContextualizerRule
;
; ContextLink <0.45, 0.48>
;     Money
;     EvaluationLink
;         Involved
;         ListLink
;             Clark
;             CanadianPlaceNames

(define step6 (ContextualizerRule step5))

; 7) Using InheritanceSubstRule and axiom8 infer how much Clark in
; involved with CanadianPeople in the context of Money
;
; ContextLink <0.45, 0.3>
;     Money
;     EvaluationLink
;         Involved
;         ListLink
;             Clark
;             CanadianPeople

(define step7 (InheritanceSubstRule axiom8 step6))

; 8) Infer the conjunction of axiom11 and the last step
;
; AndLink <0.4, 0.27>
;     InheritanceLink
;         Clark
;         NotLink
;             CanadianPeople
;     ContextLink
;         Money
;         EvaluationLink
;             Involved
;             ListLink
;                 Clark
;                 CanadianPeople

(define step8 (SimpleAndRule axiom11 step7))

; 9) Instantiate axiom10 with X = Clark
;
; ImplicationLink <0.6, 0.8>
;     AndLink
;         InheritanceLink
;             Clark
;             NotLink
;                 CanadianPeople
;         ContextLink
;             Money
;             EvaluationLink
;                 Involved
;                 ListLink
;                     Clark
;                     CanadianPeople
;     InheritanceLink
;         Clark
;         LogTrafficking

(define step9 (AverageInstantiationRule axiom10 Clark))

; 10) Apply modus ponens with step9 as implication and step8 as antecedent
;
; InheritanceLink <0.24, 0.27>
;     Clark
;     LogTrafficking

(define target (ModusPonensRule step9 step8))

; this is for PLNSchemeWrapperUTest
target
