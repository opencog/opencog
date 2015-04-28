; Examples from the Real-World Reasoning book by Ben et al.

; RelEx2Logic should generate the AtomSpace representations
; from the sentences. PLN should then perform deduction on
; the atoms to arrive at the desired conclusion.

; To do: How to structure the tests? Separate RelEx2Logic
; and PLN? Is a pipeline possible?

; Examples for PLN tests by Alex can be found at:
; ../specific_rules/DeductionRule_InheritanceLink.scm
; (uses EvaluationLinks to specify inputs, outputs, etc.
; ../specific_rules/DeductionRule.scm
; (inputs and outputs are implicitly defined)

; Temporal Inference (p. 208ff.)

; To do: How to best represent a sentence in Scheme?
; Sentence 1: “Jane is at the daycare center everyday of the
; week between 7am and 7:30am and between 16pm and 16:30pm
; (when she brings and fetch her child).”

; To do: The examples are taking directly from the book; 
; how to represent IsWeekDay, AtTime and AtPlace, During and
; OverlapTime?
; 1a
(AverageLink
    (VariableNode "$Day")
    (AndLink
        (IsWeekDay("$Day"))
        (AtTime([$Day:7am, $Day:7:30am], AtPlace(Jane, daycare)))))

; 1b
(AverageLink
    (VariableNode "$Day")
    (AndLink
        (IsWeekDay($Day))
        (AtTime([$Day:16am, $Day:16:30am], AtPlace(Jane, daycare)))))

; Sentence 2: “When Susie has an important meeting at time
; interval T, she will be in the daycare center during 30 minutes
; an hour before the beginning of T and after the end of T.”

(ImplicationLink
    (AtTime(T, ImportantMeeting(Susie)))
    (AndLink
        (AtTime
            [beginning(T)-1h, beginning(T)-1:30h]
            AtPlace(Susie, daycare))
        (AtTime
            [end(T)+1h, end(T)+1:30h]
            AtPlace(Susie, daycare))))

; Sentence 3: “Susie had an important meeting last Tuesday
; between 1:30pm and 3:15pm.”

(AtTime
    [LastTuesday:1:30pm, LastTuesday:3:15pm]
    ImportantMeeting(Susie))


; Inference chain

; Inputs: Axioms 2 + 3

; Output:
(AndLink
    (AtTime
        [LastTuesday:12:30pm, LastTuesday:1pm]
        AtPlace(Susie, daycare))
    (AtTime
        [LastTuesday:4:15pm, LastTuesday:4:45pm]
        AtPlace(Susie, daycare)))

; reduce AndLink
; “Susie was at the daycare center Tuesday between 4:15pm
; and 4:45pm”.
(AtTime
    [LastTuesday:4:15pm, LastTuesday:4:45pm]
    AtPlace(Susie, daycare))

; Inputs: 1b +
(AndLink
    (isWeekDay(Tuesday))
    (AtTime
        [LastTuesday:4pm, LastTuesday:4:45pm]
        AtPlace(Jane, daycare)) 

; reduce AndLink
; “Jane was at the daycare center Tuesday between 4:pm
; and 4:30pm”.

(AtTime
    [LastTuesday:4pm, LastTuesday:4:45pm]
    AtPlace(Jane, daycare))

; PLN Inference

(AndLink
    (AtTime
        [LastTuesday:4:15pm, LastTuesday:4:45pm]
        AtPlace(Susie, daycare))
    (AtTime
        [LastTuesday:4pm, LastTuesday:4:45pm]
        AtPlace(Jane, daycare))
    (OverlapTime
        [LastTuesday:4:15pm, LastTuesday:4:45pm]
        [LastTuesday:4pm, LastTuesday:4:45pm])
    (During
        [LastTuesday:4:15pm, LastTuesday:4:45pm]
        LastWeek)
    (During
        [LastTuesday:4pm, LastTuesday:4:45pm]
        LastWeek))

; PLN existential qualifier axioms
; "Susie was at the same place as Jane last week."

$Place=daycare
$TimeInterval1=[LastTuesday:4:15pm, LastTuesday:4:45pm],
$TimeInterval2=[LastTuesday:4pm, LastTuesday:4:45pm]

(ThereExistsLink
    (VariableNode "$Place")
    (VariableNode "$TimeInterval1")
    (VariableNode "$TimeInterval2")
    (AndLink
        (AtTime($TimeInterval1, AtPlace(Susie, $Place)))
        (AtTime($TimeInterval2, AtPlace(Jane, $Place)))
        (OverlapTime($TimeInterval1, $TimeInterval2))
        (During($TimeInterval1, LastWeek))))


