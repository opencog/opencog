; A car is not a boat.
; Some boats are water bicycles
; |- ?
; A. No boat is a water bicycle
; B. Some water bicycles are no cars
; C. No boat is a car
; D. Some cars ware no water bicycles

; Note: Original input sentence was "A car is no boat."
; Evokes relation _quantity(boat, no).
; This prompts the question: Should this be handled with a NotLink
; or use a ForAllLink which itself contains a NotLink?

(EvaluationLink (PredicateNode "inputs")
    (ListLink
    )
)
