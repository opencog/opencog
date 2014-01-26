; Inferring the TV of niceWeather(tomorrow) given the average TV of
; niceWeather
(define X (VariableNode "X"))
(define niceWeather (PredicateNode "niceWeather"))
(define tomorrow (ConceptNode "tomorrow"))
(define average (AverageLink (stv 0.5 0.5) (ListLink X) 
                 (EvaluationLink niceWeather (ListLink X))))
; inference
(AverageInstantiationRule average tomorrow)
