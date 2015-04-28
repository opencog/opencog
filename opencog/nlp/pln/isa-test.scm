;
; isa-test.scm
;
; Test input data for isa pln deduction
;
(EvaluationLink (stv 1 1)
	(DefinedLinguisticRelationshipNode "isa")
	(ListLink
		(WordInstanceNode "cat@123")
		(WordInstanceNode "Becky@456")
	)
)

(EvaluationLink (stv 1 1)
	(DefinedLinguisticRelationshipNode "isa")
	(ListLink
		(WordInstanceNode "animal@123")
		(WordInstanceNode "cat@123")
	)
)

(define q
	(EvaluationLink 
		(DefinedLinguisticRelationshipNode "isa")
		(ListLink
			(WordInstanceNode "animal@123")
			(WordInstanceNode "Becky@456")
		)
	)
)

(pln-bc q 300)



; alternate real-life 

(EvaluationLink (stv 1 0.99999988) 
   (DefinedLinguisticRelationshipNode "isa")
   (ListLink 
     (SemeNode "cat@e5c959e7-afb5-4410-9d82-da7b84b73067" (stv 1 1))
     (SemeNode "Becky@64ef3146-6ef9-4100-9354-f3d672847280" (stv 1 1))
   )
)

(EvaluationLink (stv 1 0.99999988)
   (DefinedLinguisticRelationshipNode "isa")
   (ListLink
      (SemeNode "animal@4f81d0cf-3f92-45aa-886e-12268e402a57" (stv 1 1))
      (SemeNode "cat@e5c959e7-afb5-4410-9d82-da7b84b73067" (stv 1 1))
   )
)

(define unknown
   (EvaluationLink
      (DefinedLinguisticRelationshipNode "isa")
      (ListLink
          (SemeNode "animal@4f81d0cf-3f92-45aa-886e-12268e402a57")
          (SemeNode "Becky@64ef3146-6ef9-4100-9354-f3d672847280" )
      )
   )
)


