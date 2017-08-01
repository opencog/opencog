;; Use the backward chainer to prove that TheKB2-1 is inconsistent.
;;
;; The reasoning goes as follows
;;
;; 1. If Entity2-2 is a father, then it has the attribute of
;; Male. Apply conditional instantiation over KB-test1-6 and KB-17197
;; to conclude
;;
;; (EvaluationLink
;;   (PredicateNode "attribute")
;;   (ListLink
;;     (ConceptNode "Entity2-2")
;;     (ConceptNode "Male")))
;;
;; 2. If Entity2-2 is a mother, then it has the attribute of
;; Female. Apply conditional instantiation over KB-test1-7 and
;; KB-16754 to conclude
;;
;; (EvaluationLink
;;   (PredicateNode "attribute")
;;   (ListLink
;;     (ConceptNode "Entity2-2")
;;     (ConceptNode "Female")))
;;
;; 3. If Entity2-2 has attribute of Male, then it has the property of
;; Male because attribute is a subrelation of property. Apply
;; conditional instantiation using 1. and KB-17347 as premises
;;
;; (EvaluationLink
;;   (PredicateNode "property")
;;   (ListLink
;;     (ConceptNode "Entity2-2")
;;     (ConceptNode "Male")))
;;
;; 4. If Entity2-2 has attribute of Female, then it has the property
;; of Female because attribute is a subrelation of property. Apply
;; conditional instantiation using 2. and KB-17347 as premises
;;
;; (EvaluationLink
;;   (PredicateNode "property")
;;   (ListLink
;;     (ConceptNode "Entity2-2")
;;     (ConceptNode "Female")))
;;
;; 5. Introduce the conjunction that Entity2-2 has property Male and
;; Female and that these attributes are contrary. Apply fuzzy
;; conjunction introduction using 3., 4. and KB-17527 to obtain their
;; conjunction.
;;
;; 6. Since Entity2-2 has 2 contrary properties, the KB is
;; inconsistent. Apply conditional instantiation using 5. and
;; KB-test1-5 as premises to obtain the target
;;
;; TODO: use the suppodely fact that "contraryAttribute" is symmetric
;; to get the right argument order.
;;
;; Note: use Merge.scm (instead of all-sumo-labeled-kb.scm) to be more
;; tractable.
;;
;; guile --no-auto-compile -l Merge.scm

;; Set logger to debug level
(use-modules (opencog logger))
(cog-logger-set-level! "debug")

;; Load PLN rule base
(load "pln-config2.scm")

;; Add axioms pertaining to the test
;; KB-test2-1
(MemberLink (stv 1.000000 1.000000)
  (ConceptNode "Entity2-2" (stv 0.010000 1.000000)) ; [2771084889312061962][1]
  (ConceptNode "Organism" (stv 0.010000 1.000000)) ; [6700036726303166665][1]
) ; [15556963335151315501][1]
;; KB-test2-2
(MemberLink (stv 1.000000 1.000000)
  (ConceptNode "Entity2-1" (stv 0.010000 1.000000)) ; [4514343376845643670][1]
  (ConceptNode "Organism" (stv 0.010000 1.000000)) ; [6700036726303166665][1]
) ; [17744261202630857017][1]
;; KB-test2-3
(MemberLink (stv 1.000000 1.000000)
  (ConceptNode "Inconsistent" (stv 0.010000 1.000000)) ; [6471271079275877227][1]
  (ConceptNode "Attribute" (stv 0.010000 1.000000)) ; [4582596031008048372][1]
) ; [15641830429550015449][1]
;; KB-test2-4
(MemberLink (stv 1.000000 1.000000)
  (ConceptNode "TheKB2-1" (stv 0.010000 1.000000)) ; [1885560612557942259][1]
  (ConceptNode "ComputerProgram" (stv 0.010000 1.000000)) ; [1602459263668714272][1]
) ; [18130572887050016141][1]
;; KB-test2-5
(ImplicationScopeLink (stv 1.000000 1.000000)
  (VariableList
    (TypedVariableLink
      (VariableNode "?ATTR1") ; [7171898034362421019][1]
      (TypeChoice
        (TypeNode "ConceptNode") ; [3788625745177846543][1]
        (TypeNode "SchemaNode") ; [4818308169440095275][1]
        (TypeNode "PredicateNode") ; [7305730297992941107][1]
      ) ; [17198466103586147334][1]
    ) ; [15979217339517717053][1]
    (TypedVariableLink
      (VariableNode "?ATTR2") ; [547329696171059331][1]
      (TypeChoice
        (TypeNode "ConceptNode") ; [3788625745177846543][1]
        (TypeNode "SchemaNode") ; [4818308169440095275][1]
        (TypeNode "PredicateNode") ; [7305730297992941107][1]
      ) ; [17198466103586147334][1]
    ) ; [9506019026862624933][1]
    (TypedVariableLink
      (VariableNode "?X") ; [6888048983637573642][1]
      (TypeChoice
        (TypeNode "ConceptNode") ; [3788625745177846543][1]
        (TypeNode "SchemaNode") ; [4818308169440095275][1]
        (TypeNode "PredicateNode") ; [7305730297992941107][1]
      ) ; [17198466103586147334][1]
    ) ; [15835570702452529420][1]
  ) ; [12951339569168068226][1]
  (AndLink
    (EvaluationLink
      (PredicateNode "property" (stv 0.100000 1.000000)) ; [4317465433366588984][1]
      (ListLink
        (VariableNode "?X") ; [6888048983637573642][1]
        (VariableNode "?ATTR1") ; [7171898034362421019][1]
      ) ; [13518901312528859704][1]
    ) ; [9973097162708519399][1]
    (EvaluationLink
      (PredicateNode "property" (stv 0.100000 1.000000)) ; [4317465433366588984][1]
      (ListLink
        (VariableNode "?X") ; [6888048983637573642][1]
        (VariableNode "?ATTR2") ; [547329696171059331][1]
      ) ; [16117705011192273824][1]
    ) ; [12571900861371933519][1]
    (EvaluationLink
      (PredicateNode "contraryAttribute" (stv 0.100000 1.000000)) ; [2212261046096530067][1]
      (ListLink
        (VariableNode "?ATTR1") ; [7171898034362421019][1]
        (VariableNode "?ATTR2") ; [547329696171059331][1]
      ) ; [16261351648257461457][1]
    ) ; [17030779013363383355][1]
  ) ; [14838515346647225112][1]
  (EvaluationLink
    (PredicateNode "property" (stv 0.100000 1.000000)) ; [4317465433366588984][1]
    (ListLink
      (ConceptNode "TheKB2-1" (stv 0.010000 1.000000)) ; [1885560612557942259][1]
      (ConceptNode "Inconsistent" (stv 0.010000 1.000000)) ; [6471271079275877227][1]
    ) ; [13756854775200444817][1]
  ) ; [10211050625380104512][1]
) ; [14389148767193402296][1]
;; KB-test2-6
(EvaluationLink (stv 1.000000 1.000000)
  (PredicateNode "father" (stv 0.100000 1.000000)) ; [991398774833084676][1]
  (ListLink
    (ConceptNode "Entity2-1" (stv 0.010000 1.000000)) ; [4514343376845643670][1]
    (ConceptNode "Entity2-2" (stv 0.010000 1.000000)) ; [2771084889312061962][1]
  ) ; [13796151475037793843][1]
) ; [11170612035869121070][1]
;; KB-test2-7
(EvaluationLink (stv 1.000000 1.000000)
  (PredicateNode "mother" (stv 0.100000 1.000000)) ; [4696242094007566092][1]
  (ListLink
    (ConceptNode "Entity2-1" (stv 0.010000 1.000000)) ; [4514343376845643670][1]
    (ConceptNode "Entity2-2" (stv 0.010000 1.000000)) ; [2771084889312061962][1]
  ) ; [13796151475037793843][1]
) ; [13526605089514922294][1]

;; Define taget
(define target
  (EvaluationLink
    (PredicateNode "property" (stv 0.100000 1.000000)) ; [4317465433366588984][1]
    (ListLink
      (ConceptNode "TheKB2-1" (stv 0.010000 1.000000)) ; [1885560612557942259][1]
      (ConceptNode "Inconsistent" (stv 0.010000 1.000000)) ; [6471271079275877227][1]
    ) ; [13756854775200444817][1]
  ) ; [10211050625380104512][1]
) ; [18415966981910948508][1]

;; Prove target
(pln-bc target)
