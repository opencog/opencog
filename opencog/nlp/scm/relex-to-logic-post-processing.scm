;This part of RelEx2Logic is used for post processing purpose of complex sentence 
;like adjective clause ,coordinate conjunction , correlative conjunction etc
;The rules are used to generate a new graph from already generated graphs at  pre stage of RelEx2logic 
;in this process the temporary nodes like "WhichMarker" node, "AndMarker" node, "WhoMarker" node will be removed
;-------------------------------------------------------------------------------
;Variable declarations for the purpose of post processing 
(define antecedent-inst (TypedVariableLink (VariableNode "$antecedent-inst") (VariableTypeNode "ConceptNode")))
(define concept-sub-1 (TypedVariableLink (VariableNode "$concept-sub-1") (VariableTypeNode "ConceptNode")))
(define concept-sub-2 (TypedVariableLink (VariableNode "$concept-sub-2") (VariableTypeNode "ConceptNode")))
(define concept-main-1 (TypedVariableLink (VariableNode "$concept-main-1") (VariableTypeNode "ConceptNode")))
(define sub-root-verb (TypedVariableLink (VariableNode "$sub-root-verb") (VariableTypeNode "PredicateNode")))
(define main-root-verb (TypedVariableLink (VariableNode "$main-root-verb") (VariableTypeNode "PredicateNode")))
;-------------------------------------------------------------------------------
;accept the antecedent instance  and return the antecedent 
;here, antecedent is the thing that is modified by the adjective clause
(define (find-antecedent ant-inst)
(cog-node 'ConceptNode (cog-name (word-inst-get-lemma(cog-node 'WordInstanceNode (cog-name ant-inst))))))
;-------------------------------------------------------------------------------
;re-write the graphs to new form 
(define (rewrite-graph sub-clause main-clause antecedent-inst)
    (define antecedent (find-antecedent antecedent-inst))
    (if (not (equal? (cog-name antecedent) (cog-name (cog-get-partner main-clause (VariableNode "$X")))))
    (InheritanceLink
        (SatisfyingSetLink (VariableNode "$X")
            (AndLink
            (InheritanceLink (VariableNode "$X") antecedent)
            sub-clause))
        (SatisfyingSetLink (VariableNode "$X")
           main-clause))))
;-------------------------------------------------------------------------------
;re-write the graphs to new form for sentence with definite flag 
(define (rewrite-graphs-with-definite sub-clause main-clause antecedent-inst)
    (define antecedent (find-antecedent antecedent-inst))
    (if (not (equal? (cog-name antecedent) (cog-name (cog-get-partner main-clause (VariableNode "$X")))))
    (MemberLink
        (antecedent-inst)
        (SatisfyingSetLink (VariableNode "$X")
            (AndLink main-clause sub-clause)))))
;-------------------------------------------------------------------------------
;Adjective clause rules
;
;-------------------------------------------------------------------------------
;For sentences with SVO and SVP rules at pre stage 
;Example: "Restaurants which serve frogs are famous."
;RelEx2Logic representaion after post processing 
;(InheritanceLink
;   (SatisfyingSetLink (VariableNode "$X")
;        (AndLink
;           (InheritanceLink(VariableNode "$X")(ConceptNode "Restaurant"))
;           (EvaluationLink
;               (PredicateNode "serves@12")
;               (ListLink
;                   (VariableNode "$X")
;                   (ConceptNode "frog@11")))))
;   (SatisfyingSetLink (VariableNode "$X")
;        (InheritanceLink(VariableNode "$X")(ConceptNode "Famous@45"))
;$antecedent-inst=Restaurant,$concept-main-1=famous,$sub-root-verb=serve and $concept-sub-1=frogs
(define Adjective-clause-SVO&SVP
    (BindLink
        (ListLink antecedent-inst sub-root-verb  concept-sub-1 concept-main-1)
        (ImplicationLink
            (AndLink
                (InheritanceLink (VariableNode "$antecedent-inst")(VariableNode "$concept-main-1"))
                (EvaluationLink (VariableNode "$sub-root-verb")(ListLink(VariableNode "$antecedent-inst")(VariableNode "$concept-sub-1")))
                (EvaluationLink (PredicateNode "whichmarker") (ListLink (VariableNode "$antecedent-inst")(VariableNode "$sub-root-verb"))))
        (ExecutionLink
           (GroundedSchemaNode "scm: rewrite-graph")
            (ListLink
                (EvaluationLink (VariableNode "$sub-root-verb") (ListLink (VariableNode "$X") (VariableNode "$concept-sub-1")))
                (InheritanceLink (VariableNode "$X")(VariableNode "$concept-main-1"))
                (VariableNode "$antecedent-inst"))
           ))))
;For sentences with SVO rule applied twice at pre stage 
;Example: "I like trees which have coffee beans." 
;RelEx2Logic representaion after post processing 
;(InheritanceLink
;    (SatisfyingSetLink (VariableNode "$X")
;        (AndLink
;            (InheritanceLink (VariableNode "$X")(ConceptNode "trees"))
;            (EvaluationLink
;                   (PredicateNode "have@12")
;                       (ListLink
;                           (VariableNode "$X")
;                           (ConceptNode "beans@13")))))
;    (SatisfyingSetLink (VariableNode "$X")
;        (EvaluationLink (PredicateNode "like@44")(ListLink (ConceptNode "I@09")(VariableNode "$X")))))
;$main-root-verb =like,$concept-main-1=i,$antecedent-inst=tree,$sub-root-verb=have and $concept-sub-1=been
(define adjective-clause-SVO&SVO
    (BindLink
        (ListLink antecedent-inst sub-root-verb concept-sub-1 concept-main-1 main-root-verb)
        (ImplicationLink
            (AndLink
                (EvaluationLink (VariableNode "$main-root-verb")(ListLink(VariableNode "$concept-main-1")(VariableNode "$antecedent-inst")))
                (EvaluationLink (VariableNode "$sub-root-verb")(ListLink(VariableNode "$antecedent-inst")(VariableNode "$concept-sub-1")))
                (EvaluationLink (PredicateNode "whichmarker") (ListLink (VariableNode "$antecedent-inst")(VariableNode "$sub-root-verb"))))
        (ExecutionLink
           (GroundedSchemaNode "scm: rewrite-graph")
            (ListLink
                (EvaluationLink (VariableNode "$sub-root-verb") (ListLink (VariableNode "$X") (VariableNode "$concept-sub-1")))
                (EvaluationLink (PredicateNode "main-root-verb")(ListLink (ConceptNode "concept-main-1")(VariableNode "$X")))
                (VariableNode "$antecedent-inst")))
)))
;For sentences with SVP and SVIO rule at pre stage 
;Example:  "Books which you give me are interesting."
;RelEx2Logic representaion after post processing 
;(InheritanceLink
;        (SatisfyingSetLink (VariableNode "$X")
;            (AndLink
;                (InheritanceLink (VariableNode "$X")(ConceptNode "book"))
;                (EvaluationLink (PredicateNode "give@12")(ListLink (ConceptNode "you@13")(ConceptNode "me@18")(VariableNode "$x")))))
;        (SatisfyingSetLink (VariableNode "$X")
;            (InheritanceLink(VariableNode "$X")(ConceptNode "interesting@19"))))
;$antecedent-inst = book ,$concept-main-1 = interesting,$sub-root-verb = give, $concept-sub-1 = you and $concept-sub-2= me
(define adjective-clause-SVP&SVIO
    (BindLink
        (ListLink antecedent-inst concept-main-1 sub-root-verb concept-sub-1 concept-sub-2)
        (ImplicationLink
            (AndLink
                (InheritanceLink (VariableNode "$antecedent-inst")(VariableNode "$concept-main-1"))
                (EvaluationLink (VariableNode "$sub-root-verb")(ListLink(VariableNode "$concept-sub-1")(VariableNode "$concept-sub-2")(VariableNode "$antecedent-inst")))
                (EvaluationLink (PredicateNode "whichmarker") (ListLink (VariableNode "$antecedent-inst")(VariableNode "$sub-root-verb"))))
        (ExecutionLink
           (GroundedSchemaNode "scm: rewrite-graph")
            (ListLink
                (EvaluationLink (PredicateNode "$sub-root-verb")(ListLink (ConceptNode "$concept-sub-1")(ConceptNode "$concept-sub-2")(VariableNode "$x")))
                (InheritanceLink(VariableNode "$X")(ConceptNode "$concept-main-1"))
                (VariableNode "$antecedent-inst")))
)))
;For sentences with definite flag ,SVO and passive rule at pre stage 
; Example: "The books which I read in the library were written by Charles Dickens." 
;RelEx2Logic representaion after post processing 
;(MemberLink
;    (ConceptNode Book@12)
;    (SatisfyingSetLink (VariableNode $X)
;        (AndLink
;            (EvaluationLink
;                (PredicateNode write@34)
;                (ListLink
;                    (VariableNode $X)
;                    (ConceptNode Charles_Dickens@45)))
;            (EvaluationLink
;                (PredicateNode read@39)
;                (ListLink
;                    (ConceptNode I@67)
;                    (VariableNode $X))))))
;$main-root-verb = write,$concept-main-1 = Charles_Dickens,$antecedent-inst = book,$sub-root-verb = read and $concept-sub-1= I
(define adjective-clause-SVO&PASSIVE
    (BindLink
         (ListLink)
        (ImplicationLink
            (AndLink
                (EvaluationLink (VariableNode "$main-root-verb")(ListLink(VariableNode "$concept-main-1")(VariableNode "$antecedent-inst")))
                (EvaluationLink (VariableNode "$sub-root-verb")(ListLink(VariableNode "$concept-sub-1")(VariableNode "$antecedent-inst")))
                (EvaluationLink (PredicateNode "whichmarker") (ListLink (VariableNode "$antecedent-inst")(VariableNode "$sub-root-verb"))))
                (EvaluationLink (PredicateNode "definite") (ListLink (VariableNode "$antecedent-inst")))
        (ExecutionLink
           (GroundedSchemaNode "scm: rewrite-graphs-with-definite")
            (ListLink
                (EvaluationLink (PredicateNode "$sub-root-verb")(ListLink (ConceptNode "$concept-sub-1")(VariableNode "$x")))
                (EvaluationLink (PredicateNode "$main-root-verb")(ListLink (ConceptNode "$concept-main-1")(VariableNode "$x")))
                (VariableNode "$antecedent-inst")))
)))

