;-------------------------------------------------------------------------------
;Variables used for post processing process of Relex2Logic 
(define anticident-inst (TypedVariableLink (VariableNode "$anticident-inst") (VariableTypeNode "ConceptNode")))
(define concept-sub-1 (TypedVariableLink (VariableNode "$concept-sub-1") (VariableTypeNode "ConceptNode")))
(define concept-sub-2 (TypedVariableLink (VariableNode "$concept-sub-2") (VariableTypeNode "ConceptNode")))
(define concept-main-1 (TypedVariableLink (VariableNode "$concept-main-1") (VariableTypeNode "ConceptNode")))
(define sub-root-verb (TypedVariableLink (VariableNode "$sub-root-verb") (VariableTypeNode "PredicateNode")))
(define main-root-verb (TypedVariableLink (VariableNode "$main-root-verb") (VariableTypeNode "PredicateNode")))
;-------------------------------------------------------------------------------
;find anticident of the given anticident instance 
(define (find-anticident ant-inst)
    (define temp-node)
    (let ((lst (cog-chase-link 'InheritanceLink 'ConceptNode ant-inst)))
        (for-each (lambda (i)
            (if (equal? (string-trim-right (substring (cog-name ant-inst) 0 (string-index (cog-name ant-inst) #\@)) #\s)(cog-name i))
                (set! temp-node i)
                #f))
        lst) temp-node))
;-------------------------------------------------------------------------------
;re-write the graphs
(define (rewrite-graph sub-clause main-clause anticident-inst)
    (define anticident (find-anticident anticident-inst))
    (if (not (equal? (cog-name anticident) (cog-name (cog-get-partner main-clause (VariableNode "$X")))))
    (InheritanceLink
        (SatisfyingSetLink (VariableNode "$X")
            (AndLink
            (InheritanceLink (VariableNode "$X") anticident)
            sub-clause))
        (SatisfyingSetLink (VariableNode "$X")
           main-clause))))
;-------------------------------------------------------------------------------
;re-write the graphs for sentence with definite flag 
(define (rewrite-graphs-with-definite sub-clause main-clause anticident-inst)
    (define anticident (find-anticident anticident-inst))
    (if (not (equal? (cog-name anticident) (cog-name (cog-get-partner main-clause (VariableNode "$X")))))
    (MemberLink
        (anticident-inst)
        (SatisfyingSetLink (VariableNode "$X")
            (AndLink
            (InheritanceLink (VariableNode "$X") anticident) main-clause sub-clause)))))
;-------------------------------------------------------------------------------
;Adjective clause rules
;
;-------------------------------------------------------------------------------
;For sentences with SVO and SVP rules
;Example: "Restaurants which serve frogs are famous."
;RelEx2Logic representaion:
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
;$anticident-inst=Restaurant,$concept-main-1=famous,$sub-root-verb=serve and $concept-sub-1=frogs
(define Adjective-clause-SVO&SVP
    (BindLink
        (ListLink anticident-inst sub-root-verb  concept-sub-1 concept-main-1)
        (ImplicationLink
            (AndLink
                (InheritanceLink (VariableNode "$anticident-inst")(VariableNode "$concept-main-1"))
                (EvaluationLink (VariableNode "$sub-root-verb")(ListLink(VariableNode "$anticident-inst")(VariableNode "$concept-sub-1")))
                (EvaluationLink (PredicateNode "whichmarker") (ListLink (VariableNode "$anticident-inst")(VariableNode "$sub-root-verb"))))
        (ExecutionLink
           (GroundedSchemaNode "scm: rewrite-graph")
            (ListLink
                (EvaluationLink (VariableNode "$sub-root-verb") (ListLink (VariableNode "$X") (VariableNode "$concept-sub-1")))
                (InheritanceLink (VariableNode "$X")(VariableNode "$concept-main-1"))
                (VariableNode "$anticident-inst"))
           ))))
;For sentences with SVO rule applied twice 
;Example: "I like trees which have coffee beans." 
;RelEx2Logic representaion:
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
;$main-root-verb =like,$concept-main-1=i,$anticident-inst=tree,$sub-root-verb=have and $concept-sub-1=been
(define adjective-clause-SVO&SVO
    (BindLink
        (ListLink anticident-inst sub-root-verb concept-sub-1 concept-main-1 main-root-verb)
        (ImplicationLink
            (AndLink
                (EvaluationLink (VariableNode "$main-root-verb")(ListLink(VariableNode "$concept-main-1")(VariableNode "$anticident-inst")))
                (EvaluationLink (VariableNode "$sub-root-verb")(ListLink(VariableNode "$anticident-inst")(VariableNode "$concept-sub-1")))
                (EvaluationLink (PredicateNode "whichmarker") (ListLink (VariableNode "$anticident-inst")(VariableNode "$sub-root-verb"))))
        (ExecutionLink
           (GroundedSchemaNode "scm: rewrite-graph")
            (ListLink
                (EvaluationLink (VariableNode "$sub-root-verb") (ListLink (VariableNode "$X") (VariableNode "$concept-sub-1")))
                (EvaluationLink (PredicateNode "main-root-verb")(ListLink (ConceptNode "concept-main-1")(VariableNode "$X")))
                (VariableNode "$anticident-inst")))
)))
;For sentences with SVP and SVIO rule
;Example:  "Books which you give me are interesting."
;RelEx2Logic representaion:
;(InheritanceLink
;        (SatisfyingSetLink (VariableNode "$X")
;            (AndLink
;                (InheritanceLink (VariableNode "$X")(ConceptNode "book"))
;                (EvaluationLink (PredicateNode "give@12")(ListLink (ConceptNode "you@13")(ConceptNode "me@18")(VariableNode "$x")))))
;        (SatisfyingSetLink (VariableNode "$X")
;            (InheritanceLink(VariableNode "$X")(ConceptNode "interesting@19"))))
;$anticident-inst = book ,$concept-main-1 = interesting,$sub-root-verb = give, $concept-sub-1 = you and $concept-sub-2= me
(define adjective-clause-SVP&SVIO
    (BindLink
        (ListLink anticident-inst concept-main-1 sub-root-verb concept-sub-1 concept-sub-2)
        (ImplicationLink
            (AndLink
                (InheritanceLink (VariableNode "$anticident-inst")(VariableNode "$concept-main-1"))
                (EvaluationLink (VariableNode "$sub-root-verb")(ListLink(VariableNode "$concept-sub-1")(VariableNode "$concept-sub-2")(VariableNode "$anticident-inst")))
                (EvaluationLink (PredicateNode "whichmarker") (ListLink (VariableNode "$anticident-inst")(VariableNode "$sub-root-verb"))))
        (ExecutionLink
           (GroundedSchemaNode "scm: rewrite-graph")
            (ListLink
                (EvaluationLink (PredicateNode "$sub-root-verb")(ListLink (ConceptNode "$concept-sub-1")(ConceptNode "$concept-sub-2")(VariableNode "$x")))
                (InheritanceLink(VariableNode "$X")(ConceptNode "$concept-main-1"))
                (VariableNode "$anticident-inst")))
)))
;For sentences with definite flag ,SVO and passive rule 
; Example: "The books which I read in the library were written by Charles Dickens." 
;RelEx2Logic representaion:
;(MemberLink
;    (ConceptNode Book@12)
;    (SatisfyingSetLink (VariableNode $X)
;        (AndLink
;            (InheritanceLink (VariableNode $X) (ConceptNode Book))
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
;$main-root-verb = write,$concept-main-1 = Charles_Dickens,$anticident-inst = book,$sub-root-verb = read and $concept-sub-1= I
(define adjective-clause-SVO&PASSIVE
    (BindLink
         (ListLink)
        (ImplicationLink
            (AndLink
                (EvaluationLink (VariableNode "$main-root-verb")(ListLink(VariableNode "$concept-main-1")(VariableNode "$anticident-inst")))
                (EvaluationLink (VariableNode "$sub-root-verb")(ListLink(VariableNode "$concept-sub-1")(VariableNode "$anticident-inst")))
                (EvaluationLink (PredicateNode "whichmarker") (ListLink (VariableNode "$anticident-inst")(VariableNode "$sub-root-verb"))))
                (EvaluationLink (PredicateNode "definite") (ListLink (VariableNode "$anticident-inst")))
        (ExecutionLink
           (GroundedSchemaNode "scm: rewrite-graphs-with-definite")
            (ListLink
                (EvaluationLink (PredicateNode "$sub-root-verb")(ListLink (ConceptNode "$concept-sub-1")(VariableNode "$x")))
                (EvaluationLink (PredicateNode "$main-root-verb")(ListLink (ConceptNode "$concept-main-1")(VariableNode "$x")))
                (VariableNode "$anticident-inst")))
)))

