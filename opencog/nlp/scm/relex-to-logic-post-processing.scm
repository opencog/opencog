;-------------------------------------------------------------------------------
;Variables used for post processing process of Relex2Logic 
(define anticident-inst (TypedVariableLink (VariableNode "$anticident-inst") (VariableTypeNode "ConceptNode")))
(define concept-sub-1 (TypedVariableLink (VariableNode "$concept-sub-1") (VariableTypeNode "ConceptNode")))
(define concept-main-1 (TypedVariableLink (VariableNode "$concept-main-1") (VariableTypeNode "ConceptNode")))
(define sub-root-verb (TypedVariableLink (VariableNode "$sub-root-verb") (VariableTypeNode "PredicateNode")))
(define main-root-verb (TypedVariableLink (VariableNode "$main-root-verb") (VariableTypeNode "PredicateNode")))
;-------------------------------------------------------------------------------
;find anticident of the given anticident instance 
(define (find-anticident ant-inst)
    (define temp-node '())
    (let ((lst (cog-chase-link 'InheritanceLink 'ConceptNode ant-inst)))
        (for-each (lambda (i)
            (if (equal? (cog-name ant-inst ) (cog-name i))
                (append temp-node (list i))
                #f))
        lst) temp-node))
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
;                   (ConceptNode "food@11")))))
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
            (InheritanceLink
                (SatisfyingSetLink (VariableNode "$X")
                    (AndLink
                        (InheritanceLink (VariableNode "$X") (VariableNode "$anticident-inst"))
                        (EvaluationLink (VariableNode "$sub-root-verb") (ListLink (VariableNode "$X") (VariableNode "$concept-sub-1")))))
                (SatisfyingSetLink (VariableNode "$X")
                    (InheritanceLink (VariableNode "$X") (VariableNode "$concept-main-1")))))))
