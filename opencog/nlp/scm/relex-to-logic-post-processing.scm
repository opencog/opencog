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
