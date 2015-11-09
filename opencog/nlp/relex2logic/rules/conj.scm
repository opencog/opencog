(define conjand
    (BindLink
        (VariableList
            (TypedVariableLink
                (VariableNode "$a-parse")
                (TypeNode "ParseNode")
            )
            (TypedVariableLink
                (VariableNode "$var1")
                (TypeNode "WordInstanceNode")
            )
            (TypedVariableLink
                (VariableNode "$var2")
                (TypeNode "WordInstanceNode")
            )
		    (TypedVariableLink
                (VariableNode "$pos")
                (TypeNode "DefinedLinguisticConceptNode")
            )
        )
        (AndLink
            (WordInstanceLink
                (VariableNode "$var1")
                (VariableNode "$a-parse")
            )
            (WordInstanceLink
                (VariableNode "$var2")
                (VariableNode "$a-parse")
            )
		    (PartOfSpeechLink
   				(VariableNode "$var1")
   				(VariableNode "$pos")
			)
		    (PartOfSpeechLink
                (VariableNode "$var2")
   				(VariableNode "$pos")
			)
		    (EvaluationLink
   					(PrepositionalRelationshipNode "conj_and")
   						(ListLink
      						(VariableNode "$var1")
      						(VariableNode "$var2")
   						)
				)
        )
       (ListLink
        (ExecutionOutputLink
       	   (GroundedSchemaNode "scm: pre-conjand-rule")
       	      (ListLink
       	         (VariableNode "$var1")
       	         (VariableNode "$var2")
			     (VariableNode "$pos")
            )
        )
      )
    )
)

(define (pre-conjand-rule var1 var2 pos)
  (ListLink 
    (and-rule (cog-name (word-inst-get-lemma  var1)) (cog-name var1)
              (cog-name (word-inst-get-lemma  var2)) (cog-name var2)
              (cog-name pos)
    )
  )
)