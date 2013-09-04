;
;
; Parse rules and stuff.
;

(define (stv p c) (cog-new-stv p c))

; XXX TODO
;; for future rules, make use of varscope-wrap-implication which can be
; found in triples/varscope.scm -- it just extracts the varaibles,
;

(define attach 
  (BindLink
    ; Bound variable declarations. These are the variables we expect
    ; to match up when connnecting the left and right words with a connector.
    (ListLink
      (TypedVariableLink
        (VariableNode "$left word")
        (VariableTypeNode "WordNode")
      )
      (TypedVariableLink
        (VariableNode "$right word")
        (VariableTypeNode "WordNode")
      )
      (TypedVariableLink
        (VariableNode "$connector")
        (VariableTypeNode "LgConnectorNode")
      )
    )
    (ImplicationLink
      ; First, a list of all the graphs we want to match
      (AndLink
        ; Fish out the first unconnected connector in the state pair.
        (LgStatePair
          (LgSeq
            (LgWordCset
              (VariableNode "$left word")
              (LgConnector
                (VariableNode "$connector")
                (LgConnDirNode "+")
              )
            )
            ; XXX TODO if there's more in the seqeunce, we don't care ...
          )
          ; The output state could be anything; ignore it
          ; (VariableNode "$whatever dont care")
          ; XXX TODO how do we ignore a link!??
          (LgSeq)
        )
        ; We also want to match up a word ...
        (LgWordCset
          (VariableNode "$right word")
          (LgConnector
            (VariableNode "$connector")
            (LgConnDirNode "-")
          )
        )
      )
      ; OK if the above matched, then we create the below..
      (ListLink
        (ConceptNode "whopee we got one")
        (VariableNode "$connector")
        (VariableNode "$left word")
        (VariableNode "$right word")
      )
    )
  )
)


