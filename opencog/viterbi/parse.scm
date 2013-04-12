;
;
; Parse rules and stuff.
;

(define (attach) 
  (ImplicationLink
    ; First, a list of all the graphs we want to match
    (AndLink
      ; Fish out the first unconnected connector in the state pair.
      (LgStatePair
        (LgSeq
          (LgWordCset
            ; XXX TODO the type of this varnode should be WordNode
            (VariableNode "$left word")
            (LgConnector
              ; XXX TODO the type of this should be LgConnectorNode
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
        ; XXX TODO type of this should be WordNode
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


