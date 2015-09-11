; MOSES model gotten from
;
; moses -i dataset.csv -u recovery-speed-of-injury-alpha -W 1 -Hpre -q0.7 -p0.5 -c 1 --output-format scheme
;
; Wrapped in an ImplicationLink to relate models and target feature +
; confidence.

(define moses-model
   (ImplicationLink (stv 0.875 0.44)
      (OrLink
         (PredicateNode "take-treatment-1")
         (PredicateNode "eat-lots-fruits-vegetables")
      )
      (PredicateNode "recovery-speed-of-injury-alpha")
   )
)
