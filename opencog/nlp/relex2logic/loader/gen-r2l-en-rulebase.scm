(use-modules (opencog) (opencog ure))

; Load rule files. The order of the load matters
(load "../rule-utils.scm")
(load "../loader.scm")

; Define r2l-rulebase for English.
(ure-define-rbs r2l-rules 100)

; Add rules to rulebase
; NOTE: The weight is chosen to be equal for all rules as the mode ure is
; being run in doesn't require it.

(ure-define-add-rule r2l-rules "advmod" advmod (stv 1 1))
(ure-define-add-rule r2l-rules "advmod-maybe" advmod-maybe (stv 1 1))

(ure-define-add-rule r2l-rules "adverbialpp" adverbialpp (stv 1 1))

(ure-define-add-rule r2l-rules "amod" amod (stv 1 1))

(ure-define-add-rule r2l-rules "atTime" atTime (stv 1 1))

(ure-define-add-rule r2l-rules "because" because (stv 1 1))

(ure-define-add-rule r2l-rules "be-inheritance" be-inheritance (stv 1 1))

(ure-define-add-rule r2l-rules "compmod" compmod (stv 1 1))

(ure-define-add-rule r2l-rules "comp" comp (stv 1 1))

(ure-define-add-rule r2l-rules "copula-ynq" copula-ynq (stv 1 1))

(ure-define-add-rule r2l-rules "definite" definite (stv 1 1))

;(ure-define-add-rule r2l-rules "demdet" demdet (stv 1 1))

(ure-define-add-rule r2l-rules "howdeg-q" howdeg-q (stv 1 1))

(ure-define-add-rule r2l-rules "howpredadj1-q" howpredadj1-q (stv 1 1))

(ure-define-add-rule r2l-rules "how-q" how-q (stv 1 1))

(ure-define-add-rule r2l-rules "howquant-q" howquant-q (stv 1 1))

;(ure-define-add-rule r2l-rules "IMPERATIVE" IMPERATIVE (stv 1 1))

(ure-define-add-rule r2l-rules "neg" neg (stv 1 1))

(ure-define-add-rule r2l-rules "poss" poss (stv 1 1))

;(ure-define-add-rule r2l-rules "post-processing" post-processing (stv 1 1))

(ure-define-add-rule r2l-rules "pp" pp (stv 1 1))

(ure-define-add-rule r2l-rules "pred-ynq" pred-ynq (stv 1 1))

;(ure-define-add-rule r2l-rules "prepadj" prepadj (stv 1 1))

(ure-define-add-rule r2l-rules "PREP" PREP (stv 1 1))

(ure-define-add-rule r2l-rules "quantity" quantity (stv 1 1))

(ure-define-add-rule r2l-rules "rel" rel (stv 1 1))

(ure-define-add-rule r2l-rules "rep" rep (stv 1 1))

;(ure-define-add-rule r2l-rules "rule-helpers" rule-helpers (stv 1 1))

(ure-define-add-rule r2l-rules "SP" SP (stv 1 1))

(ure-define-add-rule r2l-rules "SVIO1" SVIO1 (stv 1 1))

(ure-define-add-rule r2l-rules "SVIO2" SVIO2 (stv 1 1))

(ure-define-add-rule r2l-rules "svo" svo (stv 1 1))

(ure-define-add-rule r2l-rules "SV" SV (stv 1 1))

(ure-define-add-rule r2l-rules "TOBE" TOBE (stv 1 1))

(ure-define-add-rule r2l-rules "todo1" todo1 (stv 1 1))

(ure-define-add-rule r2l-rules "todo2" todo2 (stv 1 1))

(ure-define-add-rule r2l-rules "todo5" todo5 (stv 1 1))

(ure-define-add-rule r2l-rules "todo3" todo3 (stv 1 1))

;(ure-define-add-rule r2l-rules "utilities" utilities (stv 1 1))

(ure-define-add-rule r2l-rules "when-cop-q" when-cop-q (stv 1 1))

(ure-define-add-rule r2l-rules "when-q" when-q (stv 1 1))

(ure-define-add-rule r2l-rules "where-cop-q" where-cop-q (stv 1 1))

(ure-define-add-rule r2l-rules "where-q" where-q (stv 1 1))

(ure-define-add-rule r2l-rules "whichiobjQ" whichiobjQ (stv 1 1))

(ure-define-add-rule r2l-rules "whichobjQ" whichobjQ (stv 1 1))

;(ure-define-add-rule r2l-rules "whichobjSVIOQ" whichobjSVIOQ (stv 1 1))

;(ure-define-add-rule r2l-rules "whichpobjQ" whichpobjQ (stv 1 1))

(ure-define-add-rule r2l-rules "whichpredadjQ" whichpredadjQ (stv 1 1))

;(ure-define-add-rule r2l-rules "whichsubjpobjQ" whichsubjpobjQ (stv 1 1))

;(ure-define-add-rule r2l-rules "whichsubjQ" whichsubjQ (stv 1 1))

;(ure-define-add-rule r2l-rules "whichsubjSVIOQ" whichsubjSVIOQ (stv 1 1))

(ure-define-add-rule r2l-rules "whichsubjSVOQ" whichsubjSVOQ (stv 1 1))

;(ure-define-add-rule r2l-rules "whichsubjSVQ" whichsubjSVQ (stv 1 1))

(ure-define-add-rule r2l-rules "why-cop-q" why-cop-q (stv 1 1))

(ure-define-add-rule r2l-rules "why-q" why-q (stv 1 1))

(ure-define-add-rule r2l-rules "declarative" declarative (stv 1 1))

(ure-define-add-rule r2l-rules "truthquery" truthquery (stv 1 1))

(ure-define-add-rule r2l-rules "interrogative" interrogative (stv 1 1))

(ure-define-add-rule r2l-rules "imperative" imperative (stv 1 1))

(ure-define-add-rule r2l-rules "det" det (stv 1 1))

(ure-define-add-rule r2l-rules "nn" nn (stv 1 1))

(ure-define-add-rule r2l-rules "gender" gender (stv 1 1))

(ure-define-add-rule r2l-rules "conj" conj (stv 1 1))

(ure-define-add-rule r2l-rules "tense" tense (stv 1 1))

(ure-define-add-rule r2l-rules "passive" passive (stv 1 1))
