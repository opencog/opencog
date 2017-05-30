(use-modules (opencog) (opencog rule-engine))

; NOTE: Temporary Hack around cogservers scheme module loading issue
; See https://github.com/opencog/atomspace/issues/508 for details
; XXX Except this "hack" utterly, completel broken: since it causes
; everything to be defined in the rule-egine module, and thus everything
; become invisble outside the rule-engine module!!
; (load-from-path "opencog/rule-engine.scm")

; Load rule files. The order of the load matters
(load "../rule-utils.scm")
(load "load-rules.scm")

; Define r2l-rulebase for English.
(define-public r2l-rules (ConceptNode "R2L-en-RuleBase"))
(ure-define-rbs r2l-rules 100)

; Add rules to rulebase
; NOTE: The weight is chosen to be equal for all rules as the mode ure is
; being run in doesn't require it.

(ure-define-add-rule r2l-rules "advmod" advmod 1)
(ure-define-add-rule r2l-rules "advmod-maybe" advmod-maybe 1)

(ure-define-add-rule r2l-rules "adverbialpp" adverbialpp 1)

(ure-define-add-rule r2l-rules "amod" amod 1)

(ure-define-add-rule r2l-rules "atTime" atTime 1)

(ure-define-add-rule r2l-rules "because" because 1)

(ure-define-add-rule r2l-rules "be-inheritance" be-inheritance 1)

(ure-define-add-rule r2l-rules "compmod" compmod 1)

(ure-define-add-rule r2l-rules "comp" comp 1)

(ure-define-add-rule r2l-rules "copula-ynq" copula-ynq 1)

(ure-define-add-rule r2l-rules "definite" definite 1)

;(ure-define-add-rule r2l-rules "demdet" demdet 1)

(ure-define-add-rule r2l-rules "howdeg-q" howdeg-q 1)

(ure-define-add-rule r2l-rules "howpredadj1-q" howpredadj1-q 1)

(ure-define-add-rule r2l-rules "how-q" how-q 1)

(ure-define-add-rule r2l-rules "howquant-q" howquant-q 1)

;(ure-define-add-rule r2l-rules "IMPERATIVE" IMPERATIVE 1)

(ure-define-add-rule r2l-rules "neg" neg 1)

(ure-define-add-rule r2l-rules "poss" poss 1)

;(ure-define-add-rule r2l-rules "post-processing" post-processing 1)

(ure-define-add-rule r2l-rules "pp" pp 1)

(ure-define-add-rule r2l-rules "pred-ynq" pred-ynq 1)

;(ure-define-add-rule r2l-rules "prepadj" prepadj 1)

(ure-define-add-rule r2l-rules "PREP" PREP 1)

(ure-define-add-rule r2l-rules "quantity" quantity 1)

(ure-define-add-rule r2l-rules "rel" rel 1)

(ure-define-add-rule r2l-rules "rep" rep 1)

;(ure-define-add-rule r2l-rules "rule-helpers" rule-helpers 1)

(ure-define-add-rule r2l-rules "SP" SP 1)

(ure-define-add-rule r2l-rules "SVIO1" SVIO1 1)

(ure-define-add-rule r2l-rules "SVIO2" SVIO2 1)

(ure-define-add-rule r2l-rules "svo" svo 1)

(ure-define-add-rule r2l-rules "SV" SV 1)

(ure-define-add-rule r2l-rules "TOBE" TOBE 1)

(ure-define-add-rule r2l-rules "todo1" todo1 1)

(ure-define-add-rule r2l-rules "todo2" todo2 1)

(ure-define-add-rule r2l-rules "todo5" todo5 1)

(ure-define-add-rule r2l-rules "todo3" todo3 1)

;(ure-define-add-rule r2l-rules "utilities" utilities 1)

(ure-define-add-rule r2l-rules "when-cop-q" when-cop-q 1)

(ure-define-add-rule r2l-rules "when-q" when-q 1)

(ure-define-add-rule r2l-rules "where-cop-q" where-cop-q 1)

(ure-define-add-rule r2l-rules "where-q" where-q 1)

(ure-define-add-rule r2l-rules "whichiobjQ" whichiobjQ 1)

(ure-define-add-rule r2l-rules "whichobjQ" whichobjQ 1)

;(ure-define-add-rule r2l-rules "whichobjSVIOQ" whichobjSVIOQ 1)

;(ure-define-add-rule r2l-rules "whichpobjQ" whichpobjQ 1)

(ure-define-add-rule r2l-rules "whichpredadjQ" whichpredadjQ 1)

;(ure-define-add-rule r2l-rules "whichsubjpobjQ" whichsubjpobjQ 1)

;(ure-define-add-rule r2l-rules "whichsubjQ" whichsubjQ 1)

;(ure-define-add-rule r2l-rules "whichsubjSVIOQ" whichsubjSVIOQ 1)

(ure-define-add-rule r2l-rules "whichsubjSVOQ" whichsubjSVOQ 1)

;(ure-define-add-rule r2l-rules "whichsubjSVQ" whichsubjSVQ 1)

(ure-define-add-rule r2l-rules "why-cop-q" why-cop-q 1)

(ure-define-add-rule r2l-rules "why-q" why-q 1)

(ure-define-add-rule r2l-rules "declarative" declarative 1)

(ure-define-add-rule r2l-rules "truthquery" truthquery 1)

(ure-define-add-rule r2l-rules "interrogative" interrogative 1)

(ure-define-add-rule r2l-rules "imperative" imperative 1)

(ure-define-add-rule r2l-rules "det" det 1)

(ure-define-add-rule r2l-rules "nn" nn 1)

(ure-define-add-rule r2l-rules "gender" gender 1)

(ure-define-add-rule r2l-rules "conj" conj 1)

(ure-define-add-rule r2l-rules "tense" tense 1)

(ure-define-add-rule r2l-rules "passive" passive 1)
