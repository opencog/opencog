;; Test #2

;; Anaphora is not "that" or "enough"
;; Antecedent is a verb

;; Expected result:
;; Rejection

;; Connection between two clauses

(ListLink
    (AnchorNode "CurrentResolution")
    (WordInstanceNode "anaphor")
    (WordInstanceNode "antecedent")
)
(ListLink
    (AnchorNode "CurrentPronoun")
    (WordInstanceNode "anaphor")
)
(ListLink
    (AnchorNode "CurrentProposal")
    (WordInstanceNode "antecedent")
)
;; filter tests

(PartOfSpeechLink
    (WordInstanceNode "antecedent")
    (DefinedLinguisticConceptNode "verb")
)
