;; Test #1

;; Anaphora is not "that" or "enough"
;; Antecedent is a noun

;; Expected result:
;; Acceptance

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
    (DefinedLinguisticConceptNode "noun")
)
