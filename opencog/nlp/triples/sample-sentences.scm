scm
;
; sample-sentences.scm
;
; This file contains a sequence of example sentences, previously parsed
; by linkgrammar+relex, and output in SCM format. Many of these sentences
; are discussed in the README file; they form the raw input on which 
; prepositional relation extraction, PLN reasoning, and question-answering
; can be done.
;
; The current list of sentences are:
; [Berlin is in Germany.]
; [Berlin is a city in Germany.]
; [The heart is in the chest.]
; [The heart is inside the chest.]
; [The garage is behind the house.]
; [Lisbon is the capital of Portugaul.]
; [The capital of Germany is Berlin.]
; [The color of the sky is blue.]
; [Pottery is made from clay.]
; [Yarn is spun from fibers.]
; [Yarn is made of fibers.]
;
; January 2009
;
; Version: link-grammar-4.4.2	relex-0.98.0
; SENTENCE: [Berlin is in Germany.]
(ReferenceLink
   (WordInstanceNode "Berlin@a66eee16-1f1e-4adb-af57-54aaabc0da58")
   (WordNode "Berlin")
)
(WordInstanceLink
   (WordInstanceNode "Berlin@a66eee16-1f1e-4adb-af57-54aaabc0da58")
   (ParseNode "sentence@4bc32f08-9e5e-4af8-85db-36cb95f3d5e6_parse_0")
)
(ReferenceLink
   (WordInstanceNode "is@e61b46cc-e964-4fbd-a09f-34d8b9a0b23c")
   (WordNode "is")
)
(WordInstanceLink
   (WordInstanceNode "is@e61b46cc-e964-4fbd-a09f-34d8b9a0b23c")
   (ParseNode "sentence@4bc32f08-9e5e-4af8-85db-36cb95f3d5e6_parse_0")
)
(ReferenceLink
   (WordInstanceNode "in@b03436d8-8c1e-4774-9db5-52b7d3d562bc")
   (WordNode "in")
)
(WordInstanceLink
   (WordInstanceNode "in@b03436d8-8c1e-4774-9db5-52b7d3d562bc")
   (ParseNode "sentence@4bc32f08-9e5e-4af8-85db-36cb95f3d5e6_parse_0")
)
(ReferenceLink
   (WordInstanceNode "Germany@4f751c22-8088-4afc-8b1e-f1fee59d2186")
   (WordNode "Germany")
)
(WordInstanceLink
   (WordInstanceNode "Germany@4f751c22-8088-4afc-8b1e-f1fee59d2186")
   (ParseNode "sentence@4bc32f08-9e5e-4af8-85db-36cb95f3d5e6_parse_0")
)
(ReferenceLink
   (WordInstanceNode ".@c9b01453-1591-41ad-a84a-2998be9dcfe2")
   (WordNode ".")
)
(WordInstanceLink
   (WordInstanceNode ".@c9b01453-1591-41ad-a84a-2998be9dcfe2")
   (ParseNode "sentence@4bc32f08-9e5e-4af8-85db-36cb95f3d5e6_parse_0")
)
(ReferenceLink
   (ParseNode "sentence@4bc32f08-9e5e-4af8-85db-36cb95f3d5e6_parse_0")
   (ListLink
      (WordInstanceNode "Berlin@a66eee16-1f1e-4adb-af57-54aaabc0da58")
      (WordInstanceNode "is@e61b46cc-e964-4fbd-a09f-34d8b9a0b23c")
      (WordInstanceNode "in@b03436d8-8c1e-4774-9db5-52b7d3d562bc")
      (WordInstanceNode "Germany@4f751c22-8088-4afc-8b1e-f1fee59d2186")
      (WordInstanceNode ".@c9b01453-1591-41ad-a84a-2998be9dcfe2")
   )
)
(EvaluationLink
   (LinkGrammarRelationshipNode "Js")
   (ListLink
      (WordInstanceNode "in@b03436d8-8c1e-4774-9db5-52b7d3d562bc")
      (WordInstanceNode "Germany@4f751c22-8088-4afc-8b1e-f1fee59d2186")
   )
)
(EvaluationLink
   (LinkGrammarRelationshipNode "Xp")
   (ListLink
      (WordInstanceNode "LEFT-WALL")
      (WordInstanceNode ".@c9b01453-1591-41ad-a84a-2998be9dcfe2")
   )
)
(EvaluationLink
   (LinkGrammarRelationshipNode "Pp")
   (ListLink
      (WordInstanceNode "is@e61b46cc-e964-4fbd-a09f-34d8b9a0b23c")
      (WordInstanceNode "in@b03436d8-8c1e-4774-9db5-52b7d3d562bc")
   )
)
(EvaluationLink
   (LinkGrammarRelationshipNode "Ss")
   (ListLink
      (WordInstanceNode "Berlin@a66eee16-1f1e-4adb-af57-54aaabc0da58")
      (WordInstanceNode "is@e61b46cc-e964-4fbd-a09f-34d8b9a0b23c")
   )
)
(EvaluationLink
   (LinkGrammarRelationshipNode "Wd")
   (ListLink
      (WordInstanceNode "LEFT-WALL")
      (WordInstanceNode "Berlin@a66eee16-1f1e-4adb-af57-54aaabc0da58")
   )
)
(ParseLink
   (ParseNode "sentence@4bc32f08-9e5e-4af8-85db-36cb95f3d5e6_parse_0" (stv 1.0 0.9531))
   (SentenceNode "sentence@4bc32f08-9e5e-4af8-85db-36cb95f3d5e6")
)
(LemmaLink
   (WordInstanceNode "Berlin@a66eee16-1f1e-4adb-af57-54aaabc0da58")
   (WordNode "Berlin")
)
(LemmaLink
   (WordInstanceNode "is@e61b46cc-e964-4fbd-a09f-34d8b9a0b23c")
   (WordNode "be")
)
(LemmaLink
   (WordInstanceNode "in@b03436d8-8c1e-4774-9db5-52b7d3d562bc")
   (WordNode "in")
)
(LemmaLink
   (WordInstanceNode "Germany@4f751c22-8088-4afc-8b1e-f1fee59d2186")
   (WordNode "Germany")
)
(LemmaLink
   (WordInstanceNode ".@c9b01453-1591-41ad-a84a-2998be9dcfe2")
   (WordNode ".")
)
; DEFINITE-FLAG (Germany, T)
(InheritanceLink
   (WordInstanceNode "Germany@4f751c22-8088-4afc-8b1e-f1fee59d2186")
   (DefinedLinguisticConceptNode "definite")
)
; noun_number (Germany, singular)
(InheritanceLink
   (WordInstanceNode "Germany@4f751c22-8088-4afc-8b1e-f1fee59d2186")
   (DefinedLinguisticConceptNode "singular")
)
; inflection-TAG (Germany, .l)
(InheritanceLink
   (WordInstanceNode "Germany@4f751c22-8088-4afc-8b1e-f1fee59d2186")
   (DefinedLinguisticConceptNode ".l")
)
; location-FLAG (Germany, T)
(InheritanceLink
   (WordInstanceNode "Germany@4f751c22-8088-4afc-8b1e-f1fee59d2186")
   (DefinedLinguisticConceptNode "location")
)
; pos (Germany, noun)
(PartOfSpeechLink
   (WordInstanceNode "Germany@4f751c22-8088-4afc-8b1e-f1fee59d2186")
   (DefinedLinguisticConceptNode "noun")
)
; pos (., punctuation)
(PartOfSpeechLink
   (WordInstanceNode ".@c9b01453-1591-41ad-a84a-2998be9dcfe2")
   (DefinedLinguisticConceptNode "punctuation")
)
; DEFINITE-FLAG (Berlin, T)
(InheritanceLink
   (WordInstanceNode "Berlin@a66eee16-1f1e-4adb-af57-54aaabc0da58")
   (DefinedLinguisticConceptNode "definite")
)
; noun_number (Berlin, singular)
(InheritanceLink
   (WordInstanceNode "Berlin@a66eee16-1f1e-4adb-af57-54aaabc0da58")
   (DefinedLinguisticConceptNode "singular")
)
; pos (Berlin, noun)
(PartOfSpeechLink
   (WordInstanceNode "Berlin@a66eee16-1f1e-4adb-af57-54aaabc0da58")
   (DefinedLinguisticConceptNode "noun")
)
; in (<<_%copula>>, <<in>>) 
(EvaluationLink
   (DefinedLinguisticRelationshipNode "in")
   (ListLink
      (WordInstanceNode "is@e61b46cc-e964-4fbd-a09f-34d8b9a0b23c")
      (WordInstanceNode "in@b03436d8-8c1e-4774-9db5-52b7d3d562bc")
   )
)
; inflection-TAG (_%copula, .v)
(InheritanceLink
   (WordInstanceNode "is@e61b46cc-e964-4fbd-a09f-34d8b9a0b23c")
   (DefinedLinguisticConceptNode ".v")
)
; pos (_%copula, verb)
(PartOfSpeechLink
   (WordInstanceNode "is@e61b46cc-e964-4fbd-a09f-34d8b9a0b23c")
   (DefinedLinguisticConceptNode "verb")
)
; _pobj (<<in>>, <<Germany>>) 
(EvaluationLink
   (DefinedLinguisticRelationshipNode "_pobj")
   (ListLink
      (WordInstanceNode "in@b03436d8-8c1e-4774-9db5-52b7d3d562bc")
      (WordInstanceNode "Germany@4f751c22-8088-4afc-8b1e-f1fee59d2186")
   )
)
; _psubj (<<in>>, <<Berlin>>) 
(EvaluationLink
   (DefinedLinguisticRelationshipNode "_psubj")
   (ListLink
      (WordInstanceNode "in@b03436d8-8c1e-4774-9db5-52b7d3d562bc")
      (WordInstanceNode "Berlin@a66eee16-1f1e-4adb-af57-54aaabc0da58")
   )
)
; tense (in, present)
(InheritanceLink
   (WordInstanceNode "in@b03436d8-8c1e-4774-9db5-52b7d3d562bc")
   (DefinedLinguisticConceptNode "present")
)
; SPECIAL-PREP-FLAG (in, T)
(InheritanceLink
   (WordInstanceNode "in@b03436d8-8c1e-4774-9db5-52b7d3d562bc")
   (DefinedLinguisticConceptNode "special-prep")
)
; pos (in, verb)
(PartOfSpeechLink
   (WordInstanceNode "in@b03436d8-8c1e-4774-9db5-52b7d3d562bc")
   (DefinedLinguisticConceptNode "verb")
)

(ReferenceLink
   (WordInstanceNode "Berlin@152d5095-ff61-43b0-a98f-38cd4d249306")
   (WordNode "Berlin")
)
(WordInstanceLink
   (WordInstanceNode "Berlin@152d5095-ff61-43b0-a98f-38cd4d249306")
   (ParseNode "sentence@4bc32f08-9e5e-4af8-85db-36cb95f3d5e6_parse_1")
)
(ReferenceLink
   (WordInstanceNode "is@85c561a1-f786-4faf-8f0f-8a9e50561da8")
   (WordNode "is")
)
(WordInstanceLink
   (WordInstanceNode "is@85c561a1-f786-4faf-8f0f-8a9e50561da8")
   (ParseNode "sentence@4bc32f08-9e5e-4af8-85db-36cb95f3d5e6_parse_1")
)
(ReferenceLink
   (WordInstanceNode "in@ba85bff6-5525-47cb-bcf7-0b728a217ed5")
   (WordNode "in")
)
(WordInstanceLink
   (WordInstanceNode "in@ba85bff6-5525-47cb-bcf7-0b728a217ed5")
   (ParseNode "sentence@4bc32f08-9e5e-4af8-85db-36cb95f3d5e6_parse_1")
)
(ReferenceLink
   (WordInstanceNode "Germany@7bdf1465-8d80-49d3-8aca-b4fb43908a01")
   (WordNode "Germany")
)
(WordInstanceLink
   (WordInstanceNode "Germany@7bdf1465-8d80-49d3-8aca-b4fb43908a01")
   (ParseNode "sentence@4bc32f08-9e5e-4af8-85db-36cb95f3d5e6_parse_1")
)
(ReferenceLink
   (WordInstanceNode ".@7c5104d6-53c4-46b2-b4b9-744a1c2e2c5e")
   (WordNode ".")
)
(WordInstanceLink
   (WordInstanceNode ".@7c5104d6-53c4-46b2-b4b9-744a1c2e2c5e")
   (ParseNode "sentence@4bc32f08-9e5e-4af8-85db-36cb95f3d5e6_parse_1")
)
(ReferenceLink
   (ParseNode "sentence@4bc32f08-9e5e-4af8-85db-36cb95f3d5e6_parse_1")
   (ListLink
      (WordInstanceNode "Berlin@152d5095-ff61-43b0-a98f-38cd4d249306")
      (WordInstanceNode "is@85c561a1-f786-4faf-8f0f-8a9e50561da8")
      (WordInstanceNode "in@ba85bff6-5525-47cb-bcf7-0b728a217ed5")
      (WordInstanceNode "Germany@7bdf1465-8d80-49d3-8aca-b4fb43908a01")
      (WordInstanceNode ".@7c5104d6-53c4-46b2-b4b9-744a1c2e2c5e")
   )
)
(EvaluationLink
   (LinkGrammarRelationshipNode "MVp")
   (ListLink
      (WordInstanceNode "is@85c561a1-f786-4faf-8f0f-8a9e50561da8")
      (WordInstanceNode "in@ba85bff6-5525-47cb-bcf7-0b728a217ed5")
   )
)
(EvaluationLink
   (LinkGrammarRelationshipNode "Js")
   (ListLink
      (WordInstanceNode "in@ba85bff6-5525-47cb-bcf7-0b728a217ed5")
      (WordInstanceNode "Germany@7bdf1465-8d80-49d3-8aca-b4fb43908a01")
   )
)
(EvaluationLink
   (LinkGrammarRelationshipNode "Xp")
   (ListLink
      (WordInstanceNode "LEFT-WALL")
      (WordInstanceNode ".@7c5104d6-53c4-46b2-b4b9-744a1c2e2c5e")
   )
)
(EvaluationLink
   (LinkGrammarRelationshipNode "Ss")
   (ListLink
      (WordInstanceNode "Berlin@152d5095-ff61-43b0-a98f-38cd4d249306")
      (WordInstanceNode "is@85c561a1-f786-4faf-8f0f-8a9e50561da8")
   )
)
(EvaluationLink
   (LinkGrammarRelationshipNode "Wd")
   (ListLink
      (WordInstanceNode "LEFT-WALL")
      (WordInstanceNode "Berlin@152d5095-ff61-43b0-a98f-38cd4d249306")
   )
)
(ParseLink
   (ParseNode "sentence@4bc32f08-9e5e-4af8-85db-36cb95f3d5e6_parse_1" (stv 1.0 0.6389))
   (SentenceNode "sentence@4bc32f08-9e5e-4af8-85db-36cb95f3d5e6")
)
(LemmaLink
   (WordInstanceNode "Berlin@152d5095-ff61-43b0-a98f-38cd4d249306")
   (WordNode "Berlin")
)
(LemmaLink
   (WordInstanceNode "is@85c561a1-f786-4faf-8f0f-8a9e50561da8")
   (WordNode "be")
)
(LemmaLink
   (WordInstanceNode "in@ba85bff6-5525-47cb-bcf7-0b728a217ed5")
   (WordNode "in")
)
(LemmaLink
   (WordInstanceNode "Germany@7bdf1465-8d80-49d3-8aca-b4fb43908a01")
   (WordNode "Germany")
)
(LemmaLink
   (WordInstanceNode ".@7c5104d6-53c4-46b2-b4b9-744a1c2e2c5e")
   (WordNode ".")
)
; pos (in, prep)
(PartOfSpeechLink
   (WordInstanceNode "in@ba85bff6-5525-47cb-bcf7-0b728a217ed5")
   (DefinedLinguisticConceptNode "prep")
)
; DEFINITE-FLAG (Germany, T)
(InheritanceLink
   (WordInstanceNode "Germany@7bdf1465-8d80-49d3-8aca-b4fb43908a01")
   (DefinedLinguisticConceptNode "definite")
)
; noun_number (Germany, singular)
(InheritanceLink
   (WordInstanceNode "Germany@7bdf1465-8d80-49d3-8aca-b4fb43908a01")
   (DefinedLinguisticConceptNode "singular")
)
; inflection-TAG (Germany, .l)
(InheritanceLink
   (WordInstanceNode "Germany@7bdf1465-8d80-49d3-8aca-b4fb43908a01")
   (DefinedLinguisticConceptNode ".l")
)
; location-FLAG (Germany, T)
(InheritanceLink
   (WordInstanceNode "Germany@7bdf1465-8d80-49d3-8aca-b4fb43908a01")
   (DefinedLinguisticConceptNode "location")
)
; pos (Germany, noun)
(PartOfSpeechLink
   (WordInstanceNode "Germany@7bdf1465-8d80-49d3-8aca-b4fb43908a01")
   (DefinedLinguisticConceptNode "noun")
)
; pos (., punctuation)
(PartOfSpeechLink
   (WordInstanceNode ".@7c5104d6-53c4-46b2-b4b9-744a1c2e2c5e")
   (DefinedLinguisticConceptNode "punctuation")
)
; DEFINITE-FLAG (Berlin, T)
(InheritanceLink
   (WordInstanceNode "Berlin@152d5095-ff61-43b0-a98f-38cd4d249306")
   (DefinedLinguisticConceptNode "definite")
)
; noun_number (Berlin, singular)
(InheritanceLink
   (WordInstanceNode "Berlin@152d5095-ff61-43b0-a98f-38cd4d249306")
   (DefinedLinguisticConceptNode "singular")
)
; pos (Berlin, noun)
(PartOfSpeechLink
   (WordInstanceNode "Berlin@152d5095-ff61-43b0-a98f-38cd4d249306")
   (DefinedLinguisticConceptNode "noun")
)
; in (<<be>>, <<Germany>>) 
(EvaluationLink
   (DefinedLinguisticRelationshipNode "in")
   (ListLink
      (WordInstanceNode "is@85c561a1-f786-4faf-8f0f-8a9e50561da8")
      (WordInstanceNode "Germany@7bdf1465-8d80-49d3-8aca-b4fb43908a01")
   )
)
; _subj (<<be>>, <<Berlin>>) 
(EvaluationLink
   (DefinedLinguisticRelationshipNode "_subj")
   (ListLink
      (WordInstanceNode "is@85c561a1-f786-4faf-8f0f-8a9e50561da8")
      (WordInstanceNode "Berlin@152d5095-ff61-43b0-a98f-38cd4d249306")
   )
)
; tense (be, present)
(InheritanceLink
   (WordInstanceNode "is@85c561a1-f786-4faf-8f0f-8a9e50561da8")
   (DefinedLinguisticConceptNode "present")
)
; inflection-TAG (be, .v)
(InheritanceLink
   (WordInstanceNode "is@85c561a1-f786-4faf-8f0f-8a9e50561da8")
   (DefinedLinguisticConceptNode ".v")
)
; pos (be, verb)
(PartOfSpeechLink
   (WordInstanceNode "is@85c561a1-f786-4faf-8f0f-8a9e50561da8")
   (DefinedLinguisticConceptNode "verb")
)

; SENTENCE: [Berlin is a city in Germany.]
(ReferenceLink
   (WordInstanceNode "Berlin@b7da0d0d-75e3-4bbc-ac58-e64a7377c8e1")
   (WordNode "Berlin")
)
(WordInstanceLink
   (WordInstanceNode "Berlin@b7da0d0d-75e3-4bbc-ac58-e64a7377c8e1")
   (ParseNode "sentence@327f8945-7634-4d27-b192-ce477fec4370_parse_0")
)
(ReferenceLink
   (WordInstanceNode "is@eabce068-389e-4df0-a799-cb0176836791")
   (WordNode "is")
)
(WordInstanceLink
   (WordInstanceNode "is@eabce068-389e-4df0-a799-cb0176836791")
   (ParseNode "sentence@327f8945-7634-4d27-b192-ce477fec4370_parse_0")
)
(ReferenceLink
   (WordInstanceNode "a@fa79be35-996a-48b6-ad16-2d93237dcdf1")
   (WordNode "a")
)
(WordInstanceLink
   (WordInstanceNode "a@fa79be35-996a-48b6-ad16-2d93237dcdf1")
   (ParseNode "sentence@327f8945-7634-4d27-b192-ce477fec4370_parse_0")
)
(ReferenceLink
   (WordInstanceNode "city@f73c69fc-3683-4a22-8bf9-e705a5aa5191")
   (WordNode "city")
)
(WordInstanceLink
   (WordInstanceNode "city@f73c69fc-3683-4a22-8bf9-e705a5aa5191")
   (ParseNode "sentence@327f8945-7634-4d27-b192-ce477fec4370_parse_0")
)
(ReferenceLink
   (WordInstanceNode "in@3b8caa2e-d340-4061-9d4c-efe49ee7b7f6")
   (WordNode "in")
)
(WordInstanceLink
   (WordInstanceNode "in@3b8caa2e-d340-4061-9d4c-efe49ee7b7f6")
   (ParseNode "sentence@327f8945-7634-4d27-b192-ce477fec4370_parse_0")
)
(ReferenceLink
   (WordInstanceNode "Germany@497f22b3-53df-4bea-8789-ae1c3eda38b1")
   (WordNode "Germany")
)
(WordInstanceLink
   (WordInstanceNode "Germany@497f22b3-53df-4bea-8789-ae1c3eda38b1")
   (ParseNode "sentence@327f8945-7634-4d27-b192-ce477fec4370_parse_0")
)
(ReferenceLink
   (WordInstanceNode ".@86bad39d-9070-40f3-a1af-bd270aeb0cac")
   (WordNode ".")
)
(WordInstanceLink
   (WordInstanceNode ".@86bad39d-9070-40f3-a1af-bd270aeb0cac")
   (ParseNode "sentence@327f8945-7634-4d27-b192-ce477fec4370_parse_0")
)
(ReferenceLink
   (ParseNode "sentence@327f8945-7634-4d27-b192-ce477fec4370_parse_0")
   (ListLink
      (WordInstanceNode "Berlin@b7da0d0d-75e3-4bbc-ac58-e64a7377c8e1")
      (WordInstanceNode "is@eabce068-389e-4df0-a799-cb0176836791")
      (WordInstanceNode "a@fa79be35-996a-48b6-ad16-2d93237dcdf1")
      (WordInstanceNode "city@f73c69fc-3683-4a22-8bf9-e705a5aa5191")
      (WordInstanceNode "in@3b8caa2e-d340-4061-9d4c-efe49ee7b7f6")
      (WordInstanceNode "Germany@497f22b3-53df-4bea-8789-ae1c3eda38b1")
      (WordInstanceNode ".@86bad39d-9070-40f3-a1af-bd270aeb0cac")
   )
)
(EvaluationLink
   (LinkGrammarRelationshipNode "MVp")
   (ListLink
      (WordInstanceNode "is@eabce068-389e-4df0-a799-cb0176836791")
      (WordInstanceNode "in@3b8caa2e-d340-4061-9d4c-efe49ee7b7f6")
   )
)
(EvaluationLink
   (LinkGrammarRelationshipNode "Js")
   (ListLink
      (WordInstanceNode "in@3b8caa2e-d340-4061-9d4c-efe49ee7b7f6")
      (WordInstanceNode "Germany@497f22b3-53df-4bea-8789-ae1c3eda38b1")
   )
)
(EvaluationLink
   (LinkGrammarRelationshipNode "Xp")
   (ListLink
      (WordInstanceNode "LEFT-WALL")
      (WordInstanceNode ".@86bad39d-9070-40f3-a1af-bd270aeb0cac")
   )
)
(EvaluationLink
   (LinkGrammarRelationshipNode "Ost")
   (ListLink
      (WordInstanceNode "is@eabce068-389e-4df0-a799-cb0176836791")
      (WordInstanceNode "city@f73c69fc-3683-4a22-8bf9-e705a5aa5191")
   )
)
(EvaluationLink
   (LinkGrammarRelationshipNode "Ds")
   (ListLink
      (WordInstanceNode "a@fa79be35-996a-48b6-ad16-2d93237dcdf1")
      (WordInstanceNode "city@f73c69fc-3683-4a22-8bf9-e705a5aa5191")
   )
)
(EvaluationLink
   (LinkGrammarRelationshipNode "Ss")
   (ListLink
      (WordInstanceNode "Berlin@b7da0d0d-75e3-4bbc-ac58-e64a7377c8e1")
      (WordInstanceNode "is@eabce068-389e-4df0-a799-cb0176836791")
   )
)
(EvaluationLink
   (LinkGrammarRelationshipNode "Wd")
   (ListLink
      (WordInstanceNode "LEFT-WALL")
      (WordInstanceNode "Berlin@b7da0d0d-75e3-4bbc-ac58-e64a7377c8e1")
   )
)
(ParseLink
   (ParseNode "sentence@327f8945-7634-4d27-b192-ce477fec4370_parse_0" (stv 1.0 0.8976))
   (SentenceNode "sentence@327f8945-7634-4d27-b192-ce477fec4370")
)
(LemmaLink
   (WordInstanceNode "Berlin@b7da0d0d-75e3-4bbc-ac58-e64a7377c8e1")
   (WordNode "Berlin")
)
(LemmaLink
   (WordInstanceNode "is@eabce068-389e-4df0-a799-cb0176836791")
   (WordNode "be")
)
(LemmaLink
   (WordInstanceNode "a@fa79be35-996a-48b6-ad16-2d93237dcdf1")
   (WordNode "a")
)
(LemmaLink
   (WordInstanceNode "city@f73c69fc-3683-4a22-8bf9-e705a5aa5191")
   (WordNode "city")
)
(LemmaLink
   (WordInstanceNode "in@3b8caa2e-d340-4061-9d4c-efe49ee7b7f6")
   (WordNode "in")
)
(LemmaLink
   (WordInstanceNode "Germany@497f22b3-53df-4bea-8789-ae1c3eda38b1")
   (WordNode "Germany")
)
(LemmaLink
   (WordInstanceNode ".@86bad39d-9070-40f3-a1af-bd270aeb0cac")
   (WordNode ".")
)
; pos (in, prep)
(PartOfSpeechLink
   (WordInstanceNode "in@3b8caa2e-d340-4061-9d4c-efe49ee7b7f6")
   (DefinedLinguisticConceptNode "prep")
)
; DEFINITE-FLAG (Germany, T)
(InheritanceLink
   (WordInstanceNode "Germany@497f22b3-53df-4bea-8789-ae1c3eda38b1")
   (DefinedLinguisticConceptNode "definite")
)
; noun_number (Germany, singular)
(InheritanceLink
   (WordInstanceNode "Germany@497f22b3-53df-4bea-8789-ae1c3eda38b1")
   (DefinedLinguisticConceptNode "singular")
)
; inflection-TAG (Germany, .l)
(InheritanceLink
   (WordInstanceNode "Germany@497f22b3-53df-4bea-8789-ae1c3eda38b1")
   (DefinedLinguisticConceptNode ".l")
)
; location-FLAG (Germany, T)
(InheritanceLink
   (WordInstanceNode "Germany@497f22b3-53df-4bea-8789-ae1c3eda38b1")
   (DefinedLinguisticConceptNode "location")
)
; pos (Germany, noun)
(PartOfSpeechLink
   (WordInstanceNode "Germany@497f22b3-53df-4bea-8789-ae1c3eda38b1")
   (DefinedLinguisticConceptNode "noun")
)
; pos (., punctuation)
(PartOfSpeechLink
   (WordInstanceNode ".@86bad39d-9070-40f3-a1af-bd270aeb0cac")
   (DefinedLinguisticConceptNode "punctuation")
)
; DEFINITE-FLAG (Berlin, T)
(InheritanceLink
   (WordInstanceNode "Berlin@b7da0d0d-75e3-4bbc-ac58-e64a7377c8e1")
   (DefinedLinguisticConceptNode "definite")
)
; noun_number (Berlin, singular)
(InheritanceLink
   (WordInstanceNode "Berlin@b7da0d0d-75e3-4bbc-ac58-e64a7377c8e1")
   (DefinedLinguisticConceptNode "singular")
)
; pos (Berlin, noun)
(PartOfSpeechLink
   (WordInstanceNode "Berlin@b7da0d0d-75e3-4bbc-ac58-e64a7377c8e1")
   (DefinedLinguisticConceptNode "noun")
)
; noun_number (city, singular)
(InheritanceLink
   (WordInstanceNode "city@f73c69fc-3683-4a22-8bf9-e705a5aa5191")
   (DefinedLinguisticConceptNode "singular")
)
; inflection-TAG (city, .n)
(InheritanceLink
   (WordInstanceNode "city@f73c69fc-3683-4a22-8bf9-e705a5aa5191")
   (DefinedLinguisticConceptNode ".n")
)
; pos (city, noun)
(PartOfSpeechLink
   (WordInstanceNode "city@f73c69fc-3683-4a22-8bf9-e705a5aa5191")
   (DefinedLinguisticConceptNode "noun")
)
; in (<<be>>, <<Germany>>) 
(EvaluationLink
   (DefinedLinguisticRelationshipNode "in")
   (ListLink
      (WordInstanceNode "is@eabce068-389e-4df0-a799-cb0176836791")
      (WordInstanceNode "Germany@497f22b3-53df-4bea-8789-ae1c3eda38b1")
   )
)
; _subj (<<be>>, <<Berlin>>) 
(EvaluationLink
   (DefinedLinguisticRelationshipNode "_subj")
   (ListLink
      (WordInstanceNode "is@eabce068-389e-4df0-a799-cb0176836791")
      (WordInstanceNode "Berlin@b7da0d0d-75e3-4bbc-ac58-e64a7377c8e1")
   )
)
; _obj (<<be>>, <<city>>) 
(EvaluationLink
   (DefinedLinguisticRelationshipNode "_obj")
   (ListLink
      (WordInstanceNode "is@eabce068-389e-4df0-a799-cb0176836791")
      (WordInstanceNode "city@f73c69fc-3683-4a22-8bf9-e705a5aa5191")
   )
)
; tense (be, present)
(InheritanceLink
   (WordInstanceNode "is@eabce068-389e-4df0-a799-cb0176836791")
   (DefinedLinguisticConceptNode "present")
)
; inflection-TAG (be, .v)
(InheritanceLink
   (WordInstanceNode "is@eabce068-389e-4df0-a799-cb0176836791")
   (DefinedLinguisticConceptNode ".v")
)
; pos (be, verb)
(PartOfSpeechLink
   (WordInstanceNode "is@eabce068-389e-4df0-a799-cb0176836791")
   (DefinedLinguisticConceptNode "verb")
)
; pos (a, det)
(PartOfSpeechLink
   (WordInstanceNode "a@fa79be35-996a-48b6-ad16-2d93237dcdf1")
   (DefinedLinguisticConceptNode "det")
)

(ReferenceLink
   (WordInstanceNode "Berlin@a8914040-7dde-4c25-a1f8-946aaae11ec6")
   (WordNode "Berlin")
)
(WordInstanceLink
   (WordInstanceNode "Berlin@a8914040-7dde-4c25-a1f8-946aaae11ec6")
   (ParseNode "sentence@327f8945-7634-4d27-b192-ce477fec4370_parse_1")
)
(ReferenceLink
   (WordInstanceNode "is@68c38266-efd6-4e92-96c5-afe34dfbb1a2")
   (WordNode "is")
)
(WordInstanceLink
   (WordInstanceNode "is@68c38266-efd6-4e92-96c5-afe34dfbb1a2")
   (ParseNode "sentence@327f8945-7634-4d27-b192-ce477fec4370_parse_1")
)
(ReferenceLink
   (WordInstanceNode "a@946ecdde-2752-44e6-87b6-1eff51b10ebc")
   (WordNode "a")
)
(WordInstanceLink
   (WordInstanceNode "a@946ecdde-2752-44e6-87b6-1eff51b10ebc")
   (ParseNode "sentence@327f8945-7634-4d27-b192-ce477fec4370_parse_1")
)
(ReferenceLink
   (WordInstanceNode "city@c53f2836-9efd-4ddc-b34b-9dc87ac092bc")
   (WordNode "city")
)
(WordInstanceLink
   (WordInstanceNode "city@c53f2836-9efd-4ddc-b34b-9dc87ac092bc")
   (ParseNode "sentence@327f8945-7634-4d27-b192-ce477fec4370_parse_1")
)
(ReferenceLink
   (WordInstanceNode "in@31af58c9-e3bf-43f4-b965-9c469c68ab0e")
   (WordNode "in")
)
(WordInstanceLink
   (WordInstanceNode "in@31af58c9-e3bf-43f4-b965-9c469c68ab0e")
   (ParseNode "sentence@327f8945-7634-4d27-b192-ce477fec4370_parse_1")
)
(ReferenceLink
   (WordInstanceNode "Germany@580954fc-8be1-4e62-965a-d8271d8698b2")
   (WordNode "Germany")
)
(WordInstanceLink
   (WordInstanceNode "Germany@580954fc-8be1-4e62-965a-d8271d8698b2")
   (ParseNode "sentence@327f8945-7634-4d27-b192-ce477fec4370_parse_1")
)
(ReferenceLink
   (WordInstanceNode ".@b94aacc3-857f-4d49-b3e0-6c98a16f0c50")
   (WordNode ".")
)
(WordInstanceLink
   (WordInstanceNode ".@b94aacc3-857f-4d49-b3e0-6c98a16f0c50")
   (ParseNode "sentence@327f8945-7634-4d27-b192-ce477fec4370_parse_1")
)
(ReferenceLink
   (ParseNode "sentence@327f8945-7634-4d27-b192-ce477fec4370_parse_1")
   (ListLink
      (WordInstanceNode "Berlin@a8914040-7dde-4c25-a1f8-946aaae11ec6")
      (WordInstanceNode "is@68c38266-efd6-4e92-96c5-afe34dfbb1a2")
      (WordInstanceNode "a@946ecdde-2752-44e6-87b6-1eff51b10ebc")
      (WordInstanceNode "city@c53f2836-9efd-4ddc-b34b-9dc87ac092bc")
      (WordInstanceNode "in@31af58c9-e3bf-43f4-b965-9c469c68ab0e")
      (WordInstanceNode "Germany@580954fc-8be1-4e62-965a-d8271d8698b2")
      (WordInstanceNode ".@b94aacc3-857f-4d49-b3e0-6c98a16f0c50")
   )
)
(EvaluationLink
   (LinkGrammarRelationshipNode "Ost")
   (ListLink
      (WordInstanceNode "is@68c38266-efd6-4e92-96c5-afe34dfbb1a2")
      (WordInstanceNode "city@c53f2836-9efd-4ddc-b34b-9dc87ac092bc")
   )
)
(EvaluationLink
   (LinkGrammarRelationshipNode "Mp")
   (ListLink
      (WordInstanceNode "city@c53f2836-9efd-4ddc-b34b-9dc87ac092bc")
      (WordInstanceNode "in@31af58c9-e3bf-43f4-b965-9c469c68ab0e")
   )
)
(EvaluationLink
   (LinkGrammarRelationshipNode "Js")
   (ListLink
      (WordInstanceNode "in@31af58c9-e3bf-43f4-b965-9c469c68ab0e")
      (WordInstanceNode "Germany@580954fc-8be1-4e62-965a-d8271d8698b2")
   )
)
(EvaluationLink
   (LinkGrammarRelationshipNode "Xp")
   (ListLink
      (WordInstanceNode "LEFT-WALL")
      (WordInstanceNode ".@b94aacc3-857f-4d49-b3e0-6c98a16f0c50")
   )
)
(EvaluationLink
   (LinkGrammarRelationshipNode "Ds")
   (ListLink
      (WordInstanceNode "a@946ecdde-2752-44e6-87b6-1eff51b10ebc")
      (WordInstanceNode "city@c53f2836-9efd-4ddc-b34b-9dc87ac092bc")
   )
)
(EvaluationLink
   (LinkGrammarRelationshipNode "Ss")
   (ListLink
      (WordInstanceNode "Berlin@a8914040-7dde-4c25-a1f8-946aaae11ec6")
      (WordInstanceNode "is@68c38266-efd6-4e92-96c5-afe34dfbb1a2")
   )
)
(EvaluationLink
   (LinkGrammarRelationshipNode "Wd")
   (ListLink
      (WordInstanceNode "LEFT-WALL")
      (WordInstanceNode "Berlin@a8914040-7dde-4c25-a1f8-946aaae11ec6")
   )
)
(ParseLink
   (ParseNode "sentence@327f8945-7634-4d27-b192-ce477fec4370_parse_1" (stv 1.0 0.7527))
   (SentenceNode "sentence@327f8945-7634-4d27-b192-ce477fec4370")
)
(LemmaLink
   (WordInstanceNode "Berlin@a8914040-7dde-4c25-a1f8-946aaae11ec6")
   (WordNode "Berlin")
)
(LemmaLink
   (WordInstanceNode "is@68c38266-efd6-4e92-96c5-afe34dfbb1a2")
   (WordNode "be")
)
(LemmaLink
   (WordInstanceNode "a@946ecdde-2752-44e6-87b6-1eff51b10ebc")
   (WordNode "a")
)
(LemmaLink
   (WordInstanceNode "city@c53f2836-9efd-4ddc-b34b-9dc87ac092bc")
   (WordNode "city")
)
(LemmaLink
   (WordInstanceNode "in@31af58c9-e3bf-43f4-b965-9c469c68ab0e")
   (WordNode "in")
)
(LemmaLink
   (WordInstanceNode "Germany@580954fc-8be1-4e62-965a-d8271d8698b2")
   (WordNode "Germany")
)
(LemmaLink
   (WordInstanceNode ".@b94aacc3-857f-4d49-b3e0-6c98a16f0c50")
   (WordNode ".")
)
; DEFINITE-FLAG (Berlin, T)
(InheritanceLink
   (WordInstanceNode "Berlin@a8914040-7dde-4c25-a1f8-946aaae11ec6")
   (DefinedLinguisticConceptNode "definite")
)
; noun_number (Berlin, singular)
(InheritanceLink
   (WordInstanceNode "Berlin@a8914040-7dde-4c25-a1f8-946aaae11ec6")
   (DefinedLinguisticConceptNode "singular")
)
; pos (Berlin, noun)
(PartOfSpeechLink
   (WordInstanceNode "Berlin@a8914040-7dde-4c25-a1f8-946aaae11ec6")
   (DefinedLinguisticConceptNode "noun")
)
; pos (in, prep)
(PartOfSpeechLink
   (WordInstanceNode "in@31af58c9-e3bf-43f4-b965-9c469c68ab0e")
   (DefinedLinguisticConceptNode "prep")
)
; DEFINITE-FLAG (Germany, T)
(InheritanceLink
   (WordInstanceNode "Germany@580954fc-8be1-4e62-965a-d8271d8698b2")
   (DefinedLinguisticConceptNode "definite")
)
; noun_number (Germany, singular)
(InheritanceLink
   (WordInstanceNode "Germany@580954fc-8be1-4e62-965a-d8271d8698b2")
   (DefinedLinguisticConceptNode "singular")
)
; inflection-TAG (Germany, .l)
(InheritanceLink
   (WordInstanceNode "Germany@580954fc-8be1-4e62-965a-d8271d8698b2")
   (DefinedLinguisticConceptNode ".l")
)
; location-FLAG (Germany, T)
(InheritanceLink
   (WordInstanceNode "Germany@580954fc-8be1-4e62-965a-d8271d8698b2")
   (DefinedLinguisticConceptNode "location")
)
; pos (Germany, noun)
(PartOfSpeechLink
   (WordInstanceNode "Germany@580954fc-8be1-4e62-965a-d8271d8698b2")
   (DefinedLinguisticConceptNode "noun")
)
; pos (., punctuation)
(PartOfSpeechLink
   (WordInstanceNode ".@b94aacc3-857f-4d49-b3e0-6c98a16f0c50")
   (DefinedLinguisticConceptNode "punctuation")
)
; in (<<city>>, <<Germany>>) 
(EvaluationLink
   (DefinedLinguisticRelationshipNode "in")
   (ListLink
      (WordInstanceNode "city@c53f2836-9efd-4ddc-b34b-9dc87ac092bc")
      (WordInstanceNode "Germany@580954fc-8be1-4e62-965a-d8271d8698b2")
   )
)
; noun_number (city, singular)
(InheritanceLink
   (WordInstanceNode "city@c53f2836-9efd-4ddc-b34b-9dc87ac092bc")
   (DefinedLinguisticConceptNode "singular")
)
; inflection-TAG (city, .n)
(InheritanceLink
   (WordInstanceNode "city@c53f2836-9efd-4ddc-b34b-9dc87ac092bc")
   (DefinedLinguisticConceptNode ".n")
)
; pos (city, noun)
(PartOfSpeechLink
   (WordInstanceNode "city@c53f2836-9efd-4ddc-b34b-9dc87ac092bc")
   (DefinedLinguisticConceptNode "noun")
)
; _subj (<<be>>, <<Berlin>>) 
(EvaluationLink
   (DefinedLinguisticRelationshipNode "_subj")
   (ListLink
      (WordInstanceNode "is@68c38266-efd6-4e92-96c5-afe34dfbb1a2")
      (WordInstanceNode "Berlin@a8914040-7dde-4c25-a1f8-946aaae11ec6")
   )
)
; _obj (<<be>>, <<city>>) 
(EvaluationLink
   (DefinedLinguisticRelationshipNode "_obj")
   (ListLink
      (WordInstanceNode "is@68c38266-efd6-4e92-96c5-afe34dfbb1a2")
      (WordInstanceNode "city@c53f2836-9efd-4ddc-b34b-9dc87ac092bc")
   )
)
; tense (be, present)
(InheritanceLink
   (WordInstanceNode "is@68c38266-efd6-4e92-96c5-afe34dfbb1a2")
   (DefinedLinguisticConceptNode "present")
)
; inflection-TAG (be, .v)
(InheritanceLink
   (WordInstanceNode "is@68c38266-efd6-4e92-96c5-afe34dfbb1a2")
   (DefinedLinguisticConceptNode ".v")
)
; pos (be, verb)
(PartOfSpeechLink
   (WordInstanceNode "is@68c38266-efd6-4e92-96c5-afe34dfbb1a2")
   (DefinedLinguisticConceptNode "verb")
)
; pos (a, det)
(PartOfSpeechLink
   (WordInstanceNode "a@946ecdde-2752-44e6-87b6-1eff51b10ebc")
   (DefinedLinguisticConceptNode "det")
)

; SENTENCE: [The heart is in the chest.]
(ReferenceLink
   (WordInstanceNode "the@d088a092-8699-4d94-be08-daac4bbcefa6")
   (WordNode "the")
)
(WordInstanceLink
   (WordInstanceNode "the@d088a092-8699-4d94-be08-daac4bbcefa6")
   (ParseNode "sentence@0944bb6b-6cb9-4528-be77-7c76bfabef8f_parse_0")
)
(ReferenceLink
   (WordInstanceNode "heart@672b4579-75f2-4727-a8e8-54d229bd9d32")
   (WordNode "heart")
)
(WordInstanceLink
   (WordInstanceNode "heart@672b4579-75f2-4727-a8e8-54d229bd9d32")
   (ParseNode "sentence@0944bb6b-6cb9-4528-be77-7c76bfabef8f_parse_0")
)
(ReferenceLink
   (WordInstanceNode "is@dd6df27e-0f30-4cb8-afef-841c17ad354a")
   (WordNode "is")
)
(WordInstanceLink
   (WordInstanceNode "is@dd6df27e-0f30-4cb8-afef-841c17ad354a")
   (ParseNode "sentence@0944bb6b-6cb9-4528-be77-7c76bfabef8f_parse_0")
)
(ReferenceLink
   (WordInstanceNode "in@0d1c38e2-9605-43cc-a2fa-48748a4167d0")
   (WordNode "in")
)
(WordInstanceLink
   (WordInstanceNode "in@0d1c38e2-9605-43cc-a2fa-48748a4167d0")
   (ParseNode "sentence@0944bb6b-6cb9-4528-be77-7c76bfabef8f_parse_0")
)
(ReferenceLink
   (WordInstanceNode "the@e43834d2-9fc6-404b-b50d-51e8b0eeaf0a")
   (WordNode "the")
)
(WordInstanceLink
   (WordInstanceNode "the@e43834d2-9fc6-404b-b50d-51e8b0eeaf0a")
   (ParseNode "sentence@0944bb6b-6cb9-4528-be77-7c76bfabef8f_parse_0")
)
(ReferenceLink
   (WordInstanceNode "chest@7b9a6ba6-35db-4f76-a911-5c43ac660c70")
   (WordNode "chest")
)
(WordInstanceLink
   (WordInstanceNode "chest@7b9a6ba6-35db-4f76-a911-5c43ac660c70")
   (ParseNode "sentence@0944bb6b-6cb9-4528-be77-7c76bfabef8f_parse_0")
)
(ReferenceLink
   (WordInstanceNode ".@babaca63-40ae-4e18-8194-00f7d6713ec7")
   (WordNode ".")
)
(WordInstanceLink
   (WordInstanceNode ".@babaca63-40ae-4e18-8194-00f7d6713ec7")
   (ParseNode "sentence@0944bb6b-6cb9-4528-be77-7c76bfabef8f_parse_0")
)
(ReferenceLink
   (ParseNode "sentence@0944bb6b-6cb9-4528-be77-7c76bfabef8f_parse_0")
   (ListLink
      (WordInstanceNode "the@d088a092-8699-4d94-be08-daac4bbcefa6")
      (WordInstanceNode "heart@672b4579-75f2-4727-a8e8-54d229bd9d32")
      (WordInstanceNode "is@dd6df27e-0f30-4cb8-afef-841c17ad354a")
      (WordInstanceNode "in@0d1c38e2-9605-43cc-a2fa-48748a4167d0")
      (WordInstanceNode "the@e43834d2-9fc6-404b-b50d-51e8b0eeaf0a")
      (WordInstanceNode "chest@7b9a6ba6-35db-4f76-a911-5c43ac660c70")
      (WordInstanceNode ".@babaca63-40ae-4e18-8194-00f7d6713ec7")
   )
)
(EvaluationLink
   (LinkGrammarRelationshipNode "Js")
   (ListLink
      (WordInstanceNode "in@0d1c38e2-9605-43cc-a2fa-48748a4167d0")
      (WordInstanceNode "chest@7b9a6ba6-35db-4f76-a911-5c43ac660c70")
   )
)
(EvaluationLink
   (LinkGrammarRelationshipNode "Ds")
   (ListLink
      (WordInstanceNode "the@e43834d2-9fc6-404b-b50d-51e8b0eeaf0a")
      (WordInstanceNode "chest@7b9a6ba6-35db-4f76-a911-5c43ac660c70")
   )
)
(EvaluationLink
   (LinkGrammarRelationshipNode "Xp")
   (ListLink
      (WordInstanceNode "LEFT-WALL")
      (WordInstanceNode ".@babaca63-40ae-4e18-8194-00f7d6713ec7")
   )
)
(EvaluationLink
   (LinkGrammarRelationshipNode "Pp")
   (ListLink
      (WordInstanceNode "is@dd6df27e-0f30-4cb8-afef-841c17ad354a")
      (WordInstanceNode "in@0d1c38e2-9605-43cc-a2fa-48748a4167d0")
   )
)
(EvaluationLink
   (LinkGrammarRelationshipNode "Ss")
   (ListLink
      (WordInstanceNode "heart@672b4579-75f2-4727-a8e8-54d229bd9d32")
      (WordInstanceNode "is@dd6df27e-0f30-4cb8-afef-841c17ad354a")
   )
)
(EvaluationLink
   (LinkGrammarRelationshipNode "Wd")
   (ListLink
      (WordInstanceNode "LEFT-WALL")
      (WordInstanceNode "heart@672b4579-75f2-4727-a8e8-54d229bd9d32")
   )
)
(EvaluationLink
   (LinkGrammarRelationshipNode "D*u")
   (ListLink
      (WordInstanceNode "the@d088a092-8699-4d94-be08-daac4bbcefa6")
      (WordInstanceNode "heart@672b4579-75f2-4727-a8e8-54d229bd9d32")
   )
)
(ParseLink
   (ParseNode "sentence@0944bb6b-6cb9-4528-be77-7c76bfabef8f_parse_0" (stv 1.0 0.9084))
   (SentenceNode "sentence@0944bb6b-6cb9-4528-be77-7c76bfabef8f")
)
(LemmaLink
   (WordInstanceNode "the@d088a092-8699-4d94-be08-daac4bbcefa6")
   (WordNode "the")
)
(LemmaLink
   (WordInstanceNode "heart@672b4579-75f2-4727-a8e8-54d229bd9d32")
   (WordNode "heart")
)
(LemmaLink
   (WordInstanceNode "is@dd6df27e-0f30-4cb8-afef-841c17ad354a")
   (WordNode "be")
)
(LemmaLink
   (WordInstanceNode "in@0d1c38e2-9605-43cc-a2fa-48748a4167d0")
   (WordNode "in")
)
(LemmaLink
   (WordInstanceNode "the@e43834d2-9fc6-404b-b50d-51e8b0eeaf0a")
   (WordNode "the")
)
(LemmaLink
   (WordInstanceNode "chest@7b9a6ba6-35db-4f76-a911-5c43ac660c70")
   (WordNode "chest")
)
(LemmaLink
   (WordInstanceNode ".@babaca63-40ae-4e18-8194-00f7d6713ec7")
   (WordNode ".")
)
; DEFINITE-FLAG (chest, T)
(InheritanceLink
   (WordInstanceNode "chest@7b9a6ba6-35db-4f76-a911-5c43ac660c70")
   (DefinedLinguisticConceptNode "definite")
)
; noun_number (chest, singular)
(InheritanceLink
   (WordInstanceNode "chest@7b9a6ba6-35db-4f76-a911-5c43ac660c70")
   (DefinedLinguisticConceptNode "singular")
)
; inflection-TAG (chest, .n)
(InheritanceLink
   (WordInstanceNode "chest@7b9a6ba6-35db-4f76-a911-5c43ac660c70")
   (DefinedLinguisticConceptNode ".n")
)
; pos (chest, noun)
(PartOfSpeechLink
   (WordInstanceNode "chest@7b9a6ba6-35db-4f76-a911-5c43ac660c70")
   (DefinedLinguisticConceptNode "noun")
)
; pos (., punctuation)
(PartOfSpeechLink
   (WordInstanceNode ".@babaca63-40ae-4e18-8194-00f7d6713ec7")
   (DefinedLinguisticConceptNode "punctuation")
)
; DEFINITE-FLAG (heart, T)
(InheritanceLink
   (WordInstanceNode "heart@672b4579-75f2-4727-a8e8-54d229bd9d32")
   (DefinedLinguisticConceptNode "definite")
)
; noun_number (heart, singular)
(InheritanceLink
   (WordInstanceNode "heart@672b4579-75f2-4727-a8e8-54d229bd9d32")
   (DefinedLinguisticConceptNode "singular")
)
; inflection-TAG (heart, .n)
(InheritanceLink
   (WordInstanceNode "heart@672b4579-75f2-4727-a8e8-54d229bd9d32")
   (DefinedLinguisticConceptNode ".n")
)
; pos (heart, noun)
(PartOfSpeechLink
   (WordInstanceNode "heart@672b4579-75f2-4727-a8e8-54d229bd9d32")
   (DefinedLinguisticConceptNode "noun")
)
; in (<<_%copula>>, <<in>>) 
(EvaluationLink
   (DefinedLinguisticRelationshipNode "in")
   (ListLink
      (WordInstanceNode "is@dd6df27e-0f30-4cb8-afef-841c17ad354a")
      (WordInstanceNode "in@0d1c38e2-9605-43cc-a2fa-48748a4167d0")
   )
)
; inflection-TAG (_%copula, .v)
(InheritanceLink
   (WordInstanceNode "is@dd6df27e-0f30-4cb8-afef-841c17ad354a")
   (DefinedLinguisticConceptNode ".v")
)
; pos (_%copula, verb)
(PartOfSpeechLink
   (WordInstanceNode "is@dd6df27e-0f30-4cb8-afef-841c17ad354a")
   (DefinedLinguisticConceptNode "verb")
)
; _pobj (<<in>>, <<chest>>) 
(EvaluationLink
   (DefinedLinguisticRelationshipNode "_pobj")
   (ListLink
      (WordInstanceNode "in@0d1c38e2-9605-43cc-a2fa-48748a4167d0")
      (WordInstanceNode "chest@7b9a6ba6-35db-4f76-a911-5c43ac660c70")
   )
)
; _psubj (<<in>>, <<heart>>) 
(EvaluationLink
   (DefinedLinguisticRelationshipNode "_psubj")
   (ListLink
      (WordInstanceNode "in@0d1c38e2-9605-43cc-a2fa-48748a4167d0")
      (WordInstanceNode "heart@672b4579-75f2-4727-a8e8-54d229bd9d32")
   )
)
; tense (in, present)
(InheritanceLink
   (WordInstanceNode "in@0d1c38e2-9605-43cc-a2fa-48748a4167d0")
   (DefinedLinguisticConceptNode "present")
)
; SPECIAL-PREP-FLAG (in, T)
(InheritanceLink
   (WordInstanceNode "in@0d1c38e2-9605-43cc-a2fa-48748a4167d0")
   (DefinedLinguisticConceptNode "special-prep")
)
; pos (in, verb)
(PartOfSpeechLink
   (WordInstanceNode "in@0d1c38e2-9605-43cc-a2fa-48748a4167d0")
   (DefinedLinguisticConceptNode "verb")
)
; pos (the, det)
(PartOfSpeechLink
   (WordInstanceNode "the@e43834d2-9fc6-404b-b50d-51e8b0eeaf0a")
   (DefinedLinguisticConceptNode "det")
)
; pos (the, det)
(PartOfSpeechLink
   (WordInstanceNode "the@d088a092-8699-4d94-be08-daac4bbcefa6")
   (DefinedLinguisticConceptNode "det")
)

(ReferenceLink
   (WordInstanceNode "the@e92c8886-cbd8-4dfb-89a9-4342d713c56e")
   (WordNode "the")
)
(WordInstanceLink
   (WordInstanceNode "the@e92c8886-cbd8-4dfb-89a9-4342d713c56e")
   (ParseNode "sentence@0944bb6b-6cb9-4528-be77-7c76bfabef8f_parse_1")
)
(ReferenceLink
   (WordInstanceNode "heart@66aef304-1b5b-4e9d-b117-3a90ea6844f7")
   (WordNode "heart")
)
(WordInstanceLink
   (WordInstanceNode "heart@66aef304-1b5b-4e9d-b117-3a90ea6844f7")
   (ParseNode "sentence@0944bb6b-6cb9-4528-be77-7c76bfabef8f_parse_1")
)
(ReferenceLink
   (WordInstanceNode "is@e051638b-86cc-4d24-a4a7-7f1dc21720f8")
   (WordNode "is")
)
(WordInstanceLink
   (WordInstanceNode "is@e051638b-86cc-4d24-a4a7-7f1dc21720f8")
   (ParseNode "sentence@0944bb6b-6cb9-4528-be77-7c76bfabef8f_parse_1")
)
(ReferenceLink
   (WordInstanceNode "in@4c48e80d-8d79-4f8e-8b10-3cdf7dbc5f23")
   (WordNode "in")
)
(WordInstanceLink
   (WordInstanceNode "in@4c48e80d-8d79-4f8e-8b10-3cdf7dbc5f23")
   (ParseNode "sentence@0944bb6b-6cb9-4528-be77-7c76bfabef8f_parse_1")
)
(ReferenceLink
   (WordInstanceNode "the@12fa5e7f-f55c-4f2c-89a1-10a24c602794")
   (WordNode "the")
)
(WordInstanceLink
   (WordInstanceNode "the@12fa5e7f-f55c-4f2c-89a1-10a24c602794")
   (ParseNode "sentence@0944bb6b-6cb9-4528-be77-7c76bfabef8f_parse_1")
)
(ReferenceLink
   (WordInstanceNode "chest@6aa408df-18f7-4650-b7fe-1372f786766a")
   (WordNode "chest")
)
(WordInstanceLink
   (WordInstanceNode "chest@6aa408df-18f7-4650-b7fe-1372f786766a")
   (ParseNode "sentence@0944bb6b-6cb9-4528-be77-7c76bfabef8f_parse_1")
)
(ReferenceLink
   (WordInstanceNode ".@a83ddd72-d5a7-4545-a9f5-b842438fd298")
   (WordNode ".")
)
(WordInstanceLink
   (WordInstanceNode ".@a83ddd72-d5a7-4545-a9f5-b842438fd298")
   (ParseNode "sentence@0944bb6b-6cb9-4528-be77-7c76bfabef8f_parse_1")
)
(ReferenceLink
   (ParseNode "sentence@0944bb6b-6cb9-4528-be77-7c76bfabef8f_parse_1")
   (ListLink
      (WordInstanceNode "the@e92c8886-cbd8-4dfb-89a9-4342d713c56e")
      (WordInstanceNode "heart@66aef304-1b5b-4e9d-b117-3a90ea6844f7")
      (WordInstanceNode "is@e051638b-86cc-4d24-a4a7-7f1dc21720f8")
      (WordInstanceNode "in@4c48e80d-8d79-4f8e-8b10-3cdf7dbc5f23")
      (WordInstanceNode "the@12fa5e7f-f55c-4f2c-89a1-10a24c602794")
      (WordInstanceNode "chest@6aa408df-18f7-4650-b7fe-1372f786766a")
      (WordInstanceNode ".@a83ddd72-d5a7-4545-a9f5-b842438fd298")
   )
)
(EvaluationLink
   (LinkGrammarRelationshipNode "MVp")
   (ListLink
      (WordInstanceNode "is@e051638b-86cc-4d24-a4a7-7f1dc21720f8")
      (WordInstanceNode "in@4c48e80d-8d79-4f8e-8b10-3cdf7dbc5f23")
   )
)
(EvaluationLink
   (LinkGrammarRelationshipNode "Js")
   (ListLink
      (WordInstanceNode "in@4c48e80d-8d79-4f8e-8b10-3cdf7dbc5f23")
      (WordInstanceNode "chest@6aa408df-18f7-4650-b7fe-1372f786766a")
   )
)
(EvaluationLink
   (LinkGrammarRelationshipNode "Ds")
   (ListLink
      (WordInstanceNode "the@12fa5e7f-f55c-4f2c-89a1-10a24c602794")
      (WordInstanceNode "chest@6aa408df-18f7-4650-b7fe-1372f786766a")
   )
)
(EvaluationLink
   (LinkGrammarRelationshipNode "Xp")
   (ListLink
      (WordInstanceNode "LEFT-WALL")
      (WordInstanceNode ".@a83ddd72-d5a7-4545-a9f5-b842438fd298")
   )
)
(EvaluationLink
   (LinkGrammarRelationshipNode "Ss")
   (ListLink
      (WordInstanceNode "heart@66aef304-1b5b-4e9d-b117-3a90ea6844f7")
      (WordInstanceNode "is@e051638b-86cc-4d24-a4a7-7f1dc21720f8")
   )
)
(EvaluationLink
   (LinkGrammarRelationshipNode "Wd")
   (ListLink
      (WordInstanceNode "LEFT-WALL")
      (WordInstanceNode "heart@66aef304-1b5b-4e9d-b117-3a90ea6844f7")
   )
)
(EvaluationLink
   (LinkGrammarRelationshipNode "D*u")
   (ListLink
      (WordInstanceNode "the@e92c8886-cbd8-4dfb-89a9-4342d713c56e")
      (WordInstanceNode "heart@66aef304-1b5b-4e9d-b117-3a90ea6844f7")
   )
)
(ParseLink
   (ParseNode "sentence@0944bb6b-6cb9-4528-be77-7c76bfabef8f_parse_1" (stv 1.0 0.6089))
   (SentenceNode "sentence@0944bb6b-6cb9-4528-be77-7c76bfabef8f")
)
(LemmaLink
   (WordInstanceNode "the@e92c8886-cbd8-4dfb-89a9-4342d713c56e")
   (WordNode "the")
)
(LemmaLink
   (WordInstanceNode "heart@66aef304-1b5b-4e9d-b117-3a90ea6844f7")
   (WordNode "heart")
)
(LemmaLink
   (WordInstanceNode "is@e051638b-86cc-4d24-a4a7-7f1dc21720f8")
   (WordNode "be")
)
(LemmaLink
   (WordInstanceNode "in@4c48e80d-8d79-4f8e-8b10-3cdf7dbc5f23")
   (WordNode "in")
)
(LemmaLink
   (WordInstanceNode "the@12fa5e7f-f55c-4f2c-89a1-10a24c602794")
   (WordNode "the")
)
(LemmaLink
   (WordInstanceNode "chest@6aa408df-18f7-4650-b7fe-1372f786766a")
   (WordNode "chest")
)
(LemmaLink
   (WordInstanceNode ".@a83ddd72-d5a7-4545-a9f5-b842438fd298")
   (WordNode ".")
)
; pos (in, prep)
(PartOfSpeechLink
   (WordInstanceNode "in@4c48e80d-8d79-4f8e-8b10-3cdf7dbc5f23")
   (DefinedLinguisticConceptNode "prep")
)
; DEFINITE-FLAG (chest, T)
(InheritanceLink
   (WordInstanceNode "chest@6aa408df-18f7-4650-b7fe-1372f786766a")
   (DefinedLinguisticConceptNode "definite")
)
; noun_number (chest, singular)
(InheritanceLink
   (WordInstanceNode "chest@6aa408df-18f7-4650-b7fe-1372f786766a")
   (DefinedLinguisticConceptNode "singular")
)
; inflection-TAG (chest, .n)
(InheritanceLink
   (WordInstanceNode "chest@6aa408df-18f7-4650-b7fe-1372f786766a")
   (DefinedLinguisticConceptNode ".n")
)
; pos (chest, noun)
(PartOfSpeechLink
   (WordInstanceNode "chest@6aa408df-18f7-4650-b7fe-1372f786766a")
   (DefinedLinguisticConceptNode "noun")
)
; pos (., punctuation)
(PartOfSpeechLink
   (WordInstanceNode ".@a83ddd72-d5a7-4545-a9f5-b842438fd298")
   (DefinedLinguisticConceptNode "punctuation")
)
; pos (the, det)
(PartOfSpeechLink
   (WordInstanceNode "the@12fa5e7f-f55c-4f2c-89a1-10a24c602794")
   (DefinedLinguisticConceptNode "det")
)
; DEFINITE-FLAG (heart, T)
(InheritanceLink
   (WordInstanceNode "heart@66aef304-1b5b-4e9d-b117-3a90ea6844f7")
   (DefinedLinguisticConceptNode "definite")
)
; noun_number (heart, singular)
(InheritanceLink
   (WordInstanceNode "heart@66aef304-1b5b-4e9d-b117-3a90ea6844f7")
   (DefinedLinguisticConceptNode "singular")
)
; inflection-TAG (heart, .n)
(InheritanceLink
   (WordInstanceNode "heart@66aef304-1b5b-4e9d-b117-3a90ea6844f7")
   (DefinedLinguisticConceptNode ".n")
)
; pos (heart, noun)
(PartOfSpeechLink
   (WordInstanceNode "heart@66aef304-1b5b-4e9d-b117-3a90ea6844f7")
   (DefinedLinguisticConceptNode "noun")
)
; in (<<be>>, <<chest>>) 
(EvaluationLink
   (DefinedLinguisticRelationshipNode "in")
   (ListLink
      (WordInstanceNode "is@e051638b-86cc-4d24-a4a7-7f1dc21720f8")
      (WordInstanceNode "chest@6aa408df-18f7-4650-b7fe-1372f786766a")
   )
)
; _subj (<<be>>, <<heart>>) 
(EvaluationLink
   (DefinedLinguisticRelationshipNode "_subj")
   (ListLink
      (WordInstanceNode "is@e051638b-86cc-4d24-a4a7-7f1dc21720f8")
      (WordInstanceNode "heart@66aef304-1b5b-4e9d-b117-3a90ea6844f7")
   )
)
; tense (be, present)
(InheritanceLink
   (WordInstanceNode "is@e051638b-86cc-4d24-a4a7-7f1dc21720f8")
   (DefinedLinguisticConceptNode "present")
)
; inflection-TAG (be, .v)
(InheritanceLink
   (WordInstanceNode "is@e051638b-86cc-4d24-a4a7-7f1dc21720f8")
   (DefinedLinguisticConceptNode ".v")
)
; pos (be, verb)
(PartOfSpeechLink
   (WordInstanceNode "is@e051638b-86cc-4d24-a4a7-7f1dc21720f8")
   (DefinedLinguisticConceptNode "verb")
)
; pos (the, det)
(PartOfSpeechLink
   (WordInstanceNode "the@e92c8886-cbd8-4dfb-89a9-4342d713c56e")
   (DefinedLinguisticConceptNode "det")
)

; SENTENCE: [The heart is inside the chest.]
(ReferenceLink
   (WordInstanceNode "the@6d33cb38-2837-4a13-8de2-c6929a4dd2b1")
   (WordNode "the")
)
(WordInstanceLink
   (WordInstanceNode "the@6d33cb38-2837-4a13-8de2-c6929a4dd2b1")
   (ParseNode "sentence@43d81757-0f6c-427f-8824-5fc2326e1dfc_parse_0")
)
(ReferenceLink
   (WordInstanceNode "heart@36c9845c-7a83-4068-8b2a-b1a18b0541a1")
   (WordNode "heart")
)
(WordInstanceLink
   (WordInstanceNode "heart@36c9845c-7a83-4068-8b2a-b1a18b0541a1")
   (ParseNode "sentence@43d81757-0f6c-427f-8824-5fc2326e1dfc_parse_0")
)
(ReferenceLink
   (WordInstanceNode "is@be2e2de6-3885-4c2e-bdef-7ccfe020a65d")
   (WordNode "is")
)
(WordInstanceLink
   (WordInstanceNode "is@be2e2de6-3885-4c2e-bdef-7ccfe020a65d")
   (ParseNode "sentence@43d81757-0f6c-427f-8824-5fc2326e1dfc_parse_0")
)
(ReferenceLink
   (WordInstanceNode "inside@0106cc57-8e57-4b8f-9e1e-6fd1637b2430")
   (WordNode "inside")
)
(WordInstanceLink
   (WordInstanceNode "inside@0106cc57-8e57-4b8f-9e1e-6fd1637b2430")
   (ParseNode "sentence@43d81757-0f6c-427f-8824-5fc2326e1dfc_parse_0")
)
(ReferenceLink
   (WordInstanceNode "the@b1bfcb57-8c91-48a5-9c54-fd55aac20070")
   (WordNode "the")
)
(WordInstanceLink
   (WordInstanceNode "the@b1bfcb57-8c91-48a5-9c54-fd55aac20070")
   (ParseNode "sentence@43d81757-0f6c-427f-8824-5fc2326e1dfc_parse_0")
)
(ReferenceLink
   (WordInstanceNode "chest@d744e3b5-e5e9-412b-8399-506d9240ae18")
   (WordNode "chest")
)
(WordInstanceLink
   (WordInstanceNode "chest@d744e3b5-e5e9-412b-8399-506d9240ae18")
   (ParseNode "sentence@43d81757-0f6c-427f-8824-5fc2326e1dfc_parse_0")
)
(ReferenceLink
   (WordInstanceNode ".@0b42ddac-9470-4576-bc43-cc28e6297e93")
   (WordNode ".")
)
(WordInstanceLink
   (WordInstanceNode ".@0b42ddac-9470-4576-bc43-cc28e6297e93")
   (ParseNode "sentence@43d81757-0f6c-427f-8824-5fc2326e1dfc_parse_0")
)
(ReferenceLink
   (ParseNode "sentence@43d81757-0f6c-427f-8824-5fc2326e1dfc_parse_0")
   (ListLink
      (WordInstanceNode "the@6d33cb38-2837-4a13-8de2-c6929a4dd2b1")
      (WordInstanceNode "heart@36c9845c-7a83-4068-8b2a-b1a18b0541a1")
      (WordInstanceNode "is@be2e2de6-3885-4c2e-bdef-7ccfe020a65d")
      (WordInstanceNode "inside@0106cc57-8e57-4b8f-9e1e-6fd1637b2430")
      (WordInstanceNode "the@b1bfcb57-8c91-48a5-9c54-fd55aac20070")
      (WordInstanceNode "chest@d744e3b5-e5e9-412b-8399-506d9240ae18")
      (WordInstanceNode ".@0b42ddac-9470-4576-bc43-cc28e6297e93")
   )
)
(EvaluationLink
   (LinkGrammarRelationshipNode "Js")
   (ListLink
      (WordInstanceNode "inside@0106cc57-8e57-4b8f-9e1e-6fd1637b2430")
      (WordInstanceNode "chest@d744e3b5-e5e9-412b-8399-506d9240ae18")
   )
)
(EvaluationLink
   (LinkGrammarRelationshipNode "Ds")
   (ListLink
      (WordInstanceNode "the@b1bfcb57-8c91-48a5-9c54-fd55aac20070")
      (WordInstanceNode "chest@d744e3b5-e5e9-412b-8399-506d9240ae18")
   )
)
(EvaluationLink
   (LinkGrammarRelationshipNode "Xp")
   (ListLink
      (WordInstanceNode "LEFT-WALL")
      (WordInstanceNode ".@0b42ddac-9470-4576-bc43-cc28e6297e93")
   )
)
(EvaluationLink
   (LinkGrammarRelationshipNode "Pp")
   (ListLink
      (WordInstanceNode "is@be2e2de6-3885-4c2e-bdef-7ccfe020a65d")
      (WordInstanceNode "inside@0106cc57-8e57-4b8f-9e1e-6fd1637b2430")
   )
)
(EvaluationLink
   (LinkGrammarRelationshipNode "Ss")
   (ListLink
      (WordInstanceNode "heart@36c9845c-7a83-4068-8b2a-b1a18b0541a1")
      (WordInstanceNode "is@be2e2de6-3885-4c2e-bdef-7ccfe020a65d")
   )
)
(EvaluationLink
   (LinkGrammarRelationshipNode "Wd")
   (ListLink
      (WordInstanceNode "LEFT-WALL")
      (WordInstanceNode "heart@36c9845c-7a83-4068-8b2a-b1a18b0541a1")
   )
)
(EvaluationLink
   (LinkGrammarRelationshipNode "D*u")
   (ListLink
      (WordInstanceNode "the@6d33cb38-2837-4a13-8de2-c6929a4dd2b1")
      (WordInstanceNode "heart@36c9845c-7a83-4068-8b2a-b1a18b0541a1")
   )
)
(ParseLink
   (ParseNode "sentence@43d81757-0f6c-427f-8824-5fc2326e1dfc_parse_0" (stv 1.0 0.9084))
   (SentenceNode "sentence@43d81757-0f6c-427f-8824-5fc2326e1dfc")
)
(LemmaLink
   (WordInstanceNode "the@6d33cb38-2837-4a13-8de2-c6929a4dd2b1")
   (WordNode "the")
)
(LemmaLink
   (WordInstanceNode "heart@36c9845c-7a83-4068-8b2a-b1a18b0541a1")
   (WordNode "heart")
)
(LemmaLink
   (WordInstanceNode "is@be2e2de6-3885-4c2e-bdef-7ccfe020a65d")
   (WordNode "be")
)
(LemmaLink
   (WordInstanceNode "inside@0106cc57-8e57-4b8f-9e1e-6fd1637b2430")
   (WordNode "inside")
)
(LemmaLink
   (WordInstanceNode "the@b1bfcb57-8c91-48a5-9c54-fd55aac20070")
   (WordNode "the")
)
(LemmaLink
   (WordInstanceNode "chest@d744e3b5-e5e9-412b-8399-506d9240ae18")
   (WordNode "chest")
)
(LemmaLink
   (WordInstanceNode ".@0b42ddac-9470-4576-bc43-cc28e6297e93")
   (WordNode ".")
)
; DEFINITE-FLAG (chest, T)
(InheritanceLink
   (WordInstanceNode "chest@d744e3b5-e5e9-412b-8399-506d9240ae18")
   (DefinedLinguisticConceptNode "definite")
)
; noun_number (chest, singular)
(InheritanceLink
   (WordInstanceNode "chest@d744e3b5-e5e9-412b-8399-506d9240ae18")
   (DefinedLinguisticConceptNode "singular")
)
; inflection-TAG (chest, .n)
(InheritanceLink
   (WordInstanceNode "chest@d744e3b5-e5e9-412b-8399-506d9240ae18")
   (DefinedLinguisticConceptNode ".n")
)
; pos (chest, noun)
(PartOfSpeechLink
   (WordInstanceNode "chest@d744e3b5-e5e9-412b-8399-506d9240ae18")
   (DefinedLinguisticConceptNode "noun")
)
; pos (., punctuation)
(PartOfSpeechLink
   (WordInstanceNode ".@0b42ddac-9470-4576-bc43-cc28e6297e93")
   (DefinedLinguisticConceptNode "punctuation")
)
; DEFINITE-FLAG (heart, T)
(InheritanceLink
   (WordInstanceNode "heart@36c9845c-7a83-4068-8b2a-b1a18b0541a1")
   (DefinedLinguisticConceptNode "definite")
)
; noun_number (heart, singular)
(InheritanceLink
   (WordInstanceNode "heart@36c9845c-7a83-4068-8b2a-b1a18b0541a1")
   (DefinedLinguisticConceptNode "singular")
)
; inflection-TAG (heart, .n)
(InheritanceLink
   (WordInstanceNode "heart@36c9845c-7a83-4068-8b2a-b1a18b0541a1")
   (DefinedLinguisticConceptNode ".n")
)
; pos (heart, noun)
(PartOfSpeechLink
   (WordInstanceNode "heart@36c9845c-7a83-4068-8b2a-b1a18b0541a1")
   (DefinedLinguisticConceptNode "noun")
)
; inside (<<_%copula>>, <<inside>>) 
(EvaluationLink
   (DefinedLinguisticRelationshipNode "inside")
   (ListLink
      (WordInstanceNode "is@be2e2de6-3885-4c2e-bdef-7ccfe020a65d")
      (WordInstanceNode "inside@0106cc57-8e57-4b8f-9e1e-6fd1637b2430")
   )
)
; inflection-TAG (_%copula, .v)
(InheritanceLink
   (WordInstanceNode "is@be2e2de6-3885-4c2e-bdef-7ccfe020a65d")
   (DefinedLinguisticConceptNode ".v")
)
; pos (_%copula, verb)
(PartOfSpeechLink
   (WordInstanceNode "is@be2e2de6-3885-4c2e-bdef-7ccfe020a65d")
   (DefinedLinguisticConceptNode "verb")
)
; _pobj (<<inside>>, <<chest>>) 
(EvaluationLink
   (DefinedLinguisticRelationshipNode "_pobj")
   (ListLink
      (WordInstanceNode "inside@0106cc57-8e57-4b8f-9e1e-6fd1637b2430")
      (WordInstanceNode "chest@d744e3b5-e5e9-412b-8399-506d9240ae18")
   )
)
; _psubj (<<inside>>, <<heart>>) 
(EvaluationLink
   (DefinedLinguisticRelationshipNode "_psubj")
   (ListLink
      (WordInstanceNode "inside@0106cc57-8e57-4b8f-9e1e-6fd1637b2430")
      (WordInstanceNode "heart@36c9845c-7a83-4068-8b2a-b1a18b0541a1")
   )
)
; tense (inside, present)
(InheritanceLink
   (WordInstanceNode "inside@0106cc57-8e57-4b8f-9e1e-6fd1637b2430")
   (DefinedLinguisticConceptNode "present")
)
; inflection-TAG (inside, .r)
(InheritanceLink
   (WordInstanceNode "inside@0106cc57-8e57-4b8f-9e1e-6fd1637b2430")
   (DefinedLinguisticConceptNode ".r")
)
; SPECIAL-PREP-FLAG (inside, T)
(InheritanceLink
   (WordInstanceNode "inside@0106cc57-8e57-4b8f-9e1e-6fd1637b2430")
   (DefinedLinguisticConceptNode "special-prep")
)
; pos (inside, verb)
(PartOfSpeechLink
   (WordInstanceNode "inside@0106cc57-8e57-4b8f-9e1e-6fd1637b2430")
   (DefinedLinguisticConceptNode "verb")
)
; pos (the, det)
(PartOfSpeechLink
   (WordInstanceNode "the@b1bfcb57-8c91-48a5-9c54-fd55aac20070")
   (DefinedLinguisticConceptNode "det")
)
; pos (the, det)
(PartOfSpeechLink
   (WordInstanceNode "the@6d33cb38-2837-4a13-8de2-c6929a4dd2b1")
   (DefinedLinguisticConceptNode "det")
)

(ReferenceLink
   (WordInstanceNode "the@034cb367-e514-42a5-900e-617d8a780bda")
   (WordNode "the")
)
(WordInstanceLink
   (WordInstanceNode "the@034cb367-e514-42a5-900e-617d8a780bda")
   (ParseNode "sentence@43d81757-0f6c-427f-8824-5fc2326e1dfc_parse_1")
)
(ReferenceLink
   (WordInstanceNode "heart@bed1d7c7-81bf-4c17-9f28-df6dadc7afc0")
   (WordNode "heart")
)
(WordInstanceLink
   (WordInstanceNode "heart@bed1d7c7-81bf-4c17-9f28-df6dadc7afc0")
   (ParseNode "sentence@43d81757-0f6c-427f-8824-5fc2326e1dfc_parse_1")
)
(ReferenceLink
   (WordInstanceNode "is@3e6ba563-debc-481d-b27f-a1684e499c8c")
   (WordNode "is")
)
(WordInstanceLink
   (WordInstanceNode "is@3e6ba563-debc-481d-b27f-a1684e499c8c")
   (ParseNode "sentence@43d81757-0f6c-427f-8824-5fc2326e1dfc_parse_1")
)
(ReferenceLink
   (WordInstanceNode "inside@5ce439cf-e654-43f0-a254-3fef66ab6ac8")
   (WordNode "inside")
)
(WordInstanceLink
   (WordInstanceNode "inside@5ce439cf-e654-43f0-a254-3fef66ab6ac8")
   (ParseNode "sentence@43d81757-0f6c-427f-8824-5fc2326e1dfc_parse_1")
)
(ReferenceLink
   (WordInstanceNode "the@0fb94122-0905-42e5-a81e-277964783058")
   (WordNode "the")
)
(WordInstanceLink
   (WordInstanceNode "the@0fb94122-0905-42e5-a81e-277964783058")
   (ParseNode "sentence@43d81757-0f6c-427f-8824-5fc2326e1dfc_parse_1")
)
(ReferenceLink
   (WordInstanceNode "chest@900175b1-98b5-4cde-91a3-399a4f8b74cd")
   (WordNode "chest")
)
(WordInstanceLink
   (WordInstanceNode "chest@900175b1-98b5-4cde-91a3-399a4f8b74cd")
   (ParseNode "sentence@43d81757-0f6c-427f-8824-5fc2326e1dfc_parse_1")
)
(ReferenceLink
   (WordInstanceNode ".@4dc3d81f-dcd8-4004-b796-dc621bce96fb")
   (WordNode ".")
)
(WordInstanceLink
   (WordInstanceNode ".@4dc3d81f-dcd8-4004-b796-dc621bce96fb")
   (ParseNode "sentence@43d81757-0f6c-427f-8824-5fc2326e1dfc_parse_1")
)
(ReferenceLink
   (ParseNode "sentence@43d81757-0f6c-427f-8824-5fc2326e1dfc_parse_1")
   (ListLink
      (WordInstanceNode "the@034cb367-e514-42a5-900e-617d8a780bda")
      (WordInstanceNode "heart@bed1d7c7-81bf-4c17-9f28-df6dadc7afc0")
      (WordInstanceNode "is@3e6ba563-debc-481d-b27f-a1684e499c8c")
      (WordInstanceNode "inside@5ce439cf-e654-43f0-a254-3fef66ab6ac8")
      (WordInstanceNode "the@0fb94122-0905-42e5-a81e-277964783058")
      (WordInstanceNode "chest@900175b1-98b5-4cde-91a3-399a4f8b74cd")
      (WordInstanceNode ".@4dc3d81f-dcd8-4004-b796-dc621bce96fb")
   )
)
(EvaluationLink
   (LinkGrammarRelationshipNode "MVp")
   (ListLink
      (WordInstanceNode "is@3e6ba563-debc-481d-b27f-a1684e499c8c")
      (WordInstanceNode "inside@5ce439cf-e654-43f0-a254-3fef66ab6ac8")
   )
)
(EvaluationLink
   (LinkGrammarRelationshipNode "Js")
   (ListLink
      (WordInstanceNode "inside@5ce439cf-e654-43f0-a254-3fef66ab6ac8")
      (WordInstanceNode "chest@900175b1-98b5-4cde-91a3-399a4f8b74cd")
   )
)
(EvaluationLink
   (LinkGrammarRelationshipNode "Ds")
   (ListLink
      (WordInstanceNode "the@0fb94122-0905-42e5-a81e-277964783058")
      (WordInstanceNode "chest@900175b1-98b5-4cde-91a3-399a4f8b74cd")
   )
)
(EvaluationLink
   (LinkGrammarRelationshipNode "Xp")
   (ListLink
      (WordInstanceNode "LEFT-WALL")
      (WordInstanceNode ".@4dc3d81f-dcd8-4004-b796-dc621bce96fb")
   )
)
(EvaluationLink
   (LinkGrammarRelationshipNode "Ss")
   (ListLink
      (WordInstanceNode "heart@bed1d7c7-81bf-4c17-9f28-df6dadc7afc0")
      (WordInstanceNode "is@3e6ba563-debc-481d-b27f-a1684e499c8c")
   )
)
(EvaluationLink
   (LinkGrammarRelationshipNode "Wd")
   (ListLink
      (WordInstanceNode "LEFT-WALL")
      (WordInstanceNode "heart@bed1d7c7-81bf-4c17-9f28-df6dadc7afc0")
   )
)
(EvaluationLink
   (LinkGrammarRelationshipNode "D*u")
   (ListLink
      (WordInstanceNode "the@034cb367-e514-42a5-900e-617d8a780bda")
      (WordInstanceNode "heart@bed1d7c7-81bf-4c17-9f28-df6dadc7afc0")
   )
)
(ParseLink
   (ParseNode "sentence@43d81757-0f6c-427f-8824-5fc2326e1dfc_parse_1" (stv 1.0 0.6089))
   (SentenceNode "sentence@43d81757-0f6c-427f-8824-5fc2326e1dfc")
)
(LemmaLink
   (WordInstanceNode "the@034cb367-e514-42a5-900e-617d8a780bda")
   (WordNode "the")
)
(LemmaLink
   (WordInstanceNode "heart@bed1d7c7-81bf-4c17-9f28-df6dadc7afc0")
   (WordNode "heart")
)
(LemmaLink
   (WordInstanceNode "is@3e6ba563-debc-481d-b27f-a1684e499c8c")
   (WordNode "be")
)
(LemmaLink
   (WordInstanceNode "inside@5ce439cf-e654-43f0-a254-3fef66ab6ac8")
   (WordNode "inside")
)
(LemmaLink
   (WordInstanceNode "the@0fb94122-0905-42e5-a81e-277964783058")
   (WordNode "the")
)
(LemmaLink
   (WordInstanceNode "chest@900175b1-98b5-4cde-91a3-399a4f8b74cd")
   (WordNode "chest")
)
(LemmaLink
   (WordInstanceNode ".@4dc3d81f-dcd8-4004-b796-dc621bce96fb")
   (WordNode ".")
)
; inflection-TAG (inside, .r)
(InheritanceLink
   (WordInstanceNode "inside@5ce439cf-e654-43f0-a254-3fef66ab6ac8")
   (DefinedLinguisticConceptNode ".r")
)
; pos (inside, prep)
(PartOfSpeechLink
   (WordInstanceNode "inside@5ce439cf-e654-43f0-a254-3fef66ab6ac8")
   (DefinedLinguisticConceptNode "prep")
)
; DEFINITE-FLAG (chest, T)
(InheritanceLink
   (WordInstanceNode "chest@900175b1-98b5-4cde-91a3-399a4f8b74cd")
   (DefinedLinguisticConceptNode "definite")
)
; noun_number (chest, singular)
(InheritanceLink
   (WordInstanceNode "chest@900175b1-98b5-4cde-91a3-399a4f8b74cd")
   (DefinedLinguisticConceptNode "singular")
)
; inflection-TAG (chest, .n)
(InheritanceLink
   (WordInstanceNode "chest@900175b1-98b5-4cde-91a3-399a4f8b74cd")
   (DefinedLinguisticConceptNode ".n")
)
; pos (chest, noun)
(PartOfSpeechLink
   (WordInstanceNode "chest@900175b1-98b5-4cde-91a3-399a4f8b74cd")
   (DefinedLinguisticConceptNode "noun")
)
; pos (., punctuation)
(PartOfSpeechLink
   (WordInstanceNode ".@4dc3d81f-dcd8-4004-b796-dc621bce96fb")
   (DefinedLinguisticConceptNode "punctuation")
)
; pos (the, det)
(PartOfSpeechLink
   (WordInstanceNode "the@0fb94122-0905-42e5-a81e-277964783058")
   (DefinedLinguisticConceptNode "det")
)
; DEFINITE-FLAG (heart, T)
(InheritanceLink
   (WordInstanceNode "heart@bed1d7c7-81bf-4c17-9f28-df6dadc7afc0")
   (DefinedLinguisticConceptNode "definite")
)
; noun_number (heart, singular)
(InheritanceLink
   (WordInstanceNode "heart@bed1d7c7-81bf-4c17-9f28-df6dadc7afc0")
   (DefinedLinguisticConceptNode "singular")
)
; inflection-TAG (heart, .n)
(InheritanceLink
   (WordInstanceNode "heart@bed1d7c7-81bf-4c17-9f28-df6dadc7afc0")
   (DefinedLinguisticConceptNode ".n")
)
; pos (heart, noun)
(PartOfSpeechLink
   (WordInstanceNode "heart@bed1d7c7-81bf-4c17-9f28-df6dadc7afc0")
   (DefinedLinguisticConceptNode "noun")
)
; _subj (<<be>>, <<heart>>) 
(EvaluationLink
   (DefinedLinguisticRelationshipNode "_subj")
   (ListLink
      (WordInstanceNode "is@3e6ba563-debc-481d-b27f-a1684e499c8c")
      (WordInstanceNode "heart@bed1d7c7-81bf-4c17-9f28-df6dadc7afc0")
   )
)
; inside (<<be>>, <<chest>>) 
(EvaluationLink
   (DefinedLinguisticRelationshipNode "inside")
   (ListLink
      (WordInstanceNode "is@3e6ba563-debc-481d-b27f-a1684e499c8c")
      (WordInstanceNode "chest@900175b1-98b5-4cde-91a3-399a4f8b74cd")
   )
)
; tense (be, present)
(InheritanceLink
   (WordInstanceNode "is@3e6ba563-debc-481d-b27f-a1684e499c8c")
   (DefinedLinguisticConceptNode "present")
)
; inflection-TAG (be, .v)
(InheritanceLink
   (WordInstanceNode "is@3e6ba563-debc-481d-b27f-a1684e499c8c")
   (DefinedLinguisticConceptNode ".v")
)
; pos (be, verb)
(PartOfSpeechLink
   (WordInstanceNode "is@3e6ba563-debc-481d-b27f-a1684e499c8c")
   (DefinedLinguisticConceptNode "verb")
)
; pos (the, det)
(PartOfSpeechLink
   (WordInstanceNode "the@034cb367-e514-42a5-900e-617d8a780bda")
   (DefinedLinguisticConceptNode "det")
)

; SENTENCE: [The garage is behind the house.]
(ReferenceLink
   (WordInstanceNode "the@8376425e-390d-4ab4-a70d-c2b560407e46")
   (WordNode "the")
)
(WordInstanceLink
   (WordInstanceNode "the@8376425e-390d-4ab4-a70d-c2b560407e46")
   (ParseNode "sentence@f33be399-e41e-49b2-8610-1c01a0ee77c8_parse_0")
)
(ReferenceLink
   (WordInstanceNode "garage@ec65c67e-391d-4ab0-b149-8adba4eb4904")
   (WordNode "garage")
)
(WordInstanceLink
   (WordInstanceNode "garage@ec65c67e-391d-4ab0-b149-8adba4eb4904")
   (ParseNode "sentence@f33be399-e41e-49b2-8610-1c01a0ee77c8_parse_0")
)
(ReferenceLink
   (WordInstanceNode "is@6048aae1-84ef-4ee4-be09-cf6b46d8a0a0")
   (WordNode "is")
)
(WordInstanceLink
   (WordInstanceNode "is@6048aae1-84ef-4ee4-be09-cf6b46d8a0a0")
   (ParseNode "sentence@f33be399-e41e-49b2-8610-1c01a0ee77c8_parse_0")
)
(ReferenceLink
   (WordInstanceNode "behind@e1326093-6b14-4811-b9b3-5e83e33b7fa1")
   (WordNode "behind")
)
(WordInstanceLink
   (WordInstanceNode "behind@e1326093-6b14-4811-b9b3-5e83e33b7fa1")
   (ParseNode "sentence@f33be399-e41e-49b2-8610-1c01a0ee77c8_parse_0")
)
(ReferenceLink
   (WordInstanceNode "the@8c0e6c36-18fb-4bb6-8ad3-52603ece4383")
   (WordNode "the")
)
(WordInstanceLink
   (WordInstanceNode "the@8c0e6c36-18fb-4bb6-8ad3-52603ece4383")
   (ParseNode "sentence@f33be399-e41e-49b2-8610-1c01a0ee77c8_parse_0")
)
(ReferenceLink
   (WordInstanceNode "house@12b517d5-0627-4c81-9473-14759b9a5b12")
   (WordNode "house")
)
(WordInstanceLink
   (WordInstanceNode "house@12b517d5-0627-4c81-9473-14759b9a5b12")
   (ParseNode "sentence@f33be399-e41e-49b2-8610-1c01a0ee77c8_parse_0")
)
(ReferenceLink
   (WordInstanceNode ".@2ef2fce5-6811-4602-9106-eed4875c05fb")
   (WordNode ".")
)
(WordInstanceLink
   (WordInstanceNode ".@2ef2fce5-6811-4602-9106-eed4875c05fb")
   (ParseNode "sentence@f33be399-e41e-49b2-8610-1c01a0ee77c8_parse_0")
)
(ReferenceLink
   (ParseNode "sentence@f33be399-e41e-49b2-8610-1c01a0ee77c8_parse_0")
   (ListLink
      (WordInstanceNode "the@8376425e-390d-4ab4-a70d-c2b560407e46")
      (WordInstanceNode "garage@ec65c67e-391d-4ab0-b149-8adba4eb4904")
      (WordInstanceNode "is@6048aae1-84ef-4ee4-be09-cf6b46d8a0a0")
      (WordInstanceNode "behind@e1326093-6b14-4811-b9b3-5e83e33b7fa1")
      (WordInstanceNode "the@8c0e6c36-18fb-4bb6-8ad3-52603ece4383")
      (WordInstanceNode "house@12b517d5-0627-4c81-9473-14759b9a5b12")
      (WordInstanceNode ".@2ef2fce5-6811-4602-9106-eed4875c05fb")
   )
)
(EvaluationLink
   (LinkGrammarRelationshipNode "Js")
   (ListLink
      (WordInstanceNode "behind@e1326093-6b14-4811-b9b3-5e83e33b7fa1")
      (WordInstanceNode "house@12b517d5-0627-4c81-9473-14759b9a5b12")
   )
)
(EvaluationLink
   (LinkGrammarRelationshipNode "Ds")
   (ListLink
      (WordInstanceNode "the@8c0e6c36-18fb-4bb6-8ad3-52603ece4383")
      (WordInstanceNode "house@12b517d5-0627-4c81-9473-14759b9a5b12")
   )
)
(EvaluationLink
   (LinkGrammarRelationshipNode "Xp")
   (ListLink
      (WordInstanceNode "LEFT-WALL")
      (WordInstanceNode ".@2ef2fce5-6811-4602-9106-eed4875c05fb")
   )
)
(EvaluationLink
   (LinkGrammarRelationshipNode "Pp")
   (ListLink
      (WordInstanceNode "is@6048aae1-84ef-4ee4-be09-cf6b46d8a0a0")
      (WordInstanceNode "behind@e1326093-6b14-4811-b9b3-5e83e33b7fa1")
   )
)
(EvaluationLink
   (LinkGrammarRelationshipNode "Ss")
   (ListLink
      (WordInstanceNode "garage@ec65c67e-391d-4ab0-b149-8adba4eb4904")
      (WordInstanceNode "is@6048aae1-84ef-4ee4-be09-cf6b46d8a0a0")
   )
)
(EvaluationLink
   (LinkGrammarRelationshipNode "Wd")
   (ListLink
      (WordInstanceNode "LEFT-WALL")
      (WordInstanceNode "garage@ec65c67e-391d-4ab0-b149-8adba4eb4904")
   )
)
(EvaluationLink
   (LinkGrammarRelationshipNode "Ds")
   (ListLink
      (WordInstanceNode "the@8376425e-390d-4ab4-a70d-c2b560407e46")
      (WordInstanceNode "garage@ec65c67e-391d-4ab0-b149-8adba4eb4904")
   )
)
(ParseLink
   (ParseNode "sentence@f33be399-e41e-49b2-8610-1c01a0ee77c8_parse_0" (stv 1.0 0.9084))
   (SentenceNode "sentence@f33be399-e41e-49b2-8610-1c01a0ee77c8")
)
(LemmaLink
   (WordInstanceNode "the@8376425e-390d-4ab4-a70d-c2b560407e46")
   (WordNode "the")
)
(LemmaLink
   (WordInstanceNode "garage@ec65c67e-391d-4ab0-b149-8adba4eb4904")
   (WordNode "garage")
)
(LemmaLink
   (WordInstanceNode "is@6048aae1-84ef-4ee4-be09-cf6b46d8a0a0")
   (WordNode "be")
)
(LemmaLink
   (WordInstanceNode "behind@e1326093-6b14-4811-b9b3-5e83e33b7fa1")
   (WordNode "behind")
)
(LemmaLink
   (WordInstanceNode "the@8c0e6c36-18fb-4bb6-8ad3-52603ece4383")
   (WordNode "the")
)
(LemmaLink
   (WordInstanceNode "house@12b517d5-0627-4c81-9473-14759b9a5b12")
   (WordNode "house")
)
(LemmaLink
   (WordInstanceNode ".@2ef2fce5-6811-4602-9106-eed4875c05fb")
   (WordNode ".")
)
; DEFINITE-FLAG (house, T)
(InheritanceLink
   (WordInstanceNode "house@12b517d5-0627-4c81-9473-14759b9a5b12")
   (DefinedLinguisticConceptNode "definite")
)
; noun_number (house, singular)
(InheritanceLink
   (WordInstanceNode "house@12b517d5-0627-4c81-9473-14759b9a5b12")
   (DefinedLinguisticConceptNode "singular")
)
; inflection-TAG (house, .n)
(InheritanceLink
   (WordInstanceNode "house@12b517d5-0627-4c81-9473-14759b9a5b12")
   (DefinedLinguisticConceptNode ".n")
)
; pos (house, noun)
(PartOfSpeechLink
   (WordInstanceNode "house@12b517d5-0627-4c81-9473-14759b9a5b12")
   (DefinedLinguisticConceptNode "noun")
)
; pos (., punctuation)
(PartOfSpeechLink
   (WordInstanceNode ".@2ef2fce5-6811-4602-9106-eed4875c05fb")
   (DefinedLinguisticConceptNode "punctuation")
)
; DEFINITE-FLAG (garage, T)
(InheritanceLink
   (WordInstanceNode "garage@ec65c67e-391d-4ab0-b149-8adba4eb4904")
   (DefinedLinguisticConceptNode "definite")
)
; noun_number (garage, singular)
(InheritanceLink
   (WordInstanceNode "garage@ec65c67e-391d-4ab0-b149-8adba4eb4904")
   (DefinedLinguisticConceptNode "singular")
)
; inflection-TAG (garage, .n)
(InheritanceLink
   (WordInstanceNode "garage@ec65c67e-391d-4ab0-b149-8adba4eb4904")
   (DefinedLinguisticConceptNode ".n")
)
; pos (garage, noun)
(PartOfSpeechLink
   (WordInstanceNode "garage@ec65c67e-391d-4ab0-b149-8adba4eb4904")
   (DefinedLinguisticConceptNode "noun")
)
; behind (<<_%copula>>, <<behind>>) 
(EvaluationLink
   (DefinedLinguisticRelationshipNode "behind")
   (ListLink
      (WordInstanceNode "is@6048aae1-84ef-4ee4-be09-cf6b46d8a0a0")
      (WordInstanceNode "behind@e1326093-6b14-4811-b9b3-5e83e33b7fa1")
   )
)
; inflection-TAG (_%copula, .v)
(InheritanceLink
   (WordInstanceNode "is@6048aae1-84ef-4ee4-be09-cf6b46d8a0a0")
   (DefinedLinguisticConceptNode ".v")
)
; pos (_%copula, verb)
(PartOfSpeechLink
   (WordInstanceNode "is@6048aae1-84ef-4ee4-be09-cf6b46d8a0a0")
   (DefinedLinguisticConceptNode "verb")
)
; _pobj (<<behind>>, <<house>>) 
(EvaluationLink
   (DefinedLinguisticRelationshipNode "_pobj")
   (ListLink
      (WordInstanceNode "behind@e1326093-6b14-4811-b9b3-5e83e33b7fa1")
      (WordInstanceNode "house@12b517d5-0627-4c81-9473-14759b9a5b12")
   )
)
; _psubj (<<behind>>, <<garage>>) 
(EvaluationLink
   (DefinedLinguisticRelationshipNode "_psubj")
   (ListLink
      (WordInstanceNode "behind@e1326093-6b14-4811-b9b3-5e83e33b7fa1")
      (WordInstanceNode "garage@ec65c67e-391d-4ab0-b149-8adba4eb4904")
   )
)
; tense (behind, present)
(InheritanceLink
   (WordInstanceNode "behind@e1326093-6b14-4811-b9b3-5e83e33b7fa1")
   (DefinedLinguisticConceptNode "present")
)
; inflection-TAG (behind, .p)
(InheritanceLink
   (WordInstanceNode "behind@e1326093-6b14-4811-b9b3-5e83e33b7fa1")
   (DefinedLinguisticConceptNode ".p")
)
; SPECIAL-PREP-FLAG (behind, T)
(InheritanceLink
   (WordInstanceNode "behind@e1326093-6b14-4811-b9b3-5e83e33b7fa1")
   (DefinedLinguisticConceptNode "special-prep")
)
; pos (behind, noun)
(PartOfSpeechLink
   (WordInstanceNode "behind@e1326093-6b14-4811-b9b3-5e83e33b7fa1")
   (DefinedLinguisticConceptNode "noun")
)
; pos (the, det)
(PartOfSpeechLink
   (WordInstanceNode "the@8c0e6c36-18fb-4bb6-8ad3-52603ece4383")
   (DefinedLinguisticConceptNode "det")
)
; pos (the, det)
(PartOfSpeechLink
   (WordInstanceNode "the@8376425e-390d-4ab4-a70d-c2b560407e46")
   (DefinedLinguisticConceptNode "det")
)

(ReferenceLink
   (WordInstanceNode "the@a6f0aa84-705f-4fa1-9d69-cfd54da9f99f")
   (WordNode "the")
)
(WordInstanceLink
   (WordInstanceNode "the@a6f0aa84-705f-4fa1-9d69-cfd54da9f99f")
   (ParseNode "sentence@f33be399-e41e-49b2-8610-1c01a0ee77c8_parse_1")
)
(ReferenceLink
   (WordInstanceNode "garage@3e77ec00-be6b-4ad2-bd5f-23ee2331ff2a")
   (WordNode "garage")
)
(WordInstanceLink
   (WordInstanceNode "garage@3e77ec00-be6b-4ad2-bd5f-23ee2331ff2a")
   (ParseNode "sentence@f33be399-e41e-49b2-8610-1c01a0ee77c8_parse_1")
)
(ReferenceLink
   (WordInstanceNode "is@ec6873a1-5810-4086-873d-e9e6fad8dee4")
   (WordNode "is")
)
(WordInstanceLink
   (WordInstanceNode "is@ec6873a1-5810-4086-873d-e9e6fad8dee4")
   (ParseNode "sentence@f33be399-e41e-49b2-8610-1c01a0ee77c8_parse_1")
)
(ReferenceLink
   (WordInstanceNode "behind@2885ed9f-ef25-41e4-b8f6-6532039d2cc6")
   (WordNode "behind")
)
(WordInstanceLink
   (WordInstanceNode "behind@2885ed9f-ef25-41e4-b8f6-6532039d2cc6")
   (ParseNode "sentence@f33be399-e41e-49b2-8610-1c01a0ee77c8_parse_1")
)
(ReferenceLink
   (WordInstanceNode "the@afe5bf76-68c8-4038-9fd6-12962401ca50")
   (WordNode "the")
)
(WordInstanceLink
   (WordInstanceNode "the@afe5bf76-68c8-4038-9fd6-12962401ca50")
   (ParseNode "sentence@f33be399-e41e-49b2-8610-1c01a0ee77c8_parse_1")
)
(ReferenceLink
   (WordInstanceNode "house@c5e2f759-f418-402b-bc83-c30d8ebe3199")
   (WordNode "house")
)
(WordInstanceLink
   (WordInstanceNode "house@c5e2f759-f418-402b-bc83-c30d8ebe3199")
   (ParseNode "sentence@f33be399-e41e-49b2-8610-1c01a0ee77c8_parse_1")
)
(ReferenceLink
   (WordInstanceNode ".@90e3300b-5f58-4c34-8fe8-00f141bfdda7")
   (WordNode ".")
)
(WordInstanceLink
   (WordInstanceNode ".@90e3300b-5f58-4c34-8fe8-00f141bfdda7")
   (ParseNode "sentence@f33be399-e41e-49b2-8610-1c01a0ee77c8_parse_1")
)
(ReferenceLink
   (ParseNode "sentence@f33be399-e41e-49b2-8610-1c01a0ee77c8_parse_1")
   (ListLink
      (WordInstanceNode "the@a6f0aa84-705f-4fa1-9d69-cfd54da9f99f")
      (WordInstanceNode "garage@3e77ec00-be6b-4ad2-bd5f-23ee2331ff2a")
      (WordInstanceNode "is@ec6873a1-5810-4086-873d-e9e6fad8dee4")
      (WordInstanceNode "behind@2885ed9f-ef25-41e4-b8f6-6532039d2cc6")
      (WordInstanceNode "the@afe5bf76-68c8-4038-9fd6-12962401ca50")
      (WordInstanceNode "house@c5e2f759-f418-402b-bc83-c30d8ebe3199")
      (WordInstanceNode ".@90e3300b-5f58-4c34-8fe8-00f141bfdda7")
   )
)
(EvaluationLink
   (LinkGrammarRelationshipNode "MVp")
   (ListLink
      (WordInstanceNode "is@ec6873a1-5810-4086-873d-e9e6fad8dee4")
      (WordInstanceNode "behind@2885ed9f-ef25-41e4-b8f6-6532039d2cc6")
   )
)
(EvaluationLink
   (LinkGrammarRelationshipNode "Js")
   (ListLink
      (WordInstanceNode "behind@2885ed9f-ef25-41e4-b8f6-6532039d2cc6")
      (WordInstanceNode "house@c5e2f759-f418-402b-bc83-c30d8ebe3199")
   )
)
(EvaluationLink
   (LinkGrammarRelationshipNode "Ds")
   (ListLink
      (WordInstanceNode "the@afe5bf76-68c8-4038-9fd6-12962401ca50")
      (WordInstanceNode "house@c5e2f759-f418-402b-bc83-c30d8ebe3199")
   )
)
(EvaluationLink
   (LinkGrammarRelationshipNode "Xp")
   (ListLink
      (WordInstanceNode "LEFT-WALL")
      (WordInstanceNode ".@90e3300b-5f58-4c34-8fe8-00f141bfdda7")
   )
)
(EvaluationLink
   (LinkGrammarRelationshipNode "Ss")
   (ListLink
      (WordInstanceNode "garage@3e77ec00-be6b-4ad2-bd5f-23ee2331ff2a")
      (WordInstanceNode "is@ec6873a1-5810-4086-873d-e9e6fad8dee4")
   )
)
(EvaluationLink
   (LinkGrammarRelationshipNode "Wd")
   (ListLink
      (WordInstanceNode "LEFT-WALL")
      (WordInstanceNode "garage@3e77ec00-be6b-4ad2-bd5f-23ee2331ff2a")
   )
)
(EvaluationLink
   (LinkGrammarRelationshipNode "Ds")
   (ListLink
      (WordInstanceNode "the@a6f0aa84-705f-4fa1-9d69-cfd54da9f99f")
      (WordInstanceNode "garage@3e77ec00-be6b-4ad2-bd5f-23ee2331ff2a")
   )
)
(ParseLink
   (ParseNode "sentence@f33be399-e41e-49b2-8610-1c01a0ee77c8_parse_1" (stv 1.0 0.6089))
   (SentenceNode "sentence@f33be399-e41e-49b2-8610-1c01a0ee77c8")
)
(LemmaLink
   (WordInstanceNode "the@a6f0aa84-705f-4fa1-9d69-cfd54da9f99f")
   (WordNode "the")
)
(LemmaLink
   (WordInstanceNode "garage@3e77ec00-be6b-4ad2-bd5f-23ee2331ff2a")
   (WordNode "garage")
)
(LemmaLink
   (WordInstanceNode "is@ec6873a1-5810-4086-873d-e9e6fad8dee4")
   (WordNode "be")
)
(LemmaLink
   (WordInstanceNode "behind@2885ed9f-ef25-41e4-b8f6-6532039d2cc6")
   (WordNode "behind")
)
(LemmaLink
   (WordInstanceNode "the@afe5bf76-68c8-4038-9fd6-12962401ca50")
   (WordNode "the")
)
(LemmaLink
   (WordInstanceNode "house@c5e2f759-f418-402b-bc83-c30d8ebe3199")
   (WordNode "house")
)
(LemmaLink
   (WordInstanceNode ".@90e3300b-5f58-4c34-8fe8-00f141bfdda7")
   (WordNode ".")
)
; DEFINITE-FLAG (house, T)
(InheritanceLink
   (WordInstanceNode "house@c5e2f759-f418-402b-bc83-c30d8ebe3199")
   (DefinedLinguisticConceptNode "definite")
)
; noun_number (house, singular)
(InheritanceLink
   (WordInstanceNode "house@c5e2f759-f418-402b-bc83-c30d8ebe3199")
   (DefinedLinguisticConceptNode "singular")
)
; inflection-TAG (house, .n)
(InheritanceLink
   (WordInstanceNode "house@c5e2f759-f418-402b-bc83-c30d8ebe3199")
   (DefinedLinguisticConceptNode ".n")
)
; pos (house, noun)
(PartOfSpeechLink
   (WordInstanceNode "house@c5e2f759-f418-402b-bc83-c30d8ebe3199")
   (DefinedLinguisticConceptNode "noun")
)
; pos (., punctuation)
(PartOfSpeechLink
   (WordInstanceNode ".@90e3300b-5f58-4c34-8fe8-00f141bfdda7")
   (DefinedLinguisticConceptNode "punctuation")
)
; _obj (<<behind>>, <<house>>) 
(EvaluationLink
   (DefinedLinguisticRelationshipNode "_obj")
   (ListLink
      (WordInstanceNode "behind@2885ed9f-ef25-41e4-b8f6-6532039d2cc6")
      (WordInstanceNode "house@c5e2f759-f418-402b-bc83-c30d8ebe3199")
   )
)
; inflection-TAG (behind, .p)
(InheritanceLink
   (WordInstanceNode "behind@2885ed9f-ef25-41e4-b8f6-6532039d2cc6")
   (DefinedLinguisticConceptNode ".p")
)
; pos (behind, noun)
(PartOfSpeechLink
   (WordInstanceNode "behind@2885ed9f-ef25-41e4-b8f6-6532039d2cc6")
   (DefinedLinguisticConceptNode "noun")
)
; pos (the, det)
(PartOfSpeechLink
   (WordInstanceNode "the@afe5bf76-68c8-4038-9fd6-12962401ca50")
   (DefinedLinguisticConceptNode "det")
)
; DEFINITE-FLAG (garage, T)
(InheritanceLink
   (WordInstanceNode "garage@3e77ec00-be6b-4ad2-bd5f-23ee2331ff2a")
   (DefinedLinguisticConceptNode "definite")
)
; noun_number (garage, singular)
(InheritanceLink
   (WordInstanceNode "garage@3e77ec00-be6b-4ad2-bd5f-23ee2331ff2a")
   (DefinedLinguisticConceptNode "singular")
)
; inflection-TAG (garage, .n)
(InheritanceLink
   (WordInstanceNode "garage@3e77ec00-be6b-4ad2-bd5f-23ee2331ff2a")
   (DefinedLinguisticConceptNode ".n")
)
; pos (garage, noun)
(PartOfSpeechLink
   (WordInstanceNode "garage@3e77ec00-be6b-4ad2-bd5f-23ee2331ff2a")
   (DefinedLinguisticConceptNode "noun")
)
; behind (<<be>>, <<house>>) 
(EvaluationLink
   (DefinedLinguisticRelationshipNode "behind")
   (ListLink
      (WordInstanceNode "is@ec6873a1-5810-4086-873d-e9e6fad8dee4")
      (WordInstanceNode "house@c5e2f759-f418-402b-bc83-c30d8ebe3199")
   )
)
; _subj (<<be>>, <<garage>>) 
(EvaluationLink
   (DefinedLinguisticRelationshipNode "_subj")
   (ListLink
      (WordInstanceNode "is@ec6873a1-5810-4086-873d-e9e6fad8dee4")
      (WordInstanceNode "garage@3e77ec00-be6b-4ad2-bd5f-23ee2331ff2a")
   )
)
; tense (be, present)
(InheritanceLink
   (WordInstanceNode "is@ec6873a1-5810-4086-873d-e9e6fad8dee4")
   (DefinedLinguisticConceptNode "present")
)
; inflection-TAG (be, .v)
(InheritanceLink
   (WordInstanceNode "is@ec6873a1-5810-4086-873d-e9e6fad8dee4")
   (DefinedLinguisticConceptNode ".v")
)
; pos (be, verb)
(PartOfSpeechLink
   (WordInstanceNode "is@ec6873a1-5810-4086-873d-e9e6fad8dee4")
   (DefinedLinguisticConceptNode "verb")
)
; pos (the, det)
(PartOfSpeechLink
   (WordInstanceNode "the@a6f0aa84-705f-4fa1-9d69-cfd54da9f99f")
   (DefinedLinguisticConceptNode "det")
)

; SENTENCE: [Lisbon is the capital of Portugaul.]
(ReferenceLink
   (WordInstanceNode "Lisbon@9d5031b1-5caf-48eb-ae7c-b3a89bb11381")
   (WordNode "Lisbon")
)
(WordInstanceLink
   (WordInstanceNode "Lisbon@9d5031b1-5caf-48eb-ae7c-b3a89bb11381")
   (ParseNode "sentence@959f6ccd-5757-4a4e-9254-2351f5696400_parse_0")
)
(ReferenceLink
   (WordInstanceNode "is@234c5c1a-38a0-4728-8c02-09b2068a2c6d")
   (WordNode "is")
)
(WordInstanceLink
   (WordInstanceNode "is@234c5c1a-38a0-4728-8c02-09b2068a2c6d")
   (ParseNode "sentence@959f6ccd-5757-4a4e-9254-2351f5696400_parse_0")
)
(ReferenceLink
   (WordInstanceNode "the@219e7f7a-c6b0-4c41-bb8c-4193f02105c8")
   (WordNode "the")
)
(WordInstanceLink
   (WordInstanceNode "the@219e7f7a-c6b0-4c41-bb8c-4193f02105c8")
   (ParseNode "sentence@959f6ccd-5757-4a4e-9254-2351f5696400_parse_0")
)
(ReferenceLink
   (WordInstanceNode "capital@220cca64-e477-4246-8f2d-f0464f23b3e6")
   (WordNode "capital")
)
(WordInstanceLink
   (WordInstanceNode "capital@220cca64-e477-4246-8f2d-f0464f23b3e6")
   (ParseNode "sentence@959f6ccd-5757-4a4e-9254-2351f5696400_parse_0")
)
(ReferenceLink
   (WordInstanceNode "of@864bb7c0-1d2a-480e-9713-3a2e52d6043e")
   (WordNode "of")
)
(WordInstanceLink
   (WordInstanceNode "of@864bb7c0-1d2a-480e-9713-3a2e52d6043e")
   (ParseNode "sentence@959f6ccd-5757-4a4e-9254-2351f5696400_parse_0")
)
(ReferenceLink
   (WordInstanceNode "Portugaul@36d41058-3dc8-4e6c-82b9-300a5f48e283")
   (WordNode "Portugaul")
)
(WordInstanceLink
   (WordInstanceNode "Portugaul@36d41058-3dc8-4e6c-82b9-300a5f48e283")
   (ParseNode "sentence@959f6ccd-5757-4a4e-9254-2351f5696400_parse_0")
)
(ReferenceLink
   (WordInstanceNode ".@4859009a-e89a-44a0-9443-961306fa7af5")
   (WordNode ".")
)
(WordInstanceLink
   (WordInstanceNode ".@4859009a-e89a-44a0-9443-961306fa7af5")
   (ParseNode "sentence@959f6ccd-5757-4a4e-9254-2351f5696400_parse_0")
)
(ReferenceLink
   (ParseNode "sentence@959f6ccd-5757-4a4e-9254-2351f5696400_parse_0")
   (ListLink
      (WordInstanceNode "Lisbon@9d5031b1-5caf-48eb-ae7c-b3a89bb11381")
      (WordInstanceNode "is@234c5c1a-38a0-4728-8c02-09b2068a2c6d")
      (WordInstanceNode "the@219e7f7a-c6b0-4c41-bb8c-4193f02105c8")
      (WordInstanceNode "capital@220cca64-e477-4246-8f2d-f0464f23b3e6")
      (WordInstanceNode "of@864bb7c0-1d2a-480e-9713-3a2e52d6043e")
      (WordInstanceNode "Portugaul@36d41058-3dc8-4e6c-82b9-300a5f48e283")
      (WordInstanceNode ".@4859009a-e89a-44a0-9443-961306fa7af5")
   )
)
(EvaluationLink
   (LinkGrammarRelationshipNode "Ost")
   (ListLink
      (WordInstanceNode "is@234c5c1a-38a0-4728-8c02-09b2068a2c6d")
      (WordInstanceNode "capital@220cca64-e477-4246-8f2d-f0464f23b3e6")
   )
)
(EvaluationLink
   (LinkGrammarRelationshipNode "Mp")
   (ListLink
      (WordInstanceNode "capital@220cca64-e477-4246-8f2d-f0464f23b3e6")
      (WordInstanceNode "of@864bb7c0-1d2a-480e-9713-3a2e52d6043e")
   )
)
(EvaluationLink
   (LinkGrammarRelationshipNode "Js")
   (ListLink
      (WordInstanceNode "of@864bb7c0-1d2a-480e-9713-3a2e52d6043e")
      (WordInstanceNode "Portugaul@36d41058-3dc8-4e6c-82b9-300a5f48e283")
   )
)
(EvaluationLink
   (LinkGrammarRelationshipNode "Xp")
   (ListLink
      (WordInstanceNode "LEFT-WALL")
      (WordInstanceNode ".@4859009a-e89a-44a0-9443-961306fa7af5")
   )
)
(EvaluationLink
   (LinkGrammarRelationshipNode "D*u")
   (ListLink
      (WordInstanceNode "the@219e7f7a-c6b0-4c41-bb8c-4193f02105c8")
      (WordInstanceNode "capital@220cca64-e477-4246-8f2d-f0464f23b3e6")
   )
)
(EvaluationLink
   (LinkGrammarRelationshipNode "Ss")
   (ListLink
      (WordInstanceNode "Lisbon@9d5031b1-5caf-48eb-ae7c-b3a89bb11381")
      (WordInstanceNode "is@234c5c1a-38a0-4728-8c02-09b2068a2c6d")
   )
)
(EvaluationLink
   (LinkGrammarRelationshipNode "Wd")
   (ListLink
      (WordInstanceNode "LEFT-WALL")
      (WordInstanceNode "Lisbon@9d5031b1-5caf-48eb-ae7c-b3a89bb11381")
   )
)
(ParseLink
   (ParseNode "sentence@959f6ccd-5757-4a4e-9254-2351f5696400_parse_0" (stv 1.0 0.9194))
   (SentenceNode "sentence@959f6ccd-5757-4a4e-9254-2351f5696400")
)
(LemmaLink
   (WordInstanceNode "Lisbon@9d5031b1-5caf-48eb-ae7c-b3a89bb11381")
   (WordNode "Lisbon")
)
(LemmaLink
   (WordInstanceNode "is@234c5c1a-38a0-4728-8c02-09b2068a2c6d")
   (WordNode "be")
)
(LemmaLink
   (WordInstanceNode "the@219e7f7a-c6b0-4c41-bb8c-4193f02105c8")
   (WordNode "the")
)
(LemmaLink
   (WordInstanceNode "capital@220cca64-e477-4246-8f2d-f0464f23b3e6")
   (WordNode "capital")
)
(LemmaLink
   (WordInstanceNode "of@864bb7c0-1d2a-480e-9713-3a2e52d6043e")
   (WordNode "of")
)
(LemmaLink
   (WordInstanceNode "Portugaul@36d41058-3dc8-4e6c-82b9-300a5f48e283")
   (WordNode "Portugaul")
)
(LemmaLink
   (WordInstanceNode ".@4859009a-e89a-44a0-9443-961306fa7af5")
   (WordNode ".")
)
; DEFINITE-FLAG (Lisbon, T)
(InheritanceLink
   (WordInstanceNode "Lisbon@9d5031b1-5caf-48eb-ae7c-b3a89bb11381")
   (DefinedLinguisticConceptNode "definite")
)
; noun_number (Lisbon, singular)
(InheritanceLink
   (WordInstanceNode "Lisbon@9d5031b1-5caf-48eb-ae7c-b3a89bb11381")
   (DefinedLinguisticConceptNode "singular")
)
; pos (Lisbon, noun)
(PartOfSpeechLink
   (WordInstanceNode "Lisbon@9d5031b1-5caf-48eb-ae7c-b3a89bb11381")
   (DefinedLinguisticConceptNode "noun")
)
; of (<<capital>>, <<Portugaul>>) 
(EvaluationLink
   (DefinedLinguisticRelationshipNode "of")
   (ListLink
      (WordInstanceNode "capital@220cca64-e477-4246-8f2d-f0464f23b3e6")
      (WordInstanceNode "Portugaul@36d41058-3dc8-4e6c-82b9-300a5f48e283")
   )
)
; noun_number (capital, singular)
(InheritanceLink
   (WordInstanceNode "capital@220cca64-e477-4246-8f2d-f0464f23b3e6")
   (DefinedLinguisticConceptNode "singular")
)
; inflection-TAG (capital, .n)
(InheritanceLink
   (WordInstanceNode "capital@220cca64-e477-4246-8f2d-f0464f23b3e6")
   (DefinedLinguisticConceptNode ".n")
)
; pos (capital, noun)
(PartOfSpeechLink
   (WordInstanceNode "capital@220cca64-e477-4246-8f2d-f0464f23b3e6")
   (DefinedLinguisticConceptNode "noun")
)
; DEFINITE-FLAG (capital, T)
(InheritanceLink
   (WordInstanceNode "capital@220cca64-e477-4246-8f2d-f0464f23b3e6")
   (DefinedLinguisticConceptNode "definite")
)
; pos (of, prep)
(PartOfSpeechLink
   (WordInstanceNode "of@864bb7c0-1d2a-480e-9713-3a2e52d6043e")
   (DefinedLinguisticConceptNode "prep")
)
; DEFINITE-FLAG (Portugaul, T)
(InheritanceLink
   (WordInstanceNode "Portugaul@36d41058-3dc8-4e6c-82b9-300a5f48e283")
   (DefinedLinguisticConceptNode "definite")
)
; noun_number (Portugaul, singular)
(InheritanceLink
   (WordInstanceNode "Portugaul@36d41058-3dc8-4e6c-82b9-300a5f48e283")
   (DefinedLinguisticConceptNode "singular")
)
; pos (Portugaul, noun)
(PartOfSpeechLink
   (WordInstanceNode "Portugaul@36d41058-3dc8-4e6c-82b9-300a5f48e283")
   (DefinedLinguisticConceptNode "noun")
)
; pos (., punctuation)
(PartOfSpeechLink
   (WordInstanceNode ".@4859009a-e89a-44a0-9443-961306fa7af5")
   (DefinedLinguisticConceptNode "punctuation")
)
; _subj (<<be>>, <<Lisbon>>) 
(EvaluationLink
   (DefinedLinguisticRelationshipNode "_subj")
   (ListLink
      (WordInstanceNode "is@234c5c1a-38a0-4728-8c02-09b2068a2c6d")
      (WordInstanceNode "Lisbon@9d5031b1-5caf-48eb-ae7c-b3a89bb11381")
   )
)
; _obj (<<be>>, <<capital>>) 
(EvaluationLink
   (DefinedLinguisticRelationshipNode "_obj")
   (ListLink
      (WordInstanceNode "is@234c5c1a-38a0-4728-8c02-09b2068a2c6d")
      (WordInstanceNode "capital@220cca64-e477-4246-8f2d-f0464f23b3e6")
   )
)
; tense (be, present)
(InheritanceLink
   (WordInstanceNode "is@234c5c1a-38a0-4728-8c02-09b2068a2c6d")
   (DefinedLinguisticConceptNode "present")
)
; inflection-TAG (be, .v)
(InheritanceLink
   (WordInstanceNode "is@234c5c1a-38a0-4728-8c02-09b2068a2c6d")
   (DefinedLinguisticConceptNode ".v")
)
; pos (be, verb)
(PartOfSpeechLink
   (WordInstanceNode "is@234c5c1a-38a0-4728-8c02-09b2068a2c6d")
   (DefinedLinguisticConceptNode "verb")
)
; pos (the, det)
(PartOfSpeechLink
   (WordInstanceNode "the@219e7f7a-c6b0-4c41-bb8c-4193f02105c8")
   (DefinedLinguisticConceptNode "det")
)

; SENTENCE: [The capital of Germany is Berlin.]
(ReferenceLink
   (WordInstanceNode "the@9662bc6b-4f66-4b38-9ab3-3785dd5ca8d5")
   (WordNode "the")
)
(WordInstanceLink
   (WordInstanceNode "the@9662bc6b-4f66-4b38-9ab3-3785dd5ca8d5")
   (ParseNode "sentence@a463469f-ae51-4810-9445-795efda5adf3_parse_0")
)
(ReferenceLink
   (WordInstanceNode "capital@0bd7c26e-ed22-47dd-83fa-dc87ec73b7f8")
   (WordNode "capital")
)
(WordInstanceLink
   (WordInstanceNode "capital@0bd7c26e-ed22-47dd-83fa-dc87ec73b7f8")
   (ParseNode "sentence@a463469f-ae51-4810-9445-795efda5adf3_parse_0")
)
(ReferenceLink
   (WordInstanceNode "of@e04a4e52-edbb-4088-a1a9-faedcd7ea619")
   (WordNode "of")
)
(WordInstanceLink
   (WordInstanceNode "of@e04a4e52-edbb-4088-a1a9-faedcd7ea619")
   (ParseNode "sentence@a463469f-ae51-4810-9445-795efda5adf3_parse_0")
)
(ReferenceLink
   (WordInstanceNode "Germany@3cdca1f8-adf4-4532-a2d6-622da3f43ce6")
   (WordNode "Germany")
)
(WordInstanceLink
   (WordInstanceNode "Germany@3cdca1f8-adf4-4532-a2d6-622da3f43ce6")
   (ParseNode "sentence@a463469f-ae51-4810-9445-795efda5adf3_parse_0")
)
(ReferenceLink
   (WordInstanceNode "is@b3a3eebb-4332-4f3b-b3f1-57531530bad2")
   (WordNode "is")
)
(WordInstanceLink
   (WordInstanceNode "is@b3a3eebb-4332-4f3b-b3f1-57531530bad2")
   (ParseNode "sentence@a463469f-ae51-4810-9445-795efda5adf3_parse_0")
)
(ReferenceLink
   (WordInstanceNode "Berlin@a0d168f7-3735-48a1-b603-33dd0fc95228")
   (WordNode "Berlin")
)
(WordInstanceLink
   (WordInstanceNode "Berlin@a0d168f7-3735-48a1-b603-33dd0fc95228")
   (ParseNode "sentence@a463469f-ae51-4810-9445-795efda5adf3_parse_0")
)
(ReferenceLink
   (WordInstanceNode ".@935a8fcd-68e8-426d-94c7-8ee527b00563")
   (WordNode ".")
)
(WordInstanceLink
   (WordInstanceNode ".@935a8fcd-68e8-426d-94c7-8ee527b00563")
   (ParseNode "sentence@a463469f-ae51-4810-9445-795efda5adf3_parse_0")
)
(ReferenceLink
   (ParseNode "sentence@a463469f-ae51-4810-9445-795efda5adf3_parse_0")
   (ListLink
      (WordInstanceNode "the@9662bc6b-4f66-4b38-9ab3-3785dd5ca8d5")
      (WordInstanceNode "capital@0bd7c26e-ed22-47dd-83fa-dc87ec73b7f8")
      (WordInstanceNode "of@e04a4e52-edbb-4088-a1a9-faedcd7ea619")
      (WordInstanceNode "Germany@3cdca1f8-adf4-4532-a2d6-622da3f43ce6")
      (WordInstanceNode "is@b3a3eebb-4332-4f3b-b3f1-57531530bad2")
      (WordInstanceNode "Berlin@a0d168f7-3735-48a1-b603-33dd0fc95228")
      (WordInstanceNode ".@935a8fcd-68e8-426d-94c7-8ee527b00563")
   )
)
(EvaluationLink
   (LinkGrammarRelationshipNode "Ost")
   (ListLink
      (WordInstanceNode "is@b3a3eebb-4332-4f3b-b3f1-57531530bad2")
      (WordInstanceNode "Berlin@a0d168f7-3735-48a1-b603-33dd0fc95228")
   )
)
(EvaluationLink
   (LinkGrammarRelationshipNode "Xp")
   (ListLink
      (WordInstanceNode "LEFT-WALL")
      (WordInstanceNode ".@935a8fcd-68e8-426d-94c7-8ee527b00563")
   )
)
(EvaluationLink
   (LinkGrammarRelationshipNode "Ss")
   (ListLink
      (WordInstanceNode "capital@0bd7c26e-ed22-47dd-83fa-dc87ec73b7f8")
      (WordInstanceNode "is@b3a3eebb-4332-4f3b-b3f1-57531530bad2")
   )
)
(EvaluationLink
   (LinkGrammarRelationshipNode "Mp")
   (ListLink
      (WordInstanceNode "capital@0bd7c26e-ed22-47dd-83fa-dc87ec73b7f8")
      (WordInstanceNode "of@e04a4e52-edbb-4088-a1a9-faedcd7ea619")
   )
)
(EvaluationLink
   (LinkGrammarRelationshipNode "Js")
   (ListLink
      (WordInstanceNode "of@e04a4e52-edbb-4088-a1a9-faedcd7ea619")
      (WordInstanceNode "Germany@3cdca1f8-adf4-4532-a2d6-622da3f43ce6")
   )
)
(EvaluationLink
   (LinkGrammarRelationshipNode "Wd")
   (ListLink
      (WordInstanceNode "LEFT-WALL")
      (WordInstanceNode "capital@0bd7c26e-ed22-47dd-83fa-dc87ec73b7f8")
   )
)
(EvaluationLink
   (LinkGrammarRelationshipNode "D*u")
   (ListLink
      (WordInstanceNode "the@9662bc6b-4f66-4b38-9ab3-3785dd5ca8d5")
      (WordInstanceNode "capital@0bd7c26e-ed22-47dd-83fa-dc87ec73b7f8")
   )
)
(ParseLink
   (ParseNode "sentence@a463469f-ae51-4810-9445-795efda5adf3_parse_0" (stv 1.0 0.8976))
   (SentenceNode "sentence@a463469f-ae51-4810-9445-795efda5adf3")
)
(LemmaLink
   (WordInstanceNode "the@9662bc6b-4f66-4b38-9ab3-3785dd5ca8d5")
   (WordNode "the")
)
(LemmaLink
   (WordInstanceNode "capital@0bd7c26e-ed22-47dd-83fa-dc87ec73b7f8")
   (WordNode "capital")
)
(LemmaLink
   (WordInstanceNode "of@e04a4e52-edbb-4088-a1a9-faedcd7ea619")
   (WordNode "of")
)
(LemmaLink
   (WordInstanceNode "Germany@3cdca1f8-adf4-4532-a2d6-622da3f43ce6")
   (WordNode "Germany")
)
(LemmaLink
   (WordInstanceNode "is@b3a3eebb-4332-4f3b-b3f1-57531530bad2")
   (WordNode "be")
)
(LemmaLink
   (WordInstanceNode "Berlin@a0d168f7-3735-48a1-b603-33dd0fc95228")
   (WordNode "Berlin")
)
(LemmaLink
   (WordInstanceNode ".@935a8fcd-68e8-426d-94c7-8ee527b00563")
   (WordNode ".")
)
; of (<<capital>>, <<Germany>>) 
(EvaluationLink
   (DefinedLinguisticRelationshipNode "of")
   (ListLink
      (WordInstanceNode "capital@0bd7c26e-ed22-47dd-83fa-dc87ec73b7f8")
      (WordInstanceNode "Germany@3cdca1f8-adf4-4532-a2d6-622da3f43ce6")
   )
)
; noun_number (capital, singular)
(InheritanceLink
   (WordInstanceNode "capital@0bd7c26e-ed22-47dd-83fa-dc87ec73b7f8")
   (DefinedLinguisticConceptNode "singular")
)
; inflection-TAG (capital, .n)
(InheritanceLink
   (WordInstanceNode "capital@0bd7c26e-ed22-47dd-83fa-dc87ec73b7f8")
   (DefinedLinguisticConceptNode ".n")
)
; pos (capital, noun)
(PartOfSpeechLink
   (WordInstanceNode "capital@0bd7c26e-ed22-47dd-83fa-dc87ec73b7f8")
   (DefinedLinguisticConceptNode "noun")
)
; DEFINITE-FLAG (capital, T)
(InheritanceLink
   (WordInstanceNode "capital@0bd7c26e-ed22-47dd-83fa-dc87ec73b7f8")
   (DefinedLinguisticConceptNode "definite")
)
; pos (of, prep)
(PartOfSpeechLink
   (WordInstanceNode "of@e04a4e52-edbb-4088-a1a9-faedcd7ea619")
   (DefinedLinguisticConceptNode "prep")
)
; DEFINITE-FLAG (Germany, T)
(InheritanceLink
   (WordInstanceNode "Germany@3cdca1f8-adf4-4532-a2d6-622da3f43ce6")
   (DefinedLinguisticConceptNode "definite")
)
; noun_number (Germany, singular)
(InheritanceLink
   (WordInstanceNode "Germany@3cdca1f8-adf4-4532-a2d6-622da3f43ce6")
   (DefinedLinguisticConceptNode "singular")
)
; inflection-TAG (Germany, .l)
(InheritanceLink
   (WordInstanceNode "Germany@3cdca1f8-adf4-4532-a2d6-622da3f43ce6")
   (DefinedLinguisticConceptNode ".l")
)
; location-FLAG (Germany, T)
(InheritanceLink
   (WordInstanceNode "Germany@3cdca1f8-adf4-4532-a2d6-622da3f43ce6")
   (DefinedLinguisticConceptNode "location")
)
; pos (Germany, noun)
(PartOfSpeechLink
   (WordInstanceNode "Germany@3cdca1f8-adf4-4532-a2d6-622da3f43ce6")
   (DefinedLinguisticConceptNode "noun")
)
; DEFINITE-FLAG (Berlin, T)
(InheritanceLink
   (WordInstanceNode "Berlin@a0d168f7-3735-48a1-b603-33dd0fc95228")
   (DefinedLinguisticConceptNode "definite")
)
; noun_number (Berlin, singular)
(InheritanceLink
   (WordInstanceNode "Berlin@a0d168f7-3735-48a1-b603-33dd0fc95228")
   (DefinedLinguisticConceptNode "singular")
)
; pos (Berlin, noun)
(PartOfSpeechLink
   (WordInstanceNode "Berlin@a0d168f7-3735-48a1-b603-33dd0fc95228")
   (DefinedLinguisticConceptNode "noun")
)
; pos (., punctuation)
(PartOfSpeechLink
   (WordInstanceNode ".@935a8fcd-68e8-426d-94c7-8ee527b00563")
   (DefinedLinguisticConceptNode "punctuation")
)
; _subj (<<be>>, <<capital>>) 
(EvaluationLink
   (DefinedLinguisticRelationshipNode "_subj")
   (ListLink
      (WordInstanceNode "is@b3a3eebb-4332-4f3b-b3f1-57531530bad2")
      (WordInstanceNode "capital@0bd7c26e-ed22-47dd-83fa-dc87ec73b7f8")
   )
)
; _obj (<<be>>, <<Berlin>>) 
(EvaluationLink
   (DefinedLinguisticRelationshipNode "_obj")
   (ListLink
      (WordInstanceNode "is@b3a3eebb-4332-4f3b-b3f1-57531530bad2")
      (WordInstanceNode "Berlin@a0d168f7-3735-48a1-b603-33dd0fc95228")
   )
)
; tense (be, present)
(InheritanceLink
   (WordInstanceNode "is@b3a3eebb-4332-4f3b-b3f1-57531530bad2")
   (DefinedLinguisticConceptNode "present")
)
; inflection-TAG (be, .v)
(InheritanceLink
   (WordInstanceNode "is@b3a3eebb-4332-4f3b-b3f1-57531530bad2")
   (DefinedLinguisticConceptNode ".v")
)
; pos (be, verb)
(PartOfSpeechLink
   (WordInstanceNode "is@b3a3eebb-4332-4f3b-b3f1-57531530bad2")
   (DefinedLinguisticConceptNode "verb")
)
; pos (the, det)
(PartOfSpeechLink
   (WordInstanceNode "the@9662bc6b-4f66-4b38-9ab3-3785dd5ca8d5")
   (DefinedLinguisticConceptNode "det")
)

; SENTENCE: [The color of the sky is blue.]
(ReferenceLink
   (WordInstanceNode "the@153bd1f8-c9c0-402f-90d8-6f761a837103")
   (WordNode "the")
)
(WordInstanceLink
   (WordInstanceNode "the@153bd1f8-c9c0-402f-90d8-6f761a837103")
   (ParseNode "sentence@2477ad26-eff2-4e70-8000-5484edc874bc_parse_0")
)
(ReferenceLink
   (WordInstanceNode "color@a3f9d2c9-3214-491b-8694-e8f8e62067b2")
   (WordNode "color")
)
(WordInstanceLink
   (WordInstanceNode "color@a3f9d2c9-3214-491b-8694-e8f8e62067b2")
   (ParseNode "sentence@2477ad26-eff2-4e70-8000-5484edc874bc_parse_0")
)
(ReferenceLink
   (WordInstanceNode "of@745e0871-45dd-4405-805d-a2fb0b90fc3e")
   (WordNode "of")
)
(WordInstanceLink
   (WordInstanceNode "of@745e0871-45dd-4405-805d-a2fb0b90fc3e")
   (ParseNode "sentence@2477ad26-eff2-4e70-8000-5484edc874bc_parse_0")
)
(ReferenceLink
   (WordInstanceNode "the@c55cd923-108e-43d0-8ad5-1072d5ec62b0")
   (WordNode "the")
)
(WordInstanceLink
   (WordInstanceNode "the@c55cd923-108e-43d0-8ad5-1072d5ec62b0")
   (ParseNode "sentence@2477ad26-eff2-4e70-8000-5484edc874bc_parse_0")
)
(ReferenceLink
   (WordInstanceNode "sky@f3924dad-81a0-4967-81c8-900e31254f74")
   (WordNode "sky")
)
(WordInstanceLink
   (WordInstanceNode "sky@f3924dad-81a0-4967-81c8-900e31254f74")
   (ParseNode "sentence@2477ad26-eff2-4e70-8000-5484edc874bc_parse_0")
)
(ReferenceLink
   (WordInstanceNode "is@ddaf36ff-4c6b-44f1-8eb9-4d0e0e8e9071")
   (WordNode "is")
)
(WordInstanceLink
   (WordInstanceNode "is@ddaf36ff-4c6b-44f1-8eb9-4d0e0e8e9071")
   (ParseNode "sentence@2477ad26-eff2-4e70-8000-5484edc874bc_parse_0")
)
(ReferenceLink
   (WordInstanceNode "blue@df54a7e9-1fb2-4643-be97-ca6ee1e7a359")
   (WordNode "blue")
)
(WordInstanceLink
   (WordInstanceNode "blue@df54a7e9-1fb2-4643-be97-ca6ee1e7a359")
   (ParseNode "sentence@2477ad26-eff2-4e70-8000-5484edc874bc_parse_0")
)
(ReferenceLink
   (WordInstanceNode ".@f586c728-66e5-4096-b20a-7c4b7d6bdd41")
   (WordNode ".")
)
(WordInstanceLink
   (WordInstanceNode ".@f586c728-66e5-4096-b20a-7c4b7d6bdd41")
   (ParseNode "sentence@2477ad26-eff2-4e70-8000-5484edc874bc_parse_0")
)
(ReferenceLink
   (ParseNode "sentence@2477ad26-eff2-4e70-8000-5484edc874bc_parse_0")
   (ListLink
      (WordInstanceNode "the@153bd1f8-c9c0-402f-90d8-6f761a837103")
      (WordInstanceNode "color@a3f9d2c9-3214-491b-8694-e8f8e62067b2")
      (WordInstanceNode "of@745e0871-45dd-4405-805d-a2fb0b90fc3e")
      (WordInstanceNode "the@c55cd923-108e-43d0-8ad5-1072d5ec62b0")
      (WordInstanceNode "sky@f3924dad-81a0-4967-81c8-900e31254f74")
      (WordInstanceNode "is@ddaf36ff-4c6b-44f1-8eb9-4d0e0e8e9071")
      (WordInstanceNode "blue@df54a7e9-1fb2-4643-be97-ca6ee1e7a359")
      (WordInstanceNode ".@f586c728-66e5-4096-b20a-7c4b7d6bdd41")
   )
)
(EvaluationLink
   (LinkGrammarRelationshipNode "Ss")
   (ListLink
      (WordInstanceNode "color@a3f9d2c9-3214-491b-8694-e8f8e62067b2")
      (WordInstanceNode "is@ddaf36ff-4c6b-44f1-8eb9-4d0e0e8e9071")
   )
)
(EvaluationLink
   (LinkGrammarRelationshipNode "Pa")
   (ListLink
      (WordInstanceNode "is@ddaf36ff-4c6b-44f1-8eb9-4d0e0e8e9071")
      (WordInstanceNode "blue@df54a7e9-1fb2-4643-be97-ca6ee1e7a359")
   )
)
(EvaluationLink
   (LinkGrammarRelationshipNode "Mp")
   (ListLink
      (WordInstanceNode "color@a3f9d2c9-3214-491b-8694-e8f8e62067b2")
      (WordInstanceNode "of@745e0871-45dd-4405-805d-a2fb0b90fc3e")
   )
)
(EvaluationLink
   (LinkGrammarRelationshipNode "Jp")
   (ListLink
      (WordInstanceNode "of@745e0871-45dd-4405-805d-a2fb0b90fc3e")
      (WordInstanceNode "sky@f3924dad-81a0-4967-81c8-900e31254f74")
   )
)
(EvaluationLink
   (LinkGrammarRelationshipNode "Dmu")
   (ListLink
      (WordInstanceNode "the@c55cd923-108e-43d0-8ad5-1072d5ec62b0")
      (WordInstanceNode "sky@f3924dad-81a0-4967-81c8-900e31254f74")
   )
)
(EvaluationLink
   (LinkGrammarRelationshipNode "Wd")
   (ListLink
      (WordInstanceNode "LEFT-WALL")
      (WordInstanceNode "color@a3f9d2c9-3214-491b-8694-e8f8e62067b2")
   )
)
(EvaluationLink
   (LinkGrammarRelationshipNode "D*u")
   (ListLink
      (WordInstanceNode "the@153bd1f8-c9c0-402f-90d8-6f761a837103")
      (WordInstanceNode "color@a3f9d2c9-3214-491b-8694-e8f8e62067b2")
   )
)
(EvaluationLink
   (LinkGrammarRelationshipNode "Xp")
   (ListLink
      (WordInstanceNode "LEFT-WALL")
      (WordInstanceNode ".@f586c728-66e5-4096-b20a-7c4b7d6bdd41")
   )
)
(ParseLink
   (ParseNode "sentence@2477ad26-eff2-4e70-8000-5484edc874bc_parse_0" (stv 1.0 0.8658))
   (SentenceNode "sentence@2477ad26-eff2-4e70-8000-5484edc874bc")
)
(LemmaLink
   (WordInstanceNode "the@153bd1f8-c9c0-402f-90d8-6f761a837103")
   (WordNode "the")
)
(LemmaLink
   (WordInstanceNode "color@a3f9d2c9-3214-491b-8694-e8f8e62067b2")
   (WordNode "color")
)
(LemmaLink
   (WordInstanceNode "of@745e0871-45dd-4405-805d-a2fb0b90fc3e")
   (WordNode "of")
)
(LemmaLink
   (WordInstanceNode "the@c55cd923-108e-43d0-8ad5-1072d5ec62b0")
   (WordNode "the")
)
(LemmaLink
   (WordInstanceNode "sky@f3924dad-81a0-4967-81c8-900e31254f74")
   (WordNode "sky")
)
(LemmaLink
   (WordInstanceNode "is@ddaf36ff-4c6b-44f1-8eb9-4d0e0e8e9071")
   (WordNode "be")
)
(LemmaLink
   (WordInstanceNode "blue@df54a7e9-1fb2-4643-be97-ca6ee1e7a359")
   (WordNode "blue")
)
(LemmaLink
   (WordInstanceNode ".@f586c728-66e5-4096-b20a-7c4b7d6bdd41")
   (WordNode ".")
)
; of (<<color>>, <<sky>>) 
(EvaluationLink
   (DefinedLinguisticRelationshipNode "of")
   (ListLink
      (WordInstanceNode "color@a3f9d2c9-3214-491b-8694-e8f8e62067b2")
      (WordInstanceNode "sky@f3924dad-81a0-4967-81c8-900e31254f74")
   )
)
; noun_number (color, singular)
(InheritanceLink
   (WordInstanceNode "color@a3f9d2c9-3214-491b-8694-e8f8e62067b2")
   (DefinedLinguisticConceptNode "singular")
)
; inflection-TAG (color, .n)
(InheritanceLink
   (WordInstanceNode "color@a3f9d2c9-3214-491b-8694-e8f8e62067b2")
   (DefinedLinguisticConceptNode ".n")
)
; pos (color, noun)
(PartOfSpeechLink
   (WordInstanceNode "color@a3f9d2c9-3214-491b-8694-e8f8e62067b2")
   (DefinedLinguisticConceptNode "noun")
)
; DEFINITE-FLAG (color, T)
(InheritanceLink
   (WordInstanceNode "color@a3f9d2c9-3214-491b-8694-e8f8e62067b2")
   (DefinedLinguisticConceptNode "definite")
)
; _predadj (<<blue>>, <<color>>) 
(EvaluationLink
   (DefinedLinguisticRelationshipNode "_predadj")
   (ListLink
      (WordInstanceNode "blue@df54a7e9-1fb2-4643-be97-ca6ee1e7a359")
      (WordInstanceNode "color@a3f9d2c9-3214-491b-8694-e8f8e62067b2")
   )
)
; tense (blue, present)
(InheritanceLink
   (WordInstanceNode "blue@df54a7e9-1fb2-4643-be97-ca6ee1e7a359")
   (DefinedLinguisticConceptNode "present")
)
; inflection-TAG (blue, .a)
(InheritanceLink
   (WordInstanceNode "blue@df54a7e9-1fb2-4643-be97-ca6ee1e7a359")
   (DefinedLinguisticConceptNode ".a")
)
; pos (blue, adj)
(PartOfSpeechLink
   (WordInstanceNode "blue@df54a7e9-1fb2-4643-be97-ca6ee1e7a359")
   (DefinedLinguisticConceptNode "adj")
)
; pos (of, prep)
(PartOfSpeechLink
   (WordInstanceNode "of@745e0871-45dd-4405-805d-a2fb0b90fc3e")
   (DefinedLinguisticConceptNode "prep")
)
; DEFINITE-FLAG (sky, T)
(InheritanceLink
   (WordInstanceNode "sky@f3924dad-81a0-4967-81c8-900e31254f74")
   (DefinedLinguisticConceptNode "definite")
)
; noun_number (sky, uncountable)
(InheritanceLink
   (WordInstanceNode "sky@f3924dad-81a0-4967-81c8-900e31254f74")
   (DefinedLinguisticConceptNode "uncountable")
)
; inflection-TAG (sky, .n)
(InheritanceLink
   (WordInstanceNode "sky@f3924dad-81a0-4967-81c8-900e31254f74")
   (DefinedLinguisticConceptNode ".n")
)
; pos (sky, noun)
(PartOfSpeechLink
   (WordInstanceNode "sky@f3924dad-81a0-4967-81c8-900e31254f74")
   (DefinedLinguisticConceptNode "noun")
)
; inflection-TAG (be, .v)
(InheritanceLink
   (WordInstanceNode "is@ddaf36ff-4c6b-44f1-8eb9-4d0e0e8e9071")
   (DefinedLinguisticConceptNode ".v")
)
; pos (be, verb)
(PartOfSpeechLink
   (WordInstanceNode "is@ddaf36ff-4c6b-44f1-8eb9-4d0e0e8e9071")
   (DefinedLinguisticConceptNode "verb")
)
; pos (the, det)
(PartOfSpeechLink
   (WordInstanceNode "the@c55cd923-108e-43d0-8ad5-1072d5ec62b0")
   (DefinedLinguisticConceptNode "det")
)
; pos (., punctuation)
(PartOfSpeechLink
   (WordInstanceNode ".@f586c728-66e5-4096-b20a-7c4b7d6bdd41")
   (DefinedLinguisticConceptNode "punctuation")
)
; pos (the, det)
(PartOfSpeechLink
   (WordInstanceNode "the@153bd1f8-c9c0-402f-90d8-6f761a837103")
   (DefinedLinguisticConceptNode "det")
)

; SENTENCE: [Pottery is made from clay.]
(ReferenceLink
   (WordInstanceNode "pottery@419fc4ef-99f8-4b08-a6a4-5f8cc8e31583")
   (WordNode "pottery")
)
(WordInstanceLink
   (WordInstanceNode "pottery@419fc4ef-99f8-4b08-a6a4-5f8cc8e31583")
   (ParseNode "sentence@baec3da3-4ce6-4579-9e3a-137de7f3f773_parse_0")
)
(ReferenceLink
   (WordInstanceNode "is@28ad9a1d-53fb-415e-a520-eac2df9598a0")
   (WordNode "is")
)
(WordInstanceLink
   (WordInstanceNode "is@28ad9a1d-53fb-415e-a520-eac2df9598a0")
   (ParseNode "sentence@baec3da3-4ce6-4579-9e3a-137de7f3f773_parse_0")
)
(ReferenceLink
   (WordInstanceNode "made@6caed8ac-8403-4b47-ba50-b49d7099542b")
   (WordNode "made")
)
(WordInstanceLink
   (WordInstanceNode "made@6caed8ac-8403-4b47-ba50-b49d7099542b")
   (ParseNode "sentence@baec3da3-4ce6-4579-9e3a-137de7f3f773_parse_0")
)
(ReferenceLink
   (WordInstanceNode "from@699101df-e689-4e3a-a064-ae67375bf78d")
   (WordNode "from")
)
(WordInstanceLink
   (WordInstanceNode "from@699101df-e689-4e3a-a064-ae67375bf78d")
   (ParseNode "sentence@baec3da3-4ce6-4579-9e3a-137de7f3f773_parse_0")
)
(ReferenceLink
   (WordInstanceNode "clay@dfb83438-7e85-4d86-babc-de25e875c145")
   (WordNode "clay")
)
(WordInstanceLink
   (WordInstanceNode "clay@dfb83438-7e85-4d86-babc-de25e875c145")
   (ParseNode "sentence@baec3da3-4ce6-4579-9e3a-137de7f3f773_parse_0")
)
(ReferenceLink
   (WordInstanceNode ".@e48ed737-67a3-44ca-a89c-4bb7a483cf3f")
   (WordNode ".")
)
(WordInstanceLink
   (WordInstanceNode ".@e48ed737-67a3-44ca-a89c-4bb7a483cf3f")
   (ParseNode "sentence@baec3da3-4ce6-4579-9e3a-137de7f3f773_parse_0")
)
(ReferenceLink
   (ParseNode "sentence@baec3da3-4ce6-4579-9e3a-137de7f3f773_parse_0")
   (ListLink
      (WordInstanceNode "pottery@419fc4ef-99f8-4b08-a6a4-5f8cc8e31583")
      (WordInstanceNode "is@28ad9a1d-53fb-415e-a520-eac2df9598a0")
      (WordInstanceNode "made@6caed8ac-8403-4b47-ba50-b49d7099542b")
      (WordInstanceNode "from@699101df-e689-4e3a-a064-ae67375bf78d")
      (WordInstanceNode "clay@dfb83438-7e85-4d86-babc-de25e875c145")
      (WordInstanceNode ".@e48ed737-67a3-44ca-a89c-4bb7a483cf3f")
   )
)
(EvaluationLink
   (LinkGrammarRelationshipNode "MVp")
   (ListLink
      (WordInstanceNode "made@6caed8ac-8403-4b47-ba50-b49d7099542b")
      (WordInstanceNode "from@699101df-e689-4e3a-a064-ae67375bf78d")
   )
)
(EvaluationLink
   (LinkGrammarRelationshipNode "Jp")
   (ListLink
      (WordInstanceNode "from@699101df-e689-4e3a-a064-ae67375bf78d")
      (WordInstanceNode "clay@dfb83438-7e85-4d86-babc-de25e875c145")
   )
)
(EvaluationLink
   (LinkGrammarRelationshipNode "Xp")
   (ListLink
      (WordInstanceNode "LEFT-WALL")
      (WordInstanceNode ".@e48ed737-67a3-44ca-a89c-4bb7a483cf3f")
   )
)
(EvaluationLink
   (LinkGrammarRelationshipNode "Pvf")
   (ListLink
      (WordInstanceNode "is@28ad9a1d-53fb-415e-a520-eac2df9598a0")
      (WordInstanceNode "made@6caed8ac-8403-4b47-ba50-b49d7099542b")
   )
)
(EvaluationLink
   (LinkGrammarRelationshipNode "Ss")
   (ListLink
      (WordInstanceNode "pottery@419fc4ef-99f8-4b08-a6a4-5f8cc8e31583")
      (WordInstanceNode "is@28ad9a1d-53fb-415e-a520-eac2df9598a0")
   )
)
(EvaluationLink
   (LinkGrammarRelationshipNode "Wd")
   (ListLink
      (WordInstanceNode "LEFT-WALL")
      (WordInstanceNode "pottery@419fc4ef-99f8-4b08-a6a4-5f8cc8e31583")
   )
)
(ParseLink
   (ParseNode "sentence@baec3da3-4ce6-4579-9e3a-137de7f3f773_parse_0" (stv 1.0 0.9417))
   (SentenceNode "sentence@baec3da3-4ce6-4579-9e3a-137de7f3f773")
)
(LemmaLink
   (WordInstanceNode "pottery@419fc4ef-99f8-4b08-a6a4-5f8cc8e31583")
   (WordNode "pottery")
)
(LemmaLink
   (WordInstanceNode "is@28ad9a1d-53fb-415e-a520-eac2df9598a0")
   (WordNode "be")
)
(LemmaLink
   (WordInstanceNode "made@6caed8ac-8403-4b47-ba50-b49d7099542b")
   (WordNode "make")
)
(LemmaLink
   (WordInstanceNode "from@699101df-e689-4e3a-a064-ae67375bf78d")
   (WordNode "from")
)
(LemmaLink
   (WordInstanceNode "clay@dfb83438-7e85-4d86-babc-de25e875c145")
   (WordNode "clay")
)
(LemmaLink
   (WordInstanceNode ".@e48ed737-67a3-44ca-a89c-4bb7a483cf3f")
   (WordNode ".")
)
; pos (from, prep)
(PartOfSpeechLink
   (WordInstanceNode "from@699101df-e689-4e3a-a064-ae67375bf78d")
   (DefinedLinguisticConceptNode "prep")
)
; inflection-TAG (clay, .n)
(InheritanceLink
   (WordInstanceNode "clay@dfb83438-7e85-4d86-babc-de25e875c145")
   (DefinedLinguisticConceptNode ".n")
)
; pos (clay, noun)
(PartOfSpeechLink
   (WordInstanceNode "clay@dfb83438-7e85-4d86-babc-de25e875c145")
   (DefinedLinguisticConceptNode "noun")
)
; pos (., punctuation)
(PartOfSpeechLink
   (WordInstanceNode ".@e48ed737-67a3-44ca-a89c-4bb7a483cf3f")
   (DefinedLinguisticConceptNode "punctuation")
)
; noun_number (pottery, uncountable)
(InheritanceLink
   (WordInstanceNode "pottery@419fc4ef-99f8-4b08-a6a4-5f8cc8e31583")
   (DefinedLinguisticConceptNode "uncountable")
)
; inflection-TAG (pottery, .n)
(InheritanceLink
   (WordInstanceNode "pottery@419fc4ef-99f8-4b08-a6a4-5f8cc8e31583")
   (DefinedLinguisticConceptNode ".n")
)
; pos (pottery, noun)
(PartOfSpeechLink
   (WordInstanceNode "pottery@419fc4ef-99f8-4b08-a6a4-5f8cc8e31583")
   (DefinedLinguisticConceptNode "noun")
)
; inflection-TAG (be, .v)
(InheritanceLink
   (WordInstanceNode "is@28ad9a1d-53fb-415e-a520-eac2df9598a0")
   (DefinedLinguisticConceptNode ".v")
)
; pos (be, verb)
(PartOfSpeechLink
   (WordInstanceNode "is@28ad9a1d-53fb-415e-a520-eac2df9598a0")
   (DefinedLinguisticConceptNode "verb")
)
; _obj (<<make>>, <<pottery>>) 
(EvaluationLink
   (DefinedLinguisticRelationshipNode "_obj")
   (ListLink
      (WordInstanceNode "made@6caed8ac-8403-4b47-ba50-b49d7099542b")
      (WordInstanceNode "pottery@419fc4ef-99f8-4b08-a6a4-5f8cc8e31583")
   )
)
; from (<<make>>, <<clay>>) 
(EvaluationLink
   (DefinedLinguisticRelationshipNode "from")
   (ListLink
      (WordInstanceNode "made@6caed8ac-8403-4b47-ba50-b49d7099542b")
      (WordInstanceNode "clay@dfb83438-7e85-4d86-babc-de25e875c145")
   )
)
; tense (make, present_passive)
(InheritanceLink
   (WordInstanceNode "made@6caed8ac-8403-4b47-ba50-b49d7099542b")
   (DefinedLinguisticConceptNode "present_passive")
)
; inflection-TAG (make, .v)
(InheritanceLink
   (WordInstanceNode "made@6caed8ac-8403-4b47-ba50-b49d7099542b")
   (DefinedLinguisticConceptNode ".v")
)
; pos (make, verb)
(PartOfSpeechLink
   (WordInstanceNode "made@6caed8ac-8403-4b47-ba50-b49d7099542b")
   (DefinedLinguisticConceptNode "verb")
)

; SENTENCE: [Yarn is spun from fibers.]
(ReferenceLink
   (WordInstanceNode "yarn@02740eaf-cbf8-4cbe-b8ca-7fa883ee9b8e")
   (WordNode "yarn")
)
(WordInstanceLink
   (WordInstanceNode "yarn@02740eaf-cbf8-4cbe-b8ca-7fa883ee9b8e")
   (ParseNode "sentence@ce7dbf6b-eaf5-45a9-864c-1c3976e6a06c_parse_0")
)
(ReferenceLink
   (WordInstanceNode "is@2dd654ad-3c4c-41ce-b2c8-54cb5849af53")
   (WordNode "is")
)
(WordInstanceLink
   (WordInstanceNode "is@2dd654ad-3c4c-41ce-b2c8-54cb5849af53")
   (ParseNode "sentence@ce7dbf6b-eaf5-45a9-864c-1c3976e6a06c_parse_0")
)
(ReferenceLink
   (WordInstanceNode "spun@cf20b39e-927a-473c-9082-a74ead225fa5")
   (WordNode "spun")
)
(WordInstanceLink
   (WordInstanceNode "spun@cf20b39e-927a-473c-9082-a74ead225fa5")
   (ParseNode "sentence@ce7dbf6b-eaf5-45a9-864c-1c3976e6a06c_parse_0")
)
(ReferenceLink
   (WordInstanceNode "from@e2000d5e-26c6-4cc1-9133-6c334a91d440")
   (WordNode "from")
)
(WordInstanceLink
   (WordInstanceNode "from@e2000d5e-26c6-4cc1-9133-6c334a91d440")
   (ParseNode "sentence@ce7dbf6b-eaf5-45a9-864c-1c3976e6a06c_parse_0")
)
(ReferenceLink
   (WordInstanceNode "fibers[!]@3d627c9a-efdd-495b-9aa7-2064b20bbc84")
   (WordNode "fibers[!]")
)
(WordInstanceLink
   (WordInstanceNode "fibers[!]@3d627c9a-efdd-495b-9aa7-2064b20bbc84")
   (ParseNode "sentence@ce7dbf6b-eaf5-45a9-864c-1c3976e6a06c_parse_0")
)
(ReferenceLink
   (WordInstanceNode ".@633f9e69-2ad5-4086-bc3b-2dac2d841c8c")
   (WordNode ".")
)
(WordInstanceLink
   (WordInstanceNode ".@633f9e69-2ad5-4086-bc3b-2dac2d841c8c")
   (ParseNode "sentence@ce7dbf6b-eaf5-45a9-864c-1c3976e6a06c_parse_0")
)
(ReferenceLink
   (ParseNode "sentence@ce7dbf6b-eaf5-45a9-864c-1c3976e6a06c_parse_0")
   (ListLink
      (WordInstanceNode "yarn@02740eaf-cbf8-4cbe-b8ca-7fa883ee9b8e")
      (WordInstanceNode "is@2dd654ad-3c4c-41ce-b2c8-54cb5849af53")
      (WordInstanceNode "spun@cf20b39e-927a-473c-9082-a74ead225fa5")
      (WordInstanceNode "from@e2000d5e-26c6-4cc1-9133-6c334a91d440")
      (WordInstanceNode "fibers[!]@3d627c9a-efdd-495b-9aa7-2064b20bbc84")
      (WordInstanceNode ".@633f9e69-2ad5-4086-bc3b-2dac2d841c8c")
   )
)
(EvaluationLink
   (LinkGrammarRelationshipNode "MVp")
   (ListLink
      (WordInstanceNode "spun@cf20b39e-927a-473c-9082-a74ead225fa5")
      (WordInstanceNode "from@e2000d5e-26c6-4cc1-9133-6c334a91d440")
   )
)
(EvaluationLink
   (LinkGrammarRelationshipNode "Jp")
   (ListLink
      (WordInstanceNode "from@e2000d5e-26c6-4cc1-9133-6c334a91d440")
      (WordInstanceNode "fibers[!]@3d627c9a-efdd-495b-9aa7-2064b20bbc84")
   )
)
(EvaluationLink
   (LinkGrammarRelationshipNode "Xp")
   (ListLink
      (WordInstanceNode "LEFT-WALL")
      (WordInstanceNode ".@633f9e69-2ad5-4086-bc3b-2dac2d841c8c")
   )
)
(EvaluationLink
   (LinkGrammarRelationshipNode "Pv")
   (ListLink
      (WordInstanceNode "is@2dd654ad-3c4c-41ce-b2c8-54cb5849af53")
      (WordInstanceNode "spun@cf20b39e-927a-473c-9082-a74ead225fa5")
   )
)
(EvaluationLink
   (LinkGrammarRelationshipNode "Ss")
   (ListLink
      (WordInstanceNode "yarn@02740eaf-cbf8-4cbe-b8ca-7fa883ee9b8e")
      (WordInstanceNode "is@2dd654ad-3c4c-41ce-b2c8-54cb5849af53")
   )
)
(EvaluationLink
   (LinkGrammarRelationshipNode "Wd")
   (ListLink
      (WordInstanceNode "LEFT-WALL")
      (WordInstanceNode "yarn@02740eaf-cbf8-4cbe-b8ca-7fa883ee9b8e")
   )
)
(ParseLink
   (ParseNode "sentence@ce7dbf6b-eaf5-45a9-864c-1c3976e6a06c_parse_0" (stv 1.0 0.9417))
   (SentenceNode "sentence@ce7dbf6b-eaf5-45a9-864c-1c3976e6a06c")
)
(LemmaLink
   (WordInstanceNode "yarn@02740eaf-cbf8-4cbe-b8ca-7fa883ee9b8e")
   (WordNode "yarn")
)
(LemmaLink
   (WordInstanceNode "is@2dd654ad-3c4c-41ce-b2c8-54cb5849af53")
   (WordNode "be")
)
(LemmaLink
   (WordInstanceNode "spun@cf20b39e-927a-473c-9082-a74ead225fa5")
   (WordNode "spin")
)
(LemmaLink
   (WordInstanceNode "from@e2000d5e-26c6-4cc1-9133-6c334a91d440")
   (WordNode "from")
)
(LemmaLink
   (WordInstanceNode "fibers[!]@3d627c9a-efdd-495b-9aa7-2064b20bbc84")
   (WordNode "fiber")
)
(LemmaLink
   (WordInstanceNode ".@633f9e69-2ad5-4086-bc3b-2dac2d841c8c")
   (WordNode ".")
)
; pos (from, prep)
(PartOfSpeechLink
   (WordInstanceNode "from@e2000d5e-26c6-4cc1-9133-6c334a91d440")
   (DefinedLinguisticConceptNode "prep")
)
; noun_number (fiber, plural)
(InheritanceLink
   (WordInstanceNode "fibers[!]@3d627c9a-efdd-495b-9aa7-2064b20bbc84")
   (DefinedLinguisticConceptNode "plural")
)
; inflection-TAG (fiber, .n)
(InheritanceLink
   (WordInstanceNode "fibers[!]@3d627c9a-efdd-495b-9aa7-2064b20bbc84")
   (DefinedLinguisticConceptNode ".n")
)
; pos (fiber, noun)
(PartOfSpeechLink
   (WordInstanceNode "fibers[!]@3d627c9a-efdd-495b-9aa7-2064b20bbc84")
   (DefinedLinguisticConceptNode "noun")
)
; pos (., punctuation)
(PartOfSpeechLink
   (WordInstanceNode ".@633f9e69-2ad5-4086-bc3b-2dac2d841c8c")
   (DefinedLinguisticConceptNode "punctuation")
)
; noun_number (yarn, uncountable)
(InheritanceLink
   (WordInstanceNode "yarn@02740eaf-cbf8-4cbe-b8ca-7fa883ee9b8e")
   (DefinedLinguisticConceptNode "uncountable")
)
; inflection-TAG (yarn, .n)
(InheritanceLink
   (WordInstanceNode "yarn@02740eaf-cbf8-4cbe-b8ca-7fa883ee9b8e")
   (DefinedLinguisticConceptNode ".n")
)
; pos (yarn, noun)
(PartOfSpeechLink
   (WordInstanceNode "yarn@02740eaf-cbf8-4cbe-b8ca-7fa883ee9b8e")
   (DefinedLinguisticConceptNode "noun")
)
; inflection-TAG (be, .v)
(InheritanceLink
   (WordInstanceNode "is@2dd654ad-3c4c-41ce-b2c8-54cb5849af53")
   (DefinedLinguisticConceptNode ".v")
)
; pos (be, verb)
(PartOfSpeechLink
   (WordInstanceNode "is@2dd654ad-3c4c-41ce-b2c8-54cb5849af53")
   (DefinedLinguisticConceptNode "verb")
)
; _obj (<<spin>>, <<yarn>>) 
(EvaluationLink
   (DefinedLinguisticRelationshipNode "_obj")
   (ListLink
      (WordInstanceNode "spun@cf20b39e-927a-473c-9082-a74ead225fa5")
      (WordInstanceNode "yarn@02740eaf-cbf8-4cbe-b8ca-7fa883ee9b8e")
   )
)
; from (<<spin>>, <<fiber>>) 
(EvaluationLink
   (DefinedLinguisticRelationshipNode "from")
   (ListLink
      (WordInstanceNode "spun@cf20b39e-927a-473c-9082-a74ead225fa5")
      (WordInstanceNode "fibers[!]@3d627c9a-efdd-495b-9aa7-2064b20bbc84")
   )
)
; tense (spin, present_passive)
(InheritanceLink
   (WordInstanceNode "spun@cf20b39e-927a-473c-9082-a74ead225fa5")
   (DefinedLinguisticConceptNode "present_passive")
)
; inflection-TAG (spin, .v)
(InheritanceLink
   (WordInstanceNode "spun@cf20b39e-927a-473c-9082-a74ead225fa5")
   (DefinedLinguisticConceptNode ".v")
)
; pos (spin, verb)
(PartOfSpeechLink
   (WordInstanceNode "spun@cf20b39e-927a-473c-9082-a74ead225fa5")
   (DefinedLinguisticConceptNode "verb")
)

; SENTENCE: [Yarn is made of fibers.]
(ReferenceLink
   (WordInstanceNode "yarn@cf5ec4bb-a377-48d3-8d11-c3d8d5260b24")
   (WordNode "yarn")
)
(WordInstanceLink
   (WordInstanceNode "yarn@cf5ec4bb-a377-48d3-8d11-c3d8d5260b24")
   (ParseNode "sentence@9edd6ffe-8670-46a9-9c63-39c9637c577e_parse_0")
)
(ReferenceLink
   (WordInstanceNode "is@ddb49869-eb1a-4ba6-8242-b3fcb79f90bb")
   (WordNode "is")
)
(WordInstanceLink
   (WordInstanceNode "is@ddb49869-eb1a-4ba6-8242-b3fcb79f90bb")
   (ParseNode "sentence@9edd6ffe-8670-46a9-9c63-39c9637c577e_parse_0")
)
(ReferenceLink
   (WordInstanceNode "made@dedbc1c5-9b42-4593-88ac-c13b8537370b")
   (WordNode "made")
)
(WordInstanceLink
   (WordInstanceNode "made@dedbc1c5-9b42-4593-88ac-c13b8537370b")
   (ParseNode "sentence@9edd6ffe-8670-46a9-9c63-39c9637c577e_parse_0")
)
(ReferenceLink
   (WordInstanceNode "of@49e4c93f-c78e-4847-bb88-d1a6dcdf89b7")
   (WordNode "of")
)
(WordInstanceLink
   (WordInstanceNode "of@49e4c93f-c78e-4847-bb88-d1a6dcdf89b7")
   (ParseNode "sentence@9edd6ffe-8670-46a9-9c63-39c9637c577e_parse_0")
)
(ReferenceLink
   (WordInstanceNode "fibers[!]@7cdbe7fd-eee8-4d64-95ac-b8bccdd039d6")
   (WordNode "fibers[!]")
)
(WordInstanceLink
   (WordInstanceNode "fibers[!]@7cdbe7fd-eee8-4d64-95ac-b8bccdd039d6")
   (ParseNode "sentence@9edd6ffe-8670-46a9-9c63-39c9637c577e_parse_0")
)
(ReferenceLink
   (WordInstanceNode ".@58837325-a0f5-4a74-b628-f81158f47bc1")
   (WordNode ".")
)
(WordInstanceLink
   (WordInstanceNode ".@58837325-a0f5-4a74-b628-f81158f47bc1")
   (ParseNode "sentence@9edd6ffe-8670-46a9-9c63-39c9637c577e_parse_0")
)
(ReferenceLink
   (ParseNode "sentence@9edd6ffe-8670-46a9-9c63-39c9637c577e_parse_0")
   (ListLink
      (WordInstanceNode "yarn@cf5ec4bb-a377-48d3-8d11-c3d8d5260b24")
      (WordInstanceNode "is@ddb49869-eb1a-4ba6-8242-b3fcb79f90bb")
      (WordInstanceNode "made@dedbc1c5-9b42-4593-88ac-c13b8537370b")
      (WordInstanceNode "of@49e4c93f-c78e-4847-bb88-d1a6dcdf89b7")
      (WordInstanceNode "fibers[!]@7cdbe7fd-eee8-4d64-95ac-b8bccdd039d6")
      (WordInstanceNode ".@58837325-a0f5-4a74-b628-f81158f47bc1")
   )
)
(EvaluationLink
   (LinkGrammarRelationshipNode "Op")
   (ListLink
      (WordInstanceNode "of@49e4c93f-c78e-4847-bb88-d1a6dcdf89b7")
      (WordInstanceNode "fibers[!]@7cdbe7fd-eee8-4d64-95ac-b8bccdd039d6")
   )
)
(EvaluationLink
   (LinkGrammarRelationshipNode "Xp")
   (ListLink
      (WordInstanceNode "LEFT-WALL")
      (WordInstanceNode ".@58837325-a0f5-4a74-b628-f81158f47bc1")
   )
)
(EvaluationLink
   (LinkGrammarRelationshipNode "Pv")
   (ListLink
      (WordInstanceNode "is@ddb49869-eb1a-4ba6-8242-b3fcb79f90bb")
      (WordInstanceNode "of@49e4c93f-c78e-4847-bb88-d1a6dcdf89b7")
   )
)
(EvaluationLink
   (LinkGrammarRelationshipNode "Ss")
   (ListLink
      (WordInstanceNode "yarn@cf5ec4bb-a377-48d3-8d11-c3d8d5260b24")
      (WordInstanceNode "is@ddb49869-eb1a-4ba6-8242-b3fcb79f90bb")
   )
)
(EvaluationLink
   (LinkGrammarRelationshipNode "Wd")
   (ListLink
      (WordInstanceNode "LEFT-WALL")
      (WordInstanceNode "yarn@cf5ec4bb-a377-48d3-8d11-c3d8d5260b24")
   )
)
(EvaluationLink
   (LinkGrammarRelationshipNode "IDOB")
   (ListLink
      (WordInstanceNode "made@dedbc1c5-9b42-4593-88ac-c13b8537370b")
      (WordInstanceNode "of@49e4c93f-c78e-4847-bb88-d1a6dcdf89b7")
   )
)
(ParseLink
   (ParseNode "sentence@9edd6ffe-8670-46a9-9c63-39c9637c577e_parse_0" (stv 1.0 0.9305))
   (SentenceNode "sentence@9edd6ffe-8670-46a9-9c63-39c9637c577e")
)
(LemmaLink
   (WordInstanceNode "yarn@cf5ec4bb-a377-48d3-8d11-c3d8d5260b24")
   (WordNode "yarn")
)
(LemmaLink
   (WordInstanceNode "is@ddb49869-eb1a-4ba6-8242-b3fcb79f90bb")
   (WordNode "be")
)
(LemmaLink
   (WordInstanceNode "of@49e4c93f-c78e-4847-bb88-d1a6dcdf89b7")
   (WordNode "made_of")
)
(LemmaLink
   (WordInstanceNode "fibers[!]@7cdbe7fd-eee8-4d64-95ac-b8bccdd039d6")
   (WordNode "fiber")
)
(LemmaLink
   (WordInstanceNode ".@58837325-a0f5-4a74-b628-f81158f47bc1")
   (WordNode ".")
)
; noun_number (fiber, plural)
(InheritanceLink
   (WordInstanceNode "fibers[!]@7cdbe7fd-eee8-4d64-95ac-b8bccdd039d6")
   (DefinedLinguisticConceptNode "plural")
)
; inflection-TAG (fiber, .n)
(InheritanceLink
   (WordInstanceNode "fibers[!]@7cdbe7fd-eee8-4d64-95ac-b8bccdd039d6")
   (DefinedLinguisticConceptNode ".n")
)
; pos (fiber, noun)
(PartOfSpeechLink
   (WordInstanceNode "fibers[!]@7cdbe7fd-eee8-4d64-95ac-b8bccdd039d6")
   (DefinedLinguisticConceptNode "noun")
)
; pos (., punctuation)
(PartOfSpeechLink
   (WordInstanceNode ".@58837325-a0f5-4a74-b628-f81158f47bc1")
   (DefinedLinguisticConceptNode "punctuation")
)
; noun_number (yarn, uncountable)
(InheritanceLink
   (WordInstanceNode "yarn@cf5ec4bb-a377-48d3-8d11-c3d8d5260b24")
   (DefinedLinguisticConceptNode "uncountable")
)
; inflection-TAG (yarn, .n)
(InheritanceLink
   (WordInstanceNode "yarn@cf5ec4bb-a377-48d3-8d11-c3d8d5260b24")
   (DefinedLinguisticConceptNode ".n")
)
; pos (yarn, noun)
(PartOfSpeechLink
   (WordInstanceNode "yarn@cf5ec4bb-a377-48d3-8d11-c3d8d5260b24")
   (DefinedLinguisticConceptNode "noun")
)
; inflection-TAG (be, .v)
(InheritanceLink
   (WordInstanceNode "is@ddb49869-eb1a-4ba6-8242-b3fcb79f90bb")
   (DefinedLinguisticConceptNode ".v")
)
; pos (be, verb)
(PartOfSpeechLink
   (WordInstanceNode "is@ddb49869-eb1a-4ba6-8242-b3fcb79f90bb")
   (DefinedLinguisticConceptNode "verb")
)
; _obj (<<made_of>>, <<yarn>>) 
(EvaluationLink
   (DefinedLinguisticRelationshipNode "_obj")
   (ListLink
      (WordInstanceNode "of@49e4c93f-c78e-4847-bb88-d1a6dcdf89b7")
      (WordInstanceNode "yarn@cf5ec4bb-a377-48d3-8d11-c3d8d5260b24")
   )
)
; _iobj (<<made_of>>, <<fiber>>) 
(EvaluationLink
   (DefinedLinguisticRelationshipNode "_iobj")
   (ListLink
      (WordInstanceNode "of@49e4c93f-c78e-4847-bb88-d1a6dcdf89b7")
      (WordInstanceNode "fibers[!]@7cdbe7fd-eee8-4d64-95ac-b8bccdd039d6")
   )
)
; tense (made_of, present_passive)
(InheritanceLink
   (WordInstanceNode "of@49e4c93f-c78e-4847-bb88-d1a6dcdf89b7")
   (DefinedLinguisticConceptNode "present_passive")
)
; IDIOM-FLAG (made_of, T)
(InheritanceLink
   (WordInstanceNode "of@49e4c93f-c78e-4847-bb88-d1a6dcdf89b7")
   (DefinedLinguisticConceptNode "idiom")
)
; pos (made_of, verb)
(PartOfSpeechLink
   (WordInstanceNode "of@49e4c93f-c78e-4847-bb88-d1a6dcdf89b7")
   (DefinedLinguisticConceptNode "verb")
)

(ReferenceLink
   (DocumentNode "document@a2113cf1-b6a3-4571-bc3c-4385b7211e6d")
   (ListLink
      (SentenceNode "sentence@4bc32f08-9e5e-4af8-85db-36cb95f3d5e6")
      (SentenceNode "sentence@327f8945-7634-4d27-b192-ce477fec4370")
      (SentenceNode "sentence@0944bb6b-6cb9-4528-be77-7c76bfabef8f")
      (SentenceNode "sentence@43d81757-0f6c-427f-8824-5fc2326e1dfc")
      (SentenceNode "sentence@f33be399-e41e-49b2-8610-1c01a0ee77c8")
      (SentenceNode "sentence@959f6ccd-5757-4a4e-9254-2351f5696400")
      (SentenceNode "sentence@a463469f-ae51-4810-9445-795efda5adf3")
      (SentenceNode "sentence@2477ad26-eff2-4e70-8000-5484edc874bc")
      (SentenceNode "sentence@baec3da3-4ce6-4579-9e3a-137de7f3f773")
      (SentenceNode "sentence@ce7dbf6b-eaf5-45a9-864c-1c3976e6a06c")
      (SentenceNode "sentence@9edd6ffe-8670-46a9-9c63-39c9637c577e")
   )
)

; Bye.
.
exit
