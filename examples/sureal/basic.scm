; Copyright (C) 2016 OpenCog Foundation
;
; A simple demo for Surface Realization (SuReal)
;
; For more details about SuReal, please check:
; https://github.com/opencog/opencog/blob/master/opencog/nlp/sureal/README.md
; http://wiki.opencog.org/w/Surface_Realization_%28SuReal%29
;
; Prior to running this, the RelEx parse server needs to be set up,
; so that the `nlp-parse` call succeeds. The directory containing the
; chatbot has detailed instructions on how to do this.
;
; On the other hand, if you are running this from the OpenCog docker container,
; you can skip this step as the RelEx parse server will be started automatically
; along with the container. You may need to set the `relex-server-host` if you
; get a "Connection refused" error. For more information:
; https://github.com/opencog/docker/tree/master/opencog/README.md

; Load the needed modules!
(use-modules (opencog)
             (opencog nlp)
             (opencog nlp sureal)
             (opencog nlp chatbot) ; the chatbot defines nlp-parse
             (opencog nlp relex2logic))

; SuReal depends on the contents of the AtomSpace, specifically the existing
; sentences, i.e. the sentences/utterances that were parsed via the `nlp-parse`
; scheme function. Let's start by parsing a few sentences into AtomSpace:
(nlp-parse "Roy runs.")
(nlp-parse "She knows who is the killer.")
(nlp-parse "He drinks quickly.")

; Let's generate a new sentence by running SuReal
; Expected result: "she drinks ."
(sureal (SetLink (EvaluationLink (PredicateNode "drink") (ListLink (ConceptNode "she")))))

; Let's parse a few more sentences into the AtomSpace
(nlp-parse "That lovely pig eats the apple.")
(nlp-parse "The cat he loves can fly.")
(nlp-parse "Jumpy the dog can slowly sign his own name in green paint.")

; And then try to generate a slightly more complex sentence
; Expected result: "that green cat loves the dog ."
(sureal (SetLink (EvaluationLink (PredicateNode "love") (ListLink (ConceptNode "cat") (ConceptNode "dog")))
                 (InheritanceLink (ConceptNode "cat") (ConceptNode "green"))))

; NOTE: The word "that" was not in the input but was included in the output,
;       it's because the only syntactically matching sentence available in the
;       AtomSpace is "That lovely pig eats the apple.", with the word "that"
;       being the starting word of the sentence. Currently when SuReal finds a
;       match, it substitutes the words from that sentence by those syntactically
;       matching words from the input, while leaving the leftovers untouched.
;       This can be confusing sometimes, future version of SuReal may handle
;       this differently.
;
; Additionally, if there are two (or more) matching sentences, SuReal will return
; the best/good enough one. For example if we also have:
(nlp-parse "Tom reads quickly.")

; And then run:
; Expected result is "he eats ." instead of "he eats quickly ." though both
; of the sentences "Roy runs." and "Tom reads quickly." are matched.
(sureal (SetLink (EvaluationLink (PredicateNode "eat") (ListLink (ConceptNode "he")))))

; We can also specify the tense of the verb we want to generate, for example
; if we have:
(nlp-parse "John sits and Julia spoke.")
(nlp-parse "Batman ate the cakes.")

; and we do:
; Expected result: "she drinks and he ate ."
(sureal (SetLink
    (EvaluationLink (PredicateNode "eat") (ListLink (ConceptNode "he")))
    (InheritanceLink (PredicateNode "eat") (DefinedLinguisticConceptNode "past"))
    (EvaluationLink (PredicateNode "drink") (ListLink (ConceptNode "she")))
    (InheritanceLink (PredicateNode "drink") (DefinedLinguisticConceptNode "present"))))
