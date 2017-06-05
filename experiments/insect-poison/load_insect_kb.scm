
(use-modules (opencog nlp))
(use-modules (opencog nlp chatbot))
(use-modules (opencog nlp relex2logic))

;(primitive-load "/home/misgana/Desktop/database/kb/wordnet.scm")
;(primitive-load "/home/misgana/Desktop/database/kb/conceptnet4.scm")

(nlp-start-stimulation 70)
(parse-all nlp-parse "exp1_insects_sent.txt")
