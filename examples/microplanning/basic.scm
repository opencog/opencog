; Copyright (C) 2016 OpenCog Foundation
;
; A simple demo for Microplanner
;
; For more details about Microplanner, please check:
; https://github.com/opencog/opencog/blob/master/opencog/nlp/microplanning/README.md
; http://wiki.opencog.org/wikihome/index.php/Microplanner
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
;
; Also, you may need to do:
; (add-to-load-path "absolute/path/to/opencog/tests")
; By default, these two will be added:
(add-to-load-path "/opencog/tests")
(add-to-load-path "../tests")
(add-to-load-path "../../tests")

; Load the needed modules
(use-modules (opencog)
             (opencog nlp)
             (opencog nlp microplanning)
             (opencog nlp sureal) ; for sentence generation
             (opencog nlp chatbot)) ; the chatbot defines nlp-parse

; To begin with, let's populate the AtomSpace with some sentences, they are:
; John steals it.
; They grow.
; What burns the tree?
; I mean the tall man.
; The brown cat climbs the table.
; Sam collects the tiny bones.
; John steals the orange.
; Apple's dog grow.
(load-from-path "nlp/microplanning/r2l-atomspace.scm")

; Also load the test cases
(load-from-path "nlp/microplanning/test-atomspace.scm")

; Firstly we can run Microplanner to produce declarative utterances with no anaphora.
(microplanning test-declarative-sal "declarative" *default_chunks_option* #f)

; The output of the above is a long list SetLinks, to make it easier
; to read, we can call SuReal to generate the actual sentences.
; Expected outputs:
; (the green robot climbs the tree .) (robot grabs the apple .)
; (Bob steals the apple .) (Bob eats the apple .) (Bob collects the tiny seeds .)
; (Bob plants the seeds .) (apple 's seeds grow .)
(map sureal (car (microplanning test-declarative-sal "declarative" *default_chunks_option* #f)))

; Let's run with anaphora this time by changing the flag to #t, or just ignore
; the last two optional arguments.
; Expected outputs:
; (the green robot climbs the tree .) (robot grabs the apple .)
; (Bob steals the apple .) (he eats it .) (he collects the tiny seeds .)
; (he plants them .) (they grow .)
(map sureal (car (microplanning test-declarative-sal "declarative")))

; This time, try to produce interrogative utterances with no anaphora
; Expected outputs:
; (what climbs the tree ?) (I mean the tall tree .)
(map sureal (car (microplanning test-interrogative-sal "interrogative" *default_chunks_option* #f)))

; We can also produce interrogative utterances with lexical noun alternative
; Expected outputs:
; (what climbs the tree ?) (I mean the tall pine .)
(map sureal (car (microplanning test-interrogative-sal "interrogative")))
