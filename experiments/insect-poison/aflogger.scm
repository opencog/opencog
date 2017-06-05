;#!/usr/bin/guile

(use-modules (ice-9 rdelim))
(use-modules (opencog nlp relex2logic))
(use-modules (opencog nlp chatbot))

(define OC_DIR    "/home/misgana/OPENCOG/opencog/")
(define BASE_DIR  (string-append OC_DIR "experiments/insect-poison/"))
(define SENT_FILE (string-append BASE_DIR "exp1_poison_sent.txt"))
(define AS_FILE   (string-append BASE_DIR "exp1_afatomspace.scm"))

(define fr (open-file SENT_FILE "r"))
(define fw (open-file AS_FILE "w"))

(define (parse) 
 (display "NLP parsing started...make sure you've set stimulus(i.e nlp-stimulate <number>\n")
 (let ((sentence (read-line fr)))
       (while (not (eof-object? sentence))              
              (nlp-parse sentence)
              (sleep 2) ;sleep for few seconds to let attentino spreading happen
              (display (strftime "\n[time %H:%M:%S]\n" (localtime (current-time))) fw)
              (display (string-append "SENTENCE: " sentence "\n") fw)
              (display (object->string (cog-af)) fw) 
              (set! sentence (read-line fr))))                   
 (close-port fr)
 (close-port fw))     

