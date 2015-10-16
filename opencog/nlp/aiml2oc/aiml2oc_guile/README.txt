; Make appropriate path adjustments
; the translator
(load "/home/adminuser/share/code/OpenCogAimlProcess.scm")
; the query pattern matcher
(load "/home/adminuser/share/code/OpenCogAimlPre2.scm")
; the AIML
(processAndLoadAIML "/home/adminuser/share/knowledge.aiml" "/home/adminuser/share/knowledge.scm")	
(processAndLoadAIML "/home/adminuser/share/reductions1.aiml" "/home/adminuser/share/reductions1.scm")	
(processAndLoadAIML "/home/adminuser/share/test1.aiml" "/home/adminuser/share/test1.scm")

; Some questions
(answerInput "WHAT IS MELODRAMA")
(answerInput "WHO INVENTED RADIO")
(answerInput "HOW MANY LITERS ARE IN A GALLON")
(answerInput "I see you")
(answerInput "A DOG IS A MAMMAL")
(answerInput "A DOLPHIN IS A SMART MAMMAL")
(answerInput "WHO INVENTED PAPER")
