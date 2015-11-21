; Make appropriate path adjustments
; the translator
(load "/home/adminuser/share/code/OpenCogAimlProcess1.scm")
; the query pattern matcher
(load "/home/adminuser/share/code/OpenCogAimlReply1.scm")
; the AIML
(processAndLoadAIML "/home/adminuser/share/eliza.aiml" "/home/adminuser/share/eliza.scm")
; start talking
(chatty)

;additional AIML files 
(processAndLoadAIML "/home/adminuser/share/knowledge.aiml" "/home/adminuser/share/knowledge.scm")	
(processAndLoadAIML "/home/adminuser/share/reductions1.aiml" "/home/adminuser/share/reductions1.scm")	
(processAndLoadAIML "/home/adminuser/share/test1.aiml" "/home/adminuser/share/test1.scm")
(processAndLoadAIML "/home/adminuser/share/filtered_alice.aiml" "/home/adminuser/share/filtered_alice.scm")
(processAndLoadAIML "/home/adminuser/share/filtered_atomic.aiml" "/home/adminuser/share/filtered_atomic.scm")
(processAndLoadAIML "/home/adminuser/share/filtered_personality.aiml" "/home/adminuser/share/filtered_personality.scm")
(processAndLoadAIML "/home/adminuser/share/filtered_reductions2.aiml" "/home/adminuser/share/filtered_reductions2.scm")
(processAndLoadAIML "/home/adminuser/share/filtered_std65percent.aiml" "/home/adminuser/share/filtered_std65percent.scm")
(processAndLoadAIML "/home/adminuser/share/filtered_yes.aiml" "/home/adminuser/share/filtered_yes.scm")

; Some questions
(answerInput "WHAT IS MELODRAMA")
(answerInput "WHO INVENTED RADIO")
(answerInput "HOW MANY LITERS ARE IN A GALLON")
(answerInput "I see you")
(answerInput "A DOG IS A MAMMAL")
(answerInput "A DOLPHIN IS A SMART MAMMAL")
(answerInput "WHO INVENTED PAPER")

