;
; This file contains all sorts of speech act schemas of Dialog System used by 
; multiverse world. 
;  
; @note:  It should be loaded after rules_core.scm, multiverse_rules.scm and 
;         dialog_system.scm
;
; @author Zhenhua Cai <czhedu@gmail.com>
; @date   2011-06-23

;******************************************************************************
;******************************************************************************
;******************************************************************************

; Speech act schemas

(define (AskForFood)
    (update_utterance_node "utterance_sentences"  
        (SentenceNode "IS_NEW: TRUE, TO:  , RESPONSER: AskForFood, CONTENT: I'm so hungry")
        (SentenceNode "IS_NEW: TRUE, TO:  , RESPONSER: AskForFood, CONTENT: Could you give me some food?")
        (SentenceNode "IS_NEW: TRUE, TO:  , RESPONSER: AskForFood, CONTENT: OK. Forget it. I'm testing the dialog system :-)")
    )
)
