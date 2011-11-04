;
; This file contains all sorts of speech act schemas of Dialog System used by 
; unity world. 
;  
; @note:  It should be loaded after rules_core.scm, unity_rules.scm and 
;         dialog_system.scm
;
; @author Zhenhua Cai <czhedu@gmail.com>
; @date   2011-10-28

;******************************************************************************
;******************************************************************************
;******************************************************************************

; Speech act schemas

(define (AskForFood)
    (update_utterance_node "utterance_sentences"  
        (SentenceNode "IS_NEW: TRUE, TO:  , RESPONSER: AskForFood, CONTENT: I'm so hungry.")
        (SentenceNode "IS_NEW: TRUE, TO:  , RESPONSER: AskForFood, CONTENT: Could you give me some battery?")
    )
)

; TODO: For the moment PsiActionSelectionAgent::executeAction will handle
;       'answer_question' directly, because some c++ code is required.
;       A better approach is implementng 'answer_question' here in scheme. 
;
;(define (answer_question)
;    (update_utterance_node "utterance_sentences"  
;        (SentenceNode "IS_NEW: TRUE, TO:  , RESPONSER: answer_question, CONTENT: Test answer_question.")
;    )
;)

(define (unsolicited_observation)
    (update_utterance_node "utterance_sentences"  
        (SentenceNode "IS_NEW: TRUE, TO:  , RESPONSER: unsolicited_observation, CONTENT: Test unsolicited_observation.")
    )
)

(define (notify_external_change)
    (update_utterance_node "utterance_sentences"  
        (SentenceNode "IS_NEW: TRUE, TO:  , RESPONSER: notify_external_change, CONTENT: Test notify_external_change.")
    )
)

(define (notify_internal_change)
    (update_utterance_node "utterance_sentences"  
        (SentenceNode "IS_NEW: TRUE, TO:  , RESPONSER: notify_internal_change, CONTENT: Test notify_internal_change.")
    )
)

(define (ask_question)
    (update_utterance_node "utterance_sentences"  
        (SentenceNode "IS_NEW: TRUE, TO:  , RESPONSER: ask_question, CONTENT: Test ask_question.")
    )
)

