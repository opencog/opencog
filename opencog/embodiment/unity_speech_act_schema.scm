;
; This file contains all sorts of speech act schemas of Dialog System used by 
; unity world. 
;  
; @note:  It should be loaded after rules_core.scm, unity_rules.scm and 
;         dialog_system.scm
;
; @author Zhenhua Cai <czhedu@gmail.com>
; @date   2011-11-15

(use-modules (srfi srfi-1)
)

;******************************************************************************
;******************************************************************************
;******************************************************************************

; Speech act schemas

(define (AskForFood)
    (update_utterance_node "utterance_sentences"  
        (SentenceNode "IS_NEW: TRUE, TO:  , RESPONSER: AskForFood, CONTENT: I'm so hungry.")
        (SentenceNode "IS_NEW: TRUE, TO:  , RESPONSER: AskForFood, CONTENT: Can you please give me a battery?")
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

(define (notify_changes)
    (let* ( (changes_with_arg (get_changes_with_arg) )
            (changes_with_tv (get_changes_with_tv) )
            (changes (append changes_with_arg changes_with_tv) )
            (random_change (random_select changes) )
          )
         
          (if (not (null? random_change) )
              (update_utterance_node "utterance_sentences"  
                  (SentenceNode 
                      (string-append
                          "IS_NEW: TRUE, TO:  , RESPONSER: notify_changes, CONTENT: found change: "
                          (atom_as_string random_change)
                      )
                  )
              )
          ); if
    ); let
)

(define (ask_question)
    (update_utterance_node "utterance_sentences"  
        (SentenceNode "IS_NEW: TRUE, TO:  , RESPONSER: ask_question, CONTENT: Test ask_question.")
    )
)

