;
; This file contains all sorts of triggers, reponsers and rules of Dialog System
; inspired by Speech Act Theory. For more details of the original design of the 
; dialog system, please refer to the document written by Ben as below:
;
; ./doc/dialog_system/DialogueSystemSketch_v3.pdf
;  
; @note:  It should be loaded after rules_core.scm and pet_rules.scm
;
; @author Zhenhua Cai <czhedu@gmail.com>
; @date   2011-04-27
;
; Below is an example of how the rule of Dialog System is represented in AtomSpace
;
; ImplicationLink
;     AndLink
;         EvaluationLink
;             SpeechActTriggerNode  "truth_value_answer_trigger"
;             DialogNode  "dialog_history"
;         ExecutionOutputLink
;             SpeechActSchemaNode  "truth_value_answer_reponser"
;             DialogNode  "dialog_history"
;             UtteranceNode  "utterance_sentences"
;     EvaluationLink
;         PredicateNode  "goal_name"
;         ListLink  (empty link)
;
; ReferenceLink
;     DialogNode  "dialog_history"  
;     ListLink
;         SentenceNode  "IS_NEW: FALSE, TO: listener_id, RESPONSER: responser_name, CONTENT: sentence content"
;         SentenceNode  "IS_NEW: TRUE, FROM: speaker_id, CONTENT: sentence content"
;         ....
;
; ReferenceLink
;     UtteranceNode  "utterance_sentences"
;     ListLink
;         SentenceNode  "IS_NEW: TRUE, TO: listener_id, RESPONSER: responser_name, CONTENT: sentence content"
;         ...
;
; Since you can not modify the contents within the ListLink once it is created,
; each time when a new sentence is received or generated, you should create a 
; new ListLink and a ReferenceLink, then delete the old ones, while leave
; DialogNode and UtteranceNode untouched. Don't worry about that, triggers and 
; reponsers would do this dirty work automatically. 
;
; How this dialog system works?
; TODO: finish it once the framework basically works. 
;
;(EvaluationLink
;    PredicateNode "said"
;    ListLink
;       AvtarNode ""
;       SentenceNode
;)

;||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||
;
; Add a speech act trigger given trigger name and dialog node 
;
; A speech act trigger can be served as a precondition in 'add_rule' function 
;
; The format of a speech act trigger is as follows:
;
; EvaluationLink
;     SpeechActTriggerNode  "truth_value_answer_trigger"
;     DialogNode  "dialog_history"
;

(define (add_speech_act_trigger trigger_name . handle_dialog_node)
    (EvaluationLink (DEFAULT_STV) (DEFAULT_AV)
    
        (SpeechActTriggerNode (string-trim-both trigger_name) ) 
        handle_dialog_node
    )        
)

;||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||
;
; Add a speech act responser given responser_name, dialog node and utterance node
;
; A speech act responser can be served as an Action in 'add_rule' function
;
; The format of a speech act responser is as follows:
;
; ExecutionOutputLink
;     SpeechActSchemaNode  "truth_value_answer_reponser"
;     DialogNode  "dialog_history"
;     UtteranceNode  "utterance_sentences"

(define (add_speech_act_schema responser_name handle_dialog_node handle_utterance_node)
    (ExecutionOutputLink (DEFAULT_STV) (DEFAULT_AV)
        (SpeechActSchemaNode (string-trim-both responser_name) )
        handle_dialog_node
        handle_utterance_node
    )
)






;||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||
;
; Return the only ReferenceLink that containing the given node. We would use
; this function to get the ReferenceLink of DialogNode and UtteranceNode. 
;
; If there's none or more than 1 ReferenceLink containing the given node, it 
; woule return an empty list. 
;

(define (get_single_reference_link node)
    (let* ( (reference_link (list) )
          )

          ; Get the ReferenceLink that contains the node
          (map
              (lambda (incoming_link)
                  (if (equal? 'ReferenceLink (cog-type incoming_link) )
                      (set! reference_link 
                          (append reference_link '(incoming_link) )
                      )
                  )
              ); lambda 

              (cog-incoming-set node)
          ); map

          ; Check there's only one ReferenceLink containing the node
          (if (equal? (length reference_link) 1) 
              (set! reference_link (cat reference_link) ) 

              (begin
                  (print_debug_info INFO_TYPE_FAIL "get_single_reference_link"
                      (string-append "There should be exactly one ReferenceLink "
                                     " that contains the node named: " 
                                     (cog-name node)    
                                     ". But got " 
                                     (number->string (length reference_link) )
                      )
                  )

                  (set! reference_link (list) )
              )
          ); if

    ); let* 

    ; Return the single ReferenceLink containing the node
    reference_link

); define

;||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||
;
; Append a bunch of sentence nodes to the end of the dialog history
;
; Return the created ReferenceLink or NIL if fails
;
; The dialog history is represented in ActomSpace as follows:
;
; ReferenceLink
;     DialogNode  "dialog_history"  
;     ListLink
;         SentenceNode  "sentence content"
;         SentenceNode  "sentence content"
;         ....
;

(define (update_dialog_history handle_dialog_node . handles_sentence_node)
    (let* ( (old_reference_link (get_single_reference_link handle_dialog_node) )
            (old_list_link (list) )
            (new_reference_link (list) )
          )
    
          (if (null? old_reference_link)
              ; if failed to get the single ReferenceLink containing the DialogNode
              (print_debug_info INFO_TYPE_FAIL "update_dialog_history"
                  "Failed to get the single ReferenceLink containing " 
                  (cog-name handle_dialog_node)
              )

              ; if success to get the single ReferenceLink containing the DialogNode
              (begin
                  (set! old_list_link (cog-outcoming-set old_reference_link) ) 

                  (if (equal? (length old_list_link) 1)
                      ; if there's a single ListLink inside the ReferenceLink
                      (begin
                          (set! old_list_link (cat old_list_link) )

                          (if (equal? 'ListLink (cog-type old_list_link) )
                              (begin
                                  ; Create new ListLink and ReferenceLink
                                  (ReferenceLink
                                      handle_dialog_node 

                                      (cog-new-link 'ListLink
                                          (apply (append
                                                     (cog-outcoming-set old_reference_link)
                                                     handles_sentence_node
                                                 )
                                          )
                                      ) 
                                  ); ReferenceLink
                            
                                  ; Delete the old ReferenceLink and ListLink
                                  (cog-delete old_reference_link)
                                  (cog-delete old_list_link)
                              ) 
                          ); if
                      ); begin

                      ; if there's none or more than one ListLink inside the ReferenceLink
                      (print_debug_info INFO_TYPE_FAIL "update_dialog_history"
                          (string-append "There should be exactly one ListLink "
                                         "inside the ReferenceLink. " 
                                         "But got " 
                                         (number->string (length old_list_link) )
                                         )
                      )

                  ); if (equal? (length old_list_link) 1
              ); begin

          ); if (null? old_reference_link)

    ); let*        

); define
