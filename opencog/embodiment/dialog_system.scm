;
; TODO: There are two 'find_reference_link', one is here, 
;       and another in 'psi_util.scm', we should change the code here, and use 
;       the version in 'psi_util.scm' instead. 
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
; @date   2011-05-04
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
; Label the SentenceNode
;
; For each sentence the agent said (or observed other agent said) is labeled
; as below:
; 
; AtTimeLink
;     TimeNode "timestamp"
;     EvaluationLink
;         PredicateNode "said"
;         ListLink
;         SentenceNode "sentence content"
;            AvtarNode "speaker_id"
;            AvtarNode "listener_id"
;            SpeechActTriggerNode "responser_name"
;
; For each sentence the agent heard (or observed other agent heard) is labeled
; as below: 
;
; AtTimeLink
;     TimeNode "timestamp"
;     EvaluationLink
;         PredicateNode "heard"
;         ListLink
;         SentenceNode "sentence content"
;            AvtarNode "speaker_id"
;            AvtarNode "listener_id"
;

;(define (set_sentence_label sentence_node time_stamp predicate_name handle_speaker handle_listener . handle_responser)
;)

;||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||
;
; Return a BindLink used by pattern matcher to search a ReferenceLink
; containing the given node and a ListLink 
; 
; The format of the structure being searched is as follows:
;
; ReferenceLink
;     first_node
;     ListLink
;

(define (find_reference_link first_node)
    (BindLink
        (ListLink
            (TypedVariableLink
                (VariableNode "$var_list_link_type") 
                (VariableTypeNode "ListLink") 
            )
        ) 
        
        (ImplicationLink
                ; Pattern to be searched
                (ReferenceLink
                    first_node
                    (VariableNode "$var_list_link_type")
                ) 

                ; Return two values encapsulated by a ListLink
                (ListLink
                    (ReferenceLink
                        first_node    
                        (VariableNode "$var_list_link_type")
                    )

                    (VariableNode "$var_list_link_type")
                )

        ); ImplicationLink
    ); BindLink
); define

;||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||
;
; Append a bunch of sentence nodes to the end of the dialog history
;
; Return the created ReferenceLink or an empty list if fails
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
; @param sentences can be a series of SentenceNode or plain text or mix of them
;

(define (update_dialog_node dialog_node_name . sentences)
    (let* ( (dialog_node (cog-node 'DialogNode dialog_node_name) )
            (find_reference_link_for_dialog_node (find_reference_link dialog_node) )
            (query_result_list_link (cog-bind find_reference_link_for_dialog_node) )
            (query_result (cog-outgoing-set query_result_list_link) )
            (old_reference_link (list) )
            (old_list_link (list) )
            (old_sentence_nodes (list) )
            (new_reference_link (list) )
            (sentence_handle (list) )
          )

          ; Delete the ListLink of query, otherwise old_reference_link and 
          ; old_list_link would never be deleted, since they are contained 
          ; within this ListLink
          (cog-delete-recursive query_result_list_link) 

          (if (equal? (length query_result) 1)
      
              ; If success to get the single ReferenceLink of the DialogNode
              (begin
                  (set! query_result_list_link (car query_result) )
                  (set! query_result (cog-outgoing-set query_result_list_link) )
                  (cog-delete-recursive query_result_list_link)
      
                  (set! old_reference_link (list-ref query_result 0) )
                  (set! old_list_link (list-ref query_result 1) )
                  (set! old_sentence_nodes (cog-outgoing-set old_list_link) )

                  ; Delete the old ReferenceLink and ListLink
                  ; Try to avoid using cog-delete-recursive, unless you know 
                  ; the Atoms are definitely not used by other Links 
                  (cog-delete old_reference_link)
                  (cog-delete old_list_link)

                  ; Create new ListLink and ReferenceLink
                  (set! new_reference_link
                      (ReferenceLink (stv 1.0 1.0)
                          dialog_node 
      
                          (apply cog-new-link
                              (append
                                  (list 'ListLink) 
                                  old_sentence_nodes
                                  ; Process the new sentences
                                  (map-in-order 
                                      (lambda (sentence)
                                          (if (cog-atom? sentence)
                                              (set! sentence_handle sentence) 
                                              (set! sentence_handle
                                                  (SentenceNode sentence)
                                              )    
                                          )
                                          ; Return the handle of SentenceNode
                                          sentence_handle
                                      ); lambda 
                                     
                                      sentences
                                  ); map-in-order
                              )
                          ); apply 
                     ); ReferenceLink
                  )
      
              ); begin
      
              ; If get none or more than one ReferenceLink of the DialogNode
              (print_debug_info INFO_TYPE_FAIL "update_dialog_node"
                  (string-append "The number of ReferenceLink containing the " 
                                 " DialogNode named: " dialog_node_name 
                                 " should be exactly one. " 
                                 "But got " 
                                 (number->string (length query_result) )
                  )
              )
          ); if
          
          ; Return the newly created ReferenceLink containing the DialogNode
          new_reference_link
    ); let*
); define

;||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||
;
; Append a bunch of sentence nodes to the end of the utterance history and 
; move the old utterance sentences to the end of dialog history
;
; Return the created ReferenceLink or an empty list if fails
;
; The utterance history is represented in ActomSpace as follows:
;
; ReferenceLink
;     UtteranceNode  "utterance_sentences"
;     ListLink
;         SentenceNode  "sentence content"
;         SentenceNode  "sentence content"
;         ...
;
; @param sentences can be a series of SentenceNode or plain text or mix of them
;

(define (update_utterance_node utterance_node_name . sentences)
    (let* ( (utterance_node (cog-node 'UtteranceNode utterance_node_name) )
            (find_reference_link_for_utterance_node (find_reference_link utterance_node) )
            (query_result_list_link (cog-bind find_reference_link_for_utterance_node) )
            (query_result (cog-outgoing-set query_result_list_link) )
            (old_reference_link (list) )
            (old_list_link (list) )
            (old_sentence_nodes (list) )
            (new_reference_link (list) )
            (sentence_handle (list) )
          )

          ; Delete the ListLink of query, otherwise old_reference_link and 
          ; old_list_link would never be deleted, since they are contained 
          ; within this ListLink
          (cog-delete-recursive query_result_list_link) 

          (if (equal? (length query_result) 1)
      
              ; If success to get the single ReferenceLink of the UtteranceNode
              (begin

                  (set! query_result_list_link (car query_result) )
                  (set! query_result (cog-outgoing-set query_result_list_link) )
                  (cog-delete-recursive query_result_list_link)
      
                  (set! old_reference_link (list-ref query_result 0) )
                  (set! old_list_link (list-ref query_result 1) )
                  (set! old_sentence_nodes (cog-outgoing-set old_list_link) )

                  ; Delete the old ReferenceLink and ListLink
                  ; Try to avoid using cog-delete-recursive, unless you know 
                  ; the Atoms are definitely not used by other Links 
                  (cog-delete old_reference_link)
                  (cog-delete old_list_link)

                  ; Append the old utterance sentences to the end of dialog history
                  (apply update_dialog_node "dialog_history" old_sentence_nodes)

                  ; Create new ListLink and ReferenceLink
                  (set! new_reference_link
                      (ReferenceLink (stv 1.0 1.0)
                          utterance_node 
      
                          (apply cog-new-link
                              (append
                                  (list 'ListLink) 
                                  ; Process the new sentences
                                  (map-in-order 
                                      (lambda (sentence)
                                          (if (cog-atom? sentence)
                                              (set! sentence_handle sentence) 
                                              (set! sentence_handle
                                                  (SentenceNode sentence)
                                              )    
                                          )
                                          ; Return the handle of SentenceNode
                                          sentence_handle
                                      ); lambda 
                                     
                                      sentences
                                  ); map-in-order
                              )
                          ); apply 

                     ); ReferenceLink
                  )
      
              ); begin
      
              ; If get none or more than one ReferenceLink of the UtteranceNode
              (print_debug_info INFO_TYPE_FAIL "update_utterance_node"
                  (string-append "The number of ReferenceLink containing the " 
                                 " UtteranceNode named: " utterance_node_name 
                                 " should be exactly one. " 
                                 "But got " 
                                 (number->string (length query_result) )
                  )
              )
          ); if
          
          ; Return the newly created ReferenceLink containing the UtteranceNode
          new_reference_link
    ); let*
); define

;******************************************************************************
;******************************************************************************
;******************************************************************************

;||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||
(ReferenceLink (stv 1.0 1.0)
    (DialogNode "dialog_history")
    (ListLink)
)    

(ReferenceLink (stv 1.0 1.0) 
    (UtteranceNode "utterance_sentences")
    (ListLink)
)




