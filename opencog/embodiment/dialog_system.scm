;
; This file contains all the helper functions of Dialog System inspired by 
; Speech Act Theory. For more details of the original design of the dialog 
; system, please refer to the document written by Ben as below:
;
; ./doc/dialog_system/DialogueSystemSketch_v3.pdf
;  
; @note:  It should be loaded after rules_core.scm and xxx_rules.scm
;
; @author Zhenhua Cai <czhedu@gmail.com>
; @date   2011-06-23
;
; Below is an example of how the rule of Dialog System is represented in AtomSpace
;
; ImplicationLink
;     AndLink
;         Context
;         ExecutionLink
;             SpeechActSchemaNode "truth_value_answer_reponser"
;             ListLink
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
            (old_list_link (get_reference dialog_node) )
            (old_sentence_nodes_list (list) )
          )

          (if (not (null? old_list_link) )
              (set! old_sentence_nodes_list 
                    (cog-outgoing-set old_list_link)
              )
          )

          (update_reference_link
              dialog_node 

              (apply cog-new-link
                  (append
                      (list 'ListLink) 
                      old_sentence_nodes_list
                      ; Process the new sentences
                      (map-in-order 
                          (lambda (sentence)
                              (if (cog-atom? sentence)
                                  sentence 
                                  (SentenceNode sentence)
                              )
                          ); lambda 
                         
                          sentences
                      ); map-in-order
                  )
              ); apply 
         ); ReferenceLink

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
            (old_reference_link (get_reference_link utterance_node) )
            (old_list_link (list-ref (cog-outgoing-set old_reference_link) 1) )
            (old_sentence_nodes_list (list) )
          )

          (if (not (null? old_list_link) )
              (set! old_sentence_nodes_list 
                    (cog-outgoing-set old_list_link)
              )
          )

          (apply update_dialog_node 
              (append (list "dialog_history") old_sentence_nodes_list) 
          )

          (cog-delete old_reference_link)

          (update_reference_link
              utterance_node 

              (apply cog-new-link
                  (append
                      (list 'ListLink) 
                      ; Process the new sentences
                      (map-in-order 
                          (lambda (sentence)
                              (if (cog-atom? sentence)
                                  sentence
                                  (SentenceNode sentence)
                              )
                          ); lambda 
                         
                          sentences
                      ); map-in-order
                  )
              ); apply 

         ); ReferenceLink

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

