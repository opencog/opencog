;
; This file contains all the helper functions of Dialog System inspired by 
; Speech Act Theory. For more details of the original design of the dialog 
; system, please refer to the document written by Ben as below:
;
; ./doc/dialog_system/DialogueSystemSketch_v3.pdf
;  
; @note:  It should be loaded after rules_core.scm and xxx_rules.scm
;         Some code are directly copied from Linas's NLP pipeline 
;
; @author Zhenhua Cai <czhedu@gmail.com>
; @date   2011-08-23
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
    (EvaluationLink (cog-new-stv 1.0 1.0) (cog-new-av 1 1 1)
    
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
    (ExecutionOutputLink (cog-new-stv 1.0 1.0) (cog-new-av 1 1 1)
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
; Append a bunch of sentence nodes to the end of the utterance node. 
;
; @return the newly created ReferenceLink or an empty list if fails
;
; The utterance sentences are organized in ActomSpace as follows:
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

          ; Get previous utterance sentences
          (if (not (null? old_list_link) )
              (set! old_sentence_nodes_list 
                    (cog-outgoing-set old_list_link)
              )
          )

          ; Delete old ReferenceLink containing only old utterance sentences
          (cog-delete old_reference_link)

          ; Create a new ReferenceLink containing both previous and new 
          ; utterance sentences
          (update_reference_link
              utterance_node 

              (apply cog-new-link
                  (append
                      (list 'ListLink) 

                      ; Old utterance sentences
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
; Move the old utterance sentences to the end of dialog history. 
;
; @return the newly created ReferenceLink with empty ListLink as below or an 
;         empty list if fails
;
;         ReferenceLink
;             UtteranceNode  "utterance_sentences"
;             ListLink (empty)
;
; @note This function is invoded by PsiActionSelectionAgent::executeAction after
;       creating 'say' actions for all the utterance sentences, that prevents the 
;       agent speaking the same sentences twice or even more. 
;

(define (reset_utterance_node utterance_node_name)
    (let* ( (utterance_node (cog-node 'UtteranceNode utterance_node_name) )
            (old_reference_link (get_reference_link utterance_node) )
            (old_list_link (list-ref (cog-outgoing-set old_reference_link) 1) )
            (old_sentence_nodes_list (list) )
          )

          ; Get old utterance sentences
          (if (not (null? old_list_link) )
              (set! old_sentence_nodes_list 
                    (cog-outgoing-set old_list_link)
              )
          )

          ; Append old utterance sentences to the end of dialog history
          (apply update_dialog_node 
              (append (list "dialog_history") old_sentence_nodes_list) 
          )

          ; Delete the old ReferenceLink containing old utterance sentences
          (cog-delete old_reference_link)

          ; Create a new ReferenceLink with empty ListLink
          (update_reference_link
              utterance_node 
              (ListLink)
          ); ReferenceLink

    ); let*
); define

;******************************************************************************
;******************************************************************************
;******************************************************************************

; Clean up information of previous sentences and prepare for new incoming sentence 
(define (reset_dialog_system)
    (let* ( (new_parsed_anchor_node (AnchorNode "# New Parsed Sentence") )
            (new_parsed_list_link_list (cog-incoming-set new_parsed_anchor_node) ) 
          )

          ; Delete all the ListLink that contains the New Parsed AnchorNode
          (map
              (lambda (new_parsed_list_link)
                  (if (equal? (cog-type new_parsed_list_link) 'ListLink)
                      (cog-delete-recursive new_parsed_list_link)
                  )
              )
              new_parsed_list_link_list
          )
    )
)

;******************************************************************************
;******************************************************************************
;******************************************************************************

;||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||
(ReferenceLink (stv 1.0 1.0) (cog-new-av 1 1 1)
    (DialogNode "dialog_history")
    (ListLink)
)    

(ReferenceLink (stv 1.0 1.0) (cog-new-av 1 1 1) 
    (UtteranceNode "utterance_sentences")
    (ListLink)
)

; -----------------------------------------------------------------------
; Release items attached to the named anchor
;
(define (release-from-anchor anchor)
   (for-each (lambda (x) (cog-delete x))
      (cog-incoming-set anchor)
   )
)

; -----------------------------------------------------------------------
; global vars:
; new-sent anchor points at the node to which all new sentences are connected
;
(define *new-parsed-sent-anchor* (AnchorNode "# New Parsed Sentence" (stv 1 1)))

; Return the list of SentenceNodes that are attached to the 
; freshly-parsed anchor.  This list will be non-empty if relex-parse
; has been recently run. This list can be emptied with the call
; release-new-parsed-sent below.
;
(define (get-new-parsed-sentences)
	(cog-chase-link 'ListLink 'SentenceNode *new-parsed-sent-anchor*)
)

; release-new-parsed-sents deletes the links that anchor sentences to 
; to new-parsed-sent anchor.
;
(define (release-new-parsed-sents)
	(release-from-anchor *new-parsed-sent-anchor*)
)

; -----------------------------------------------------------------------
; get-parses-of-sents -- return parses of the sentences
; Given a list of sentences, return a list of parses of those sentences.
; That is, given a List of SentenceNode's, return a list of ParseNode's
; associated with those sentences.
;
; OPENCOG RULE: FYI this could be easily implemented as a pattern match,
; and probably should be, when processing becomes fully rule-driven.

(define (get-parses-of-sents sent-list)
	(define (get-parses sent)
		(cog-chase-link 'ParseLink 'ParseNode sent)
	)
	(concatenate! (map get-parses sent-list))
)

; -----------------------------------------------------------------------
; attach-parses-to-anchor -- given sentences, attach the parses to anchor.
; 
; Given a list of sentences i.e. a list of SentenceNodes, go through them,
; locate the ParseNodes, and attach the parse nodes to the anchor.
;
; return value is undefined (no return value).
;
; OPENCOG RULE: FYI this could be easily implemented as a pattern match,
; and probably should be, when processing becomes fully rule-driven.
;
(define (attach-parses-to-anchor sent-list anchor)

	;; Attach all parses of a sentence to the anchor.
	(define (attach-parses sent)
		;; Get list of parses for the sentence.
		(define (get-parses sent)
			(cog-chase-link 'ParseLink 'ParseNode sent)
		)
		;; Attach all parses of the sentence to the anchor.
		;; This must have a true/confident TV so that the pattern
		;; matcher will find and use this link.
		(for-each (lambda (x) (ListLink anchor x (stv 1 1)))
			(get-parses sent)
		)
	)
	;; Attach all parses of all sentences to the anchor.
	(for-each attach-parses sent-list)
)

