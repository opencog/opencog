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




