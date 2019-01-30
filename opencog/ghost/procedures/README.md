# Procedures
Here is a list of procedures (predicates and schemas) that can be used in GHOST. Predicates should be used in the context of a rule while schemas should be used in the action.

## Predicates
- is_after_min
- is_answer_received
- is_neck_direction
- is_word_perceived
- is_talking
- is_not_talking
- is_face_perceived
- is_emotion

## Schemas
- animation
- blink
- blink_cancel
- decrease_urge
- emote
- expression
- fallback_on
- look
- look_cancel
- gesture
- get_neck_dir
- increase_urge
- max_sti_concepts
- max_sti_rules
- max_sti_words
- min_sti_concepts
- min_sti_rules
- min_sti_words
- print-by-action-logger
- saccade_cancel
- saccade_explore
- saccade_listen
- set_rule_sti
- shutup
- sing
- soma
- soma_cancel
- start_timer
- stimulate_concepts
- stimulate_rules
- stimulate_words

### Schemas for Question Answering
- send_query
- get_answer
- get_answer_source

Currently it supports sending query to WolframAlpha, DuckDuckGo, and PLN.
Note, a WolframAlpha AppID is needed and should be set via `set-wa-appid!` in advance. `pln_start_recording` is needed before calling PLN.
