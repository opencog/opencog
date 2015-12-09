;
; behavior-cfg.scm
;
; Behavior Tree configuration parameters.
;
; This is (meant to be) a verbatim translation of the contents of the
; old `behavior.cfg` file.
;
; --------------------------------------------------------
; Emotional-state to expression mapping. For a given emotional state
; (for example, happy, bored, excited) this specifies a range of
; expressions to display for that emotional state, as well as the
; intensities and durations.

; Columns (in order) are:
; * expression (emotion) class
; * blender emotion animation name
; * probability of selecting this animation from this class
; * min intensity of expression
; * max intensity of expression
; * min duration of expression
; * max duration of expression

; Translation of behavior.cfg line 9 ff
(emo-expr-spec "new-arrival" "surprised"  1.0 0.2 0.4 10 15)

(emo-expr-spec "frustrated" "confused"    0.4 0.4 0.6 3 7)
(emo-expr-spec "frustrated" "recoil"      0.3 0.4 0.6 3 7)
(emo-expr-spec "frustrated" "surprised"   0.3 0.1 0.2 3 7)

(emo-expr-spec "positive" "happy"         0.2 0.6 0.8 10 15)
(emo-expr-spec "positive" "comprehending" 0.3 0.5 0.8 10 15)
(emo-expr-spec "positive" "engaged"       0.4 0.5 0.8 10 15)

(emo-expr-spec "bored"    "bored"         0.7 0.4 0.7 10 15)
(emo-expr-spec "bored"    "sad"           0.1 0.1 0.3 10 15)
(emo-expr-spec "bored"    "happy"         0.2 0.1 0.3 10 15)

(emo-expr-spec "sleep"    "happy"         1.0  0.0 0.1 5 15)

(emo-expr-spec "wake-up"  "surprised"     0.45 0.2 0.6 5 15)
(emo-expr-spec "wake-up"  "happy"         0.2  0.5 0.7 5 15)
(emo-expr-spec "wake-up"  "irritated"     0.6  0.1 0.4 1  4)

(emo-expr-spec "neutral-speech"  "happy"         0.2  0.1 0.3 4 8)
(emo-expr-spec "neutral-speech"  "comprehending" 0.4  0.5 0.8 4 8)
(emo-expr-spec "neutral-speech"  "engaged"       0.4  0.5 0.8 4 8)

; --------------------------------------------------------
; Emotional-state to gesture mapping. For a given emotional state
; (for example, happy, bored, excited) this specifies a range of
; gestures to display for that emotional state, as well as the
; intensities and durations.
;
; Columns (in order) are:
; * expression (emotion) class
; * blender gesture animation name
; * probability of selecting this animation from this class
; * min intensity of gesture
; * max intensity of gesture
; * min number of repetitions of this gesture
; * max number of repetitions of this gesture
; * min speed of gesture
; * max speed of gesture
;
; The "noop" gesture is a special no-operation gesture; if selected,
; then nothing is done. This allows gestures to be generated only some
; of the time; the "noop" is what is "done" the rest of the time.

; Translation of behavior.cfg line 75 ff
(emo-gest-spec "positive" "nod-1"  0.1 0.6 0.9 1 1 0.5 0.8)
(emo-gest-spec "positive" "nod-2"  0.1 0.2 0.4 1 1 0.8 0.9)
(emo-gest-spec "positive" "noop"   0.8 0   0   1 1 0   0)

(emo-gest-spec "bored"   "yawn-1"  0.01 0.6 0.9 1 1 1 1)
(emo-gest-spec "bored"   "noop"    0.99 0   0   1 1 1 1)

(emo-gest-spec "sleep"  "blink-sleepy"  1 0.7 1.0 1 1 1 1)

(emo-gest-spec "wake-up" "shake-2"  0.4 0.7 1.0 1 1 0.7 0.8)
(emo-gest-spec "wake-up" "shake-3"  0.3 0.6 1.0 1 1 0.7 0.8)
(emo-gest-spec "wake-up" "blink"    0.3 0.8 1.0 2 4 0.9 1.0)

; New beavior.cfg line 120 "listening_gestures"
(emo-gest-spec "listening" "think-browsUp.001"  0.4 0.7 1.0 1 1 0.6 0.8)
(emo-gest-spec "listening" "think-browsUp.003"  0.3 0.6 1.0 1 1 0.6 0.8)
(emo-gest-spec "listening" "think-L.up"         0.3 0.8 1.0 1 1 0.6 1.0)

; New beavior.cfg line 149
(emo-gest-spec "chat-positive-nod" "nod-6"  0.5 0.8 0.9 1 1 0.2 0.4)
(emo-gest-spec "chat-positive-nod" "noop"   0.5 0   0   1 1 0   0)

(emo-gest-spec "chat-negative-shake" "shake-3"  0.9 0.9 0.9 1 1 0.4 0.7)
(emo-gest-spec "chat-negative-shake" "noop"     0.1 0   0   1 1 0   0  )

; line 160 plus lines 1351ff of new general_behavior.py
; aka stuff for "chatbot_positive_reply_think"
(emo-gest-spec "chat-pos-think" "think-browsUp"  0.8 0.5 0.7 1 1 0.3 0.5)
(emo-gest-spec "chat-pos-think" "noop"           0.2 0   0   1 1 0   0  )

(emo-gest-spec "chat-neg-think" "think-browsDown.003"  0.8 0.5 0.7 1 1 0.3 0.5)
(emo-gest-spec "chat-neg-think" "noop"                 0.2 0   0   1 1 0   0  )

; --------------------------------------------------------
; Dice-roll.  Perform some action some fraction of the time.

(dice-roll "glance new face"   0.5) ; line 590 -- glance_probability_for_new_faces
(dice-roll "glance lost face"  0.5) ; -- glance_probability_for_lost_faces
(dice-roll "group interaction" 0.7) ; line 599 -- glance_probability
(dice-roll "go to sleep"       0.1) ; line 699 -- sleep_probability
(dice-roll "wake up"           0.5) ; line 619 -- wake_up_probability

; --------------------------------------------------------
; Time-related conf paramters

; All numbers are in seconds.
; line 115 of behavior.cfg - time_to_change_face_target_min
(State (Schema "time_to_change_face_target_min") (Number 8))
(State (Schema "time_to_change_face_target_max") (Number 10))

(State (Schema "time_to_make_gesture_min") (Number 6))
(State (Schema "time_to_make_gesture_max") (Number 10))

(State (Schema "time_to_wake_up") (Number 25))

; --------------------------------------------------------
; Misc other config parameters

; blink_randomly_interval_mean and blink_randomly_interval_var
(DefineLink (DefinedSchema "blink normal mean") (Number 3.5))
(DefineLink (DefinedSchema "blink normal var")  (Number 0.2))

; blink_chat_faster_mean
(DefineLink (DefinedSchema "blink chat fast mean") (Number 2.0))
(DefineLink (DefinedSchema "blink chat fast var")  (Number 0.12))

; blink_chat_slower_mean
(DefineLink (DefinedSchema "blink chat slow mean") (Number 4.5))
(DefineLink (DefinedSchema "blink chat slow var")  (Number 0.12))

; --------------------------------------------------------
