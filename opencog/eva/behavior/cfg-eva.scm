;
; cfg-eva.scm
;
; Behavior Tree configuration parameters for the Eva blender model.
;
; The configuration controls her "personality": the kinds of
; expressions she shows on her face, how intensely she shows them,
; how long she shows them for. It also controls the probability
; of her reacting in certain ways to situations, or doing certain
; things in given situations.
;
; --------------------------------------------------------
; Emotional-state to expression mapping. For a given emotional state
; (for example, happy, bored, excited), this specifies a range of
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
;
; rostopic echo /blender_api/available_emotion_states
; ['irritated', 'happy', 'recoil', 'surprised', 'sad', 'confused',
;  'worry', 'bored', 'engaged', 'amused', 'comprehending', 'afraid']
;
; Of the above, 'worry' and 'afraid' are not used, below.
; 'worry' is interesting, but not clear when to use it...
; 'afraid' ... when could we use this?
;
; Cheat sheet: to display just one of these:
; (cog-evaluate! (Evaluation  (DefinedPredicate "Show facial expression")
;      (ListLink (Concept "worry") (Number 5) (Number 1))))

; The animations are weird: if the time is too short, then no animation
; plays, unless it is very strong (0.6 and stronger). The weaker
; strengths work, but only if the duration is long.  Strengths below
; 0.4 seem to have no effect, except for "surprised".
; This makes tuning these things tricky.
(emo-expr-spec "new-arrival" "surprised"  1.0 0.25 0.4 10 15)

; Used when chatbot is not happy; also, when someone leaves.
(emo-expr-spec "frustrated" "confused"    0.4 0.6 0.8 3 7)
(emo-expr-spec "frustrated" "recoil"      0.3 0.6 0.8 3 7)
(emo-expr-spec "frustrated" "surprised"   0.3 0.6 0.8 3 7)

(emo-expr-spec "positive" "happy"         0.2 0.6 0.8 10 15)
(emo-expr-spec "positive" "comprehending" 0.3 0.5 0.8 10 15)
(emo-expr-spec "positive" "engaged"       0.4 0.5 0.8 10 15)

(emo-expr-spec "bored"    "bored"         0.7 0.4 0.7 10 15)
(emo-expr-spec "bored"    "sad"           0.1 0.4 0.6 10 15)
(emo-expr-spec "bored"    "amused"        0.2 0.7 1.0 10 15)

(emo-expr-spec "sleepy"   "amused"        1.0  0.8 1.0 5 15)

(emo-expr-spec "wake-up"  "surprised"     0.45 0.4 0.6 5 15)
(emo-expr-spec "wake-up"  "happy"         0.2  0.6 0.7 5 15)
(emo-expr-spec "wake-up"  "irritated"     0.6  0.6 0.9 1  4)

; Used when chatbot is happy
(emo-expr-spec "neutral-speech"  "happy"         0.2  0.1 0.3 4 8)
(emo-expr-spec "neutral-speech"  "comprehending" 0.4  0.5 0.8 4 8)
(emo-expr-spec "neutral-speech"  "engaged"       0.4  0.5 0.8 4 8)

(emo-expr-spec "neutral-listen"  "happy"         0.2  0.1 0.3 4 8)
(emo-expr-spec "neutral-listen"  "comprehending" 0.4  0.5 0.8 4 8)
(emo-expr-spec "neutral-listen"  "engaged"       0.4  0.5 0.8 4 8)

(emo-expr-spec "neutral-keep-alive"  "happy"          0.2  0.4 1.0 2 8)
(emo-expr-spec "neutral-keep-alive"  "confused"       0.2  0.6 1.0 3 8)
(emo-expr-spec "neutral-keep-alive"  "engaged"        0.2  0.4 1.0 4 9)
(emo-expr-spec "neutral-keep-alive"  "amused"         0.2  0.3 1.0 2 5)
(emo-expr-spec "neutral-keep-alive"  "recoil"         0.1  0.2 0.5 1 4)
(emo-expr-spec "neutral-keep-alive"  "irritated"      0.1 0.1 0.7 1 4)
(emo-expr-spec "neutral-keep-alive"  "angry"          0.1 0.1 0.6 1 4)
(emo-expr-spec "neutral-keep-alive"  "bored"          0.1 0.1 1.0 3 7)

; Used for imperatives, i.e. when she is verbally told to do something.
; Thus, we list all of them here. The probability column is ignored.
; The strength has to be 0.6 or more, or else blender doesn't play the
; animation.
(emo-expr-spec "imperative"  "afraid"        1  0.4 0.6 6 12)
(emo-expr-spec "imperative"  "amused"        1  0.9 1.0 6 12)
(emo-expr-spec "imperative"  "bored"         1  0.6 0.9 6 12)
(emo-expr-spec "imperative"  "comprehending" 1  0.6 0.9 6 12)
(emo-expr-spec "imperative"  "confused"      1  0.6 0.9 6 12)
(emo-expr-spec "imperative"  "engaged"       1  0.6 0.9 6 12)
(emo-expr-spec "imperative"  "happy"         1  0.6 0.9 6 12)
(emo-expr-spec "imperative"  "irritated"     1  0.7 1.0 6 12)
(emo-expr-spec "imperative"  "recoil"        1  0.6 0.9 6 12)
(emo-expr-spec "imperative"  "sad"           1  0.6 0.9 6 12)
(emo-expr-spec "imperative"  "surprised"     1  0.6 0.9 6 12)
(emo-expr-spec "imperative"  "worry"         1  0.7 1.0 6 12)

; --------------------------------------------------------
; Emotional-state to gesture mapping. For a given emotional state
; (for example, happy, bored, excited), this specifies a range of
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
;
; rostopic echo /blender_api/available_gestures
; ['all', 'amused', 'blink', 'blink-micro', 'blink-relaxed',
;  'blink-sleepy', 'nod-1', 'nod-2', 'nod-3', 'shake-2', 'shake-3',
;  'thoughtful', 'yawn-1']
;
; Cheat sheet:
; (cog-evaluate! (Evaluation  (DefinedPredicate "Show gesture")
;    (ListLink (Concept "thoughtful") (Number 0.2) (Number 2) (Number 0.8))))

(emo-gest-spec "positive" "nod-1"  0.1 0.6 0.9 1 1 0.5 0.8)
(emo-gest-spec "positive" "nod-2"  0.1 0.2 0.4 1 1 0.8 0.9)
(emo-gest-spec "positive" "noop"   0.8 0   0   1 1 0   0)

; If bored, then 1/10th of the time, yawn.
; Rest of the time, don't do anything.
(emo-gest-spec "bored"   "yawn-1"  0.1 0.6 0.9 1 1 1 1)
(emo-gest-spec "bored"   "noop"    0.9 0   0   1 1 1 1)

(emo-gest-spec "sleepy"  "blink-sleepy"  1 0.7 1.0 1 1 1 1)

(emo-gest-spec "wake-up" "shake-2"  0.4 0.7 1.0 1 1 0.7 0.8)
(emo-gest-spec "wake-up" "shake-3"  0.3 0.6 1.0 1 1 0.7 0.8)
(emo-gest-spec "wake-up" "blink"    0.3 0.8 1.0 2 4 0.9 1.0)

; Gestures to use during conversations.
; Note that the gesture strengths are reduced from normal.
; "thoughtful" is very pronounced, so keep it light.
(emo-gest-spec "conversing" "thoughtful"  1.0 0.2 0.4 1 1 0.2 0.6)
; none-such animations
; (emo-gest-spec "conversing" "think-browsUp.001"  0.4 0.7 1.0 1 1 0.6 0.8)
; (emo-gest-spec "conversing" "think-browsUp.003"  0.3 0.6 1.0 1 1 0.6 0.8)
; (emo-gest-spec "conversing" "think-L.up"         0.3 0.8 1.0 1 1 0.6 1.0)

(emo-gest-spec "chat-positive-nod" "nod-3"  0.5 0.8 0.9 2 2 0.2 0.4)
(emo-gest-spec "chat-positive-nod" "noop"   0.5 0   0   1 1 0   0)

(emo-gest-spec "chat-negative-shake" "shake-3"  0.9 0.9 0.9 1 1 0.4 0.7)
(emo-gest-spec "chat-negative-shake" "noop"     0.1 0   0   1 1 0   0  )

(emo-gest-spec "chat-pos-think" "thoughtful"  0.8 0.2 0.4 1 1 0.2 0.6)
; This animation doesn't exist ...
; (emo-gest-spec "chat-pos-think" "think-browsUp"  0.8 0.5 0.7 1 1 0.3 0.5)
(emo-gest-spec "chat-pos-think" "noop"           0.2 0   0   1 1 0   0  )

; Will do negative by doig two repeats, subtle and fast.
(emo-gest-spec "chat-neg-think" "thoughtful"  0.8 0.1 0.2 2 2 0.5 0.9)
; This animation doesn't exist ...
; (emo-gest-spec "chat-neg-think" "think-browsDown.003"  0.8 0.5 0.7 1 1 0.3 0.5)
(emo-gest-spec "chat-neg-think" "noop"                 0.2 0   0   1 1 0   0  )

; Gestures to use during listening.
(emo-gest-spec "listening" "thoughtful"  0.4 0.7 1.0 1 1 0.6 0.8)
(emo-gest-spec "listening" "noop"        0.6 0.0 1.0 1 1 0.0 0.0)

; Used for imperatives, i.e. when she is verbally told to do something.
; Thus, we list all of them here. The probability column is ignored.
; The shake and nod gestures are very weak, for some reason.
(emo-gest-spec "imperative"   "amused"        0.1 0.6 0.9 1 1 1 1)
(emo-gest-spec "imperative"   "blink"         0.1 0.6 0.9 1 1 1 1)
(emo-gest-spec "imperative"   "blink-micro"   0.1 0.6 0.9 1 1 1 1)
(emo-gest-spec "imperative"   "blink-relaxed" 0.1 0.6 0.9 1 1 1 1)
(emo-gest-spec "imperative"   "blink-sleepy"  0.1 0.6 0.9 1 1 1 1)
(emo-gest-spec "imperative"   "nod-1"         0.1 0.8 1.0 1 1 1 1)
(emo-gest-spec "imperative"   "nod-2"         0.1 0.8 1.0 1 1 1 1)
(emo-gest-spec "imperative"   "nod-3"         0.1 0.8 1.0 1 1 1 1)
(emo-gest-spec "imperative"   "shake-2"       0.1 0.8 1.0 1 1 1 1)
(emo-gest-spec "imperative"   "shake-3"       0.1 0.8 1.0 1 1 1 1)
(emo-gest-spec "imperative"   "thoughtful"    0.1 0.2 0.4 1 1 1 1)
(emo-gest-spec "imperative"   "yawn-1"        0.1 0.6 0.9 1 1 1 1)

; --------------------------------------------------------
; Dice-roll.  Probability of performing some action as the result of
;    some event.

; Probability of looking at someone who entered the room.
(dice-roll "glance new face"   0.5)

; Probability of looking at spot where someone was last seen.
(dice-roll "glance lost face"  0.5)

(dice-roll "group interaction" 0.7)

; Probability of performing the face-study saccade.
(dice-roll "face study" 0.2)

; --------------------------------------------------------
; Time-related conf paramters

; All numbers are in seconds.
(State (Schema "time_to_change_face_target_min") (Number 8))
(State (Schema "time_to_change_face_target_max") (Number 10))

; Specify how long to hold off between making gestures.
; This prevents gestures from occuring too often.
(State (Schema "time_since_last_gesture_min") (Number 6))
(State (Schema "time_since_last_gesture_max") (Number 10))

; Specify how long to hold off between making facial expressions.
(State (Schema "time_since_last_expr_min") (Number 6.0))
(State (Schema "time_since_last_expr_max") (Number 10.0))

; If no one has said anything after 40-80 seconds, say something.
(State (Schema "silence_min") (Number 40))
(State (Schema "silence_max") (Number 80))

; Sleep at least 25 seconds ... at most 160
(State (Schema "time_sleeping_min") (Number 25))
(State (Schema "time_sleeping_max") (Number 160))

; After 25 seconds of boredom, maybe fall asleep.
; Fall asleep for sure after 125 seconds.
(State (Schema "time_boredom_min") (Number 25))
(State (Schema "time_boredom_max") (Number 125))

; How long to look in one direction, before changing gaze,
; when searching for atention in an empty room.
(State (Schema "time_search_attn_min") (Number 1.0))
(State (Schema "time_search_attn_max") (Number 4.0))

(State (Schema "time_search_glance_min") (Number 0.5))
(State (Schema "time_search_glance_max") (Number 2.0))

;; During search-for-attention, how far to look to left or right.
;; XXX Right now, search for attention turns the whole head;
;; perhaps only the eyes should move?
(DefineLink (DefinedSchema "gaze left max") (Number 0.35))
(DefineLink (DefinedSchema "gaze right max") (Number -0.4))

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
; Silence the output.
*unspecified*
