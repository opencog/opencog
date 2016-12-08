;
; cfg-sophia.scm
;
; Behavior Tree configuration parameters for the Sophia blender model.
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
;
; Columns (in order) are:
; * expression (emotion) class
; * blender emotion animation name
; * probability of selecting this animation from this class
; * min intensity of expression
; * max intensity of expression
; * min duration of expression
; * max duration of expression
;
; To view the available expressions, do this:
;   `rostopic echo /blender_api/available_emotion_states`
; You should see a response similar to the below:
; ['worry', 'happySurprise', 'happy.001', 'recoil', 'happyDisgust', 'happy',
; 'surprisedSad', 'surprised', 'sad', 'irritated', 'happy.002', 'fearSuprise',
; 'fear', 'engaged', 'disgustSurprise', 'disgust.Sad', 'disgust', 'contempt',
; 'confused', 'comprehending', 'bored', 'awe', 'angrySad', 'angryDisgust',
; 'angry', 'amused', 'none']

;
; Cheat sheet: to display just one of these:
; (cog-evaluate! (Evaluation  (DefinedPredicate "Show facial expression")
;      (ListLink (Concept "worry") (Number 5) (Number 1))))

; new-arrival face tracking
(emo-expr-spec "new-arrival" "happy"            0.2 0.4 1.0 2 8)
(emo-expr-spec "new-arrival" "happy.001"        0.2 0.4 1.0 2 8)
(emo-expr-spec "new-arrival" "happy.002"        0.2 0.4 1.0 2 8)
(emo-expr-spec "new-arrival" "happySurprise"    0.1 0.4 0.8 2 5)
(emo-expr-spec "new-arrival" "amused"           0.2 0.5 1.0 2 5)
(emo-expr-spec "new-arrival" "engaged"          0.1 0.6 1.0 2 8)

; Used when chatbot is not happy; also, when someone leaves. face tracking
(emo-expr-spec "frustrated" "confused"          0.1 0.5 1.0 1 4)
(emo-expr-spec "frustrated" "worry"             0.1 0.4 0.9 2 4)
(emo-expr-spec "frustrated" "recoil"            0.1 0.5 0.9 2 3)
(emo-expr-spec "frustrated" "happyDisgust"      0.1 0.5 0.9 2 4)
(emo-expr-spec "frustrated" "surprisedSad"      0.1 0.5 0.8 2 5)
(emo-expr-spec "frustrated" "sad"               0.05 0.5 0.8 2 4)
(emo-expr-spec "frustrated" "irritated"         0.05 0.4 0.8 2 4)
(emo-expr-spec "frustrated" "disgustSurprise"   0.05 0.4 0.8 2 4)
(emo-expr-spec "frustrated" "disgust.Sad"       0.1 0.4 0.7 2 4)
(emo-expr-spec "frustrated" "angry"             0.05 0.3 0.7 2 4)
(emo-expr-spec "frustrated" "angrySad"          0.1 0.3 0.8 2 4)
(emo-expr-spec "frustrated" "angryDisgust"      0.1 0.3 0.8 2 4)

; positive face tracking
(emo-expr-spec "positive" "happy"               0.2 0.6 0.8 2 6)
(emo-expr-spec "positive" "happy.001"           0.1 0.4 0.7 2 8)
(emo-expr-spec "positive" "happy.002"           0.1 0.4 1.0 2 8)
(emo-expr-spec "positive" "comprehending"       0.1 0.5 0.8 1 6)
(emo-expr-spec "positive" "amused"              0.1 0.5 0.8 3 6)
(emo-expr-spec "positive" "confused"            0.1 0.5 0.8 3 6)
(emo-expr-spec "positive" "happySurprise"       0.1 0.4 0.8 2 5)
(emo-expr-spec "positive" "engaged"             0.1 0.6 1.0 2 8)
(emo-expr-spec "positive" "surprised"           0.1 0.6 1.0 2 8)

; bored face tracking
(emo-expr-spec "bored"    "bored"               0.7 0.4 0.7 3 6)
(emo-expr-spec "bored"    "sad"                 0.1 0.1 0.3 3 6)
(emo-expr-spec "bored"    "confused"            0.2 0.1 0.3 2 4)

; sleep
(emo-expr-spec "sleep"    "happy"               1.0  0.0 0.5 5 8)

; wake-up
(emo-expr-spec "wake-up"  "surprised"           0.3 0.2 0.6 1 4)
(emo-expr-spec "wake-up"  "happy"               0.6  0.5 0.7 3 6)
(emo-expr-spec "wake-up"  "irritated"           0.1  0.1 0.4 1  3)


; NO face tracking -
; ==================
; Used when chatbot is happy and NO face tracking
(emo-expr-spec "neutral-speech"  "happy"              0.2  0.3 0.6 3 8)
(emo-expr-spec "neutral-speech"  "happy.001"          0.1  0.3 0.6 3 8)
(emo-expr-spec "neutral-speech"  "happy.002"          0.1  0.3 0.6 3 8)
(emo-expr-spec "neutral-speech"  "happySurprise"      0.1  0.1 0.6 2 6)
(emo-expr-spec "neutral-speech"  "surprised"          0.1  0.1 0.5 1 5)
(emo-expr-spec "neutral-speech"  "comprehending"      0.1  0.7 1.0 4 8)
(emo-expr-spec "neutral-speech"  "amused"             0.1  0.6 0.8 3 8)
(emo-expr-spec "neutral-speech"  "confused"           0.1  0.6 1.0 2 8)
(emo-expr-spec "neutral-speech"  "engaged"            0.1  0.5 1.0 2 6)

; Used when listening and NO face tracking
(emo-expr-spec "neutral-listen"  "happy"              0.1  0.5 1.0 2 8)
(emo-expr-spec "neutral-listen"  "happy.001"          0.1  0.5 0.65 2 8)
(emo-expr-spec "neutral-listen"  "happy.002"          0.1  0.5 0.7 2 8)
(emo-expr-spec "neutral-listen"  "happySurprise"      0.1  0.2 0.5 2 4)
(emo-expr-spec "neutral-listen"  "surprised"          0.05 0.3 0.6 2 6)
(emo-expr-spec "neutral-listen"  "surprisedSad"       0.05 0.3 0.7 2 5)
(emo-expr-spec "neutral-listen"  "comprehending"      0.1  0.8 1.0 2 5)
(emo-expr-spec "neutral-listen"  "amused"             0.1  0.4 1.0 2 8)
(emo-expr-spec "neutral-listen"  "confused"           0.05 0.5 1.0 4 8)
(emo-expr-spec "neutral-listen"  "engaged"            0.05 0.4 1.0 4 9)
(emo-expr-spec "neutral-listen"  "fear"               0.05 0.1 0.4 2 4)
(emo-expr-spec "neutral-listen"  "fearSuprise"        0.05 0.1 0.7 2 4)
(emo-expr-spec "neutral-listen"  "angrySad"           0.05 0.2 0.6 2 6)
(emo-expr-spec "neutral-listen"  "sad"                0.05 0.1 0.5 2 6)

; Used when neutral keep alive NO face tracking
(emo-expr-spec "neutral-keep-alive"  "happy"          0.1  0.4 1.0 2 8)
(emo-expr-spec "neutral-keep-alive"  "happy.001"      0.1  0.3 0.65 2 8)
(emo-expr-spec "neutral-keep-alive"  "happy.002"      0.1  0.3 0.7 2 8)
(emo-expr-spec "neutral-keep-alive"  "happySurprised" 0.05 0.2 0.6 2 5)
(emo-expr-spec "neutral-keep-alive"  "confused"       0.1  0.6 1.0 3 8)
(emo-expr-spec "neutral-keep-alive"  "engaged"        0.1  0.4 1.0 4 9)
(emo-expr-spec "neutral-keep-alive"  "amused"         0.1  0.3 1.0 2 5)
(emo-expr-spec "neutral-keep-alive"  "recoil"         0.1  0.2 0.5 1 4)
(emo-expr-spec "neutral-keep-alive"  "irritated"      0.05 0.1 0.7 1 4)
(emo-expr-spec "neutral-keep-alive"  "fearSuprise"    0.05 0.2 0.7 3 6)
(emo-expr-spec "neutral-keep-alive"  "angry"          0.05 0.1 0.6 1 4)
(emo-expr-spec "neutral-keep-alive"  "angrySad"       0.05 0.1 0.6 1 5)
(emo-expr-spec "neutral-keep-alive"  "bored"          0.05 0.1 1.0 3 7)


; imperative
; ==========
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

; for sound reaction
(emo-expr-spec "sound-happy"     "happy"     1 0.5 0.9 3 6)
(emo-expr-spec "sound-amused"    "amused"    1 0.5 0.9 3 6)
(emo-expr-spec "sound-afraid"    "afraid"    1 0.5 0.9 3 6)
(emo-expr-spec "luminance-happy" "happy"     1 0.5 0.9 3 6)
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
;
; Cheat sheet:
; (cog-evaluate! (Evaluation  (DefinedPredicate "Show gesture")
;    (ListLink (Concept "thoughtful") (Number 0.2) (Number 2) (Number 0.8))))
;
(emo-gest-spec "positive" "think-browsUp"        0.05 0.5 0.9 1 1 0.6 1.1)
(emo-gest-spec "positive" "think-browsUp.001"    0.05 0.5 0.9 1 1 0.6 1.1)
(emo-gest-spec "positive" "think-browsUp.002"    0.05 0.5 0.9 1 1 0.6 1.1)
(emo-gest-spec "positive" "think-browsUp.003"    0.05 0.5 0.9 1 1 0.6 1.1)
(emo-gest-spec "positive" "think-L"              0.02 0.5 0.9 1 1 0.6 1.1)
(emo-gest-spec "positive" "think-L.001"          0.02 0.5 0.9 1 1 0.6 1.1)
(emo-gest-spec "positive" "think-L.UP"           0.02 0.5 0.9 1 1 0.6 1.1)
(emo-gest-spec "positive" "think-R"              0.02 0.5 0.9 1 1 0.6 1.1)
(emo-gest-spec "positive" "think-R.001"          0.02 0.5 0.9 1 1 0.6 1.1)
(emo-gest-spec "positive" "think-R.UP"           0.02 0.5 0.9 1 1 0.6 1.1)
(emo-gest-spec "positive" "think-UP"             0.02 0.5 0.9 1 1 0.6 1.1)
; If bored, then 1/10th of the time, yawn.
; Rest of the time, don't do anything.
(emo-gest-spec "bored"   "yawn-1"  0.1 0.6 0.9 1 1 1 1)
(emo-gest-spec "bored"   "noop"    0.9 0   0   1 1 1 1)

(emo-gest-spec "sleepy"  "blink-sleepy"  1 0.7 1.0 1 1 1 1)

(emo-gest-spec "wake-up" "shake-2"  0.4 0.7 1.0 1 1 0.7 0.8)
(emo-gest-spec "wake-up" "shake-3"  0.3 0.6 1.0 1 1 0.7 0.8)
(emo-gest-spec "wake-up" "blink"    0.3 0.8 1.0 2 4 0.9 1.0)

; Gestures to use during speaking.
(emo-gest-spec "conversing" "think-browsUp.001"  0.4 0.7 1.0 1 1 0.6 0.8)
(emo-gest-spec "conversing" "think-browsUp.003"  0.3 0.6 1.0 1 1 0.6 0.8)
(emo-gest-spec "conversing" "think-L.up"         0.3 0.8 1.0 1 1 0.6 1.0)

(emo-gest-spec "chat-positive-nod" "nod-6"  0.5 0.8 0.9 1 1 0.2 0.4)
(emo-gest-spec "chat-positive-nod" "noop"   0.5 0   0   1 1 0   0)

(emo-gest-spec "chat-negative-shake" "shake-3"  0.9 0.9 0.9 1 1 0.4 0.7)
(emo-gest-spec "chat-negative-shake" "noop"     0.1 0   0   1 1 0   0  )

(emo-gest-spec "chat-pos-think" "think-browsUp"  0.8 0.5 0.7 1 1 0.3 0.5)
(emo-gest-spec "chat-pos-think" "noop"           0.2 0   0   1 1 0   0  )

(emo-gest-spec "chat-neg-think" "think-browsDown.003"  0.8 0.5 0.7 1 1 0.3 0.5)
(emo-gest-spec "chat-neg-think" "noop"                 0.2 0   0   1 1 0   0  )

; Gestures to use during listening.
(emo-gest-spec "listening" "think-browsUp.001"  0.4 0.7 1.0 1 1 0.6 0.8)
(emo-gest-spec "listening" "think-browsUp.003"  0.3 0.6 1.0 1 1 0.6 0.8)
(emo-gest-spec "listening" "think-L.up"         0.3 0.8 1.0 1 1 0.6 1.0)


; Gestures keep-alive
; ===================

; Gestures to use during keep-alive.
; positive
; positive + think
; positive + look
; negative + think


; gestures positive
(emo-gest-spec "gest-keep-alive-pos" "nod-1"            0.1  0.6 1.0 1 1 0.6 1.1)
(emo-gest-spec "gest-keep-alive-pos" "nod-2"            0.05 0.6 1.0 1 1 0.4 1.0)
(emo-gest-spec "gest-keep-alive-pos" "nod-3"            0.1  0.5 1.0 1 1 0.6 1.1)
(emo-gest-spec "gest-keep-alive-pos" "nod-4"            0.1  0.5 1.0 1 1 0.6 1.1)
(emo-gest-spec "gest-keep-alive-pos" "nod-5"            0.1  0.5 1.0 1 1 0.6 1.1)
(emo-gest-spec "gest-keep-alive-pos" "nod-6"            0.05 0.5 1.0 1 1 0.6 1.1)
(emo-gest-spec "gest-keep-alive-pos" "thoughtful"       0.05 0.4 0.9 1 1 0.4 1.0)
(emo-gest-spec "gest-keep-alive-pos" "thoughtful.001"   0.05 0.4 0.9 1 1 0.4 1.0)
(emo-gest-spec "gest-keep-alive-pos" "noop"             0.4  0.0 0.0 1 1 0.0 0.0)

; gestures negative
(emo-gest-spec "gest-keep-alive-neg" "shake-2"          0.1  0.4 1.0 1 1 0.4 0.9)
(emo-gest-spec "gest-keep-alive-neg" "shake-3"          0.05 0.4 0.7 1 1 0.4 0.9)
(emo-gest-spec "gest-keep-alive-neg" "shake-4"          0.05 0.3 0.7 1 1 0.4 0.9)
(emo-gest-spec "gest-keep-alive-neg" "shake-5"          0.05 0.3 0.7 1 1 0.4 0.9)
(emo-gest-spec "gest-keep-alive-neg" "shake-6"          0.05 0.3 0.7 1 1 0.4 0.9)
(emo-gest-spec "gest-keep-alive-neg" "noop"             0.7  0   0   1 1 0   0  )

; gestures think
(emo-gest-spec "gest-keep-alive-pos-think" "think-browsUp"        0.05 0.5 0.9 1 1 0.6 1.1)
(emo-gest-spec "gest-keep-alive-pos-think" "think-browsUp.001"    0.05 0.5 0.9 1 1 0.6 1.1)
(emo-gest-spec "gest-keep-alive-pos-think" "think-browsUp.002"    0.05 0.5 0.9 1 1 0.6 1.1)
(emo-gest-spec "gest-keep-alive-pos-think" "think-browsUp.003"    0.05 0.5 0.9 1 1 0.6 1.1)
(emo-gest-spec "gest-keep-alive-pos-think" "think-L"              0.02 0.5 0.9 1 1 0.6 1.1)
(emo-gest-spec "gest-keep-alive-pos-think" "think-L.001"          0.02 0.5 0.9 1 1 0.6 1.1)
(emo-gest-spec "gest-keep-alive-pos-think" "think-L.UP"           0.02 0.5 0.9 1 1 0.6 1.1)
(emo-gest-spec "gest-keep-alive-pos-think" "think-R"              0.02 0.5 0.9 1 1 0.6 1.1)
(emo-gest-spec "gest-keep-alive-pos-think" "think-R.001"          0.02 0.5 0.9 1 1 0.6 1.1)
(emo-gest-spec "gest-keep-alive-pos-think" "think-R.UP"           0.02 0.5 0.9 1 1 0.6 1.1)
(emo-gest-spec "gest-keep-alive-pos-think" "think-UP"             0.02 0.5 0.9 1 1 0.6 1.1)
(emo-gest-spec "gest-keep-alive-neg-think" "think-browsDown"      0.05 0.5 0.9 1 1 0.6 1.1)
(emo-gest-spec "gest-keep-alive-neg-think" "think-browsDown.001"  0.05 0.5 0.9 1 1 0.6 1.1)
(emo-gest-spec "gest-keep-alive-neg-think" "think-browsDown.002"  0.05 0.5 0.9 1 1 0.6 1.1)
(emo-gest-spec "gest-keep-alive-neg-think" "think-browsDown.003"  0.05 0.5 0.9 1 1 0.6 1.1)
(emo-gest-spec "gest-keep-alive-neg-think" "think-DWN"            0.02 0.5 0.9 1 1 0.6 1.1)
(emo-gest-spec "gest-keep-alive-neg-think" "think-R.DWN"          0.02 0.5 0.9 1 1 0.6 1.1)
(emo-gest-spec "gest-keep-alive-neg-think" "noop"                 0.42 0   0   1 1 0   0  )


; gestures look
(emo-gest-spec "gest-keep-alive-look" "look-L"            0.05 0.2 0.9 1 1 0.6 1.1)
(emo-gest-spec "gest-keep-alive-look" "look-L.001"        0.05 0.2 0.9 1 1 0.6 1.1)
(emo-gest-spec "gest-keep-alive-look" "look-L.002"        0.05 0.2 0.9 1 1 0.6 1.1)
(emo-gest-spec "gest-keep-alive-look" "look-R"            0.05 0.2 0.9 1 1 0.6 1.1)
(emo-gest-spec "gest-keep-alive-look" "look-R.001"        0.05 0.2 0.9 1 1 0.6 1.1)
(emo-gest-spec "gest-keep-alive-look" "look-R.002"        0.05 0.2 0.9 1 1 0.6 1.1)
(emo-gest-spec "gest-keep-alive-look" "look-R.UP"         0.05 0.2 0.9 1 1 0.6 1.1)
(emo-gest-spec "gest-keep-alive-look" "look-LR"           0.05 0.2 0.9 1 1 0.6 1.1)
(emo-gest-spec "gest-keep-alive-look" "look-RL"           0.05 0.2 0.9 1 1 0.6 1.1)
(emo-gest-spec "gest-keep-alive-look" "look-L.UP"         0.05 0.2 0.9 1 1 0.6 1.1)
(emo-gest-spec "gest-keep-alive-look" "noop"              0.5  0   0   1 1 0   0  )


; Thus, we list all of them here. The probability colu


; Used for imperatives, i.e. when she is verbally mn is ignored.
(emo-gest-spec "imperative"   "amused"        0.1 0.6 0.9 1 1 1 1)
(emo-gest-spec "imperative"   "blink"         0.1 0.6 0.9 1 1 1 1)
(emo-gest-spec "imperative"   "blink-micro"   0.1 0.6 0.9 1 1 1 1)
(emo-gest-spec "imperative"   "blink-relaxed" 0.1 0.6 0.9 1 1 1 1)
(emo-gest-spec "imperative"   "blink-sleepy"  0.1 0.6 0.9 1 1 1 1)
(emo-gest-spec "imperative"   "nod-1"         0.1 0.6 0.9 1 1 1 1)
(emo-gest-spec "imperative"   "nod-2"         0.1 0.6 0.9 1 1 1 1)
(emo-gest-spec "imperative"   "nod-3"         0.1 0.6 0.9 1 1 1 1)
(emo-gest-spec "imperative"   "shake-2"       0.1 0.6 0.9 1 1 1 1)
(emo-gest-spec "imperative"   "shake-3"       0.1 0.6 0.9 1 1 1 1)
(emo-gest-spec "imperative"   "thoughtful"    0.1 0.2 0.4 1 1 1 1)
(emo-gest-spec "imperative"   "yawn-1"        0.1 0.6 0.9 1 1 1 1)

;salient-gesture
(emo-gest-spec "salient-curious" "think-browsDown.002"    1 0.5 0.9 1 1 0.4 1)

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
(State (Schema "time_sleeping_min") (Number 1))
(State (Schema "time_sleeping_max") (Number 5))

; After 25 seconds of boredom, maybe fall asleep.
; Fall asleep for sure after 125 seconds.
(State (Schema "time_boredom_min") (Number 3))
(State (Schema "time_boredom_max") (Number 4))

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
