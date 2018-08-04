
Architecture/Design for Affective Behavior
==========================================

#### March 2016, Linas Vepstas

Below follows an explicit proposal of how to implement affects and
behaviors within the Eva behavior infrastructure. This includes a
learning component, through which Eva could expand her emotional and
social repertoire.

The core ideas are inspired by work from Donald Nathanson and his book
"Pride and Shame". This book names nine basic affects. These form a
substrate to emotional experience.  The book is reviewed by Richard
Ostrofsky  ["Affect Theory, Shame and the Logic of Personality"]
(http://www.secthoughts.com/Misc%20Essays/Shame%20and%20Personality.pdf)
(January, 2003)

TLDR; Executive Summary
-----------------------
The proposal is to make triggers of behavior searchable.  Instead of
writing behavior rules as "if condition X holds then perform behavior
Y", one should instead build a library of statements such as "behavior
Y can be appropriate when condition X holds".  This library can hold
several possibilities for Y, such as Y1, Y2, ... which might be also
be preference weighted. 

This library can be searched by the regular pattern-matcher or by the
fuzzy matcher to retreive the set of candidate behaviors, given
condition X.

If no exact answer is found for X, and several fuzzy answers are
available, then a random genetic cross-over or MOSES-style knob-turn
of the candidates can be attempted (maybe can we use the "conceptual
blending" code??).  If the result is good, we memorize it for next
time. If the result is bad, we add it to our "don't do this next time"
list.

Its a good idea because:
1. Learning by conceptual blending.
2. Not hard to implement.
3. Modularizes the set of behaviors into classes that make them easier
   to write/review.


Basic Terminology
-----------------

* _"Affect is biology, emotion is biography."_
* _"Affects are sensed."_
* _"A feeling is one or more affects that attain consciousness."_
* _"An emotion is a bundle of feelings, given a (cultural) label."_
* _"Affects are transient, feelings may linger."_
* _"A mood is a set of chronically triggered feelings."_
   (mood is to feeling what climate is to weather)
* _"A drive is body-specific need (e.g.) oxygen, water, food, sex."_
* _"Personality is a collection of behavior-scripts in response to
   affects."_

The Nine Affects
----------------
The below is a very short summary, focusing almost entirely on the
associated facial expressions.  What to do with these is given in a later
section.

The affects are:

* Interest-excitement
  (Brow creases. Eyes focus narrowly and track. Mouth may open.
  Head may turn to listen.  Body posture is rapt attention.)

* Enjoyment-joy/contentment
  (Opposite and complementary to interest affect. Relief or
  release from tension. Satisfaction.)

* Surprise-startle
  (Eyebrows lift; eyes open wide and blink; mouth opens. Rapid
  in-breath, out-breath with lips protruded, vocal exclamation.)

* Distress-anguish
  (Crying infant: loud bawl or rhythmic sobbing, arched or knit
  eyebrows, mouth slightly open , tears)

* Anger-rage
  (Red face, narrowed eyes, muscle groups of the jaw, face and body
  in isometric tension, quivering with anger, deep, rapid breathing)

* Fear-terror
  (Fixed stare with eyes frozen open, a face that is pale, cold and
  sweaty, and hairs standing up.  Fear was evolved to punish self
  for getting into trouble, prompting retreat to safety, dissuading
  self from trying that again.)

* Dis-smell (something smells bad, is not edible)
  (Upper lip wrinkles, head is pulled back, whole body withdraws,
  head thrust forward, ready to vomit.)

* Disgust, disdain, contempt (similar to dis-smell)
  (Head tilted back, upper lip wrinkled, squinting down the nose.)

* Shame-humiliation (variants: shy, bashful, inhibited, embarrassed,
  humbled, humiliated, chagrined, disgraced, dishonored, mortified)
  "any time desire outruns fulfilment"
  (Eyes and face averted and downcast, eyelids lower, loss of muscle
  tone in the face and neck.)
  All other negative affects are turned outside, this one turns in.

Affect Processing Outline
-------------------------

Quote Ostrofsky:
_"Affect gives charge of significance to each situation, directing
attention to it and making it salient, noticeable, and requiring of
classification. Second, affect discriminates and classifies like and
unlike situations, since those that trigger differing affects will be
experienced as significantly different. Finally, affect can be
triggered by memory, and so it can modify memories retroactively."_

Affect processing phases:

1. The trigger, the affect itself, the physiological response.
2. Association to precedent (search through script library)
3. Choice of script for present situation (or creation of random
   variant by splicing several older scripts)
4. Current emotion, activity patterned by the selected script.
5. Check for success or failure, log resulting new script.
6. Analyze scripts for rigidity, harmfulness, inappropriateness.
   Deconstruct, criticize, reconstruct. (employ psychotherapy).

Step 5 provides the needed learning mechanism.

Step 6 is performed off-line, via analytic reasoning.

Affect Processing, in detail
----------------------------
Below are the same steps, but laid out in greater detail.

1. The trigger, the affect itself, the physiological response. This is
triggered via external stimulus, just like today, in the behavior tree:
something is heard via audio or STT or seen via vision processing; some
behavior-tree snippet plays out, generating an animation.

2. Association to precedent (search through script library). Step (1)
only played a brief reactive animation, lasting a few seconds, maybe
five at most. But what to do for the next few minutes? There's a
library of previously learned/authored "scripts" to guide behavior. The
library is indexed according to the triggering stimulus and current
situation. We search to find one or more scripts that fit. Currently,
this library consists of a bunch of hand-authored scripts,
[here](/opencog/eva/behavior/behavior.scm)
-- this would need to be expanded, and made more flexible. The
coupling of "current situation" to "script" needs to be
revamped/redesigned. We also need either better SQL management tools,
or some way of managing(dumping) AtomSpace contents into a file.

3. Choice of script for present situation (or creation of random variant
by splicing several older scripts) If step (2) returns multiple scripts,
we could try to randomly mash them up. OpenCog has a "concept blending"
component, created by prior GSOC student(s), but I have a feeling its too
broken to use. Maybe not.  We also have experience doing genetic
crossover in MOSES. At any rate, we need to either splice together
scripts, or randomize parts of them. We've done this before in opencog,
we've got some scripts, some tech for this; it needs to be dusted off
and made working again.

4. Current emotion, activity patterned by the selected script. i.e. run
that script. Currently, we only have two such scripts: talking, and
listening. These two differ primarily in the number of eye-blinks, the
breathing rate, and the facial expression choices and strengths. (These
two modes were originally created by David DeMaris.) The proposal here
is that there would be more modes, besides talking/listening, and that
some of these modes are randomly cobbled together from steps 2-3 above.

5. Check for success or failure, log resulting new script. So, some
3-5 minutes later, the mode has been running for a while. How are
things going?  Is there feedback for our chosen behaviors? How do we
get feedback? Some ideas: get an audio-power envelope, add it as
time-stamped data to the AtomSpace.  I am thinking this can be used
to check for room ambient noise, e.g. rapt attention, or general
restlessness, loud talking, silence (rapt attention? Empty room?).
Gasps! Laughter!  Applause! Whistles! Loud bang!

   Video feedback might show rapid movement or slow movement or empty room.
We need a "video chaos" indicator to indicate the general excitement
level in the room.

   How to convert this into a positive/negative signal is unclear ... but
...doable, and ideally, even learn-able. We can get positive/negative
affect from the text analysis, and also partly audio analysis.  Learning
algo needs to stitch together perceptions (audio volume, pitch, noise,
video chaos, number of people in room, history of chat affect).

6. Analyze scripts for rigidity, harmfulness, inappropriateness.
Deconstruct, criticize, reconstruct. (employ psychotherapy). Yes, this
sounds like a doozey, but this is actually almost doable. I'm still in
the very early stages of hooking up language to perception, but she's
now got at least a basic self-model, and has some language attached to
it. I plan to expand this in the coming weeks/months, if I am not
derailed. Eddie's working on a variant of this. I don't see, right now,
any particular hurdles preventing introspective chat. Perhaps I'm
deluding myself: much of it would be quasi-hard-wired, and there might
be a combinatoric explosion. However, the fuzzy matcher isn't all that
bad, and it can certainly gloss over parts that are mis-understood or
can't be matched up. She may be confused, but ... at least something
could/should work.


Prognosis
---------

I'm excited. All this is doable. Its a fairly large project, but we've
already got all the pieces and parts in place; we don't really have to
invent any kind of new or magic technology; we just have to clean up
various parts and hook them together.

Design issues
-------------

The relationship to OpenPsi is completely unclear. OpenPsi has "drives"
as one of its input-concepts, and Ostrofsky kind of takes apart the idea
of "drives" as an out-moded, out-dated psychological idea, clashing or
inconsistent with "affects".  This clash needs resolution, from Ben or
Amen to figure out if there is a clash, and/or how affects fit into
OpenPsi, and also, how the above algo fits into OpenPsi.

One option is to partly/mostly or completely bypass OpenPsi, and instead
integrate the affect mechanism directly into the existing behavior tree,
as described above.

Task list
---------
To recap: the core requirements/tasks are these:

A.  Get audio-power-envelope and other audio signals (voice frequency
fundamental, rising/falling tone, excitement, pause/silence intervals,
ambient background noise level) into the AtomSpace. Time-stamped.
Get video-"chaos" power into AtomSpace.

B.  Add timestamps to the audio/video data, and the affect percepts.
Dust off the TimeServer code. Perform memory management, deleting stale
data.

C.  Extend list of animations/poses/gestures (?) to match what's needed
for the affects. Perhaps we have almost everything we need here? Need to
try this out.  See also the README-animation.md file.

D.  Hand-author additional behavior snippets, to bring them more in line
with Ostrovsky's description. These would form a core-set of expressive
animations to work from.

E.  Clearly delineate behavior-script triggers from the behaviors
themselves. Currently, they are lumped into one "if someone entered
room, do xyz". This needs to be converted into a library: "when someone
enters the room, here is a choice of scripts that could be performed.
You can refine your search for scripts by specifying additional state.
Each script is tagged with state." We can use either the crisp pattern
matcher to query these, or the fuzzy matcher. Or both. The tagging need
not be crisp.

(This requires refactoring [behavior.scm](/opencog/eva/behavior/behavior.scm))

F.  Attach language descriptors to the behaviors as well. The idea: if
someone says "Eva, please quiet down and listen!", we could fuzzy-match
on the word "listen", and put her into the listen-mode.  This is an
extension of the current verbal commands, which associate verb+object to
a single blender animation ("look afraid"), and instead associate verb
or verb-phrase with a behavior tree snippet/script, the same script
library as in task (E), steps (2)-(3) above.  (The current verbal
subsystem is located in the [opencog nlp subsystem](/opencog/eva/chatbot-eva)
mostly in the imperative.scm, knowledge.scm, imperative-rules.scm files.

G.  Dust off the concept blending code, or redesign it from scratch, to
work more like the MOSES genetic cross-over and "knob-turning" steps.
The "knob-turning" and crossover is needed to generate new behaviors,
when the existing stock is inadequate, when the current situation is not
understood, and more generally, to power learning of new behaviors.

H.  Provide a maintainer-friendly database infrastructure, so that Eva's
memory of learned behaviors can be saved/restored.  This might be doable
via either the AtomSpace SQL subsystem, enhanced with some friendly
tools, or could be a raw (scheme) dump of selected contents of the
AtomSpace.  Maybe both.

Status
------
Work has not started.
