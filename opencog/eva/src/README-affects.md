

So: first off the bat: Ostrovsky associates explicit facial expressions
with the affects: e.g. -- Interest-excitement  (Brow creases. Eyes focus
narrowly and track. Mouth may open.  Head may turn to listen.  Body
posture is rapt attention.)

The first "obvious" thing is that we do not have explicit animation
sequences for many of these.  (see full list in the commit).  Right now,
its not obvious how to get all the animations set up.   There are two
paths:  (a) beat on Mark or someone to create animations for these (b)
implement a "learning" mechanism to learn new animations.

The learning mechanism (b) is a fairly big project in itself: it would
require taking the existing animations, randomly tweaking various
movements, remembering what tweaks were made, and then getting
positive/negative reinforcement for each.   This requires lots of new
code, and the time-span to  "learn"  anything useful will almost surely
be very long.  However, something like this seems to be hip and trendy,
e.g. Baby-X and work by Mark Sagar.  Refining this project to make it
more acheivable would be excellent.

Next, the question is "great, what do we do with these affects?"  There
are several approaches, here, also.  (1) integrate them into Amen's
OpenPsi code.  It is not clear how to do this.  OpenPsi has "drives" as
one of its input-concepts, and Ostrovsky kind of takes apart the idea of
"drives" as an out-moded, out-dated psychological idea, clashing or
inconsistent with "affects". To resolve this clash, I'd need Ben or Amen
to figure out if there is a clash, and/or how affects fit into OpenPsi.

Option (2) is to partly/mostly or completely bypass openpsi, and instead
integrate them with the existing behavior tree, so that they underpin
the behaviors.  This is exicting, because Ostrovsky sketches the
algorithm, and what's more, the algorithm has explicit learning stages,
(which are not as hard as the face-expression learning task above)  and
what's more: the learning stages integrate directly with the verbal
acting-direction code that Eddie just barely started hacking on.
 (Currently, the robot can follow simple verbal commands. The verbal
acting-direction is to string a bunch of these together, and remember
them, for later performance.  Right now, these just trigger blender
animations, but could possibly dig deeper into the blender stack, if we
had an API for it.)

So, here's my Ostrovsky paraphrase:

1) The trigger, the affect itself, the physiological response.  (This is
triggered via external stimulus, just like today, in the behavior tree:
something heard via audio or STT or vision processing; some
behavior-tree snippet plays out, generating an animation)

2) Association to precedent (search through script library).  Step (1)
only played a brief reactive animation, lasting a few seconds, maybe
five at most.  But what to do for the next few minutes?  There's a
library of previously learned/authored "scripts" to guide behavior.  The
library is indexed according to the triggereing stimulus and current
situation. We search to find one or more scripts that fit.  Currently,
this library consists of a bunch of hand-authored scripts,
here: https://github.com/opencog/ros-behavior-scripting/blob/master/src/behavior.scm
 -- this would need to be expanded, and made more flexible.  The
coupling of "current situation" to "script" needs to be
revamped/redesigned.  We also need either better SQL management tools,
or some way of managing(dumping) atomspace contents into a file.

3) Choice of script for present situation (or creation of random variant
by splicing several older scripts) If step (2) returns multiple scripts,
we could try to randomly mash them up.  OpenCog has a "concept blending"
component, creted by prior GSOC student(s), but I have a feeling its too
broken to use. Maybe not.   We also have experience doing genetic
crossover in MOSES.  At any rate, we need to either splice together
scripts, or randomize parts of them.  We've done this before in opencog,
we've got some scripts, some tech for this; it needs to be dusted off
and made working again.

4) Current emotion, activity patterned by the selected script.  i.e. run
that script.  Currently, we only have two such scripts: talking, and
listening.  These two differ primarily in the number of eye-blinks, the
breathing rate, and the facial expression choices and strengths.  David
DeMaris knows this well, he set these two modes up.  The proposal here
is that there would be more modes, besides talking/listening, and that
some of these modes are randomly cobbled together from steps 2-3 above.

5) Check for success or failure, log resulting new script.  So, some 3-5
minutes later, the mode has been running for a while.  How's it going?
 How do we get feedback?  I'm told that Wenwei has some microphone
audio-power-envelope code somewhere.  Right now, that's not being dumped
into opencog; it should be.  I am thinking this can be used to check for
room ambient noise, e.g. clapping, or restlessness, loud talking,
silence.  Video feedback might show rapid movement or slow movement or
empty room.   We also don't have this being piped into opencog, but we
really really need this.   How to convert this into a positive/negative
signal is still a bit uncertain .. but ...doable. We do have
positive/negative affect from the speech analysis (DeMaris code) I'm
hoping that we can also do some sort of learning algo to sharpen these
perceptions togethre (audio volume, pitch, noise, video chaos, numbre of
people in room, history of chat affect)

6) Analyze scripts for rigidity, harmfulness, inappropriateness.
 Deconstruct, criticize, reconstruct. (employ psychotherapy).  Yes, this
sounds like a doozey, but this is actually almost doable. I'm still in
the very early stages of hooking up language to perception, but she's
now got at least a basic self-model, and has some language attached to
it.  I plan to expand this in the coming weeks/months, if I am not
derailed. Eddie's working on a variant of this.  I don't see, right now,
any particular hurdles preventing introspective chat.  Perhaps I'm
deluding myself: much of it would be quasi-hard-wired, and there might
be a combinatoric explosion. However, the fuzzy matcher isn't all that
bad, and it can certainly gloss over parts that are mis-understood or
can't be matched up. She may be confused, but ... at least something
could/should work.

----
So, now you see why I'm excited and am cc'ing everybody.  All this is
doable.  Its a fairly large project, but we've already got all the
pieces and parts in place; we don't really have to invent any kind of
new or magic technology; we just have to clean up various parts and hook
them together.

I'm not sure where this project might fit on the grand scheme of
priorities of things to do, but, given that its within reach and doable,
that has me excited. 





To recap: the core requirements/tasks are these:

A) get audio-power-envelope and other audio signals into opencog.  Get
video-"chaos" power into opencog.  

B) add timestamp's to the audio/video data, and the affect percepts.  
Dust off the TimeServer code. (currently used only by minecraft people,
may be wonky).

C) extend list of animations/poses/gestures (?) to match what's needed
for the affects. Perhaps we have almost everything we need here? Need to
try this out.

D) hand-author additional behavior snippets, to bring them more in line
with Ostrovsky's description. These would form a core-set of expressive
animations to work from.

E) Clearly delineate behavior-script triggers from the behaviors
themselves.  Currently, they are lumped into one "if someone entered
room, do xyz".  This needs to be converted into a library: "when someone
enters the room, here is a choice of scripts that could be performed.
 You can refine your search for scripts be specifying additional state.
Each script is tagged with state."  We can use either the crisp pattern
matcher to query these, or the fuzzy matcher. 

F) Attach language descriptions to the behaviors as well.  The idea: if
someone says "Eva, please quiet down and listen!", we could fuzzy-match
on the word "listen", and put her into the listen-mode.   This is an
extension of the current verbal commands, which associate verb+object to
a single blender animation ("look afraid"), and instead associate verb
or verb-phrase with a behavior tree snippet/script, the same script
library as in task (E), steps (2)-(3) above.

