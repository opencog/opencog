
=Notes/Proposal for Fine-grained Animation
==March 2016, Linas Vepstas

The current animation subsystem suffers from having a fixed set of
non-alterable animations. When played, they each play out in exactly
the same way. We need to do several things:

(1) randomize the detailed motor positions during animation play, so
that no animation is ver repeated quite exactly.

(2) eveluate and memorize particularly good animation sequences (that
were arrived via part (1) above).

(3) associate verbal tags to the animation sequences, so that verbal
direction can cause these to be selected and replayed.


The generation and learning of new animations is a fairly big project.
First, the bones and rig are in Blender, so we have to either create the
randomized sequences in Blender, or we have to extend the ROS API to
allow bone-level control in Blender (from ROS).

Since only OpenCog or the AtomSpace has the ability to "remember"
animation sequences, and to tag them with words, or emotions, or
affects, anything done in blender has to be ROS-messages back to the
AtomSpace in some way.  This suggests that its probably easier if we
have low-level ROS interfaces to Blender.

Getting positive/negative reinforcement to particular expressions is
harder.  See the README-affect.md for a general discussion of positive
and negative feedback from audio, video (background noise and movement)
and speech.

Not clear how long the learning time-frame is, and if the positive /
negative signal will be adequate to accomplish much.  Learning times
might be too long.

Steal ideas from work on Baby-X and work by Mark Sagar.

The above loose ideas need refinement and conversion into an architecture.

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


