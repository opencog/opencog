
Notes/Proposal for Fine-grained Animation
=========================================

#### March 2016, Linas Vepstas

The current animation subsystem suffers from having a fixed set of
non-alterable animations. When played, they each play out in exactly
the same way. We need to do several things:

(1) randomize the detailed motor positions during animation play, so
that no animation is ever repeated quite exactly.

(2) evaluate and memorize particularly good animation sequences (that
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
