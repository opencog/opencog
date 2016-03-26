
=Architecture/Design for Affective Behavior
==March 2016, Linas Vepstas


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


