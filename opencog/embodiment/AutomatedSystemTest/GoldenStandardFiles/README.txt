This document describes the existing golden standards for automated tests. They
do not explore all system functionalities, but at least they can be used to
check basic parts of them, as follows:

1) gsfile_0.txt (Duration ~4min) - Test the execution of the physiological
   needs actions: eat, drink, pee and poo. For executing this test, the
   parameter MILLISECONDS_PER_TICK was setted to 5000 at
   PetaverseProxy.properties (mv-pvp proxy). The original milliseconds per tick
   is 500. That configuration put the system to run 10 times faster than usual.
   petaverse_proxy revision used: 261

2) gsfile_1.txt (Duration ~4min) - The owner goes near an accessory and waits
   the pet plays with it. Then goes to another accessory and repeats that
   behavior. Eventually the onwer grabs an accessory that the pet played with
   and throws or dropps it in another place and waits the pet plays with it
   again. Pet recognize the new positions of the accessories and go to play
   with them.
   petaverse_proxy revision used: 261

3) gsfile_2.txt (Duration ~3min) - Test a simple learning process (grab stick),
   asking the pet to execute it in the playing mode after that.
   petaverse_proxy revision used: 261

After each release, it will necessary to create a 'bigger golden' standard that
covers as many as possible funcionalities of the pet brain.
