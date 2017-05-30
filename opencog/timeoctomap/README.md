TimeOctomap API:
  Purpose: This API stores Atoms in a 4d space time coordinate map. One can query the past or current location of an atom. Spatial coordinates are stored in an octomap. A circular buffer of octomaps is iused to represent time.  

Map representation is inherited from octomap library and stores atom at an x,y,z location.

Note:
 A little problem due to map being accessed probabilistic-ally is that deletion acts probabilistic as well.
 currently put a hack to change node value for full delete in case all atom references for particular atom need to be forgotten.
 RemoveAtomAtTime removes probabilistic while RemoveAtom to forget all atoms removes by changing value to UndefinedHandle
