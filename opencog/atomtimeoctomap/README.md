/*
 *
 * Copyright (c) 2016, Mandeep Singh Bhatia, OpenCog Foundation
 * All rights reserved.
 * License: AGPL
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the University of Freiburg nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */
 
TimeOctomap API:
  Purpose: This API is meant to store Atom(representing a spacial thing) coordinates in a map, and to create time buffer of maps, such that we can query the past or current location of an atom. Time goes on infinitely long, so a circular time buffer has been implemented and maps beyond some time units are automatically lost when newer time units are added. The time duration in the buffer can vary due to the fact that developer can vary time duration limit for each new time unit created. If the user maintains consistency in time durations then buffer can have some degree of predictability or total predictability depending on consistency. 

The library is supposed to be exposed to atomspace via scheme module functions and initially is meant to store perceived face and sound locations in a map, which can be queried by inquery engine for spatial relations between things. 
Perceptual (cognitive) relations may differ from mathematical relations based on context the relations are observed in, however this is a larger problem for later.

Map representation is inherited from octomap library and stores atom handle at an x,y,z location(user is responsible for conversion of coordinates based on tranform frames). Atom orientation is not stored, as several locations might be taken up by same atom type(eg: point cloud of face atoms).

Note:
 A little problem due to map being accessed probabilistic-ally is that deletion acts probabilistic as well.
 currently put a hack to change node value for full delete in case all atom references for particular atom need to be forgotten.
 The aHandle type(currently int but will be changed) is supposed to hold atom reference.
 RemoveAtomAtTime removes probabilistic while RemoveAtom to forget all atoms removes by changing value to UndefinedHandle
