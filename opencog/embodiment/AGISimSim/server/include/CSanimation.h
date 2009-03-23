/*
    Copyright (C) 2001 by Teemu Keinonen

    This library is free software; you can redistribute it and/or
    modify it under the terms of the GNU Library General Public
    License as published by the Free Software Foundation; either
    version 2 of the License, or (at your option) any later version.

    This library is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
    Library General Public License for more details.


    You should have received a copy of the GNU Library General Public
    License along with this library; if not, write to the Free
    Software Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
*/
#ifndef CSANIMATION_H
#define CSANIMATION_H

#include <iostream>
#include "simconfig.h"
#include "log.h"
#include <crystalspace.h>

//#define HAS_CAL3D

#ifdef HAS_CAL3D
#   include <cal3d/cal3d.h>
#endif

class CSAgent;

class CSanimation : public iAnimTimeUpdateHandler
{
 private:
  csRef<iSpriteCal3DState> cal3d;
  iMeshWrapper * m_pMesh_w;

 public:
  SCF_DECLARE_IBASE;

  CSanimation(csRef<iMeshWrapper> model);
  virtual ~CSanimation()
  {
  }

  void ExecuteMoveAnimation(const char * anim, csVector3 start, csVector3 end, float speed);
  bool IsMoveAnimationActive() {return m_moveAnimationActive;}
  void UpdateMoveAnimation(float time);

  void ExecuteRotateAnimation(const char * anim, float start_a, float end_a, float speed);
  bool IsRotateAnimationActive() {return m_rotateAnimationActive;}
  void UpdateRotateAnimation(float time);


  bool AnimationActive() {return (m_rotateAnimationActive||m_moveAnimationActive);}
  void UpdateAnimation(float time)
  {
    UpdateMoveAnimation(time);
    UpdateRotateAnimation(time);
  }

 private:
  csVector3 m_startPos, m_endPos, m_movePos;
  float m_moveSpeed;
  float m_moveTime;
  bool m_moveAnimationActive;
  std::string m_moveAnimationName;


  float m_startRotAngle, m_endRotAngle, m_rotAngle; 
  bool m_rotateAnimationActive; 
  float m_rotSpeed, m_rotTime;
  std::string m_rotateAnimationName;

  virtual void UpdatePosition(float delta, CalModel * cm)
  {
  }

};

#endif
