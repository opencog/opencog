/*
 * opencog/embodiment/AGISimSim/server/src/CSanimation.cpp
 *
 * Copyright (C) 2002-2009 Novamente LLC
 * All Rights Reserved
 * Author(s): Ari A. Heljakka
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU Affero General Public License v3 as
 * published by the Free Software Foundation and including the exceptions
 * at http://opencog.org/wiki/Licenses
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU Affero General Public License
 * along with this program; if not, write to:
 * Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
 */

/**

Copyright Ari A. Heljakka / GenMind Ltd.
Not part of AGI-Sim.
Do not distribute.

*/

#include "CSanimation.h"

#ifdef HAS_CAL3D

void print_anim_info(CalAnimation * c)
{
  const char * types[] = 
    {
      "TYPE_NONE",
      "TYPE_CYCLE",
      "TYPE_POSE",
      "TYPE_ACTION"
    };
  
  const char * states[] = 
    {
      "STATE_NONE",
      "STATE_SYNC",
      "STATE_ASYNC",
      "STATE_IN",
      "STATE_STEADY",
      "STATE_OUT",
      "STATE_STOPPED"
    };
  CalCoreAnimation * cc = c->getCoreAnimation();
  std::cout << "Active anim name: " << cc->getName() << std::endl;
  std::cout << "Active anim addr: " << c << std::endl;
  std::cout << "Active anim duration: " << cc->getDuration() << std::endl;
  std::cout << "Active anim type: " << types[c->getType()] << std::endl;
  std::cout << "Active anim state: " << states[c->getState()] << std::endl;
  std::cout << "Active anim time: " << c->getTime()  << std::endl;
}

#endif 

SCF_IMPLEMENT_IBASE(CSanimation);
SCF_IMPLEMENT_IBASE_END;

CSanimation::CSanimation(csRef<iMeshWrapper> model) :
  m_pMesh_w(model)
{
  m_rotateAnimationActive = false;
  m_moveAnimationActive = false;

  cal3d  = SCF_QUERY_INTERFACE(model->GetMeshObject(), iSpriteCal3DState);
  if (cal3d)
    {
      LOG("CSanimation", 2, "Got Cal3D object, set control handler.");
      //      cal3d->SetAnimTimeUpdateHandler(this);
       cal3d->AddAnimCycle(StringConfig("AnimatinIdle").c_str(), 1.0, 0.0);
      //cal3d->SetDefaultIdleAnim(StringConfig("AnimatinIdle").c_str());
    }
}

void CSanimation::ExecuteMoveAnimation(const char * anim, csVector3 start, csVector3 end, float speed)
{
  m_startPos = start;
  m_endPos = end;
  m_moveSpeed = speed;
  m_moveAnimationActive = true;
  m_moveTime = 0.0;
  m_moveAnimationName = std::string(anim);

  float duration = (end-start).SquaredNorm()/speed;
  float blend_in_time = duration * FloatConfig("AnimationBlendTimePercentage");
  if (blend_in_time > FloatConfig("AnimationBlendTimeTop"))
    blend_in_time = FloatConfig("AnimationBlendTimeTop");

  if (cal3d)
    {
      cal3d->SetTimeFactor(speed);
      cal3d->AddAnimCycle(m_moveAnimationName.c_str(), 1.0, blend_in_time);
    }
}


void CSanimation::UpdateMoveAnimation(float time)
{
  if (!m_moveAnimationActive)
    return;

  m_moveTime += time;
  csVector3 delta = m_endPos-m_startPos;
  float dist = delta.SquaredNorm();
  float duration = dist/m_moveSpeed;
  float move_delta = m_moveTime/duration;
  //delta.Normalize();
  if (move_delta >= 1.0)
    {
      m_movePos = m_endPos;
      m_moveAnimationActive = false;
    }
  else
    {
      m_movePos = m_startPos+delta*move_delta;//*dist;
    }

  if (cal3d)
    {
      float blend_out_time = duration * FloatConfig("AnimationBlendTimePercentage");     
      if (blend_out_time > FloatConfig("AnimationBlendTimeTop"))
	blend_out_time = FloatConfig("AnimationBlendTimeTop");

      if (m_moveTime*m_moveSpeed >= duration-blend_out_time)
	{
	  cal3d->ClearAnimCycle(m_moveAnimationName.c_str(), blend_out_time);
	}
    }

  m_pMesh_w->GetMovable()->SetPosition(m_movePos);
  m_pMesh_w->GetMovable()->UpdateMove();
}


void CSanimation::ExecuteRotateAnimation(const char * anim, float start_a, float end_a, float speed)
{
  m_rotateAnimationName = std::string(anim);

  m_startRotAngle = start_a;
  m_endRotAngle = end_a;
  m_rotSpeed = speed;
  m_rotTime = 0;
  m_rotateAnimationActive = true;

  float dist = fabs(end_a-start_a);
  float duration = dist/speed;
  float blend_in_time = duration * FloatConfig("AnimationBlendTimePercentage");
  if (blend_in_time > FloatConfig("AnimationBlendTimeTop"))
    blend_in_time = FloatConfig("AnimationBlendTimeTop");

  if (cal3d)
    {
      cal3d->SetTimeFactor(speed);
      cal3d->AddAnimCycle(m_rotateAnimationName.c_str(), 1.0, blend_in_time);
    }
}

void CSanimation::UpdateRotateAnimation(float time)
{
  if (!m_rotateAnimationActive)
    return;
  m_rotTime += time;
  float delta = m_endRotAngle-m_startRotAngle;
  float distance = fabs(delta);
  float duration = distance/m_rotSpeed;
  float rot_delta = (m_rotTime/duration);
  m_rotAngle = m_startRotAngle+delta*rot_delta;
  /*
  if (m_rotAngle < 0.0f)
    {
      m_rotAngle = 3.14159265*2.0f - m_rotAngle;
    }
  else if (m_rotAngle > 3.14159265*2.0f)
    {
      m_rotAngle -= 3.14159265*2.0f;
    }
  */
  std::cout << "Turning angle: " << m_rotAngle << " rot_delta: " << rot_delta << std::endl;

  if (cal3d)
    {
      float blend_out_time = duration * FloatConfig("AnimationBlendTimePercentage");
      
      if (blend_out_time > FloatConfig("AnimationBlendTimeTop"))
	blend_out_time = FloatConfig("AnimationBlendTimeTop");
      if (m_rotTime*m_rotSpeed >= duration-blend_out_time)
	cal3d->ClearAnimCycle(m_rotateAnimationName.c_str(), blend_out_time);
    }

  if (rot_delta >= 1.0f)
    {
      m_rotateAnimationActive = false;
      m_rotAngle = m_endRotAngle;
      //      if (m_endRotAngle < 0.0f)
      //m_rotAngle = 3.14159265*2.0f-m_endRotAngle;
      m_rotTime = 0;
      if (cal3d)
	cal3d->ClearAnimCycle(m_rotateAnimationName.c_str(), 0);
    }

  m_pMesh_w->GetMovable()->SetTransform(csYRotMatrix3(-m_rotAngle));
  m_pMesh_w->GetMovable()->UpdateMove();
}
