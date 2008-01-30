/**
 **	\file Semaphore.h
 **	\date  2007-04-13
 **	\author grymse@alhem.net
**/
/*
Copyright (C) 2007  Anders Hedstrom

This program is free software; you can redistribute it and/or
modify it under the terms of the GNU General Public License
as published by the Free Software Foundation; either version 2
of the License, or (at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program; if not, write to the Free Software
Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.
*/
#ifndef _SOCKETS_Semaphore_H
#define _SOCKETS_Semaphore_H

#include "sockets-config.h"
#include <pthread.h>
#ifdef MACOSX
#include <sys/semaphore.h>
#else
#include <semaphore.h>
#endif


#ifdef SOCKETS_NAMESPACE
namespace SOCKETS_NAMESPACE {
#endif


/** pthread semaphore wrapper.
	\ingroup threading */
class Semaphore
{
public:
	Semaphore();
	Semaphore(unsigned int start_val);
	~Semaphore();

	int Post();
	int Wait();
	int TryWait();
	int GetValue(int&);

private:
	Semaphore(const Semaphore& ) {} // copy constructor
	Semaphore& operator=(const Semaphore& ) { return *this; } // assignment operator
	sem_t m_sem;
};




#ifdef SOCKETS_NAMESPACE
} // namespace SOCKETS_NAMESPACE {
#endif
#endif // _SOCKETS_Semaphore_H

