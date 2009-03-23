/***************************************************************************
 *  Singleton.h: interface for the Singleton class.
 *	Singleton provides a class the properties of 'singleton' design pattern.
 *
 *  Project: AgiSim
 *
 *  See implementation (.cc, .cpp) files for license details (GPL).
 *
 *  Fri Feb 18 11:35:16 2005
 *  Copyright  2005  Ari A. Heljakka / Novamente LLC
 *  Email [heljakka at iki dot fi]
 *
 *	19.01.06	FP	formatting 
 ****************************************************************************/

#ifndef SINGLETON_H
#define SINGLETON_H

//------------------------------------------------------------------------------------------------------------
/** @class Singleton
	\brief The superclass for all singleton classes.
	Provides the \i singleton design pattern. */
//------------------------------------------------------------------------------------------------------------
template <typename T>
class Singleton
{
private:
	Singleton (const Singleton&) {};
	Singleton& operator = (const Singleton&) {};
protected:
	static T* instance;
	Singleton () { };
	~Singleton() { }
public:
	static void ResetSingleton() {
		if (instance) {
			delete instance;
			instance = NULL;
		}
	}
	static T& Instance() {
		if (!instance) {
			return *(instance = new T());
		}
		return *instance;
	}
	static bool Exists() {
		return (instance != NULL);
	}
};

template <typename T>
T* Singleton<T>::instance = NULL;

#endif
