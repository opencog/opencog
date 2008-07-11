// Singleton.h: interface for the Singleton class.
//
//////////////////////////////////////////////////////////////////////

#ifndef AFX_SINGLETON_H__C1C32EB0_0ED2_4DC8_86C4_8FE0BD0E5B80__INCLUDED_
#define AFX_SINGLETON_H__C1C32EB0_0ED2_4DC8_86C4_8FE0BD0E5B80__INCLUDED_

#if _MSC_VER > 1000
#pragma once
#endif // _MSC_VER > 1000

template <typename T>
class Singleton
{
public:
	static T& Instance() {
		if (!instance) {
			return *(instance = new T());
		}
		return *instance;
	}
protected:
	static T* instance;
	Singleton() { };
	~Singleton() { delete instance; }
private:
	Singleton(const Singleton&) {};
	Singleton& operator=(const Singleton&) {};
};

template <typename T>
T* Singleton<T>::instance = 0;

#endif // !defined(AFX_SINGLETON_H__C1C32EB0_0ED2_4DC8_86C4_8FE0BD0E5B80__INCLUDED_)
