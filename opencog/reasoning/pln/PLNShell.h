#ifndef _PLNSHELL_H
#define _PLNSHELL_H

#include "utils/Singleton.h"

#ifdef __cplusplus
extern "C"
{
#endif

/*	
	namespace reasoning
	{
		void RunPLNTests();
	}
*/
class PLNShell : public Singleton<PLNShell>
{
	void Init();
public:
	void Launch(vtree *target);
	void Launch();
};

#define ThePLNShell PLNShell::Instance()

#define LOG_ON_FILE 0
 
extern bool RunPLNtest;

#ifdef __cplusplus
}
#endif

#endif /* _PLNSHELL_H */
