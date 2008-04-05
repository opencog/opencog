/**
* VersionHandle.h
*
* @author Welter Silva
*/
#ifndef _VERSIONHANDLE_H_
#define _VERSIONHANDLE_H_

#include "CoreUtils.h" 

enum IndicatorType {HYPOTHETICAL=0, CONTEXTUAL, UNKNOWN}; 

struct VersionHandle {
    IndicatorType indicator;
    Handle substantive;

    // Default constructor, gets a NULL_VERSION_HANDLE.
    VersionHandle();
    VersionHandle(IndicatorType ind, Handle subs);

    static const char* indicatorToStr(IndicatorType) throw (InvalidParamException);
    static IndicatorType strToIndicator(const char*) throw (InvalidParamException);
};

#define NULL_VERSION_HANDLE VersionHandle()

struct hashVersionHandle{
    int operator()(VersionHandle vh) const;
};

struct eqVersionHandle{
    bool operator()(VersionHandle vh1, VersionHandle vh2) const;
}; 

#define isNullVersionHandle(vh) TLB::isInvalidHandle(vh.substantive)

#endif //_VERSIONHANDLE_H_
