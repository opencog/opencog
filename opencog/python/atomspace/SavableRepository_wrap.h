#ifndef _OPENCOG_SAVABLE_REPOSITORY_WRAP_H
#define _OPENCOG_SAVABLE_REPOSITORY_WRAP_H

#include <stdio.h>

#include <opencog/persist/file/SavableRepository.h>
#include <boost/python/wrapper.hpp>

using namespace opencog;
using namespace boost::python;

/** Exposes the SavableRepository class. */
void init_SavableRepository_py();

/** A class wrapper of the SavableRepository class.
 *
 * Dummy classes like these are necessary for wrapping the virtual functions 
 * of classes.
 */
struct SavableRepositoryWrap : SavableRepository, wrapper<SavableRepository>
{
    // Pure virtual functions.

    const char* getId() const;
    void saveRepository(FILE *) const;
    void loadRepository(FILE *, HandleMap<Atom*>*);
    void clear();
};

#endif // _OPENCOG_SAVABLE_REPOSITORY_WRAP_H
