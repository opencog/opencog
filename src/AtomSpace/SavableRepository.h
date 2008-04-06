/**
 * SavableRepository.h
 *
 *
 * Copyright(c) 2004 Rodrigo Barra
 * All rights reserved.
 */
#ifndef SAVABLEREPOSITORY_H
#define SAVABLEREPOSITORY_H
#include <stdio.h>
#include <HandleMap.h>

/**
 * This interface should be implemented by any 
 * Repositories that want to be called by the 
 * SavingLoading. These repositories should
 * registyer in the SavingLoading using the 
 * method SavingLoading::addSavableRepository.
 */
class SavableRepository
{
protected:
	/**
	 * Protected constructor so that this class is abstract.
	 */
	SavableRepository(){}
public:
	virtual ~SavableRepository(){}
	
	/**
	 * Returns an identifier for the Repository.
	 */
	virtual const char* getId() const = 0;
	
	/**
	 * Saves the repository to a file.
	 *
	 * @param The file where the repository should be saved.
	 */
	virtual void saveRepository(FILE *) const = 0;
	
	/**
	 * Loads the repository from a file.
	 *
	 * @param The file from where the repository should be loaded.
	 */
	virtual void loadRepository(FILE *, HandleMap<Atom *> *) = 0;


	/**
	 * This method is used to clear the Repository.
	 */
	virtual void clear() = 0;

};

#endif //SAVABLEREPOSITORY_H
