/**
 * Defaults.h
 *
 *
 * Copyright(c) 2001 Thiago Maia, Andre Senna
 * All rights reserved.
 */

#ifndef DEFAULTS_H
#define DEFAULTS_H

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include "ClassServer.h"
#include "types.h"

/**
 * This class is a temporary solution for dynamics values until a more
 * thorough parameter system is defined.
 */
class Defaults {

    private:

        /**
         * Private default constructor for this class to make it abstract.
         */
        Defaults() {}

    public:

        /**
         * Returns the default importance value for a given atom type.
         * 
         * @param Atom type.
         * @return Default importance value for a given atom type.
         */
        static float getDefaultImportance(Type type) {
            return 1;
        }

        /**
         * Returns the default heat value for a given atom type.
         * 
         * @param Atom type.
         * @return Default heat value for a given atom type.
         */
        static float getDefaultHeat(Type type) {
            return 0;
        }
};

#endif
