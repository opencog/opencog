/*
 * opencog/embodiment/AGISimSim/shared/include/singleton.h
 *
 * Copyright (C) 2002-2007 Novamente LLC
 * All Rights Reserved
 *
 * Written by Ari A. Heljakka
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
