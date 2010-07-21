/*
 * opencog/server/DimEmbedModule.h
 *
 * Copyright (C) 2010 by Singularity Institute for Artificial Intelligence
 * All Rights Reserved
 *
 * Written by David Crane <dncrane@gmail.com>
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

#ifndef _OPENCOG_DIM_EMBED_MODULE_H
#define _OPENCOG_DIM_EMBED_MODULE_H

#include <opencog/learning/dimensionalembedding/DimensionalEmbedding.h>
#include <opencog/server/Factory.h>
#include <opencog/server/Module.h>
#include <opencog/server/Agent.h>
#include <opencog/server/CogServer.h>

namespace opencog
{
    
    class DimEmbedModule : public Module
    {
    private:
        Factory<DimensionalEmbedding, Agent> dimEmbedFactory;
    public:
        /*
        static inline const char* id() {
            static const char* _id = "opencog::DimEmbedModule";
            return _id;
        }
        */
        const char* id();

        DimEmbedModule();
        virtual ~DimEmbedModule();
        virtual void init();
    }; // class
} //namespace

#endif // _OPENCOG_DIM_EMBED_MODULE_H
        



