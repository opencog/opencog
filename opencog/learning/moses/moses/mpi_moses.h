/*
 * mpi_moses.h --- 
 *
 * Copyright (C) 2012 Poulin Holdings LLC
 *
 * Author: Linas Vepstas
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


#ifndef _OPENCOG_MPI_MOSES_H
#define _OPENCOG_MPI_MOSES_H

#include "metapopulation.h"
#include "moses_params.h"

namespace opencog { namespace moses {

#ifdef HAVE_MPI
class moses_mpi
{
    public:
        moses_mpi();
        ~moses_mpi();

        bool is_mpi_master();
};


template<typename Scoring, typename BScoring, typename Optimization>
void mpi_moses(metapopulation<Scoring, BScoring, Optimization>& mp,
               const moses_parameters& pa,
               moses_statistics& stats)
{
    logger().info("MPI MOSES starts");
    moses_mpi mompi;

};

#else

class moses_mpi
{
    public:
        moses_mpi() {}
        ~moses_mpi() {}

        bool is_mpi_master() { return true; }
};

#endif

} // ~namespace moses
} // ~namespace opencog

#endif // _OPENCOG_MPI_MOSES_H
