/*
 * Modifications of original are
 *
 * Copyright (C) 2010-2011 OpenCog Foundation
 * All rights reserved.
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

#ifndef _OPENCOG_MT19937AR_H
#define _OPENCOG_MT19937AR_H

/** @todo Once C++11 is fully supported by GCC feel free to remove anything
 * wrapped by #ifndef USE_STL_RANDOM  
 * The situation is confusing... According to gnu.org, this has been
 * supported since gcc-4.3 (but possibly in the tr1 namespace ??)
 * In Ubuntu oneiric, with gcc-4.6.2, the build failed.
 *  With gcc-4.6.3, things work.  I'm confused. This
 * needs more user reports.
 */
#if (__GNUC__ >= 4) && (__GNUC_MINOR__ >= 6)
#define USE_STL_RANDOM
#endif

#ifndef USE_STL_RANDOM
/* 
   A C-program for MT19937, with initialization improved 2002/1/26.
   Coded by Takuji Nishimura and Makoto Matsumoto.

   Before using, initialize the state by using init_genrand(seed)  
   or init_by_array(init_key, key_length).

   Copyright (C) 1997 - 2002, Makoto Matsumoto and Takuji Nishimura,
   All rights reserved.    
   
   Broken down into header, source and unit test file by Cassio Pennachin,
   2008.                      

   Redistribution and use in source and binary forms, with or without
   modification, are permitted provided that the following conditions
   are met:

     1. Redistributions of source code must retain the above copyright
        notice, this list of conditions and the following disclaimer.

     2. Redistributions in binary form must reproduce the above copyright
        notice, this list of conditions and the following disclaimer in the
        documentation and/or other materials provided with the distribution.

     3. The names of its contributors may not be used to endorse or promote 
        products derived from this software without specific prior written 
        permission.

   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
   "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
   LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
   A PARTICULAR PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT OWNER OR
   CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
   EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
   PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
   PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
   LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
   NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
   SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.


   Any feedback is very welcome.
   http://www.math.sci.hiroshima-u.ac.jp/~m-mat/MT/emt.html
   email: m-mat @ math.sci.hiroshima-u.ac.jp (remove space)
*/
#endif

#include <opencog/util/RandGen.h>

#ifdef USE_STL_RANDOM
#include <random>
#endif

namespace opencog
{
/** \addtogroup grp_cogutil
 *  @{
 */

class MT19937RandGen : public RandGen
{

private: 

#ifndef USE_STL_RANDOM

    /* Period parameters */  
    static int  N;
    static int  M;
    static unsigned long MATRIX_A;   /**< constant vector a */
    static unsigned long UPPER_MASK; /**< most significant w-r bits */
    static unsigned long LOWER_MASK; /**< least significant r bits */

    unsigned long *mt; /**< the array for the state vector  */
    int mti; /**< mti==N+1 means mt[N] is not initialized */

    //! creates the data structures 
    void init();

    //! initializes mt[N] with a seed 
    void init_genrand(unsigned long s);

    //! initialize by an array with array-length 
    /// \param init_key is the array for initializing keys 
    /// \param key_length is its length 
    void init_by_array(unsigned long init_key[], int key_length);
    
    //! generates a random number on [0,0xffffffff]-interval 
    unsigned long genrand_int32(void);

    //! generates a random number on [0,0x7fffffff]-interval 
    long genrand_int31(void);

    //! generates a random number on [0,1]-real-interval 
    double genrand_real1(void);

    //! generates a random number on [0,1)-real-interval 
    double genrand_real2(void);

    //! generates a random number on (0,1)-real-interval 
    double genrand_real3(void);

    //! generates a random number on [0,1) with 53-bit resolution
    double genrand_res53(void);

#else

    std::mt19937 randomGen;

#endif

public: 

    MT19937RandGen(unsigned long s);

#ifndef USE_STL_RANDOM
    MT19937RandGen(unsigned long init_key[], int key_length);
    ~MT19937RandGen();
#endif

    //! reset the random seed
    void seed(unsigned long); 
    
    //! random int between 0 and max rand number.
    int randint();

    //! random float in [0,1]
    float randfloat(); 

    //! random double in [0,1]
    double randdouble();

    //! random double in [0,1)
    double randDoubleOneExcluded();

    //! random int in [0,n)
    int randint(int n);

    //! return -1 or 1 randonly
    int randPositiveNegative();

    //! random boolean
    bool randbool();
};

//! singleton instance (following Meyer's design pattern)
RandGen& randGen();

/** @}*/
} // namespace opencog

#endif // _OPENCOG_MT19937AR_H
