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

#ifndef USE_STL_RANDOM
/* Original:
 
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

#include "mt19937ar.h"

#include <opencog/util/numeric.h>

//#define  DEBUG_RAND_CALLS
#ifdef DEBUG_RAND_CALLS
#include <opencog/atomspace/Logger.h>
#endif

using namespace opencog;

#ifndef USE_STL_RANDOM
/* Period parameters */  
int MT19937RandGen::N = 624;
int MT19937RandGen::M = 397;
unsigned long MT19937RandGen::MATRIX_A = 0x9908b0dfUL;   /* constant vector a */
unsigned long MT19937RandGen::UPPER_MASK = 0x80000000UL; /* most significant w-r bits */
unsigned long MT19937RandGen::LOWER_MASK = 0x7fffffffUL; /* least significant r bits */
#endif

// PUBLIC METHODS: 

MT19937RandGen::MT19937RandGen(unsigned long s) {
#ifndef USE_STL_RANDOM
    init();
    init_genrand(s);
#else
    randomGen.seed(s);
#endif
}

#ifndef USE_STL_RANDOM
MT19937RandGen::MT19937RandGen(unsigned long init_key[], int key_length) {
    init();
    init_by_array(init_key, key_length);
}

MT19937RandGen::~MT19937RandGen() {
    delete[] mt;
}
#endif

void MT19937RandGen::seed(unsigned long s) {
#ifndef USE_STL_RANDOM
    init_genrand(s);
#else
    randomGen.seed(s);
#endif
}

// random int between 0 and max rand number.
int MT19937RandGen::randint() {
#ifndef USE_STL_RANDOM
    return (int)genrand_int31();
#else
    std::uniform_int_distribution<int> dis;
    return dis(randomGen);
#endif
}

// random float in [0,1]
float MT19937RandGen::randfloat() {
#ifndef USE_STL_RANDOM
    return (float) genrand_real1();
#else
    return randomGen() / (float)randomGen.max();
#endif
}

// random double in [0,1]
double MT19937RandGen::randdouble() { 
#ifndef USE_STL_RANDOM
    return genrand_real1();
#else
    return randomGen() / (double)randomGen.max();
#endif
}
  
//random double in [0,1)
double MT19937RandGen::randDoubleOneExcluded() {
#ifndef USE_STL_RANDOM
    return genrand_real2();
#else
    std::uniform_real_distribution<double> dis;
    return dis(randomGen);
#endif
}

//random int in [0,n)
int MT19937RandGen::randint(int n) {
#ifndef USE_STL_RANDOM
    return (int)genrand_int31() % n;
#else
    return (int)randint() % n;
#endif
}

// return -1 or 1 randonly
int MT19937RandGen::randPositiveNegative(){
    return (this->randint(2) == 0) ? 1 : -1;
}

//random boolean
bool MT19937RandGen::randbool() {
#ifndef USE_STL_RANDOM
    return genrand_int31() % 2 == 0;
#else
    return randint() % 2 == 0;
#endif
}

#ifndef USE_STL_RANDOM
// PRIVATE METHODS

/* creates the data structures */
void MT19937RandGen::init() {
    mt = new unsigned long[N];
    mti=N+1;
}

/* initializes mt[N] with a seed */
void MT19937RandGen::init_genrand(unsigned long s)
{
    mt[0]= s & 0xffffffffUL;
    for (mti=1; mti<N; mti++) {
        mt[mti] = 
	    (1812433253UL * (mt[mti-1] ^ (mt[mti-1] >> 30)) + mti); 
        /* See Knuth TAOCP Vol2. 3rd Ed. P.106 for multiplier. */
        /* In the previous versions, MSBs of the seed affect   */
        /* only MSBs of the array mt[].                        */
        /* 2002/01/09 modified by Makoto Matsumoto             */
        mt[mti] &= 0xffffffffUL;
        /* for >32 bit machines */
    }
}

/* initialize by an array with array-length */
/* init_key is the array for initializing keys */
/* key_length is its length */
/* slight change for C++, 2004/2/26 */
void MT19937RandGen::init_by_array(unsigned long init_key[], int key_length)
{
    int i, j, k;
    init_genrand(19650218UL);
    i=1; j=0;
    k = (N>key_length ? N : key_length);
    for (; k; k--) {
        mt[i] = (mt[i] ^ ((mt[i-1] ^ (mt[i-1] >> 30)) * 1664525UL))
          + init_key[j] + j; /* non linear */
        mt[i] &= 0xffffffffUL; /* for WORDSIZE > 32 machines */
        i++; j++;
        if (i>=N) { mt[0] = mt[N-1]; i=1; }
        if (j>=key_length) j=0;
    }
    for (k=N-1; k; k--) {
        mt[i] = (mt[i] ^ ((mt[i-1] ^ (mt[i-1] >> 30)) * 1566083941UL))
          - i; /* non linear */
        mt[i] &= 0xffffffffUL; /* for WORDSIZE > 32 machines */
        i++;
        if (i>=N) { mt[0] = mt[N-1]; i=1; }
    }

    mt[0] = 0x80000000UL; /* MSB is 1; assuring non-zero initial array */ 
}

/* generates a random number on [0,0xffffffff]-interval */
unsigned long MT19937RandGen::genrand_int32(void)
{
    unsigned long y;
    static unsigned long mag01[2]={0x0UL, MATRIX_A};
    /* mag01[x] = x * MATRIX_A  for x=0,1 */

    if (mti >= N) { /* generate N words at one time */
        int kk;

        if (mti == N+1)   /* if init_genrand() has not been called, */
            init_genrand(5489UL); /* a default initial seed is used */

        for (kk=0;kk<N-M;kk++) {
            y = (mt[kk]&UPPER_MASK)|(mt[kk+1]&LOWER_MASK);
            mt[kk] = mt[kk+M] ^ (y >> 1) ^ mag01[y & 0x1UL];
        }
        for (;kk<N-1;kk++) {
            y = (mt[kk]&UPPER_MASK)|(mt[kk+1]&LOWER_MASK);
            mt[kk] = mt[kk+(M-N)] ^ (y >> 1) ^ mag01[y & 0x1UL];
        }
        y = (mt[N-1]&UPPER_MASK)|(mt[0]&LOWER_MASK);
        mt[N-1] = mt[M-1] ^ (y >> 1) ^ mag01[y & 0x1UL];

        mti = 0;
    }
  
    y = mt[mti++];

    /* Tempering */
    y ^= (y >> 11);
    y ^= (y << 7) & 0x9d2c5680UL;
    y ^= (y << 15) & 0xefc60000UL;
    y ^= (y >> 18);

    return y;
}

/* generates a random number on [0,0x7fffffff]-interval */
long MT19937RandGen::genrand_int31(void)
{
    long result = (long)(genrand_int32()>>1);
    return result;
}

/* generates a random number on [0,1]-real-interval */
double MT19937RandGen::genrand_real1(void)
{
    return genrand_int32()*(1.0/4294967295.0); 
    /* divided by 2^32-1 */ 
}

/* generates a random number on [0,1)-real-interval */
double MT19937RandGen::genrand_real2(void)
{
    return genrand_int32()*(1.0/4294967296.0); 
    /* divided by 2^32 */
}

/* generates a random number on (0,1)-real-interval */
double MT19937RandGen::genrand_real3(void)
{
    return (((double)genrand_int32()) + 0.5)*(1.0/4294967296.0); 
    /* divided by 2^32 */
}

/* generates a random number on [0,1) with 53-bit resolution*/
double MT19937RandGen::genrand_res53(void) 
{ 
    unsigned long a=genrand_int32()>>5, b=genrand_int32()>>6; 
    return(a*67108864.0+b)*(1.0/9007199254740992.0); 
} 
#endif

// Create and return the signle instance. The initial seed is zero but
// can be changed with the public method RandGen::seed(unsigned long)
RandGen& opencog::randGen()
{
    static MT19937RandGen instance(0);
    return instance;
}

/* These real versions are due to Isaku Wada, 2002/01/09 added */
