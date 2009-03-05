#include "mt19937ar.h"

#include <opencog/util/numeric.h>

//#define  DEBUG_RAND_CALLS
#ifdef DEBUG_RAND_CALLS
#include <opencog/atomspace/Logger.h>
#endif

using namespace opencog;

/* Period parameters */  
int MT19937RandGen::N = 624;
int MT19937RandGen::M = 397;
unsigned long MT19937RandGen::MATRIX_A = 0x9908b0dfUL;   /* constant vector a */
unsigned long MT19937RandGen::UPPER_MASK = 0x80000000UL; /* most significant w-r bits */
unsigned long MT19937RandGen::LOWER_MASK = 0x7fffffffUL; /* least significant r bits */

// PUBLIC METHODS: 

MT19937RandGen::MT19937RandGen(unsigned long s) {
    init();
    init_genrand(s);
}

MT19937RandGen::MT19937RandGen(unsigned long init_key[], int key_length) {
    init();
    init_by_array(init_key, key_length);
}

MT19937RandGen::~MT19937RandGen() {
    delete[] mt;
}

// random int between 0 and max rand number.
int MT19937RandGen::randint() {
    int result;
    result = (int) genrand_int31();
#ifdef DEBUG_RAND_CALLS
    logger().debug("MT19937RandGen::randint() => %d", result);
#endif
    return result;
}

//random float in [0,1]
float MT19937RandGen::randfloat() { 
    float result;
    result = (float) genrand_real1();
#ifdef DEBUG_RAND_CALLS
    logger().debug("MT19937RandGen::randfloat() => %f", result);
#endif
    return result;
}

//random double in [0,1]
double MT19937RandGen::randdouble() { 
    double result;
    result = genrand_real1();
#ifdef DEBUG_RAND_CALLS
    logger().debug("MT19937RandGen::randfloat() => %f", result);
#endif
    return result;
}
  
//random double in [0,1)
double MT19937RandGen::randDoubleOneExcluded() { 
    double result;
    result = genrand_real2();
#ifdef DEBUG_RAND_CALLS
    logger().debug("MT19937RandGen::randDoubleOneExcluded() => %f", result);
#endif
    return result;
}

//random int in [0,n)
int MT19937RandGen::randint(int n) { 
    int result;
    result = (int) genrand_int31()%n;
#ifdef DEBUG_RAND_CALLS
    logger().debug("MT19937RandGen::randint(%d) => %d", n, result);
#endif
    return result;
}

// return -1 or 1 randonly
int MT19937RandGen::randPositiveNegative(){
    int result;
    result = (this->randint(2) == 0) ? 1 : -1;
#ifdef DEBUG_RAND_CALLS
    logger().debug("MT19937RandGen::randPositiveNegative() => %d", result);
#endif
    return result;
}

// Random int from a gaussian distribution. Neg numbers are clipped to 0
unsigned int MT19937RandGen::pos_gaussian_rand(unsigned int std_dev, unsigned int mean){
    int random = mean + 
                static_cast<unsigned int>(std_dev *
                std::sqrt(-2 * std::log(this->randDoubleOneExcluded())) * 
                std::cos(2 * PI * this->randDoubleOneExcluded()));

    unsigned int result = (random >= 0 ? (unsigned) random : 0);
#ifdef DEBUG_RAND_CALLS
    logger().debug("MT19937RandGen::pos_gaussian_rand() => %u", result);
#endif
    return result;
}

//random boolean
bool MT19937RandGen::randbool() { 
    bool result;
    result = genrand_int31()%2==0;
#ifdef DEBUG_RAND_CALLS
    logger().debug("MT19937RandGen::randbool() => %s", result?"true":"false");
#endif
    return result;
}

//linear biased random bool, b in [0,1]
//when b tends to 1 the result tends to be true
bool MT19937RandGen::biased_randbool(float b) {
    return b > randfloat();
#ifdef DEBUG_RAND_CALLS
    logger().debug("MT19937RandGen::biased_randbool() => %s", result?"true":"false");
#endif
}


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

/* These real versions are due to Isaku Wada, 2002/01/09 added */
