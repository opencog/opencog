/*
 *
 * This is a wrapper class with the Martin Porter implementation
 *
 */

/* This is the Porter stemming algorithm, coded up as thread-safe ANSI C
   by the author.

   It may be be regarded as cononical, in that it follows the algorithm
   presented in

   Porter, 1980, An algorithm for suffix stripping, Program, Vol. 14,
   no. 3, pp 130-137,

   only differing from it at the points maked --DEPARTURE-- below.

   See also http://www.tartarus.org/~martin/PorterStemmer

   The algorithm as described in the paper could be exactly replicated
   by adjusting the points of DEPARTURE, but this is barely necessary,
   because (a) the points of DEPARTURE are definitely improvements, and
   (b) no encoding of the Porter stemmer I have seen is anything like
   as exact as this version, even with the points of DEPARTURE!

   You can compile it on Unix with 'gcc -O3 -o stem stem.c' after which
   'stem' takes a list of inputs and sends the stemmed equivalent to
   stdout.

   The algorithm as encoded here is particularly fast.

   Release 2 (the more old-fashioned, non-thread-safe version may be
   regarded as release 1.)
*/

#ifndef OPENCOG_PORTERSTEMMER_H_
#define OPENCOG_PORTERSTEMMER_H_

#include <string>
#include <stdlib.h>  /* for malloc, free */

namespace opencog
{

class PorterStemmer
{

private:
    struct stemmer {
        char *b;
        int k;
        int j;
    };

    /* cons(z, i) is TRUE <=> b[i] is a consonant. ('b' means 'z->b', but here
       and below we drop 'z->' in comments.
    */
    static int cons(struct stemmer * z, int i);

    /* m(z) measures the number of consonant sequences between 0 and j. if c is
       a consonant sequence and v a vowel sequence, and <..> indicates arbitrary
       presence,

          <c><v>       gives 0
          <c>vc<v>     gives 1
          <c>vcvc<v>   gives 2
          <c>vcvcvc<v> gives 3
          ....
    */

    static int m(struct stemmer * z);

    /* vowelinstem(z) is TRUE <=> 0,...j contains a vowel */

    static int vowelinstem(struct stemmer * z);

    /* doublec(z, j) is TRUE <=> j,(j-1) contain a double consonant. */

    static int doublec(struct stemmer * z, int j);

    /* cvc(z, i) is TRUE <=> i-2,i-1,i has the form consonant - vowel - consonant
       and also if the second c is not w,x or y. this is used when trying to
       restore an e at the end of a short word. e.g.

          cav(e), lov(e), hop(e), crim(e), but
          snow, box, tray.

    */

    static int cvc(struct stemmer * z, int i);

    /* ends(z, s) is TRUE <=> 0,...k ends with the string s. */

    static int ends(struct stemmer * z, const char * s);

    /* setto(z, s) sets (j+1),...k to the characters in the string s, readjusting
       k. */

    static void setto(struct stemmer * z, const char * s);

    /* r(z, s) is used further down. */

    static void r(struct stemmer * z, const char * s);

    /* step1ab(z) gets rid of plurals and -ed or -ing. e.g.

           caresses  ->  caress
           ponies    ->  poni
           ties      ->  ti
           caress    ->  caress
           cats      ->  cat

           feed      ->  feed
           agreed    ->  agree
           disabled  ->  disable

           matting   ->  mat
           mating    ->  mate
           meeting   ->  meet
           milling   ->  mill
           messing   ->  mess

           meetings  ->  meet

    */

    static void step1ab(struct stemmer * z);

    /* step1c(z) turns terminal y to i when there is another vowel in the stem. */

    static void step1c(struct stemmer * z);


    /* step2(z) maps double suffices to single ones. so -ization ( = -ize plus
       -ation) maps to -ize etc. note that the string before the suffix must give
       m(z) > 0. */

    static void step2(struct stemmer * z);

    /* step3(z) deals with -ic-, -full, -ness etc. similar strategy to step2. */

    static void step3(struct stemmer * z);

    /* step4(z) takes off -ant, -ence etc., in context <c>vcvc<v>. */

    static void step4(struct stemmer * z);

    /* step5(z) removes a final -e if m(z) > 1, and changes -ll to -l if
       m(z) > 1. */

    static void step5(struct stemmer * z);

public:
    static std::string getStem( std::string word );

};  // class
}   // namespace

#endif /* OPENCOG_PORTERSTEMMER_H_ */
