#include <string>
#include <vector>
#include <iostream>
#include <sys/time.h>

#include <AttentionValue.h>
#include <mt19937ar.h>

#include "Pattern.h"

using namespace std;

Util::RandGen* patternRng = new Util::MT19937RandGen(1);

Pattern::Pattern(int w, int h, float density)
{
    width = w;
    height = h;
    rng = patternRng;

    for (int i = 0; i < width*height; i++) {
	push_back(rng->randfloat()< density);
    }
}

Pattern::~Pattern()
{
}


Pattern Pattern::binarisePattern(AttentionValue::sti_t vizThreshold)
{
    Pattern out(width, height);
    Pattern::iterator out_i = out.begin();
    Pattern::iterator i;

    for (i = begin(); i != end(); i++) {
	int val = *i;
	*out_i = (int) (val >= vizThreshold) ;
	out_i++;
    }
    
    return out;
}

std::vector< Pattern > Pattern::generateRandomPatterns(int amount, int width, int height, float density)
{
    std::vector< Pattern > patterns;

    for (; amount>0; amount--) {
	Pattern p(width, height, density);
	patterns.push_back(p);
    }
    return patterns;
}


std::vector< Pattern > Pattern::mutatePatterns(std::vector< Pattern > &patterns, float error)
{
    std::vector< Pattern >::iterator i;
    std::vector< Pattern > mutants;

    for (i = patterns.begin(); i != patterns.end(); i++) {
	Pattern p = (*i).mutatePattern(error);
	mutants.push_back(p);
    }
    return mutants;


}

Pattern Pattern::mutatePattern(float error)
{
    Pattern result(width, height);
    Pattern::iterator r_i = result.begin();
    Pattern::iterator p;

    for (p = begin(); p != end(); p++) {
	int val = *p;
	// Flip bit with error probability
	if (rng->randfloat() < error) { val = !val; }
	*r_i = val; r_i++;
    }
    return result;
}

float Pattern::hammingSimilarity(const Pattern &a)
{
    float diff = 0.0f;
    if (a.size() != size())
	return -1.0f;

    for (unsigned int i = 0; i < size(); i++) {
	if ((*this)[i] != a[i]) diff++;
    }

    return 1.0f - (diff / size());

}

