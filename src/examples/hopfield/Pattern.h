
#ifndef HDEMO_PATTERNMATRIX_H
#define HDEMO_PATTERNMATRIX_H

#include <AttentionValue.h>
#include <RandGen.h>

class Pattern : public std::vector< int > {

    private:
	// Height and width of pattern
	int width;
	int height;
	
	Util::RandGen *rng;

    public: 

	/**
	 * Constructor
	 *
	 * @param width
	 * @param height
	 * @param density of random ones
	 */
	Pattern(int w, int h, float density = 0.0f);

	~Pattern();

	/**
	 * Similarity based on Hamming distance to another Pattern.
	 *
	 * @param p Pattern to compare with.
	 * @return hamming distance as proportion of total possible (i.e.
	 * len(Pattern).
	 */
	float hammingSimilarity(const Pattern &p);

	/**
	 * binarise pattern by making values > threshold 1, otherwise 0.
	 *
	 * @param threshold
	 * @return new pattern of 1 and 0.
	 */
	Pattern binarisePattern(AttentionValue::sti_t threshold);

	/** 
	 * Mutate pattern, each value having error chance of mutating to 0, or
	 * 0 to 1.
	 *
	 * @param error rate of mutation
	 * @return new mutated pattern
	 */
	Pattern mutatePattern(float error);

	int getWidth();
	int getHeight();

	/**
	 * Generate random patterns.
	 *
	 * @param amount number of patterns to generate.
	 * @param width
	 * @param height
	 * @param density the probability of value 1.
	 * @return vector of generated patterns.
	 */
	static std::vector< Pattern > generateRandomPatterns(int amount, int w, int h, float density);

	/**
	 * Mutate a series of patterns.
	 *
	 * @param patterns to mutate.
	 * @param mutation rate.
	 * @return new mutated patterns.
	 */
	static std::vector< Pattern > mutatePatterns( std::vector< Pattern > &patterns, float error);

	/**
	 * Load a series of patterns from a file. The format must use O (letter
	 * o) as the active nodes, and a space for inactive nodes. The patterns
	 * must be square and separated by a blank line.
	 *
	 * @param file to load 
	 * @param size equal to width or height (patterns must be square)
	 * @return the loaded patterns.
	 */
	static std::vector< Pattern > loadPatterns( std::string fn, int size);

};

#endif // HDEMO_PATTERNMATRIX_H
