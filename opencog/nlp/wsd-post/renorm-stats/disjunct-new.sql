--
-- disjunct.sql
-- Tables for holding link-grammar disjunct-usage statistics.
-- The actual entries are computed by opencog MindAgents.
--
-- Conditional probablities, entropies, etc. are computed by
-- opencog/nlp/scm/compute-mi.scm
-- opencog/nlp/wsd-post/synset-renorm.pl
-- opencog/nlp/wsd-post/dj-probs.pl
--
-- The inflected word is the word, as it would appear in a link-grammar
-- dictionary. The disjunct the a list of connectors used. The count is
-- how often a particular word-disjunct pair has been seen in the text.
--
-- log_probability is minus the log_2 of the probabilty of seeing this
-- word-disjunct pair (i.e. the -log_2 of (count divided by the total count))
--
-- log_cond_probability is minus the log_2 of the conditional probabilty
-- of seeing this disjunct, given this inflected word.
--
-- entropy is equal to - \sum_s p(s|d,w) log_2 p(s|d,w) 
-- where s==sense, d==disjunct, w==word, and the sum is over senses.
--
-- senses_observed is a count of the number of different senses seen for
-- this particular word-disjunct pair.
--
-- sense_count is a total of the count column from the NewDisjunctSenses
-- table. This will be smaller than count, because sense-counting is 
-- done using a completely different manner.
--
-- sense_obscnt is a total of the obscnt column from the NewDisjunctSenses
-- table. This will be smaller than obscnt, because not all words have 
-- senses that get observed.
--
CREATE TABLE NewDisjuncts (
	inflected_word TEXT NOT NULL,
	disjunct TEXT NOT NULL,
	count FLOAT,
	obscnt INT,
	log_probability FLOAT,
	log_cond_probability FLOAT,
	sense_count FLOAT,
	sense_obscnt INT,
	senses_observed INT,
	entropy FLOAT
);

CREATE INDEX ndiw ON NewDisjuncts (inflected_word);
CREATE UNIQUE INDEX ndiwdj ON NewDisjuncts (inflected_word, disjunct);

-- The InflectMarginal table simply stores a count of how often a given
-- inflected word was seen. This is required for normalizing the entries
-- in the disjunct table. The log_probability field holds minus the
-- log_2 probability of seeing this word.
--
CREATE TABLE NewInflectMarginal (
	inflected_word TEXT NOT NULL UNIQUE,
	count FLOAT,
	obscnt INT,
	log_probability FLOAT,
	PRIMARY KEY (inflected_word)
);

-- The DisjunctSenses table associates word senses to disjuncts
-- The log_cond_probability field records the (minus log_2) 
-- conditional probability of seeing this sense, given a particular
-- word and disjuncte. The "marginal" variant of this table, which
-- sums/averages over the senses, is the NewDisjuncts table, above.
-- 
CREATE TABLE NewDisjunctSenses (
	word_sense TEXT NOT NULL,
	inflected_word TEXT NOT NULL,
	disjunct TEXT NOT NULL,
	count FLOAT,
	obscnt INT,
	log_cond_probability FLOAT
);

CREATE INDEX nsds ON NewDisjunctSenses (word_sense);
CREATE INDEX nsdiw ON NewDisjunctSenses (inflected_word);
CREATE INDEX nsdd ON NewDisjunctSenses (disjunct);
CREATE UNIQUE INDEX nsdu ON NewDisjunctSenses (word_sense,inflected_word, disjunct);

-- XXX Is this table good for anything? Its a marginal ... 
-- but doesn't seem to be a useful marginal ??
-- Table counting frequency of occurence of a word sense, associated with
-- a particular inflected word. 
-- log_probability is the minus log_2 of the probability of seeing this
--           particular word_sense/inflected_word pair.
-- The log_cond_probability is the log_2 conditional probability of seeing
-- this particular word sense, given the inflected word.
--
CREATE TABLE NewWordSenseFreq (
	word_sense TEXT NOT NULL,
	inflected_word TEXT NOT NULL,
	count FLOAT,
	obscnt INT,
	log_probability FLOAT,
	log_cond_probability FLOAT
);

CREATE INDEX nwsds ON NewWordSenseFreq (word_sense);
CREATE INDEX nwsdiw ON NewWordSenseFreq (inflected_word);
CREATE UNIQUE INDEX nwsdu ON NewWordSenseFreq (word_sense,inflected_word);
