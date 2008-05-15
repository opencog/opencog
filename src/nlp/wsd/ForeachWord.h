
#include <ForeachChaseLink.h>

namespace opencog {

/**
 * Call the callback for each parse in a sentence.  The argument handle
 * is presumed to identify  a SentenceNode, which is linked to parses 
 * via a ParseLink:
 * 
 *    <ParseLink>
 *      <ConceptNode name="parse_2" strength=0.8 confidence=0.5/>
 *      <SentenceNode name="sentence_22" />
 *    </ParseLink>
 */
template<class T>
inline void foreach_parse(Handle h, bool (T::*cb)(Handle), T *data)
{
	ForeachChaseLink<T> chase;
	chase.backtrack_binary_link(h, PARSE_LINK, cb, data);
}

#if 0
/**
 * Anotate every word in the given parse with every possible word sense
 * for that word. The argument handle is presumed to identify a specific
 * parse. Each word-instance in the parse is linked to it via a 
 * ParseInstanceLink:
 *
 *    <ParseInstanceLink>
 *       <ConceptNode name="bark_169" />
 *       <ConceptNode name="parse_3" />
 *    </ParseInstanceLink>
 */
template class<T>
inline void annotate_parse(Handle h)
{
	printf("found parse %x\n", (unsigned long) h);
	ForeachChaseLink<MihalceaLabel> chase;
	chase.backtrack_binary_link(h, PARSE_INSTANCE_LINK,
	                            &MihalceaLabel::annotate_word, this);
	return false;
}

/**
 * Anotate the given word with every possible word sense, given its 
 * part-of-speech. The argument handle is assumed to point at a specific 
 * word-instance in some parse.
 *
 * Each word-instance is assumed to be link to a single WordNode via 
 * a ReferenceLink:
 *
 *    <ReferenceLink>
 *      <ConceptNode name="bark_169" />
 *      <WordNode name="bark">
 *    </ReferenceLink>
 *
 * Each word-instance is assumed to be linked to a part-of-speech via
 *
 *    <PartOfSpeechLink>
 *       <ConceptNode name="bark_169" />
 *       <DefinedLinguisticConceptNode name="#noun" />
 *    </PartOfSpeechLink>
 *
 * Each dictionary-word is assumed to be linked to word senses via
 *
 *    <WordSenseLink>
 *       <WordNode name="bark" />
 *       <ConceptNode name="bark_sense_23" />
 *    </WordSenseLink>
 *  
 * Each word-sense is assumed to be linked to a prt-of-speech via
 *
 *    <PartOfSpeechLink>
 *       <ConceptNode name="bark_sense_23" />
 *       <ConceptNode name="noun" />
 *    </PartOfSpeechLink>
 *
 */
bool MihalceaLabel::annotate_word(Handle h)
{
	word_instance = TLB::getAtom(h);

	Atom *dict_word = fl.follow_binary_link(word_instance, REFERENCE_LINK);
	Handle dict_word_h = TLB::getHandle(dict_word);
n = dynamic_cast<Node *>(dict_word);
printf("found word-dict %s\n",  n->toString().c_str());
 
	ForeachChaseLink<MihalceaLabel> chase;
	chase.follow_binary_link(dict_word_h, WORD_SENSE_LINK,
	                            &MihalceaLabel::annotate_word_sense, this);
	return false;
}
#endif

}

/* ============================== END OF FILE ====================== */
