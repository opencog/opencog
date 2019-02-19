#include <iostream>

#include <opencog/atomspace/AtomSpace.h>
#include <opencog/attentionbank/bank/AttentionBank.h>
#include <opencog/atoms/truthvalue/SimpleTruthValue.h>

using namespace opencog;
using namespace std::placeholders;

/**
 * Sample code that shows how to listen for AtomSpace related events.
 * The following are all of the events/signals emitted by the AtomSpace.
 */

void AtomAddedCBHandler(const Handle& h);
void AVChangedCBHandler(const Handle& h, const AttentionValuePtr& av_old,
                        const AttentionValuePtr& av_new);
void TVChangedCBHandler(const Handle& h, const TruthValuePtr& tv_old,
                        const TruthValuePtr& tv_new);
void AtomRemovedCBHandler(const AtomPtr&);
void AtomAddedToAFCBHandler(const Handle& h, const AttentionValuePtr& av_old,
                            const AttentionValuePtr& av_new);
void AtomRemovedFromAFCBHandler(const Handle& h,
                                const AttentionValuePtr& av_old,
                                const AttentionValuePtr& av_new);

int main(int argc, char **args)
{
	AtomSpace as;
	AttentionBank& bank(attentionbank(&as));

	// Register Atomspace and AttentionBank event callback handlers.
	as.atomAddedSignal().connect(std::bind(&AtomAddedCBHandler, _1));
	bank.getAVChangedSignal().connect(
		std::bind(&AVChangedCBHandler, _1, _2, _3));
	as.TVChangedSignal().connect(std::bind(&TVChangedCBHandler, _1, _2, _3));
	as.atomRemovedSignal().connect(
		std::bind(&AtomRemovedCBHandler, _1));
	bank.AddAFSignal().connect(
		std::bind(&AtomAddedToAFCBHandler, _1, _2, _3));
	bank.RemoveAFSignal().connect(
		std::bind(&AtomRemovedFromAFCBHandler, _1, _2, _3));

	// create atoms
	Handle h = as.add_node(CONCEPT_NODE, "Cat");
	Handle h1 = as.add_node(CONCEPT_NODE, "Human");
	Handle h2 = as.add_node(CONCEPT_NODE, "Animal");

	// Links can be ordered or unordered.InheritanceLink is a type of ordered link
	// Which means its element's ( outgoing sets) order is kept intact.
	HandleSeq hseq = {h1, h2};
	Handle hinheritance = as.add_link(INHERITANCE_LINK, hseq);

	// Update the atom's truth value.
	SimpleTruthValuePtr tvinheritance =
		std::make_shared<SimpleTruthValue>(0.5, 100);
	hinheritance->setTruthValue(tvinheritance);

	// Update the atom's Attention value. Attention values are managed
	// by ECAN agents. See
	// http://wiki.opencog.org/w/Attention_Allocation
	bank.change_av(hinheritance, AttentionValue::createAV(15, 30, 45));

	// ListLink is an example of unordered type of Link.
	// Thus order is not guaranteed in this case.
	hseq = {h, h2};
	Handle hllink = as.add_link(LIST_LINK, hseq);

	// Set the recursive delete flag to delete h including
	// everything pointing to it.
	assert(as.remove_atom(h, true));

	return 0;
}


void AtomAddedCBHandler(const Handle& h)
{
	std::cout << "An atom added.\n" << h->to_short_string() << std::endl;
}

void AVChangedCBHandler(const Handle& h, const AttentionValuePtr& av_old,
                        const AttentionValuePtr& av_new)
{
	std::cout << "An atom's attention value changed.\n" << h->to_short_string()
	          << std::endl;
}

void TVChangedCBHandler(const Handle& h, const TruthValuePtr& tv_old,
                        const TruthValuePtr& tv_new)
{
	std::cout << "An atom's truth value changed.\n" << h->to_short_string()
	          << std::endl;
}

void AtomRemovedCBHandler(const AtomPtr& a)
{
	std::cout << "An atom is removed.\n" << std::endl;
}

void AtomAddedToAFCBHandler(const Handle& h, const AttentionValuePtr& av_old,
                            const AttentionValuePtr& av_new)
{
	std::cout << "An atom was added to Attentional Focus.\n"
	          << h->to_short_string() << std::endl;
}

void AtomRemovedFromAFCBHandler(const Handle& h,
                                const AttentionValuePtr& av_old,
                                const AttentionValuePtr& av_new)
{
	std::cout << "An atom was removed from Attentional Focus.\n"
	          << h->to_short_string() << std::endl;
}
