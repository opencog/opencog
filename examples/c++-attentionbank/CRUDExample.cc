#include <iostream>

#include <opencog/atomspace/AtomSpace.h>
#include <opencog/attentionbank/bank/AttentionBank.h>
#include <opencog/attentionbank/avalue/AttentionValue.h>
#include <opencog/atoms/truthvalue/SimpleTruthValue.h>
#include <opencog/atoms/truthvalue/TruthValue.h>

using namespace opencog;

/**
 * Sample code that demonstrates CRUD operations.
 * XXX FIXME what the heck does CRUD mean??
 */
int main(int argc, char ** args)
{
    AtomSpace as;
    AttentionBank& bank(attentionbank(&as));

    // create atoms
    Handle h = as.add_node(CONCEPT_NODE, "Cat");
    Handle h1 = as.add_node(CONCEPT_NODE, "Human");
    Handle h2 = as.add_node(CONCEPT_NODE, "Animal");

    // Links can be ordered or unordered. InheritanceLink is a type
    // of ordered link, which means the order of it's elements (the
    // outgoing set) is kept intact.
    HandleSeq hseq = { h1, h2 };
    Handle hinheritance = as.add_link(INHERITANCE_LINK, hseq);

    // Update atom's truth value
    SimpleTruthValuePtr tvinheritance(new SimpleTruthValue(0.5, 100));
    hinheritance->setTruthValue(tvinheritance);

    // Update an atom's Attention value. Attention values are managed by
    // ECAN agents. See
    // http://wiki.opencog.org/w/Attention_Allocation
    bank.change_av(hinheritance, AttentionValue::createAV(15, 30, 45));

    // ListLink is an example of unordered type of Link.
    // Thus order is not guaranteed in this case.
    hseq = {h, h2};
    Handle hllink = as.add_link(LIST_LINK, hseq);

    // Two ways to get outgoing set of a Link and incoming set of a node.
    HandleSeq outgoing = hinheritance->getOutgoingSet();

    outgoing = h->getOutgoingSet();

    opencog::IncomingSet incoming = h->getIncomingSet();

    // Atoms that have incoming Links cannot be deleted before the
    // links are deleted first.
    assert(not as.remove_atom(h));
    // Set the recursive delete flag to delete h including everything
    // pointing to it.
    assert(as.remove_atom(h, true));

    // Creating hierarchical atomspace
    // http://wiki.opencog.org/w/Multiple_AtomSpaces#Hierarchical_AtomSpaces
    AtomSpace child_as(&as);
    child_as.add_node(CONCEPT_NODE, "Cat"); // Not visible to parent.

    // Print outs of the content of the two atomspaces
    std::cout << "Main atomspace:" << std::endl << as << std::endl
              << "Child atomspace: " << std::endl << child_as << std::endl;

    return 0;
}
