#include "AtomspaceHTabler.h"
#include "opencog/atomspace/Atom.h"
#include "opencog/atomspace/Link.h"
#include "opencog/atomspace/Node.h"
#include "opencog/atomspace/SimpleTruthValue.h"
#include "opencog/atomspace/TLB.h"

using namespace opencog;

int atomCompare(Atom *a, Atom *b) //TODO: Test name, use == ?
{
    int rc = 0;
    if (NULL == b) {
        fprintf(stderr, "Error: No atom found\n");
        return -1;
    }

    if (a->getType() != b->getType()) {
        fprintf(stderr, "Error, type mis-match, a=%d b=%d\n", a->getType(), b->getType());
        --rc;
    }
    Link *na = dynamic_cast<Link *>(a);
    Link *nb = dynamic_cast<Link *>(b);
    if (na && nb) {
        if (na->getArity() != nb->getArity()) {
            fprintf(stderr, "Error, arity mis-match, a=%d b=%d\n", na->getArity(), nb->getArity());
            --rc;
        }
            if (0 < na->getArity()) {
            std::vector<Handle> outa = na->getOutgoingSet();
            std::vector<Handle> outb = nb->getOutgoingSet();
            for (int i =0; i< na->getArity(); i++) {
                if (outa[i] != outb[i]) {
                    fprintf(stderr, "Error, outgoing set mis-match:");
                    fprintf(stderr, "i=%d a[i]=%ld b[i]=%ld\n",i,
                            outa[i].value(), outb[i].value());
                    --rc;
                }
            }
        }
    }
    if (!(a->getTruthValue() == b->getTruthValue())) {
        const TruthValue &ta = a->getTruthValue();
        const TruthValue &tb = b->getTruthValue();
        fprintf(stderr, "Error, truth value mis-match, "
                "ma=%f mb=%f ca=%f cb=%f\n",
                ta.getMean(), tb.getMean(), ta.getCount(), tb.getCount());
        --rc;
    }
    if (!(a->getAttentionValue() == b->getAttentionValue())) {
        const AttentionValue &ava = a->getAttentionValue();
        const AttentionValue &avb = b->getAttentionValue();
        fprintf(stderr, "Error, attention value mis-match, "
                "stia=%hd stib=%hd ltia=%hd ltib=%hd vltia=%hd vltib=%hd\n",
                ava.getSTI(), avb.getSTI(), ava.getLTI(), avb.getLTI(),
                ava.getVLTI(), avb.getVLTI());
        --rc;
    }
    return rc;
}

int main (int argc, char **argv){
    try {
        AtomspaceHTabler table;

	    // Create an atom ... 
	    Atom *a = new Node(SCHEMA_NODE, "someNode");
	    SimpleTruthValue stv(0.55, 0.6);
	    a->setTruthValue(stv);
	    const AttentionValue av(42, 64, 1);
	    a->setAttentionValue(av);
	    TLB::addAtom(a);
	    Handle h = TLB::getHandle(a);
	
	    // Store it
	    table.storeAtom(h);

	    // Get it back
	    Atom *b = table.getAtom(h);
	    
		    // Are they equal?
        int rc = atomCompare(a,b);
        if (!rc) {
            printf("node compare success\n");
        }
        else {
            printf("node compare failure\n");
        }
        
        // Get it back through get node
        Atom *c = table.getNode(a->getType(), ((Node *)a)->getName().c_str());
        rc = atomCompare(a,c);
        if (!rc) {
            printf("node compare success\n");
        }
        else {
            printf("node compare failure\n");
        }
        
        // Create a second atom, connect it to the first
	    // with a link. Save it, fetch it ... are they equal?
	    Atom *a2 = new Node(SCHEMA_NODE, "otherNode");
	    TLB::addAtom(a2);
	    table.storeAtom(TLB::getHandle(a2));

	    std::vector<Handle> hvec;
	    hvec.push_back(TLB::getHandle(a));
	    hvec.push_back(TLB::getHandle(a2));

	    Link *l = new Link(SET_LINK, hvec);
	    TLB::addAtom(l);
	    table.storeAtom(TLB::getHandle(l));

	    Atom *lb = table.getAtom(TLB::getHandle(l));
	    rc = atomCompare(l,lb);
	    if (!rc) {
            std::cout << "link compare success" << std::endl;
	    }
	    else {
	        std::cout << "link compare failure" << std::endl;
	    }
	    
	    // Get the link back through get link
	    Atom *lc = table.getLink(l->getType(), l->getOutgoingSet());
	    rc = atomCompare(l,lc);
	    if (!rc) {
            std::cout << "link compare success" << std::endl;
	    }
	    else {
	        std::cout << "link compare failure" << std::endl;
	    }
	        
	    return 0;
	} catch (Exception &e) {
	    std::cerr << e << std::endl;
	    return 1;
	}
}
