#include <platform.h>
#include <CogServer.h>
#include <Node.h>
#include <Link.h>
#include <string>
#include <queue>

using namespace opencog;

class DottyGrapher
{
public:
    DottyGrapher(AtomSpace *space) : space(space), withIncoming(false) { answer = ""; }
    AtomSpace *space;
    std::string answer;
    bool withIncoming;

    /**
     * Outputs a dotty node for an atom.
     */
    bool do_nodes(Atom *a)
    {
        std::ostringstream ost;
        ost << TLB::getHandle(a) << " [";
        if (!space->isNode(a->getType()))
            ost << "shape=\"diamond\" ";
        ost << "label=\"[" << ClassServer::getTypeName(a->getType()) << "]";
        if (space->isNode(a->getType())) {
            Node *n = (Node*)a;
            ost << " " << n->getName();
        } else {
            Link *l = (Link*)a;
            l = l; // TODO: anything to output for links?
        }
        ost << "\"];\n";
        answer += ost.str();
        return false;
    }

    /**
     * Outputs dotty links for an atom's outgoing connections.
     */
    bool do_links(Atom *a)
    {
        Handle h = TLB::getHandle(a);
        std::ostringstream ost;
        const std::vector<Handle> &out = a->getOutgoingSet();
        for (size_t i = 0; i < out.size(); i++) {
            ost << h << "->" << out[i] << " [label=\"" << i << "\"];\n";
        }

        if (withIncoming) {
            HandleEntry *he = a->getIncomingSet();
            int i = 0;
            while (he) {
                ost << h << "->" << he->handle << " [style=\"dotted\" label=\"" << i << "\"];\n";
                he = he->next;
                i++;
            }
        }

        answer += ost.str();
        return false;
    }

    void graph()
    {
        answer += "\ndigraph OpenCog {\n";
        space->getAtomTable().foreach_atom(&DottyGrapher::do_nodes, this);
        space->getAtomTable().foreach_atom(&DottyGrapher::do_links, this);
        answer += "}\n";
    }

};

extern "C" std::string cmd_dotty(std::queue<std::string> &args)
{
    AtomSpace *space = CogServer::getAtomSpace();
    DottyGrapher g(space);
    while (!args.empty()) {
        if (args.front() == "with-incoming")
            g.withIncoming = true;
        args.pop();
    }
    g.graph();
    return g.answer;
}
