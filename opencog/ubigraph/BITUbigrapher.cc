/*
 * opencog/ubigraph/BITUbigrapher.cc
 *
 * Copyright (C) 2008-2009 by Singularity Institute for Artificial Intelligence
 * All Rights Reserved
 *
 * Written by Jared Wigmore <jared.wigmore@gmail.com>
 * Some code adapted from Ubigrapher
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU Affero General Public License v3 as
 * published by the Free Software Foundation and including the exceptions
 * at http://opencog.org/wiki/Licenses
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU Affero General Public License
 * along with this program; if not, write to:
 * Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
 */

#include "BITUbigrapher.h"

#ifdef USE_BITUBIGRAPHER

#include <limits.h>
#include <ctype.h>
#include <sstream>
#include <unistd.h>
#include <opencog/util/Logger.h>

extern "C"
{
#include "UbigraphAPI.h"
}

using namespace std;

// TODO duplicated from Ubigrapher.cc
std::string initials(std::string s)
{
    std::string ret;
    foreach (char c,  s) {
        if (/*isUpperCase(c)*/ toupper(c) == c) {
            ret += c;
        }
    }
    return ret;
}


namespace opencog
{

// use StringManipulator
template <typename T>
string to_string ( T arg )
{
    ostringstream ss;
    ss << arg;
    return ss.str();
}

BITUbigrapher::BITUbigrapher()
{
    ubigraph_clear();
    setStyles();
}

void BITUbigrapher::setStyles() {}

void BITUbigrapher::drawRoot ( BITNode* root )
{
    cout << "drawRoot" << endl;

    //int root_id = ( int ) root;
    int root_id = ( int ) (((long)root) % INT_MAX); //! @todo haxx:: use a map of BITNodes to ids, or something.
    int status = ubigraph_new_vertex_w_id ( root_id );
    if ( status )
        logger().error ( "Drawing BITNodeRoot: Status was %d", status );
    ubigraph_set_vertex_attribute ( root_id, "shape", "sphere" );
    ubigraph_set_vertex_attribute ( root_id, "color", "#ff0000" );
    //ubigraph_change_vertex_style(id, _____Style);
}

// Uses Node ids that are 32-bit ints (required by ubigraph)
void BITUbigrapher::drawBITNodeFitness(int node_id, float fitness)
{
    // Since the exact value would be too precise to visualize;
    // also C++ modulus doesn't work on floats
    int approx_fitness = (int) fitness;
    cout << "approx fitness: " << approx_fitness << endl;
    float norm_fitness; // A form of normalized fitness
    norm_fitness = (approx_fitness % 100) / 10.0f + 10;
//        cout << "norm_fitness (before scaling): " << norm_fitness << endl;
//        norm_fitness /= 10;

    cout << "Normalized fitness: " << norm_fitness << endl;

    // The ubigraph size of the vertex for this node
    double size;
    if (norm_fitness < 0.1) { // Happens with the BITNodeRoot
        size = 1.0;
    } else {
        size = 4 * norm_fitness;
    }

    std::ostringstream ost;
    ost << size;
    //ubigraph_set_vertex_attribute(node_id, "size", ost.str().c_str());
}

void BITUbigrapher::drawBITNodeLabel(BITNode * node, int node_id)
{
    // The BITNodeRoot and (I think) the root variable scoper don't have a rule
    if (node->rule != NULL) {
        //ostringstream label;
        string label;
        label = node->rule->name;
        //label << node->rules
        if (label.rfind("Rule") != string::npos ||
                label.rfind("(=>)") != string::npos)
            label.erase(label.size() - 4 - 1);
        label = initials(label);

        ubigraph_set_vertex_attribute(node_id, "label", label.c_str());
    }
}

// Actually just draws the args and children, doesn't draw the node itself.
// children is a vector of arguments, each having a set of ParametrizedBITNode's
// ParametrizedBITNode is not a subclass of BITNode, it is a wrapper (i.e. they have a BITNode as a member).

// Sometimes it draws a node at the top (because its parent hasn't been established yet. Possibly because of the BITNode cloning). Might need to call the function from more places in the BITNode/BITNodeRoot code.

void BITUbigrapher::drawBITNode ( BITNode* node, vector<set<ParametrizedBITNode> > children )
{
    //logger().fine("Drawing BITNode with %d args", children.size());
    cout << "Drawing BITNode with " << children.size() << " args" << endl;

    //int node_id = ( int ) node;
    int node_id = ( int ) (((long)node) % INT_MAX);

    // For some reason the fitness doesn't work on the BITNodeRoot
    if (node->root != node) {
        cout << "node fitness: " << node->fitness() << endl;

        drawBITNodeFitness(node_id, node->fitness());
        drawBITNodeLabel(node, node_id);
    }

    // Remember that this draws BITNodes, and the children are actually ParametrizedBITNodes referring to (often shared) BITNodes
    for (unsigned int i = 0; i < children.size(); i++ ) {
        cout << "Drawing BITNode arg #" << i << endl;
        // Display and attach that arg
        unsigned int arg_id = node_id + i + 1; // haxx:: since a BITNode takes up a lot more than a few bytes presumably
        int status = ubigraph_new_vertex_w_id ( arg_id ); // TODO use a different id system
        if ( status )
            logger().error ( "Drawing arg: Status was %d", status );
        else {
            //        int arg_id = ubigraph_new_vertex();
            ostringstream oss;
            oss << "#" << i;
            cout << oss.str() << endl;
            //ubigraph_set_vertex_attribute ( arg_id, "label", toString ( i ).c_str() );
            ubigraph_set_vertex_attribute ( arg_id, "label", oss.str().c_str() );
            status = ubigraph_new_edge ( node_id, arg_id );
            if ( status == -1 ) {
                logger().error ( "Attaching arg: Status was %d", status );
            } else {
                ubigraph_set_edge_attribute ( status, "oriented", "true" );
                ubigraph_set_edge_attribute ( status, "arrow", "true" );
            }

            // for each of its children
            foreach ( const ParametrizedBITNode& child, children[i] ) {
                // Display and attach them
                //int child_id = ( int ) ( child.prover );
                int child_id = ( int ) (((long)child.prover) % INT_MAX);
                status = ubigraph_new_vertex_w_id ( child_id );
                if ( status )
                    logger().error ( "Drawing BITNode child: Status was %d", status );

                // Do this even if it failed, because it'll be attaching this child node to a different parent node than before

                // Should be the child number!
                //ubigraph_set_vertex_attribute(child_id, "label", toString(i).c_str());
                ubigraph_set_vertex_attribute ( child_id, "shape", "sphere" );
                ubigraph_set_vertex_attribute ( child_id, "color", "#00ff00" );

                cout << "child-node fitness: " << child.prover->fitness() << endl;
                drawBITNodeFitness(child_id, child.prover->fitness());
                drawBITNodeLabel(child.prover, child_id);

                status = ubigraph_new_edge ( arg_id, child_id );
                if ( status == -1 ) {
                    logger().error ( "Attaching child: Status was %d", status );
                } else {
                    ubigraph_set_edge_attribute ( status, "oriented", "true" );
                    ubigraph_set_edge_attribute ( status, "arrow", "true" );
                }
            }
        }
    }
}

void BITUbigrapher::graph()
{
    ubigraph_clear();
    setStyles();
}

} // namespace opencog

#endif // USE_BITUBIGRAPHER

