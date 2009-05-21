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

#include <sstream>
/*#include <iomanip>
#include <string>
#include <unistd.h>
#include <limits.h>
#include <tr1/functional>
using namespace std::tr1::placeholders;
*/
#include <vector>
#include <set>
#include <opencog/util/Logger.h>
/*#include <opencog/atomspace/utils.h>
#include <opencog/atomspace/Link.h>
#include <opencog/atomspace/Node.h>
#include <opencog/atomspace/TLB.h>
#include <opencog/server/CogServer.h>
*/

// just used for a haxx
#include <iostream>

extern "C"
{
#include <UbigraphAPI.h>
}
#include "BITUbigrapher.h"

//#include <opencog/reasoning/pln/PLN.h>
#include <opencog/reasoning/pln/AtomSpaceWrapper.h> // BackInferenceTreeNode.h breaks otherwise
#include <opencog/reasoning/pln/BackInferenceTreeNode.h>

using namespace std;

namespace opencog
{

// use StringManipulator
    template <typename T>
    string to_string ( T arg )
    {
        stringstream ss;
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

        int root_id = ( int ) root;
        int status = ubigraph_new_vertex_w_id ( root_id );
        if ( status )
            logger().error ( "Drawing BITNodeRoot: Status was %d", status );
        ubigraph_set_vertex_attribute ( root_id, "shape", "sphere" );
        ubigraph_set_vertex_attribute ( root_id, "color", "#ff0000" );
        //ubigraph_change_vertex_style(id, _____Style);
    }

// Actually just draws the args and children, doesn't draw the node itself
// children is a vector of arguments, each having a set of ParametrizedBITNode's
// ParametrizedBITNode is not a subclass of BITNode, it is a wrapper (i.e. they have a BITNode as a member).

// Sometimes it draws a node at the top (because its parent hasn't been established yet?)
    void BITUbigrapher::drawBITNode ( BITNode* node, vector<set<ParametrizedBITNode> > children )
    {
        //logger().fine("Drawing BITNode with %d args", children.size());
        cout << "Drawing BITNode with " << children.size() << " args" << endl;

        int node_id = ( int ) node;
//    int node_id = (int) (node->prover);

        // Since this draws BITNodes, and the children are actually ParametrizedBITNodes referring to BITNodes, the same BITNode may


//    foreach (set<ParametrizedBITNode> arg, children) {
        for ( int i = 0; i < children.size(); i++ )
        {
            cout << "Drawing BITNode arg #" << i << endl;
            // Display and attach that arg
            int arg_id = node_id + i + 1; // since a BITNode takes up a lot more than a few bytes presumably
            int status = ubigraph_new_vertex_w_id ( arg_id ); // TODO use a different id system
            if ( status )
                logger().error ( "Drawing arg: Status was %d", status );
            else
            {
                //        int arg_id = ubigraph_new_vertex();
                ubigraph_set_vertex_attribute ( arg_id, "label", toString ( i ).c_str() );
                status = ubigraph_new_edge ( node_id, arg_id );
                if ( status == -1 )
                {
                    logger().error ( "Attaching arg: Status was %d", status );
                }
                ubigraph_set_edge_attribute ( status, "oriented", "true" );
                ubigraph_set_edge_attribute ( status, "arrow", "true" );

                // for each of its children
                //        for (vector<set<ParametrizedBITNode> >::const_iterator i =  children.begin(); i!=children.end(); i++) {
                foreach ( const ParametrizedBITNode& child, children[i] )
                {
                    // Display and attach them
                    //            int child_id = (int) &child;
                    int child_id = ( int ) ( child.prover );
                    status = ubigraph_new_vertex_w_id ( child_id );
                    if ( status )
                        logger().error ( "Drawing BITNode child: Status was %d", status );
                    //else
                    //{
                    // Do this even if it failed, because it'll be attaching this child node to a different parent node than before
                        // Should be the child number!
                        //ubigraph_set_vertex_attribute(child_id, "label", toString(i).c_str());
                        ubigraph_set_vertex_attribute ( child_id, "shape", "sphere" );
                        ubigraph_set_vertex_attribute ( child_id, "color", "#00ff00" );

                        status = ubigraph_new_edge ( arg_id, child_id );
                        if ( status == -1 )
                        {
                            logger().error ( "Attaching child: Status was %d", status );
                        }
                        else
                        {
                            ubigraph_set_edge_attribute ( status, "oriented", "true" );
                            ubigraph_set_edge_attribute ( status, "arrow", "true" );
                        }
                    //}
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
