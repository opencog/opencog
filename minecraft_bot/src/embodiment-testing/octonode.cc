/*
 * OctoMap - An Efficient Probabilistic 3D Mapping Framework Based on Octrees
 * http://octomap.github.com/
 *
 * Copyright (c) 2009-2013, K.M. Wurm and A. Hornung, University of Freiburg
 * All rights reserved.
 * License: New BSD
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the University of Freiburg nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */
#include "ros/ros.h"
#include "sensor_msgs/PointCloud2.h"
#include "sensor_msgs/PointField.h"
#include <sstream>

#include <octomap/octomap.h>
#include <octomap/OcTree.h>
#include <octomap_ros/conversions.h>
//#include <octomap_msgs/*>

#include "minecraft_bot/visible_blocks_srv.h"
#include "minecraft_bot/map_block_msg.h"

using namespace std;
using namespace octomap;


void print_query_info(point3d query, OcTreeNode* node)
{
	if (node != NULL)
	{
		cout << "occupancy probability at " << query << ":\t " << node->getOccupancy() << endl;
  	}
  	else
		cout << "occupancy probability at " << query << ":\t is unknown" << endl;    
}

int main(int argc, char** argv)
{

  	cout << endl;
  	cout << "generating minecraft visibility data" << endl;
 	
	// create empty tree with resolution 1
  	// no need for higher resolution since all blocks in MC are 1x1 unit
	OcTree tree (1);

	//initialize the octovis listener	
	ros::init(argc, argv, "octomap_visualization_node");
	ros::NodeHandle n;
	ros::ServiceClient vis_client = n.serviceClient<minecraft_bot::visible_blocks_srv>("get_visible_blocks");
	minecraft_bot::visible_blocks_srv service;
	service.request.x = -29.2;
	service.request.y = 14;
	service.request.z = -41;
	service.request.pitch = 20.3;
	service.request.yaw = -92;

	ros::Publisher octo_pub = n.advertise<sensor_msgs::PointCloud2>("octovis_data", 10000);
	
	ros::Rate loop_rate(1);

	//int count = 0;
	
	while (ros::ok())
	{	
		//sensor_msgs::PointField pf;
		//pf.name = "just a name";
		//pf.offset = 0;
		//pf.datatype = FLOAT64;
		//pf.count = 1;

		if (vis_client.call(service))
		{
			cout << "gathering visible blocks";
			cout << endl;

			minecraft_bot::visible_blocks_srvResponse_<std::allocator<void> >::_visible_blocks_type
				blocks = service.response.visible_blocks;

			for (int i=0; i < blocks.size(); i++)
			{
				float x = blocks[i].x;
				float y = blocks[i].y;
				float z = blocks[i].z;
				cout << "x: " << x << "y: " << y << "z: " << z;

				point3d coords(x, y, z);
				tree.updateNode(coords, true);

			}
		}
				//sensor_msgs::PointCloud2 msg;

		//msg.header = "just a header";

		//msg.height = 1;
		//msg.width = 1;
		ros::spinOnce();
		loop_rate.sleep();
	}
	

  // insert some measurements of occupied cells
/*
  for (int x=-20; x<20; x++) {
    for (int y=-20; y<20; y++) {
      for (int z=-20; z<20; z++) {
        point3d endpoint ((float) x*0.05f, (float) y*0.05f, (float) z*0.05f);
        tree.updateNode(endpoint, true); // integrate 'occupied' measurement
      }
    }
  }

  cout << endl;
  cout << "performing some queries:" << endl;
  
  point3d query (0., 0., 0.);
  OcTreeNode* result = tree.search (query);
  print_query_info(query, result);

  query = point3d(-1.,-1.,-1.);
  result = tree.search (query);
  print_query_info(query, result);

  query = point3d(1.,1.,1.);
  result = tree.search (query);
  print_query_info(query, result);
*/

  cout << endl;
  tree.writeBinary("minecraft_tree.bt");
  cout << "wrote example file minecraft_tree.bt" << endl << endl;
  cout << "now you can use octovis to visualize: octovis minecraft_tree.bt"  << endl;
  cout << "Hint: hit 'F'-key in viewer to see the freespace" << endl  << endl;  

}

