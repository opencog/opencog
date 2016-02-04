/*
 *
 * Copyright (c) 2016, Mandeep Singh Bhatia, OpenCog Foundation
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
 
//TestAtomOcTree.cpp
#include "assert.h"
#include "TimeSpaceAtom.h"

using namespace std;
using namespace octomap;


void print_query_info(point3d query, aHandle ato) {
  if (ato != UndefinedHandle) {
    cout << "occupancy probability at " << query << ":\t " << ato<< endl;
  }
  else 
    cout << "occupancy probability at " << query << ":\t is unknown" << endl;    
}

int main(int argc, char** argv) {

  cout << endl;
  cout << "generating example map" << endl;

  double hi[4]={0.1,0.1,0.1,0.1};//0.2 failed
  vector<double> res(hi,hi+4);
  TimeSpaceAtom tsa(3,res);  // create empty tree with resolution 0.1
  time_pt t1=std::chrono::system_clock::now();
  duration_c dd=std::chrono::seconds(10);
  tsa.CreateNewTimeUnit(t1,dd);


  // insert some measurements of occupied cells

  for (int x=-20; x<20; x++) {
    for (int y=-20; y<20; y++) {
      for (int z=-20; z<20; z++) {
        point3d endpoint ((float) x*0.05f, (float) y*0.05f, (float) z*0.05f);
        assert(tsa.PutAtomAtCurrentTime(1,endpoint,21));
        //tree.updateNode(endpoint, true); // integrate 'occupied' measurement
        //tree.setNodeData(endpoint, 21);//if omitted prune value of 0 is assigned?
      }
    }
  }
/*//removal is probabilistic
  for (int x=-20; x<20; x++) {
    for (int y=-20; y<20; y++) {
      for (int z=-20; z<20; z++) {
        point3d endpoint ((float) x*0.05f, (float) y*0.05f, (float) z*0.05f);
        assert(tsa.RemoveAtomAtTime(t1,1,endpoint));
        //tree.updateNode(endpoint, true); // integrate 'occupied' measurement
        //tree.setNodeData(endpoint, 21);//if omitted prune value of 0 is assigned?
      }
    }
  }
*/
  // insert some measurements of free cells
// Need to put something to delete atom
  for (int x=-30; x<30; x++) {
    for (int y=-30; y<30; y++) {
      for (int z=-30; z<30; z++) {
        point3d endpoint ((float) x*0.02f-1.0f, (float) y*0.02f-1.0f, (float) z*0.02f-1.0f);
        tsa.RemoveAtomAtCurrentTime(1,endpoint);
        //tree.updateNode(endpoint, false);  // integrate 'free' measurement
      }
    }
  }

  cout << endl;
  cout << "performing some queries:" << endl;
  aHandle result;
  point3d query (0., 0., 0.);
//tsa.RemoveAtomAtCurrentTime(1,query);  
  tsa.GetAtomCurrentTime(1,query,result);
  print_query_info(query, result);

  query = point3d(-1.,-1.,-1.);
  tsa.GetAtomCurrentTime(1,query,result);
  print_query_info(query, result);

  query = point3d(100.,1.,1.);
  tsa.GetAtomCurrentTime(1,query,result);
  print_query_info(query, result);

  cout<<"at time:";
  query = point3d(0., 0., 0.);
  tsa.GetAtomAtTime(t1,1,query,result);
  print_query_info(query, result);
  
  cout<<"get times at location:";
  TimeList tl=tsa.GetTimesOfAtomOccurenceAtLocation(1,query,21);
  cout<<tl.size()<<endl;

  cout<<"get times in Map:";
  tl=tsa.GetTimesOfAtomOccurenceInMap(1,21);
  cout<<tl.size()<<endl;
  
  cout<<"get locations at current time:";
  point3d_list pl=tsa.GetLocationsOfAtomOccurenceNow(1,21);
  cout<<pl.size()<<endl;
/*
  cout<<"get locations at time:";
  pl=tsa.GetLocationsOfAtomOccurenceAtTime(t1,1,21);
  cout<<pl.size()<<endl;
*/
  cout<<"removing atom=21"<<endl;
  tsa.RemoveAtom(21);
  query = point3d(0., 0., 0.);
  tsa.GetAtomCurrentTime(1,query,result);
  print_query_info(query, result);
  
  cout<<"get locations at time:";
  pl=tsa.GetLocationsOfAtomOccurenceAtTime(t1,1,21);
  cout<<pl.size()<<endl;

  cout<<"adding units.."<<endl;
  assert(tsa.CreateNewTimeUnit(t1+dd,dd));
  assert(tsa.CreateNewTimeUnit(t1+dd+dd,dd));
  assert(tsa.CreateNewTimeUnit(t1+dd+dd+dd,dd));
  
  cout<<"get locations at time:";
  pl=tsa.GetLocationsOfAtomOccurenceAtTime(t1,1,21);
  cout<<pl.size()<<endl;
/*
  cout<<"removing atom=21"<<endl;
  tsa.RemoveAtom(21);
  query = point3d(0., 0., 0.);
  tsa.GetAtomCurrentTime(1,query,result);
  print_query_info(query, result);
*/
  cout << endl;
  /*
  tree.writeBinary("simple_tree.bt");
  cout << "wrote example file simple_tree.bt" << endl << endl;
  cout << "now you can use octovis to visualize: octovis simple_tree.bt"  << endl;
  cout << "Hint: hit 'F'-key in viewer to see the freespace" << endl  << endl;  
  */
  return 0;
}
