#include "Block3D.h"
#include "Block3DMapUtil.h"
#include "OctomapOctree.h"

using namespace opencog;
using namespace opencog::spatial;
using namespace octomap;

void OctomapOcTree::addSolidBlock( Block3D * _block,
								   bool atKnownIndexes, 
								   int _x, int _y, int _z)
{
	BlockVector vec(3,4,5);
	_block= new Block3D(1,vec,"testmaterial","testcolor");
}

bool OctomapOcTree::checkIsSolid(const BlockVector& _pos, Block3D* & _block3d) const
{
	BlockVector vec(3,4,5);
	_block3d= new Block3D(1,vec,"testmaterial","testcolor");
	return true;
}
