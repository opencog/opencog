#include "Block3DMapUtil.h"
#include <climits>

using namespace opencog;
using namespace opencog::spatial;


const BlockVector BlockVector::ZERO(0,0,0);
const BlockVector BlockVector::X_UNIT(1,0,0);
const BlockVector BlockVector::Y_UNIT(0,1,0);
const BlockVector BlockVector::Z_UNIT(0,0,1);
const BlockVector BlockVector::NEG_X_UNIT(-1,0,0);
const BlockVector BlockVector::NEG_Y_UNIT(0,-1,0);
const BlockVector BlockVector::NEG_Z_UNIT(0,0,-1);

const AxisAlignedBox AxisAlignedBox::ZERO((BlockVector&)BlockVector::ZERO,0);




