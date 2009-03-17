#include "Vector3.h"
#include "LineSegment.h"

using namespace::Spatial::Math;

const Vector3 Vector3::ZERO;
const Vector3 Vector3::X_UNIT( 1, 0, 0 );
const Vector3 Vector3::Y_UNIT( 0, 1, 0 );
const Vector3 Vector3::Z_UNIT( 0, 0, 1 );
const Vector3 Vector3::NEG_X_UNIT( -1, 0, 0 );
const Vector3 Vector3::NEG_Y_UNIT( 0, -1, 0 );
const Vector3 Vector3::NEG_Z_UNIT( 0, 0, -1 );

const double LineSegment::TOLERANCE_DISTANCE = 0.001;
