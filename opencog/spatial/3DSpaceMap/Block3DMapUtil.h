#ifndef _SPATIAL_BLOCK3DMAPUTIL_H
#define _SPATIAL_BLOCK3DMAPUTIL_H

#define BLOCK_LIST            "block-list" // all the unit block nodes in an entity

#include <sstream>
#include <math.h>
#include <vector>

using namespace std;

namespace opencog
{
/** \addtogroup grp_spatial
 *  @{
 */

    namespace spatial
    {

        class BlockVector
        {
        public:
            static const BlockVector ZERO;
            static const BlockVector X_UNIT;
            static const BlockVector Y_UNIT;
            static const BlockVector Z_UNIT;
            static const BlockVector NEG_X_UNIT;
            static const BlockVector NEG_Y_UNIT;
            static const BlockVector NEG_Z_UNIT;

            double x;
            double y;
            double z;
            BlockVector()
                { x = 0.0; y = 0.0; z = 0.0; }
            BlockVector(double _x, double _y, double _z)
                { x = _x; y = _y; z = _z; }
            BlockVector(const BlockVector& other)
                { x = other.x; y = other.y; z = other.z; }

            inline BlockVector& operator = ( const BlockVector& other )
            {
                x = other.x;
                y = other.y;
                z = other.z;
                return *this;
            }

            static inline bool eq (double a, double b)
                { return fabs(a-b) < 1.0e-6; }

            inline bool operator == (const BlockVector& other) const
            {
                if (eq(other.x, x) and eq(other.y, y) and eq(other.z, z))
                    return true;
                else
                    return false;
            }

            inline bool operator != (const BlockVector& other) const
            {
                return not this->operator==(other);
            }

            inline BlockVector operator + (const BlockVector& other) const
            {
                return BlockVector(x + other.x, y + other.y, z + other.z);
            }

            inline void operator += (const BlockVector& other)
            {
                 x += other.x;
                 y += other.y;
                 z += other.z;
            }

            // the distance of this two position
            inline double operator - (const BlockVector& other) const
            {

                    return sqrt ( (x - other.x )*(x - other.x ) +
                                  (y - other.y )*(y - other.y ) +
                                  (z - other.z )*(z - other.z ));
            }

            // It doesn't make sense for the operator < in BlockVectors,
            // - we implement operator < for BlockVector just in case
            // it would be used as key of map
            inline bool operator < (const BlockVector& other) const
            {
                // XXX FIXME this should use eq
                if ((x < other.x) )
                    return true;
                else if (x > other.x)
                    return false;
                else
                {
                    if (y < other.y)
                        return true;
                    else if (y > other.y)
                        return false;
                    else
                    {
                        if (z < other.z)
                            return true;
                        else
                            return false;
                    }
                }
            }

            inline bool isFaceTouching(const BlockVector& other) const
            {
                if ( (eq(x, other.x) and eq(y, other.y) and
                     (eq(z, other.z - 1) or eq(z, other.z + 1)))
                   or (eq(x, other.x) and eq(z, other.z) and
                     (eq(y, other.y - 1) or eq(y, other.y + 1)))
                   or (eq(y, other.y) and eq(z, other.z) and
                      (eq(x, other.x - 1) or eq(x, other.x + 1))) )
                    return true;
                else
                    return false;
            }

            inline bool isSideTouching(const BlockVector& other) const
            {
                if (  (eq(x, other.x) and
                      (eq(y, other.y - 1) or eq(y, other.y + 1)) and
                      (eq(z, other.z - 1) or eq(z, other.z + 1)))
                   or (eq(z, other.z) and
                      (eq(y, other.y - 1) or eq(y, other.y + 1)) and
                      (eq(x, other.x - 1) or eq(x, other.x + 1)))
                   or (eq(y, other.y) and
                      (eq(z, other.z - 1) or eq(z, other.z + 1)) and
                      (eq(x, other.x - 1) or eq(x, other.x + 1))))
                    return true;
                else
                    return false;
            }

            inline bool isCornerTouching(const BlockVector& other) const
            {
                if ( (eq(x, other.x - 1) or eq(x, other.x + 1)) and
                     (eq(y, other.y - 1) or eq(y, other.y + 1)) and
                     (eq(z, other.z - 1) or eq(z, other.z + 1)) )
                    return true;
                else
                    return false;
            }

            inline std::string toString( ) const
            {
                std::stringstream response;
                response << x << " " << y << " " << z;
                return response.str( );
            }
         };

        // when size != 0, it is a cube, cube is usually used in blocks and octrees
        // when size == 0 ,it's not a cube, it is usually used in Entities
        class AxisAlignedBox
        {
        public:
            static const AxisAlignedBox ZERO;

            BlockVector nearLeftBottomConer;
            int size; // how many units per edge, it's usually zero, unless it's a cube
            int size_x;
            int size_y;
            int size_z;

            AxisAlignedBox(BlockVector& _neaLeftBottomPos, int _size_x, int _size_y, int _size_z ):
                size_x(_size_x), size_y(_size_y), size_z(_size_z)
            {
                nearLeftBottomConer = BlockVector(_neaLeftBottomPos);
                if ((size_x == size_y) and (size_x == size_z))
                    size = size_x;
                else
                    size = 0;
            }

            AxisAlignedBox():
            size(0), size_x(0), size_y(0), size_z(0)
            {nearLeftBottomConer = BlockVector::ZERO;}

            AxisAlignedBox(BlockVector& _neaLeftBottomPos, int _size = 1):
            size(_size), size_x(_size), size_y(_size), size_z(_size)
            {
                nearLeftBottomConer = BlockVector(_neaLeftBottomPos);
            }

            BlockVector getCenterPoint() const
            {
                return BlockVector((nearLeftBottomConer.x + 0.5*size_x),
                                   (nearLeftBottomConer.y + 0.5*size_y),
                                   (nearLeftBottomConer.z + 0.5*size_z));
            }

            float getRadius() const
            {
                if (size!=0)
                    return size;
                else
                    return (size_x + size_y + size_z)/3.0;
            }

            inline bool operator==(const AxisAlignedBox& other) const
            {
                if((nearLeftBottomConer == other.nearLeftBottomConer) and
                        (size == other.size) and
                        (size_x == other.size_x) and
                        (size_y == other.size_y) and
                        (size_z == other.size_z))
                    return true;
                else
                    return false;
            }

            inline bool operator!=(const AxisAlignedBox& other) const
            {
                return not this->operator==(other);
            }

            // add this AxisAlignedBox with another AxisAlignedBox,
            // Note: this AxisAlignedBox may become a new bigger one,
            // and will contain the two AxisAlignedBoxs
            inline void operator+=(const AxisAlignedBox& other)
            {
                BlockVector farRightUp;

                if ((other.nearLeftBottomConer.x + other.size_x) > (nearLeftBottomConer.x + size_x))
                    farRightUp.x = other.nearLeftBottomConer.x + other.size_x;
                else
                    farRightUp.x = nearLeftBottomConer.x + size_x;

                if ((other.nearLeftBottomConer.y + other.size_y) > (nearLeftBottomConer.y + size_y))
                    farRightUp.y = other.nearLeftBottomConer.y + other.size_y;
                else
                    farRightUp.y = nearLeftBottomConer.y + size_y;

                if ((other.nearLeftBottomConer.z + other.size_z) > (nearLeftBottomConer.z + size_z))
                    farRightUp.z = other.nearLeftBottomConer.z + other.size_z;
                else
                    farRightUp.z = nearLeftBottomConer.z + size_z;

                if (other.nearLeftBottomConer.x < nearLeftBottomConer.x)
                    nearLeftBottomConer.x = other.nearLeftBottomConer.x;

                if (other.nearLeftBottomConer.y < nearLeftBottomConer.y)
                    nearLeftBottomConer.y = other.nearLeftBottomConer.y;

                if (other.nearLeftBottomConer.z < nearLeftBottomConer.z)
                    nearLeftBottomConer.z = other.nearLeftBottomConer.z;

                size_x = round(farRightUp.x - nearLeftBottomConer.x);
                size_y = round(farRightUp.y - nearLeftBottomConer.y);
                size_z = round(farRightUp.z - nearLeftBottomConer.z);

                if ((size_x == size_y) and (size_x == size_z))
                    size = size_x;
                else
                    size = 0;
            }


            // is this UnitBlock inside this AxisAlignedBox
            // @ point is the nearleftbottom point of the block
            inline bool isUnitBlockInsideMe(const BlockVector& point) const
            {

                if ((point.x >= nearLeftBottomConer.x) &&
                    (point.y >= nearLeftBottomConer.y) &&
                    (point.z >= nearLeftBottomConer.z) &&
                    (point.x < (nearLeftBottomConer.x + size_x)) &&
                    (point.y < (nearLeftBottomConer.y + size_y)) &&
                    (point.z < (nearLeftBottomConer.z + size_z)) )
                {
                    return true;
                }
                else
                    return false;

            }

            // is other AxisAlignedBox insidAxisAlignedBoxAxisAlignedBox
            inline bool isInsideMe(const AxisAlignedBox& other)
            {
                if ((other.nearLeftBottomConer.x >= nearLeftBottomConer.x) &&
                    (other.nearLeftBottomConer.y >= nearLeftBottomConer.y) &&
                    (other.nearLeftBottomConer.z >= nearLeftBottomConer.z) &&
                    ((other.nearLeftBottomConer.x + other.size_x) <= (nearLeftBottomConer.x + size_x)) &&
                    ((other.nearLeftBottomConer.y + other.size_y) <= (nearLeftBottomConer.y + size_y)) &&
                    ((other.nearLeftBottomConer.z + other.size_z) <= (nearLeftBottomConer.z + size_z)) )
                {
                    return true;
                }
                else
                    return false;
            }

            static inline bool eq (double a, double b)
                { return BlockVector::eq(a,b); }

            inline bool isFaceTouching(const AxisAlignedBox& other) const
            {
                std::vector<BlockVector> coners1 = getAllCorners();
                std::vector<BlockVector> coners2 = other.getAllCorners();

                // is my bottom face touching the bottom face of the other
                if ( (eq(((BlockVector)coners1[0]).z, ((BlockVector)coners2[0]).z + 1) or
                      eq(((BlockVector)coners1[0]).z, ((BlockVector)coners2[0]).z - 1) ) and
                        is_2DRectangleOverlap(((BlockVector)coners1[0]).x, ((BlockVector)coners1[0]).y, ((BlockVector)coners1[2]).x, ((BlockVector)coners1[2]).y,
                                           ((BlockVector)coners2[0]).x, ((BlockVector)coners2[0]).y, ((BlockVector)coners2[2]).x, ((BlockVector)coners2[2]).y ) )
                    return true;

                // is my bottom face touching the top face of the other
                if ( (eq(((BlockVector)coners1[0]).z, ((BlockVector)coners2[4]).z + 1) or
                      eq(((BlockVector)coners1[0]).z, ((BlockVector)coners2[4]).z - 1) ) and
                        is_2DRectangleOverlap(((BlockVector)coners1[0]).x, ((BlockVector)coners1[0]).y, ((BlockVector)coners1[2]).x, ((BlockVector)coners1[2]).y,
                                           ((BlockVector)coners2[4]).x, ((BlockVector)coners2[4]).y, ((BlockVector)coners2[6]).x, ((BlockVector)coners2[6]).y ) )
                    return true;

                // is my top face touching the bottom face of the other
                if ( (eq(((BlockVector)coners1[4]).z, ((BlockVector)coners2[0]).z + 1) or
                      eq(((BlockVector)coners1[4]).z, ((BlockVector)coners2[0]).z - 1) ) and
                        is_2DRectangleOverlap(((BlockVector)coners1[4]).x, ((BlockVector)coners1[4]).y, ((BlockVector)coners1[6]).x, ((BlockVector)coners1[6]).y,
                                           ((BlockVector)coners2[0]).x, ((BlockVector)coners2[0]).y, ((BlockVector)coners2[2]).x, ((BlockVector)coners2[2]).y ) )
                    return true;

                // is my top face touching the top face of the other
                if ( (eq(((BlockVector)coners1[4]).z, ((BlockVector)coners2[4]).z + 1) or
                      eq(((BlockVector)coners1[4]).z, ((BlockVector)coners2[4]).z - 1) ) and
                        is_2DRectangleOverlap(((BlockVector)coners1[4]).x, ((BlockVector)coners1[4]).y, ((BlockVector)coners1[6]).x, ((BlockVector)coners1[6]).y,
                                           ((BlockVector)coners2[4]).x, ((BlockVector)coners2[4]).y, ((BlockVector)coners2[6]).x, ((BlockVector)coners2[6]).y ) )
                    return true;

                // is my left face touching the left face of the other
                if ( (eq(((BlockVector)coners1[0]).x, ((BlockVector)coners2[0]).x + 1) or
                      eq(((BlockVector)coners1[0]).x, ((BlockVector)coners2[0]).x - 1) ) and
                        is_2DRectangleOverlap(((BlockVector)coners1[0]).y, ((BlockVector)coners1[0]).z, ((BlockVector)coners1[7]).y, ((BlockVector)coners1[7]).z,
                                           ((BlockVector)coners2[0]).y, ((BlockVector)coners2[0]).z, ((BlockVector)coners2[7]).y, ((BlockVector)coners2[7]).z ) )
                    return true;

                // is my left face touching the right face of the other
                if ( (eq(((BlockVector)coners1[0]).x, ((BlockVector)coners2[1]).x + 1) or
                      eq(((BlockVector)coners1[0]).x, ((BlockVector)coners2[1]).x - 1) ) and
                        is_2DRectangleOverlap(((BlockVector)coners1[0]).y, ((BlockVector)coners1[0]).z, ((BlockVector)coners1[7]).y, ((BlockVector)coners1[7]).z,
                                           ((BlockVector)coners2[1]).y, ((BlockVector)coners2[1]).z, ((BlockVector)coners2[6]).y, ((BlockVector)coners2[6]).z ) )
                    return true;

                // is my right face touching the left face of the other
                if ( (eq(((BlockVector)coners1[1]).x, ((BlockVector)coners2[0]).x + 1) or
                      eq(((BlockVector)coners1[1]).x, ((BlockVector)coners2[0]).x - 1) ) and
                        is_2DRectangleOverlap(((BlockVector)coners1[1]).y, ((BlockVector)coners1[1]).z, ((BlockVector)coners1[6]).y, ((BlockVector)coners1[6]).z,
                                           ((BlockVector)coners2[0]).y, ((BlockVector)coners2[0]).z, ((BlockVector)coners2[7]).y, ((BlockVector)coners2[7]).z ) )
                    return true;

                // is my right face touching the right face of the other
                if ( (eq(((BlockVector)coners1[1]).x, ((BlockVector)coners2[1]).x + 1) or
                      eq(((BlockVector)coners1[1]).x, ((BlockVector)coners2[1]).x - 1) ) and
                        is_2DRectangleOverlap(((BlockVector)coners1[1]).y, ((BlockVector)coners1[1]).z, ((BlockVector)coners1[6]).y, ((BlockVector)coners1[6]).z,
                                           ((BlockVector)coners2[1]).y, ((BlockVector)coners2[1]).z, ((BlockVector)coners2[6]).y, ((BlockVector)coners2[6]).z ) )
                    return true;

                // is my near face touching the near face of the other
                if ( (eq(((BlockVector)coners1[0]).y, ((BlockVector)coners2[0]).y + 1) or
                      eq(((BlockVector)coners1[0]).y, ((BlockVector)coners2[0]).y - 1) ) and
                        is_2DRectangleOverlap(((BlockVector)coners1[0]).x, ((BlockVector)coners1[0]).z, ((BlockVector)coners1[5]).x, ((BlockVector)coners1[5]).z,
                                           ((BlockVector)coners2[0]).x, ((BlockVector)coners2[0]).z, ((BlockVector)coners2[5]).x, ((BlockVector)coners2[5]).z ) )
                    return true;

                // is my near face touching the far face of the other
                if ( (eq(((BlockVector)coners1[0]).y, ((BlockVector)coners2[3]).y + 1) or
                      eq(((BlockVector)coners1[0]).y, ((BlockVector)coners2[3]).y - 1) ) and
                        is_2DRectangleOverlap(((BlockVector)coners1[0]).x, ((BlockVector)coners1[0]).z, ((BlockVector)coners1[5]).x, ((BlockVector)coners1[5]).z,
                                           ((BlockVector)coners2[3]).x, ((BlockVector)coners2[3]).z, ((BlockVector)coners2[6]).x, ((BlockVector)coners2[6]).z ) )
                    return true;

                // is my far face touching the near face of the other
                if ( (eq(((BlockVector)coners1[3]).y, ((BlockVector)coners2[0]).y + 1) or
                      eq(((BlockVector)coners1[3]).y, ((BlockVector)coners2[0]).y - 1) ) and
                        is_2DRectangleOverlap(((BlockVector)coners1[3]).x, ((BlockVector)coners1[3]).z, ((BlockVector)coners1[6]).x, ((BlockVector)coners1[6]).z,
                                           ((BlockVector)coners2[0]).x, ((BlockVector)coners2[0]).z, ((BlockVector)coners2[5]).x, ((BlockVector)coners2[5]).z ) )
                    return true;

                // is my far face touching the far face of the other
                if ( (eq(((BlockVector)coners1[3]).y, ((BlockVector)coners2[3]).y + 1) or
                      eq(((BlockVector)coners1[3]).y, ((BlockVector)coners2[3]).y - 1) ) and
                        is_2DRectangleOverlap(((BlockVector)coners1[3]).x, ((BlockVector)coners1[3]).z, ((BlockVector)coners1[6]).x, ((BlockVector)coners1[6]).z,
                                           ((BlockVector)coners2[3]).x, ((BlockVector)coners2[3]).z, ((BlockVector)coners2[6]).x, ((BlockVector)coners2[6]).z ) )
                    return true;

                return false;

            }



            /**
             * Corners
             *
             *      7+------+6
             *      /|     /|
             *     / |    / |
             * z 4+------+5 |
             *    | 3+---|--+2
             *    | / y  | /
             *    |/     |/
             *   0+------+1  x
             */
            std::vector<BlockVector> getAllCorners() const
            {
                std::vector<BlockVector> coners;
                coners.push_back(nearLeftBottomConer);
                coners.push_back(nearLeftBottomConer + BlockVector(size_x, 0 , 0));
                coners.push_back(nearLeftBottomConer + BlockVector(size_x, size_y , 0));
                coners.push_back(nearLeftBottomConer + BlockVector(0, size_y , 0));
                coners.push_back(nearLeftBottomConer + BlockVector(0, 0 , size_z));
                coners.push_back(nearLeftBottomConer + BlockVector(size_x, 0 , size_z));
                coners.push_back(nearLeftBottomConer + BlockVector(size_x, size_y , size_z));
                coners.push_back(nearLeftBottomConer + BlockVector(0, size_y , size_z));

                return coners;
            }

            bool static is_2DRectangleOverlap(int left1, int bottom1, int right1,int top1,
                                       int left2, int bottom2, int right2,int top2)
            {
                if ((right1 > left2)&&(left1 < right2)&&(top1 > bottom2) && (bottom1 < top2))
                    return true;
                else
                    return false;
            }


        };

        class BlockMaterial
        {
        public:
            string materialType;
            string color;
            BlockMaterial(){materialType = ""; color = "";}
            BlockMaterial(string _materialType, string _color):materialType(_materialType),color(_color){}
            BlockMaterial(const BlockMaterial& other){materialType = other.materialType; color = other.color; }

            inline bool operator==(const BlockMaterial& other) const
            {
                if (other.materialType == materialType and other.color == color)
                    return true;
                else
                    return false;
            }

            inline bool operator!=(const BlockMaterial& other) const
            {
                return not this->operator==(other);
            }
        };

    }
/** @}*/
}


#endif // _SPATIAL_BLOCK3DMAPUTIL_H
