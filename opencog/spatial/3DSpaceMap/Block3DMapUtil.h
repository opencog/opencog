#ifndef _SPATIAL_BLOCK3DMAPUTIL_H
#define _SPATIAL_BLOCK3DMAPUTIL_H

#define BLOCK_LIST            "block-list" // all the unit block nodes in an entity

#include <sstream>
#include <math.h>
#include <vector>

using namespace std;

namespace opencog
{

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

            int x;
            int y;
            int z;
            BlockVector(){x = 0; y = 0; z = 0;}
            BlockVector(int _x, int _y, int _z){x = _x; y = _y; z = _z;}
            BlockVector(const BlockVector& other){x = other.x; y = other.y; z = other.z;}


            inline BlockVector& operator = ( const BlockVector& other )
            {
                x = other.x;
                y = other.y;
                z = other.z;
                return *this;
            }

            inline bool operator == (const BlockVector& other) const
            {
                if((other.x == x) && (other.y == y) && (other.z == z) )
                    return true;
                else
                    return false;
            }

            inline bool operator != (const BlockVector& other) const
            {
                if((other.x != x) || (other.y != y) || (other.z != z) )
                    return true;
                else
                    return false;
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
                if ((size_x == size_y) && (size_x == size_z) )
                    size = size_x;
                else
                    size = 0;
            }

            AxisAlignedBox():
            size_x(0), size_y(0), size_z(0), size(0)
            {nearLeftBottomConer = BlockVector::ZERO;}

            AxisAlignedBox(BlockVector& _neaLeftBottomPos, int _size = 1):
            size_x(_size), size_y(_size), size_z(_size), size(_size)
            {
                nearLeftBottomConer = BlockVector(_neaLeftBottomPos);
            }

            inline bool operator==(const AxisAlignedBox& other) const
            {
                if((nearLeftBottomConer == other.nearLeftBottomConer) &&
                        (size == other.size) &&
                        (size_x == other.size_x) &&
                        (size_y == other.size_y) &&
                        (size_z == other.size_z) )
                    return true;
                else
                    return false;
            }

            inline bool operator!=(const AxisAlignedBox& other) const
            {
                return((nearLeftBottomConer != other.nearLeftBottomConer) || (other.size != size) ||
                        (size_x != other.size_x) || (size_y != other.size_y) || (size_z != other.size_z) );

            }

            // add this AxisAlignedBox with another AxisAlignedBox,
            // Note: this AxisAlignedBox may become a new bigger one , and will contain the two AxisAlignedBoxs
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

                size_x = farRightUp.x - nearLeftBottomConer.x;
                size_y = farRightUp.y - nearLeftBottomConer.y;
                size_z = farRightUp.z - nearLeftBottomConer.z;

                if ((size_x == size_y) && (size_x == size_z) )
                    size = size_x;
                else
                    size = 0;

            }


            // is this UnitBlock inside this AxisAlignedBox
            // @ point is the nearleftbottom point of the block
            inline bool isUnitBlockInsideMe(BlockVector& point)
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
            inline bool isInsideMe(AxisAlignedBox& other)
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
                if(other.materialType == materialType && other.color == color)
                    return true;
                else
                    return false;
            }

            inline bool operator!=(const BlockMaterial& other) const
            {
                if(other.materialType != materialType || other.color != color)
                    return true;
                else
                    return false;
            }
        };

    }
}


#endif // _SPATIAL_BLOCK3DMAPUTIL_H
