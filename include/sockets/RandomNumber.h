/**
 * @author Adam McLaurin
 * @date   September 2006
 */
/*
This program is free software; you can redistribute it and/or
modify it under the terms of the GNU General Public License
as published by the Free Software Foundation; either version 2
of the License, or (at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program; if not, write to the Free Software
Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.
*/

#ifndef _SOCKET_RandomNumber_H
#define _SOCKET_RandomNumber_H

#include "sockets-config.h"
#include <limits>

#ifdef SOCKETS_NAMESPACE
namespace SOCKETS_NAMESPACE {
#endif

/**
 * The following class uses an xorshift algorithm proposed in the following
 * paper:
 *  - http://www.jstatsoft.org/v08/i14/xorshift.pdf
 *
 * The algorithm provides a PRNG with a period of (2^128)-1
 *
 * This PRNG is *not* intended for cryptographic purposes
 */
class RandomNumber
{

public:
  /**
   * Default constructor
   *
   * NOTE: Internal seeds are set to defaults proposed by the paper
   */
  RandomNumber(bool time_shuffle = false);

  /**
   * Custom constructor
   *
   * @param x_seed X seed
   *
   * @param y_seed Y seed
   *
   * @param z_seed Z seed
   *
   * @param w_seed W seed
   */
  RandomNumber(
    unsigned long int x_seed,
    unsigned long int y_seed,
    unsigned long int z_seed,
    unsigned long int w_seed);

  /**
   * Destructor
   */
  ~RandomNumber();

  // public methods

  /**
   * Reset internal state to initial seed values
   */
  void reset();

  /**
   * Cast operator to obtain current random value in the PRNG
   *
   * @return Current random value in the PRNG
   */
  operator unsigned long int() const;

  /**
   * Go to the next number in the PRNG sequence
   *
   * NOTE: This method is a slightly modified implementation of the xor128()
   * function proposed in the paper
   *
   * @return Next value produced by the PRNG (after updating)
   */
  unsigned long int next();

  /**
   * Skip ahead in the PRNG sequence by a given number of iterations
   *
   * @param s Number of iterations to skip ahead
   *
   * @return Value produced by the PRNG after skipping ahead in the sequence
   */
  unsigned long int skip(unsigned long int s);

  /**
   * Obtain all the initial seeds for this PRNG
   *
   * @param x_seed X seed (output)
   *
   * @param y_seed Y seed (output)
   *
   * @param z_seed Z seed (output)
   *
   * @param w_seed W seed (output)
   */
  void getSeed(
    unsigned long int& x_seed,
    unsigned long int& y_seed,
    unsigned long int& z_seed,
    unsigned long int& w_seed);

  /**
   * Get the maximum possible random number from this PRNG
   *
   * @return Maximum possible random number from this PRNG
   */
  static unsigned long int max_random();

  // public constants

  /**
   * Default x-seed as proposed by the paper
   */
  static const unsigned long int X_SEED_DEFAULT;

  /**
   * Default y-seed as proposed by the paper
   */
  static const unsigned long int Y_SEED_DEFAULT;

  /**
   * Default z-seed as proposed by the paper
   */
  static const unsigned long int Z_SEED_DEFAULT;

  /**
   * Default w-seed as proposed by the paper
   */
  static const unsigned long int W_SEED_DEFAULT;

private:
  /**
   * X seed
   */
  unsigned long int mXSeed;

  /**
   * Y seed
   */
  unsigned long int mYSeed;

  /**
   * Z seed
   */
  unsigned long int mZSeed;

  /**
   * W seed
   */
  unsigned long int mWSeed;

  /**
   * X value
   */
  unsigned long int mX;

  /**
   * Y value
   */
  unsigned long int mY;

  /**
   * Z value
   */
  unsigned long int mZ;

  /**
   * W value
   *
   * NOTE: This is the externally-visible next value produced by the PRNG
   */
  unsigned long int mW;

};

#ifdef SOCKETS_NAMESPACE
} // namespace SOCKETS_NAMESPACE {
#endif

#endif // _SOCKETS_RandomNumber_H

