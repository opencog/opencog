/*
 * PointMemorySCM.h
 *
 * Copyright (C) 2016 OpenCog Foundation
 *
 * Author: Mandeep Singh Bhatia <https://github.com/yantrabuddhi>
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

#ifndef _OPENCOG_POINT_MEM_SCM_H
#define _OPENCOG_POINT_MEM_SCM_H


#include <map>
#include <vector>
#include <opencog/atoms/base/Handle.h>
#include <opencog/atomtimeoctomap/TimeSpaceAtom.h>

#define FPS 30
//at-least 10 seconds records
#define MEM_SEC 10
//assuming face height of 0.22 meters
#define SPACE_RES_M 0.01

using namespace std;

namespace opencog
{
namespace ato
{
class PointMemorySCM
{
private:
    static void* init_in_guile(void*);
    static void init_in_module(void*);
    void init(void);
    //bool .. handle float float float
    bool map_ato(Handle,double,double,double);
    get_first_ato(Handle,double,double);
    get_last_ato(Handle,double,double);
    get_mem_elapse();
    //list .. handle float float
    TimeSpaceAtom* tsa;
    vector<double> space_res;
public:
    PointMemorySCM();
    ~PointMemorySCM();
};
}
}
extern "C" {
void opencog_ato_pointmem_init(void);
};
#endif
