/*
 * opencog/embodiment/Control/OperationalPetController/OutputRelexColor.h
 *
 * Copyright (C) 2009 Novamente LLC
 * All Rights Reserved
 * Author(s): Fabricio Silva 
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

#ifndef OUTPUTRELEXCOLOR_H
#define OUTPUTRELEXCOLOR_H

#include "OutputRelex.h"

using namespace opencog;

namespace OperationalPetController
{
    class OutputRelexColor : public OutputRelex
    {
        public:
            OutputRelexColor() : OutputRelex() {}
            virtual std::string getOutput( const AtomSpace &atomSpace, std::set<Handle> handles ){

                if(handles.size() != 2){
                    printf( "Wrong number of handles for OutputRelexColor. It is expected 2, but it was %d\n",handles.size() );
                    return "";
                }

                std::vector<Handle> handleVector;
                std::set<Handle>::iterator it;
                for(it=handles.begin(); it!=handles.end(); ++it){
                    handleVector.push_back(*it);
                }

                //TODO set ordered must be considered here according to the name
                //of the element. Once the elements are Entity and Color, Color
                //is the first one in the set order
                std::string attr2 = atomSpace.getName(handleVector[0]);
                std::string attr1 = atomSpace.getName(handleVector[1]);

                std::string output = "_subj("+attr1+","+attr2+")\n";
                output.append("present("+attr2+")\n");
                output.append(".a("+attr2+")\n");
                output.append("adj("+attr2+")\n");
                output.append("_predadj("+attr1+","+attr2+")\n");
                output.append("definite("+attr1+")\n");
                output.append(".n("+attr1+")\n");
                output.append("noun("+attr1+")\n");
                output.append("singular("+attr1+")\n");
                output.append(".v(be)\n");
                output.append(" verb(be)\n");
                output.append("punctuation(.)\n");
                output.append("det(the)\n");

                return output;
            }
    };
};

#endif // OUTPUTRELEXCOLOR_H
