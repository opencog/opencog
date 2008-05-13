/**
* Trail.h
*
*
* Copyright(c) 2001 Guilherme Lamacie
* All rights reserved.
*/

#ifndef TRAIL_H
#define TRAIL_H


#include <deque>
#include <exception>

#include "types.h"
#include "exceptions.h"

using namespace std;

class Trail {

        int maxSize;
        deque<Handle>* trail;

        void init(int,int) throw (InvalidParamException, std::bad_exception);

    public:

        
        Trail() throw (InvalidParamException, std::bad_exception);
        Trail(int) throw (InvalidParamException, std::bad_exception);
        Trail(int,int) throw (InvalidParamException, std::bad_exception);

        ~Trail();

        bool isInTrail(Handle);

        void insert(Handle, bool = true); 

        void append(Trail*);
        
        int getSize();

        void print();
        void print(FILE*);

        Handle getElement(int) throw (IndexErrorException, std::bad_exception);
};

#endif
