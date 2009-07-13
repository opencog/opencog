#ifndef HELLO_H
#define HELLO_H

//  Copyright Joel de Guzman 2002-2004. Distributed under the Boost
//  Software License, Version 1.0. (See accompanying file LICENSE_1_0.txt 
//  or copy at http://www.boost.org/LICENSE_1_0.txt)
//  Hello World Example from the tutorial
//  [Joel de Guzman 10/9/2002]

#include <iostream>
//#include <boost/python.hpp>
//using namespace boost::python;

// Top-level functions.
const char *greet();
const char *foo(int x, int y);

// Sample class using a constructor.
struct World
{
    World(std::string msg);
    void set(std::string msg);
    std::string greet();
    std::string msg;
};

struct Var
{
    Var(std::string name);
    std::string const name;
    float value;
};

struct Num
{
    Num();
    float value;
    float get() const;
    void set(float newValue);
};

struct Base
{
    // Virtual function with default implementation.
    virtual ~Base();
    // Virtual function with default implementation.
    virtual int notpure();
    // Pure virtual function.
    virtual int pure() = 0;
    // Virtual class operator with.
    virtual bool operator==(const Base& rhs) const = 0;
};

struct Derived : Base
{
    int pure();
    bool operator==(const Base& rhs) const;
};

void b(Base *);
void d(Derived *);
Base* factory();

class FilePos {
public:
    FilePos operator+ (FilePos);
};
//FilePos FilePos::operator+ (FilePos fp) { return fp; }
//FilePos     operator+(FilePos, int);
FilePos     operator+(int, FilePos);
int         operator-(FilePos, FilePos);
FilePos     operator-(FilePos, int);
FilePos&    operator+=(FilePos&, int);
FilePos&    operator-=(FilePos&, int);
bool        operator<(FilePos, FilePos);

struct X
{
    bool f(int);
    bool f(int, double);
    bool f(int, double, char);
    static bool f(int, int, int, int);
    int f(int, int, int);
};

struct A
{
    void f();

    struct Y {
        int g();
    };
};

#endif
