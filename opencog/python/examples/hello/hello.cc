//  Copyright Joel de Guzman 2002-2004. Distributed under the Boost
//  Software License, Version 1.0. (See accompanying file LICENSE_1_0.txt 
//  or copy at http://www.boost.org/LICENSE_1_0.txt)
//  Hello World Example from the tutorial
//  [Joel de Guzman 10/9/2002]

#include "hello.h"
#include <boost/python/module.hpp>
#include <boost/python/def.hpp>

char const *greet() { return "hello, world"; }
char const *foo(int x, int y) { return "foo"; }

World::World(std::string msg): msg(msg) {} // added constructor
void World::set(std::string msg)
{
    this->msg = msg;
}
std::string World::greet()
{
    return msg;
}

Var::Var(std::string name):
    name(name), value()
{}

Num::Num() { value = 2; }
float Num::get() const { return value; }
void Num::set(float newValue) { value = newValue; }

Base::~Base() {}
int Base::notpure() { return 0; }

int Derived::pure() { return 0; }
bool Derived::operator==(const Base& rhs) const { return true; }

void b(Base *aBase) {}
void d(Derived *aDerived) {}
Base* factory() { return new Derived; }

FilePos FilePos::operator+ (FilePos fp) { return fp; }
//FilePos     operator+(FilePos, int);
/*FilePos     operator+(int, FilePos);
int         operator-(FilePos, FilePos);
FilePos     operator-(FilePos, int);
FilePos&    operator+=(FilePos&, int);
FilePos&    operator-=(FilePos&, int);
bool        operator<(FilePos, FilePos);*/

bool X::f(int a) { return true; }
bool X::f(int a, double b) { return true; }
bool X::f(int a, double b, char c) { return true; }
int X::f(int a, int b, int c) { return a + b + c; }

void A::f() {}
int A::Y::g() { return 42; }
