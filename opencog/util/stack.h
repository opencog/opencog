#ifndef _STACK_H
#define _STACK_H
#include <stdlib.h>
#include <opencog/util/Logger.h>

template<class T> class v_array{
 public:
  int index;
  int length;
  T* elements;

  T last() { return elements[index-1];}
  void decr() { index--;}
  v_array() { index = 0; length=0; elements = NULL;}
  T& operator[](unsigned int i) { return elements[i]; }
};

template<class T> void push(v_array<T>& v, const T &new_ele)
{
  while(v.index >= v.length)
    {
        v.length = 2*v.length + 3;
        v.elements = (T *)realloc(v.elements,sizeof(T) * v.length);
    }
  v[v.index++] = new_ele;
}

template<class T> void alloc(v_array<T>& v, int length)
{
  v.elements = (T *)realloc(v.elements, sizeof(T) * length);
  v.length = length;
}
 
template<class T> v_array<T> pop(v_array<v_array<T> > &stack)
{
  if (stack.index > 0)
    return stack[--stack.index];
  else
    return v_array<T>();
}

#endif
