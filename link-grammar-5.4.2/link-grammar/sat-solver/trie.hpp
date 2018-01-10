#ifndef __TRIE_HPP__
#define __TRIE_HPP__

#include <stdlib.h>

#include "lg_assert.h"

/*
   Trie that supports strings made out of alphabeth letters,
   digits and underscores
*/
template<class T>
class Trie {
public:
  Trie();
  ~Trie();

  void insert(const char* key, T value);
  T lookup(const char* key);

  // returned in the key is not found in the trie
  const static int NOT_FOUND = -1;

private:
  // no copying
  Trie(const Trie&);
  void operator=(const Trie& t);



  // Number of supported chars - digits + upper + lower + other + addition-for-hexadecimal-base
  const static int NUM_CHARS = 10 + 1 + 10 + 1 + 6;
  // hash chars
  ssize_t char_to_pos(char c);

  bool _terminal;
  Trie* _next[NUM_CHARS];
  T _value;
};


template <class T>
Trie<T>::Trie()
  : _terminal(false) {
  memset(_next, 0, NUM_CHARS*sizeof(Trie*));
}

template <class T>
Trie<T>::~Trie() {
  for (int i = 0; i < NUM_CHARS; i++)
    if (_next[i]) {
      delete _next[i];
    }
}


template <class T>
ssize_t Trie<T>::char_to_pos(char c) {
  static ssize_t pos[] = {
    -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,
    -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,
    -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,
    0, 1, 2, 3, 4, 5, 6, 7, 8, 9,
    22, 23, 24, 25, 26, 27, -1,
//   A   B   C   D   E   F   G   H   I   J   K   L   M   N   O   P   Q   R   S   T   U   V   W   X   Y   Z
    -1, -1, -1, 10, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,
    -1, -1, -1, -1, 11, -1,
//   a   b   c   d   e   f   g   h   i   j   k   l   m   n   o   p   q   r   s   t   u   v   w   x   y   z
    -1, -1, 12, 13, 14, 15, -1, -1, 16, -1, -1, 17, -1, 18, -1, -1, -1, 19, -1, -1, 20, -1, 21, -1, -1, -1};
  assert(pos[(short)c] != -1, "NOT FOUND");
  return pos[(short)c];
  /*
  if ('0' <= c && c <= '9')
    return c-'0' + 26 + 26;
  if (c == 'c')
    return 2;
  if (c == 'd')
    return 3;
  if (c == '_')
    return 26 + 26 + 10;
  if ('a' <= c && c <= 'z')
    return c-'a';
  if ('A' <= c && c <= 'Z')
    return c-'A' + 26;
  if (c == '*')
    return 26 + 26 + 10 + 1;
  throw std::string("Trie::char ") + c + " is not supported";
  */
}

template <class T>
void Trie<T>::insert(const char* key, T value) {
  Trie* t = this;
  while(*key != '\0') {
    ssize_t pos = char_to_pos(*key);
    if (!t->_next[pos]) {
      t->_next[pos] = new Trie();
    }
    t = t->_next[pos];
    key++;
  }
  t->_terminal = true;
  t->_value = value;
}

template <class T>
T Trie<T>::lookup(const char* key) {
  Trie* t = this;
  while(*key != '\0') {
    ssize_t pos = char_to_pos(*key);
    t = t->_next[pos];
    if (!t) {
      return NOT_FOUND;
    }
    key++;
  }
  return t->_terminal ? t->_value : NOT_FOUND;
}

#endif
