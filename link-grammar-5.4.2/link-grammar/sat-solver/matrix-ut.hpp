#ifndef __MATRIX_UPPER_TRIANGLE_HPP__
#define __MATRIX_UPPER_TRIANGLE_HPP__

#include <vector>

#include "lg_assert.h"

template<class T>
class Matrix {
public:
  Matrix(int n = 1, T init = T()) {
    resize(n, init);
  }

  virtual void resize(int n, T init = T()) {
    _n = n;
    _data.resize(size(_n), init);
  }

  virtual const T& operator() (int i, int j) const {
    return _data[pos(i, j, _n)];
  }

  virtual T& operator() (int i, int j) {
    return _data[pos(i, j, _n)];
  }

  void set(int i, int j, T t) {
    (*this)(i, j) = t;
  }

protected:
  virtual int pos (int i, int j, int n) const {
    return n*i + j;
  }

  virtual int size(int n) const {
    return n*n;
  }

  std::vector<T> _data;
  int _n;
};

template <class T>
class MatrixUpperTriangle : public Matrix<T> {
public:
  MatrixUpperTriangle(int n = 1, T init = T())
    : Matrix<T>(n, init) {
  }

  const T& operator() (int i, int j) const {
    assert(i < j, "MatrixUpperTriangle: i >= j");
    return Matrix<T>::_data[pos(i, j, Matrix<T>::_n)];
  }

  T& operator() (int i, int j) {
    assert(i < j, "MatrixUpperTriangle: i >= j");
    return Matrix<T>::_data[pos(i, j, Matrix<T>::_n)];
  }

protected:
  virtual int size(int n) const {
    return n*(n-1)/2;
  }

  virtual int pos (int i, int j, int n) const {
    return i*(n-2) - i*(i-1)/2 + j - 1;
  }
};

#endif
