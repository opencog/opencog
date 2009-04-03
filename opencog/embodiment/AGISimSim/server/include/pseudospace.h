/*
 Modified by Arzah 2006 for AGI-PseudoSim

    Copyright (C) 1998,1999,2000 by Jorrit Tyberghein
    Largely rewritten by Ivan Avramovic <ivan@avramovic.com>

    This library is free software; you can redistribute it and/or
    modify it under the terms of the GNU Library General Public
    License as published by the Free Software Foundation; either
    version 2 of the License, or (at your option) any later version.

    This library is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
    Library General Public License for more details.

    You should have received a copy of the GNU Library General Public
    License along with this library; if not, write to the Free
    Software Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
*/

#ifndef __PSEUDO_SPACE_H__
#define __PSEUDO_SPACE_H__

#define csPrintf printf

/*#include "csextern.h"

#include "csgeom/math3d_d.h"*/

#define SMALL_EPSILON 0.0000001f
#define ABS(x) ( ((x)>0) ? (x) : -(x) )


class csVector3
{
public:
    /// The X component of the vector
    float x;
    /// The Y component of the vector
    float y;
    /// The Z component of the vector
    float z;

    /**
     * Make a new vector. The vector is not
     * initialized. This makes the code slightly faster as
     * csVector3 objects are used a lot.
     */
    csVector3 () : x(0), y(0), z(0) {}

    /**
     * Make a new initialized vector.
     * Creates a new vector and initializes it to m*<1,1,1>.  To create
     * a vector initialized to the zero vector, use csVector3(0)
     */
    csVector3 (float m) : x(m), y(m), z(m) {}

    /// Make a new vector and initialize with the given values.
    csVector3 (float ix, float iy, float iz = 0) : x(ix), y(iy), z(iz) {}

    /// Copy Constructor.
    csVector3 (const csVector3& v) : x(v.x), y(v.y), z(v.z) {}

    /// Add two vectors.
    csVector3 operator+(const csVector3& v2) {
        return csVector3(this->x + v2.x, this->y + v2.y, this->z + v2.z);
    }

    /// Add two vectors of differing type, raise the csVector3 to DVector3.

    /// Subtract two vectors.
    csVector3 operator- (const csVector3& v2) {
        return csVector3(this->x - v2.x, this->y - v2.y, this->z - v2.z);
    }

    /// Take the dot product of two vectors.
    float operator* (const csVector3& v2) {
        return this->x*v2.x + this->y*v2.y + this->z*v2.z;
    }

    /// Take the cross product of two vectors.
    csVector3 operator% (const csVector3& v2) {
        return csVector3 (this->y*v2.z - this->z*v2.y,
                          this->z*v2.x - this->x*v2.z,
                          this->x*v2.y - this->y*v2.x);
    }

    /// Take cross product of two vectors and put result in this vector.
    void Cross (const csVector3 & px, const csVector3 & py) {
        x = px.y * py.z - px.z * py.y;
        y = px.z * py.x - px.x * py.z;
        z = px.x * py.y - px.y * py.x;
    }

    /// Multiply a vector and a scalar.
    inline friend csVector3 operator* (const csVector3& v, float f) {
        return csVector3(v.x*f, v.y*f, v.z*f);
    }

    /// Multiply a vector and a scalar.
    inline friend csVector3 operator* (float f, const csVector3& v) {
        return csVector3(v.x*f, v.y*f, v.z*f);
    }


    /// Multiply a vector and a scalar int.
    inline friend csVector3 operator* (const csVector3& v, int f) {
        return v * (float)f;
    }

    /// Multiply a vector and a scalar int.
    inline friend csVector3 operator* (int f, const csVector3& v) {
        return v * (float)f;
    }

    /// Divide a vector by a scalar.
    inline friend csVector3 operator/ (const csVector3& v, float f) {
        f = 1.0f / f; return csVector3(v.x*f, v.y*f, v.z*f);
    }

    /// Divide a vector by a scalar int.
    inline friend csVector3 operator/ (const csVector3& v, int f) {
        return v / (float)f;
    }

    /// Check if two vectors are equal.
    inline friend bool operator== (const csVector3& v1, const csVector3& v2) {
        return v1.x == v2.x && v1.y == v2.y && v1.z == v2.z;
    }

    /// Check if two vectors are not equal.
    inline friend bool operator!= (const csVector3& v1, const csVector3& v2) {
        return v1.x != v2.x || v1.y != v2.y || v1.z != v2.z;
    }

    /// Test if each component of a vector is less than a small epsilon value.
    inline friend bool operator< (const csVector3& v, float f) {
        return ABS(v.x) < f && ABS(v.y) < f && ABS(v.z) < f;
    }

    /// Test if each component of a vector is less than a small epsilon value.
    inline friend bool operator> (float f, const csVector3& v) {
        return ABS(v.x) < f && ABS(v.y) < f && ABS(v.z) < f;
    }

    /// Returns n-th component of the vector.
    inline float operator[] (int n) const {
        return !n ? x : n&1 ? y : z;
    }

    /// Returns n-th component of the vector.
    inline float & operator[] (int n) {
        return !n ? x : n&1 ? y : z;
    }

    /// Add another vector to this vector.
    inline csVector3& operator+= (const csVector3& v) {
        x += v.x;
        y += v.y;
        z += v.z;

        return *this;
    }

    /// Subtract another vector from this vector.
    inline csVector3& operator-= (const csVector3& v) {
        x -= v.x;
        y -= v.y;
        z -= v.z;

        return *this;
    }

    /// Multiply this vector by a scalar.
    inline csVector3& operator*= (float f) {
        x *= f; y *= f; z *= f; return *this;
    }

    /// Divide this vector by a scalar.
    inline csVector3& operator/= (float f) {
        f = 1.0f / f; x *= f; y *= f; z *= f; return *this;
    }

    /// Unary + operator.
    inline csVector3 operator+ () const {
        return *this;
    }

    /// Unary - operator.
    inline csVector3 operator- () const {
        return csVector3(-x, -y, -z);
    }

    /// Set the value of this vector.
    inline void Set (float sx, float sy, float sz) {
        x = sx; y = sy; z = sz;
    }

    /// Set the value of this vector.
    inline void Set (csVector3 const& v) {
        x = v.x; y = v.y; z = v.z;
    }

    /// Set the value of this vector.
    inline void Set (float const* v) {
        x = v[0]; y = v[1]; z = v[2];
    }

    /// Set the value of this vector so that all components are the same.
    inline void Set (float v) {
        x = y = z = v;
    }

    /// Get the value of this vector.
    inline void Get (float* v) {
        v[0] = x; v[1] = y; v[2] = z;
    }

    /// Returns the norm of this vector.
    float Norm () const;

    /// Return the squared norm (magnitude) of this vector.
    float SquaredNorm () const {
        return x * x + y * y + z * z;
    }

    /**
     * Returns the unit vector in the direction of this vector.
     * Attempting to normalize a zero-vector will result in a divide by
     * zero error.  This is as it should be... fix the calling code.
     */
    csVector3 Unit () const {
        return (*this) / (this->Norm());
    }

    /// Returns the norm (magnitude) of a vector.
    inline static float Norm (const csVector3& v) {
        return v.Norm();
    }

    /// Normalizes a vector to a unit vector.
    inline static csVector3 Unit (const csVector3& v) {
        return v.Unit();
    }

    /// Scale this vector to length = 1.0;
    void Normalize ();

    /// Query if the vector is zero
    inline bool IsZero (float precision = SMALL_EPSILON) const {
        return (ABS(x) < precision) && (ABS(y) < precision)
               && (ABS(z) < precision);
    }
};


struct csBox2 {
    float minX, minZ, maxX, maxZ;
    csBox2(float _minX, float _minZ, float _maxX, float _maxZ)
            : minX(_minX), minZ(_minZ), maxX(_maxX), maxZ(_maxZ) {}
    float MinX() const {
        return minX;
    }
    float MaxX() const {
        return maxX;
    }
    float MinY() const {
        return minZ;
    }
    float MaxY() const {
        return maxZ;
    }
};

struct csBox3 {
    float minX, minY, minZ, maxX, maxY, maxZ;
    csBox3()
            : minX(0), minY(0), minZ(0), maxX(0), maxY(0), maxZ(0) {}
    csBox3(float _minX, float _minY, float _minZ, float _maxX, float _maxY, float _maxZ)
            : minX(_minX), minY(_minY), minZ(_minZ), maxX(_maxX), maxY(_maxY), maxZ(_maxZ) {}
    float MinX() const {
        return minX;
    }
    float MaxX() const {
        return maxX;
    }
    float MinY() const {
        return minY;
    }
    float MaxY() const {
        return maxY;
    }
    float MinZ() const {
        return minZ;
    }
    float MaxZ() const {
        return maxZ;
    }
};

/** @} */

#endif // __PSEUDO_SPACE_H__


/*
  Modified by Arzah 2006 for AGI-PseudoSim

  Crystal Space Smart Pointers
  Copyright (C) 2002 by Jorrit Tyberghein and Matthias Braun

  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Library General Public
  License as published by the Free Software Foundation; either
  version 2 of the License, or (at your option) any later version.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  Library General Public License for more details.

  You should have received a copy of the GNU Library General Public
  License along with this library; if not, write to the Free
  Software Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
*/

#if 0
#ifndef __CS_REF_H__
#define __CS_REF_H__

#define CS_VOIDED_PTR 0xffffffff

template <class T> class csRef;

#if defined(CS_DEBUG)
#  define CS_TEST_VOIDPTRUSAGE
#else
#  undef CS_TEST_VOIDPTRUSAGE
#endif

#ifdef CS_REF_TRACKER
#include <typeinfo>
#include "csutil/reftrackeraccess.h"

#define CSREF_TRACK(x, cmd, refCount, obj, tag)    \
    {          \
        const int rc = obj ? refCount : -1;      \
        if (obj) cmd;        \
        if (obj)         \
        {          \
            csRefTrackerAccess::SetDescription (obj,     \
                                                typeid(T).name());       \
            csRefTrackerAccess::Match ## x (obj, rc, tag);\
        }          \
    }
#define CSREF_TRACK_INCREF(obj,tag) \
    CSREF_TRACK(IncRef, obj->IncRef(), obj->GetRefCount(), obj, tag);
#define CSREF_TRACK_DECREF(obj,tag) \
    CSREF_TRACK(DecRef, obj->DecRef(), obj->GetRefCount(), obj, tag);
#define CSREF_TRACK_ASSIGN(obj,tag) \
    CSREF_TRACK(IncRef, (0), obj->GetRefCount() - 1, obj, tag);
#else
#define CSREF_TRACK_INCREF(obj,tag) \
    if (obj) obj->IncRef();
#define CSREF_TRACK_DECREF(obj,tag) \
    if (obj) obj->DecRef();
#define CSREF_TRACK_ASSIGN(obj,tag)
#endif

/**
 * A normal pointer. This class should ONLY be used for functions
 * returning pointers that are already IncRef()'ed for the caller.
 * This class behaves like a pointer encapsulator. Please do NOT
 * use this for anything else (for example, don't use this class
 * to remember pointers to anything. Use csRef for that).
 * It only stores the pointer. Nothing else. When it is assigned to
 * a csRef, the csRef smart pointer will 'inherit' the reference
 * (so no IncRef() happens).
 */
template <class T>
class  csPtr
{
private:
    friend class csRef<T>;
    T* obj;

public:
    csPtr (T* p) : obj (p) {
        CSREF_TRACK_ASSIGN(obj, this);
    }

    template <class T2>
    explicit csPtr (csRef<T2> const& r) : obj((T2*)r) {
        CSREF_TRACK_INCREF (obj, this);
    }

#ifdef CS_TEST_VOIDPTRUSAGE
    ~csPtr () {
        // If not assigned to a csRef we have a problem (leak).
        // So if this assert fires for you, then you are calling
        // a function that returns a csPtr and not using the result
        // (or at least not assigning it to a csRef). This is a memory
        // leak and you should fix that.
        CS_ASSERT (obj == (T*)CS_VOIDED_PTR);
    }
#endif

    csPtr (const csPtr<T>& copy) {
        obj = copy.obj;
#ifdef CS_TEST_VOIDPTRUSAGE
        ((csPtr<T>&)copy).obj = (T*)CS_VOIDED_PTR;
#endif
    }
};

/**
 * A smart pointer.  Maintains and correctly manages a reference to a
 * reference-counted object.  This template requires only that the object type
 * T implement the methods IncRef() and DecRef().  No other requirements are
 * placed upon T.
 */
template <class T>
class  csRef
{
private:
    T* obj;

public:
    /**
     * Construct an invalid smart pointer (that is, one pointing at nothing).
     * Dereferencing or attempting to use the invalid pointer will result in a
     * run-time error, however it is safe to invoke IsValid().
     */
    csRef () : obj (0) {}

    /**
     * Construct a smart pointer from a csPtr. Doesn't call IncRef() on
     * the object since it is assumed that the object in csPtr is already
     * IncRef()'ed.
     */
    csRef (const csPtr<T>& newobj) {
        obj = newobj.obj;
#   ifdef CS_TEST_VOIDPTRUSAGE
        CS_ASSERT (newobj.obj != (T*)CS_VOIDED_PTR);
#   endif
        // The following line is outside the ifdef to make sure
        // we have binary compatibility.
        ((csPtr<T>&)newobj).obj = (T*)CS_VOIDED_PTR;
    }

    /**
     * Construct a smart pointer from a raw object reference. Calls IncRef()
     * on the object.
     */
    csRef (T* newobj) : obj (newobj) {
        CSREF_TRACK_INCREF (obj, this);
    }

    /**
     * Smart pointer copy constructor from assignment-compatible csRef<T2>.
     */
    template <class T2>
    csRef (csRef<T2> const& other) : obj ((T2*)other) {
        CSREF_TRACK_INCREF (obj, this);
    }

    /**
     * Smart pointer copy constructor.
     */
    csRef (csRef const& other) : obj (other.obj) {
        CSREF_TRACK_INCREF (obj, this);
    }

    /**
     * Smart pointer destructor.  Invokes DecRef() upon the underlying object.
     */
    ~csRef () {
        CSREF_TRACK_DECREF (obj, this);
    }

    /**
     * Assign a csPtr to a smart pointer. Doesn't call IncRef() on
     * the object since it is assumed that the object in csPtr is already
     * IncRef()'ed.
     * \remarks
     * After this assignment, the csPtr<T> object is invalidated and cannot
     * be used. You should not (and in fact cannot) decref the csPtr<T> after
     * this assignment has been made.
     */
    csRef& operator = (const csPtr<T>& newobj) {
        T* oldobj = obj;
        // First assign and then DecRef() of old object!
        obj = newobj.obj;
#   ifdef CS_TEST_VOIDPTRUSAGE
        CS_ASSERT (newobj.obj != (T*)CS_VOIDED_PTR);
#   endif
        // The following line is outside the ifdef to make sure
        // we have binary compatibility.
        ((csPtr<T>&)newobj).obj = (T*)CS_VOIDED_PTR;
        CSREF_TRACK_DECREF (oldobj, this);
        return *this;
    }

    /**
     * Assign a raw object reference to this smart pointer.
     * \remarks
     * This function calls the object's IncRef() method. Because of this you
     * should not assign a reference created with the new operator to a csRef
     * object driectly. The following code will produce a memory leak:
     * \code
     * csRef<iEvent> event = new csEvent;
     * \endcode
     * If you are assigning a new object to a csRef, use AttachNew(T* newObj)
     * instead.
     */
    csRef& operator = (T* newobj) {
        if (obj != newobj) {
            T* oldobj = obj;
            // It is very important to first assign the new value to
            // 'obj' BEFORE calling DecRef() on the old object. Otherwise
            // it is easy to get in infinite loops with objects being
            // destructed forever (when ref=0 is used for example).
            obj = newobj;
            CSREF_TRACK_INCREF (newobj, this);
            CSREF_TRACK_DECREF (oldobj, this);
        }
        return *this;
    }

    /**
     * Assign an object reference created with the new operator to this smart
     * pointer.
     * \remarks
     * This function allows you to assign an object pointer created with the
     * \c new operator to the csRef object. Proper usage would be:
     * \code
     * csRef<iEvent> event;
     * event.AttachNew (new csEvent);
     * \endcode
     * While not recommended, you can also use this function to assign a csPtr
     * object or csRef object to the csRef. In both of these cases, using
     * AttachNew is equivalent to performing a simple assignment using the
     * \c = operator.
     * \note
     * Calling this function is equivalent to casting an object to a csPtr<T>
     * and then assigning the csPtr<T> to the csRef, as follows:
     * \code
     * // Same effect as above code.
     * csRef<iEvent> event = csPtr<iEvent> (new csEvent);
     * \endcode
     */
    void AttachNew (csPtr<T> newObj) {
        // Note: The parameter usage of csPtr<T> instead of csPtr<T>& is
        // deliberate and not to be considered a bug.

        // Just Re-use csPtr assignment logic
        *this = newObj;
    }

    /// Assign another assignment-compatible csRef<T2> to this one.
    template <class T2>
    csRef& operator = (csRef<T2> const& other) {
        T* p = (T2*)other;
        this->operator=(p);
        return *this;
    }

    /// Assign another csRef<> of the same type to this one.
    csRef& operator = (csRef const& other) {
        this->operator=(other.obj);
        return *this;
    }

    /// Test if the two references point to same object.
    inline friend bool operator == (const csRef& r1, const csRef& r2) {
        return r1.obj == r2.obj;
    }
    /// Test if the two references point to different object.
    inline friend bool operator != (const csRef& r1, const csRef& r2) {
        return r1.obj != r2.obj;
    }
    /// Test if object pointed to by reference is same as obj.
    inline friend bool operator == (const csRef& r1, T* obj) {
        return r1.obj == obj;
    }
    /// Test if object pointed to by reference is different from obj.
    inline friend bool operator != (const csRef& r1, T* obj) {
        return r1.obj != obj;
    }
    /// Test if object pointed to by reference is same as obj.
    inline friend bool operator == (T* obj, const csRef& r1) {
        return r1.obj == obj;
    }
    /// Test if object pointed to by reference is different from obj.
    inline friend bool operator != (T* obj, const csRef& r1) {
        return r1.obj != obj;
    }

    /// Dereference underlying object.
    T* operator -> () const {
        return obj;
    }

    /// Cast smart pointer to a pointer to the underlying object.
    operator T* () const {
        return obj;
    }

    /// Dereference underlying object.
    T& operator* () const {
        return *obj;
    }

    /**
     * Smart pointer validity check.  Returns true if smart pointer is pointing
     * at an actual object, otherwise returns false.
     */
    bool IsValid () const {
        return (obj != 0);
    }
};

#undef CSREF_TRACK_INCREF
#undef CSREF_TRACK_DECREF
#undef CSREF_TRACK_ASSIGN

#endif //CS_REF disabler!

#endif // __CS_REF_H__
