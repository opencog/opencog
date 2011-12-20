/*
 * opencog/learning/moses/eda/field_set.h
 *
 * Copyright (C) 2002-2008 Novamente LLC
 * All Rights Reserved
 *
 * Written by Moshe Looks
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
#ifndef _EDA_FIELD_SET_H
#define _EDA_FIELD_SET_H

#include <map>

#include <opencog/util/RandGen.h>
#include <opencog/util/dorepeat.h>
#include <opencog/util/oc_assert.h>

#include "../eda/eda.h"

namespace opencog { 
namespace moses {

 /**
  * A field set describes a compact encoding of a set of continuous,
  * discrete and 'ontological' variables.  The field set does not
  * itself hold the values of these variables; it simply describes
  * how they are packed into a bit-string. The field set provides
  * a collection of iterators for walking over such bit strings, which
  * may then be used to extract values from the bit string (or to
  * change them).  
  *
  * Some terminology: 
  * 'Discrete' variables, or discs, are just variables that range over
  * a set of n discrete values. These take ciel(log_2(n)) bits to store.
  *
  * 'Ontological' variables, or 'onto's, are sets of strings, and thus
  * are essentially discrete variables, except that when unpacked, they
  * have string values.  The field set handles these as a distinct group.
  *
  * 'Continuous' variables are variables that can range over a continuum.
  * Although one might want to think of these as floats or doubles,
  * they're not.  They're represented very, very differently: as
  * variable-length bit strings which encode intervals on the real-number
  * line. The motivation for this, as opposed to using floats or doubles,
  * is that EDA algorithms are more efficient for such an encoding.
  * Further details, motivation and documentation can be found here:
  * http://code.google.com/p/moses/wiki/ModelingAtomSpaces
  *
  * 'Boolean' or 'bit' variables. This is a special case of the discrete
  * variables, and are handled distinctly in the code and API's below:
  * they have their own iterators, etc. Only teh multi-bit discrete 
  * variables are called 'disc'.
  *
  * The variable values are packed into bit strings, which are chunked
  * as vector arrays of 32 or 64-bit unsigned ints, depending on the
  * C library execution environment. Thus, instead of having a single
  * offset, two are used: a "major offset", pointing to the appropriate 
  * 32/64-bit int, and a "minor offset", ranging over 0-31/63. The
  * "width" is the width of the field, in bits.
  */
struct field_set
{
    typedef unsigned int arity_t;
    typedef std::size_t size_t;

    struct bit_iterator;
    struct const_bit_iterator;
    struct disc_iterator;
    struct const_disc_iterator;
    struct contin_iterator;
    struct const_contin_iterator;
    struct const_onto_iterator;

    /**
     * The field struct provides the information needed to get/set the 
     * value of some variable(of any type) in an instance (i.e. a packed/vector)
     * 
     * The major_offset is the index of the element in the instance where the
     * value of the field is stored in. But since there may be values of many
     * different variables stored in the same packed_t value of the instance,
     * this is not enough. We need minor_offset to tell us where inside the
     * packed_t value instance[major_offset] the value is stored. And we need
     * width to tell us how many bits (starting at this position) are used to
     * encode the value.
     * 
     * There is a wiki page about the field_set in the opencog, you could find it on
     * http://www.opencog.org/wiki/Field_set_struct_in_MOSES
     */
    struct field
    {
        field() { }
        field(arity_t w, size_t ma, size_t mi)
                : width(w), major_offset(ma), minor_offset(mi) { }
        arity_t width;
        size_t major_offset, minor_offset;
    };
    typedef std::vector<field>::const_iterator field_iterator;

    struct disc_spec
    {
        disc_spec(arity_t a) : arity(a) { }
        arity_t arity;
        bool operator<(const disc_spec& rhs) const { //sort descending by arity
            return arity > rhs.arity;
        }
        bool operator==(const disc_spec& rhs) const { //don't know why this is needed
            return arity == rhs.arity;
        }
    };

    /**
     * The basic idea is to represent continuous quantities,
     * conceptually, as variable-length sequences of bits, with a
     * simplicity prior towards shorter sequences. Bit-sequences are
     * mapped to continuous values using a simple
     * information-theoretic approach.  (i.e. consider representing a
     * value in the range (-1,1) with a uniform prior)
     * 
     * There is a paper about the how to represent continous
     * quantities , you could find it on
     * http://code.google.com/p/moses/wiki/ModelingAtomSpaces
     */
    struct contin_spec
    {
        contin_spec(contin_t m, contin_t ss, contin_t ex, arity_t d)
                : mean(m), step_size(ss), expansion(ex), depth(d) { }
        contin_t mean, step_size, expansion;
        size_t depth;

        bool operator<(const contin_spec& rhs) const
        { 
            //sort descending by depth
            return (depth > rhs.depth
                    || (depth == rhs.depth 
                        && (expansion > rhs.expansion
                            || (expansion == rhs.expansion
                                && (step_size > rhs.step_size
                                    || (step_size == rhs.step_size
                                        && mean > rhs.mean))))));
        }
        bool operator==(const contin_spec& rhs) const //don't know why this is needed
        {
            return (mean == rhs.mean &&
                    step_size == rhs.step_size &&
                    expansion == rhs.expansion &&
                    depth == rhs.depth);
        }

        // half the smallest possible difference between two values represented
        // according to the spec
        contin_t epsilon() const
        {
            return step_size / contin_t(size_t(1) << depth);
        }

        static const disc_t Stop;  // 0
        static const disc_t Left;  // 1
        static const disc_t Right; // 2
        // This method returns Left if lr is Right
        // and Right if lr is Left
        // otherwise is raises an assert
        static disc_t switchLR(disc_t lr)
        {
            if(lr == Left)
                return Right;
            else if(lr == Right)
                return Left;
            else { 
                OC_ASSERT(false);
                return disc_t(); // to keep the compiler quiet
            }
        }
    };

    struct contin_stepper
    {
        contin_stepper(const contin_spec& c_)
                : c(c_), value(c.mean),
                _all_left(true), _all_right(true), _step_size(c.step_size) { }
        const contin_spec& c;
        contin_t value;

        void left()
        {
            if (_all_left) {
                value -= _step_size;
                _step_size *= c.expansion;
                _all_right = false;
            } else {
                if (_all_right) {
                    _all_right = false;
                    _step_size /= (c.expansion * 2);
                }
                value -= _step_size;
                _step_size /= 2;
            }
        }
        void right()
        {
            if (_all_right) {
                value += _step_size;
                _step_size *= c.expansion;
                _all_left = false;
            } else {
                if (_all_left) {
                    _all_left = false;
                    _step_size /= (c.expansion * 2);
                }
                value += _step_size;
                _step_size /= 2;
            }
        }
    protected:
        bool _all_left;
        bool _all_right;
        contin_t _step_size;
    };

    struct onto_spec
    {
        onto_spec(const onto_tree& t)
                : tr(&t), depth(t.max_depth(t.begin())),
                branching(next_power_of_two(1 + t.max_branching(t.begin()))) { }
        // @todo: could be a source of bug if such order is not total
        // as it's gonna make problems with field_set(from, to)
        bool operator<(const onto_spec& rhs) const { //sort descending by size
            return (depth*branching > rhs.depth*rhs.branching);
        }

        const onto_tree* tr;
        size_t depth, branching;

        bool operator==(const onto_spec& rhs) const { //don't know why this is needed
            return (depth == rhs.depth && branching == rhs.branching && *tr == *(rhs.tr));
        }

        static const disc_t Stop;
        static disc_t to_child_idx(disc_t d) {
            return d -1;
        }
        static disc_t from_child_idx(disc_t d) {
            return d + 1;
        }
    };

    // A spec, in general, is one of the above  three specs.
    typedef boost::variant<onto_spec, contin_spec, disc_spec> spec;

    // Default constructor for an empty field set
    field_set() { }

    // Copy cconstructor
    field_set(const field_set& x)
        : _fields(x._fields), _onto(x._onto), _contin(x._contin),
          _disc(x._disc), _nbool(x._nbool)
    {
        compute_starts();
    }

    // Constructor for a single spec, repeated n times
    field_set(const spec& s, size_t n) : _nbool(0)
    {
        build_spec(s, n);
        compute_starts();
    }

    // Constructor for a range of specs.
    template<typename It>
    field_set(It from, It to) : _nbool(0)
    {
        typedef std::map<spec, size_t> spec_map;

        // identical specs are merged and there are sorted (by the map)
        // The sorting seems to follows the order of the spec boost::variant.
        // Also, since disc_spec::operator< is sorting by descending arity,
        // it is ensured that bit_spec are at the end.
        spec_map spec_counts;
        while (from != to)
            ++spec_counts[*from++];

        foreach(const spec_map::value_type& v, spec_counts) //build them
            build_spec(v.first, v.second);

        compute_starts();                 //compute iterator positions
    }

    // Assignment and equality
    field_set& operator=(const field_set&);
    bool operator==(const field_set&) const;

    size_t packed_width() const {
        return _fields.empty() ? 0 : _fields.back().major_offset + 1;
    }
    bool empty() const {
        return _fields.empty();
    }
    size_t raw_size() const {
        return _fields.size();
    }
    // Dimension size, number of actual knobs to consider, as onto and
    // contin may take several raw knobs
    size_t dim_size() const {
        return n_bits() + n_disc() + contin().size() + onto().size();
    }

    // counts the number of nonzero (raw) settings in an instance
    size_t count(const instance& inst) const {
        return raw_size() - std::count(begin_raw(inst), end_raw(inst), 0);
    }

    // Return vector of discrete specs. This vector includes the single
    // bit (boolean) specs, which are at the end of the array. thus the
    // name "disc and bits".
    const vector<disc_spec>& disc_and_bits() const {
        return _disc;
    }
    const vector<contin_spec>& contin() const {
        return _contin;
    }
    const vector<onto_spec>& onto() const {
        return _onto;
    }

    disc_t get_raw(const instance& inst, size_t idx) const { //nth encoded var
        const field& f = _fields[idx];
        return (inst[f.major_offset] >> f.minor_offset)&packed_t((1 << f.width) - 1);
    }
    void set_raw(instance& inst, size_t idx, disc_t v) const {
        const field& f = _fields[idx];
        inst[f.major_offset] ^= ((inst[f.major_offset] ^
                                  packed_t(v << f.minor_offset)) &
                                 ((packed_t(1) << f.width) - 1) << f.minor_offset);
    }

    // returns a reference of the onto at idx, idx is relative to
    // onto_iterator
    const onto_t& get_onto(const instance& inst, size_t idx) const;
    // returns the contin at idx, idx is relative to contin_iterator
    contin_t get_contin(const instance& inst, size_t idx) const;
    void set_contin(instance& inst, size_t idx, contin_t v) const;

    // pack the data in [from,from+dof) according to our scheme, copy to out
    template<typename It, typename Out>
    Out pack(It from, Out out) const;

    std::string stream(const instance&) const;
    std::string stream_raw(const instance&) const;

    int hamming_distance(const instance& inst1, const instance& inst2) const
    {
        int d = 0;
        for (const_disc_iterator it1 = begin_raw(inst1), it2 = begin_raw(inst2);
                it1 != end_raw(inst1);++it1, ++it2)
            d += (*it1 != *it2);
        return d;
    }

    // The fields are organized so that onto fields come first,
    // followed by the continuous fields, and then the discrete
    // fields. These are then followed by the 1-bit (boolean)
    // discrete fields, tacked on at the very end. Thus, the
    // start/end values below reflect this structure.

    field_iterator begin_onto_fields() const {
        return _fields.begin();
    }
    field_iterator end_onto_fields() const {
        return _contin_start;
    }

    field_iterator begin_contin_fields() const {
        return _contin_start;
    }
    field_iterator end_contin_fields() const {
        return _disc_start;
    }

    field_iterator begin_disc_fields() const {
        return _disc_start;
    }
    field_iterator end_disc_fields() const {
        return _fields.end() - _nbool;
    }

    field_iterator begin_bit_fields() const {
        return _fields.end() - _nbool;
    }
    field_iterator end_bit_fields() const {
        return _fields.end();
    }

    field_iterator begin_fields() const {
        return _fields.begin();
    }
    field_iterator end_fields() const {
        return _fields.end();
    }

    //* number of discrete fields that are single bits
    // (i.e. are booleans)
    size_t n_bits() const {
        return _nbool;
    }

    //* number of discrete fields, not counting the booleans
    // i.e. not counting the 1-bit discrete fields.
    size_t n_disc() const {
        return distance(_disc_start, _fields.end()) - _nbool;
    }

    //* number of continuous fields.
    size_t n_contin() const {
        // Funny math because _contin_end is same as _disc_start.
        return distance(_contin_start, _disc_start);
    }

    //* number of "ontologicial" fields.
    size_t n_onto() const {
        // The ontological fields come first in the vector.
        return distance(_fields.begin(), _contin_start);
    }

    size_t contin_to_raw_idx(size_t idx) const
    {
        // @todo: compute at the start in _fields - could be faster..
        size_t raw_idx = distance(_fields.begin(), _contin_start);
        for (vector<contin_spec>::const_iterator it = _contin.begin();
                it != _contin.begin() + idx;++it)
            raw_idx += it->depth;
        return raw_idx;
    }

    // It is the exact reverse of contin_to_raw_idx, if the idx does
    // not correspond to any contin then an OC_ASSERT is raised.
    size_t raw_to_contin_idx(size_t idx) const
    {
        // @todo: compute at the start in _fields - could be faster..
        size_t begin_contin_idx = n_onto();
        size_t end_contin_idx = begin_contin_idx + n_contin();
        OC_ASSERT(idx >= begin_contin_idx && idx < end_contin_idx);
        int contin_raw_idx = idx - begin_contin_idx;
        for(size_t i = 0; i < _contin.size(); ++i) {
            contin_raw_idx -= _contin[i].depth;
            if(contin_raw_idx < 0) return i;
        }
        OC_ASSERT(false, "Impossible case");
        return size_t(); // to make the compiler quiet
    }

    size_t onto_to_raw_idx(size_t idx) const
    {
        // @todo: compute at the start in _fields - could be faster..
        size_t raw_idx = 0;
        for (vector<onto_spec>::const_iterator it = _onto.begin();
                it != _onto.begin() + idx;++it)
            raw_idx += it->depth;
        return raw_idx;
    }

    /**
     *  Get number of non-stop in the contin encode at the idx-th
     *  contin. For instance, the inst is {LRSSRLLS} of two
     *  contin_spec with the depth 4. So if the idx = 1, then it will
     *  return 3 since there are one 'R' and 2 'L's before the
     *  'S'(stop). if the idx = 0, then, 2 will be returned.
     *
     * @param inst the instance to look at
     * @param idx the index of the contin to look at, idx is relative
     *            to contin_iterator
     */
    size_t count_n_before_stop(const instance& inst, size_t idx) const
    {
        size_t raw_begin = contin_to_raw_idx(idx);
        size_t raw_end = raw_begin + _contin[idx].depth;
        size_t current = raw_begin;
        // get the number of raw code for contin before the first *Stop*
        while(current != raw_end) {
            if (get_raw(inst, current) != contin_spec::Stop)
                ++current;
            else break;
        }
        return current - raw_begin;
    }

protected:
    
    // _fields holds all of the different field types in one array.
    // They are arranged in order, so that the "ontological" fields
    // come first, followed by the continuous fields, then the 
    // (multi-bit) discrete fields (i.e. the discrete fields that
    // require more than one bit), and finally the one-bit or boolean
    // fields.
    // 
    // The locations and sizes of these can be gotten using the methods:
    // begin_onto_fields() - end_onto_fields()
    // begin_contin_fields() - end_contin_fields()
    // begin_disc_fields() - end_disc_fields()
    // begin_bit_fields() - end_bit_fields()
    vector<field> _fields;
    vector<onto_spec> _onto;
    vector<contin_spec> _contin;
    vector<disc_spec> _disc; // included bits
    size_t _nbool; // the number of disc_spec that requires only 1 bit to pack

    // Cached values for start location of the continuous and discrete
    // fields in the _fields array.  We don't need to cache the onto 
    // start, as that is same as start of _field. Meanwhile, the start
    // of the booleans is just _nbool back from the end of the array, 
    // so we don't cache that either. These can be computed from
    // scratch (below, with compute_starts()) and are cached here for
    // performance resons.
    field_iterator _contin_start, _disc_start;

    // Figure out where, in the field array, the varous different
    // field types start. Cache these, as they're handy to have around.
    void compute_starts()
    {
        _contin_start = _fields.begin();
        foreach(const onto_spec& o, _onto)
            _contin_start += o.depth; //# of fields
        _disc_start = _contin_start;
        foreach(const contin_spec& c, _contin)
            _disc_start += c.depth;
    }

    size_t back_offset() const
    {
        return _fields.empty() ? 0 :
               _fields.back().major_offset*bits_per_packed_t +
               _fields.back().minor_offset + _fields.back().width;
    }

    //* Build spec s, n times, that is:
    // Fill the corresponding field in _fields, n times.
    // Add the spec n times in _onto, _contin or _disc depending on its type.
    void build_spec(const spec& s, size_t n);

    //* Build onto_spec os, n times
    // Fill the corresponding field, n times.
    // Add the spec in _onto, n times.
    void build_onto_spec(const onto_spec& os, size_t n);

    // Like above but for contin
    void build_contin_spec(const contin_spec& cs, size_t n);

    // Like above but for disc, and also,
    // increment _nbool by n if ds has multiplicity 2 (i.e. only needs one bit).
    void build_disc_spec(const disc_spec& ds, size_t n);

    template<typename Self, typename Iterator>
    struct bit_iterator_base
        : boost::random_access_iterator_helper<Self, bool>
    {
        typedef std::ptrdiff_t Distance;
        
        Self& operator++()
        {
            _mask <<= 1;
            if (!_mask) {
                _mask = packed_t(1);
                ++_it;
            }
            return (*((Self*)this));
        }

        Self& operator--()
        {
            static const packed_t reset = packed_t(1 << (bits_per_packed_t - 1));
            _mask >>= 1;
            if (!_mask) {
                _mask = reset;
                --_it;
            }
            return (*((Self*)this));
        }

        Self& operator+=(Distance n)
        {
            if (n < 0)
                return (*this) -= (-n);
            _it += n / bits_per_packed_t;
            dorepeat(n % bits_per_packed_t) //could be faster...
                ++(*this);
            return (*((Self*)this));
        }

        Self& operator-=(Distance n)
        {
            if (n < 0)
                return (*this) += (-n); 
            _it -= n / bits_per_packed_t;
            dorepeat(n % bits_per_packed_t) //could be faster...
                --(*this);
            return (*((Self*)this));
        }

        bool operator<(const Self& x) const
        {
            return (_it < x._it ? true : integer_log2(_mask) < integer_log2(x._mask));
        }

        friend Distance operator-(const Self& x, const Self& y)
        {
            return (bits_per_packed_t*(x._it - y._it) +
                    integer_log2(x._mask) - integer_log2(y._mask));
        }

        bool operator==(const Self& rhs) const
        {
            return (_it == rhs._it && _mask == rhs._mask);
        }

    protected:
        bit_iterator_base(Iterator it, arity_t offset)
            : _it(it), _mask(packed_t(1) << offset) { }
        bit_iterator_base(packed_t mask, Iterator it) : _it(it), _mask(mask) { }
        bit_iterator_base() : _it(), _mask(0) { }

        Iterator _it; // instance iterator
        packed_t _mask; // mask over the packed_t pointed by _it,
                        // i.e. the data of interest
    };

    template<typename Iterator, typename Value>
    struct iterator_base
        : boost::random_access_iterator_helper<Iterator, Value>
    {
        typedef std::ptrdiff_t Distance;

        struct reference
        {
            reference(const Iterator* it, size_t idx) : _it(it), _idx(idx) { }

            operator Value() const {
                return do_get();
            }

            reference& operator=(Value x) {
                do_set(x); return *this;
            }
            reference& operator=(const reference& rhs) {
                do_set(rhs);
                return *this;
            }

            reference& operator+=(Value x) {
                do_set(do_get() + x);   return *this;
            }
            reference& operator-=(Value x) {
                do_set(do_get() - x); return *this;
            }
            reference& operator*=(Value x) {
                do_set(do_get()*x);  return *this;
            }
            reference& operator/=(Value x) {
                do_set(do_get() / x); return *this;
            }
        protected:
            const Iterator* _it;
            size_t _idx;

            Value do_get() const;
            void do_set(Value x);
        };

        Iterator& operator++() {
            ++_idx;
            return (*((Iterator*)this));
        }
        Iterator& operator--() {
            --_idx;
            return (*((Iterator*)this));
        }
        Iterator& operator+=(Distance n) {
            _idx += n;
            return (*((Iterator*)this));
        }
        Iterator& operator-=(Distance n) {
            _idx -= n;
            return (*((Iterator*)this));
        }
        bool operator<(const Iterator& x) const {
            return (_idx < x._idx);
        }
        friend Distance operator-(const Iterator& x, const Iterator& y) {
            return (x._idx -y._idx);
        }

        bool operator==(const Iterator& rhs) const {
            return (_idx == rhs._idx);
        }

        int idx() const {
            return _idx;
        }
    protected:
        iterator_base(const field_set& fs, size_t idx) : _fs(&fs), _idx(idx) { }
        iterator_base() : _fs(NULL), _idx(0) { }

        const field_set* _fs;
        size_t _idx;
    };
public:
    struct bit_iterator
        : public bit_iterator_base<bit_iterator, instance::iterator>
    {
        friend struct field_set;

        struct reference
        {
            reference(instance::iterator it, packed_t mask)
                : _it(it), _mask(mask) {}

            operator bool() const {
                return (*_it & _mask) != 0;
            }

            bool operator~() const {
                return (*_it & _mask) == 0;
            }
            reference& flip() {
                do_flip(); return *this;
            }

            reference& operator=(bool x) {
                do_assign(x); return *this;
            }
            reference& operator=(const reference& rhs) {
                do_assign(rhs);
                return *this;
            }

            reference& operator|=(bool x) {
                if  (x) do_set();   return *this;
            }
            reference& operator&=(bool x) {
                if (!x) do_reset(); return *this;
            }
            reference& operator^=(bool x) {
                if  (x) do_flip();  return *this;
            }
            reference& operator-=(bool x) {
                if  (x) do_reset(); return *this;
            }
        protected:
            instance::iterator _it;
            packed_t _mask;

            void do_set() {
                *_it |= _mask;
            }
            void do_reset() {
                *_it &= ~_mask;
            }
            void do_flip() {
                *_it ^= _mask;
            }
            void do_assign(bool x) {
                x ? do_set() : do_reset();
            }
        };

        reference operator*() const {
            return reference(_it, _mask);
        }
        friend class const_bit_iterator;

        bit_iterator() { }
    protected:
        bit_iterator(instance::iterator it, arity_t offset)
            : bit_iterator_base<bit_iterator, instance::iterator>(it, offset)
        { }
    };

    struct const_bit_iterator
        : public bit_iterator_base<const_bit_iterator, instance::const_iterator>
    {
        friend class field_set;
        bool operator*() const {
            return (*_it & _mask) != 0;
        }
        const_bit_iterator(const bit_iterator& bi)
            : bit_iterator_base < const_bit_iterator,
                                  instance::const_iterator > (bi._mask, bi._it) { }

        const_bit_iterator() { }
    protected:
        const_bit_iterator(instance::const_iterator it, arity_t offset)
            : bit_iterator_base < const_bit_iterator,
                                  instance::const_iterator > (it, offset) { }
    };

    struct disc_iterator : public iterator_base<disc_iterator, disc_t>
    {
        friend struct field_set;
        friend struct reference;

        reference operator*() const {
            return reference(this, _idx);
        }
        friend class const_disc_iterator;

        disc_iterator() : _inst(NULL) { }

        //for convenience, but will only work over discrete & boolean
        arity_t arity() const {
            return
                _idx < _fs->onto().size() ? _fs->onto()[_idx].branching :
                _idx < _fs->onto().size() + _fs->contin().size() ? 3 :
                _fs->disc_and_bits()[_idx-_fs->onto().size()-_fs->contin().size()].arity;
        }
        void randomize(RandGen& rng) {
            _fs->set_raw(*_inst, _idx, rng.randint(arity()));
        }
    protected:
        disc_iterator(const field_set& fs, size_t idx, instance& inst)
            : iterator_base<disc_iterator, disc_t>(fs, idx), _inst(&inst) { }
        instance* _inst;
    };

    struct const_disc_iterator
        : public iterator_base<const_disc_iterator, disc_t>
    {
        friend class field_set;
        disc_t operator*() const {
            return _fs->get_raw(*_inst, _idx);
        }
        const_disc_iterator(const disc_iterator& bi) :
            iterator_base<const_disc_iterator, disc_t>(*bi._fs, bi._idx),
            _inst(bi._inst) { }
        const_disc_iterator() : _inst(NULL) { }
        //for convenience, but will only work over disc & bool
        arity_t arity() const {
            return
                _idx < _fs->onto().size() ? _fs->onto()[_idx].branching :
                _idx < _fs->onto().size() + _fs->contin().size() ? 3 :
                _fs->disc_and_bits()[_idx-_fs->onto().size()-_fs->contin().size()].arity;
        }
    protected:
        const_disc_iterator(const field_set& fs, size_t idx, const instance& inst)
            : iterator_base<const_disc_iterator, disc_t>(fs, idx), _inst(&inst) { }
        const instance* _inst;
    };

    struct contin_iterator : public iterator_base<contin_iterator, contin_t>
    {
        friend struct field_set;
        friend struct reference;

        reference operator*() const {
            return reference(this, _idx);
        }
        friend class const_contin_iterator;

        contin_iterator() : _inst(NULL) { }
    protected:
        contin_iterator(const field_set& fs, size_t idx, instance& inst)
            : iterator_base<contin_iterator, contin_t>(fs, idx), _inst(&inst)
        { }
        instance* _inst;
    };

    struct const_contin_iterator
        : public iterator_base<const_contin_iterator, contin_t>
    {
        friend class field_set;
        contin_t operator*() const {
            return _fs->get_contin(*_inst, _idx);
        }
        const_contin_iterator(const contin_iterator& bi)
            : iterator_base<const_contin_iterator, contin_t>(*bi._fs, bi._idx),
              _inst(bi._inst) { }
        const_contin_iterator() : _inst(NULL) { }
    protected:
        const_contin_iterator(const field_set& fs, size_t idx,
                              const instance& inst)
            : iterator_base<const_contin_iterator, contin_t>(fs, idx),
              _inst(&inst) { }
        const instance* _inst;
    };

    struct const_onto_iterator
        : public iterator_base<const_onto_iterator, onto_t>
    {
        friend class field_set;
        const onto_t& operator*() {
            return _fs->get_onto(*_inst, _idx);
        }
        const_onto_iterator() : _inst(NULL) { }
    protected:
        const_onto_iterator(const field_set& fs, size_t idx,
                            const instance& inst)
            : iterator_base<const_onto_iterator, onto_t>(fs, idx),
              _inst(&inst) { }
        const instance* _inst;
    };

    const_bit_iterator begin_bits(const instance& inst) const
    {
        return (begin_bit_fields() == _fields.end() ? const_bit_iterator() :
                const_bit_iterator(inst.begin() + begin_bit_fields()->major_offset,
                                   begin_bit_fields()->minor_offset));
    }

    const_bit_iterator end_bits(const instance& inst) const
    {
        return (begin_bit_fields() == _fields.end() ? const_bit_iterator() :
                ++const_bit_iterator(--inst.end(), _fields.back().minor_offset));
    }

    bit_iterator begin_bits(instance& inst) const
    {
        return (begin_bit_fields() == _fields.end() ? bit_iterator() :
                bit_iterator(inst.begin() + begin_bit_fields()->major_offset,
                             begin_bit_fields()->minor_offset));
    }

    bit_iterator end_bits(instance& inst) const
    {
        return (begin_bit_fields() == _fields.end() ? bit_iterator() :
                ++bit_iterator(--inst.end(), _fields.back().minor_offset));
    }

    const_disc_iterator begin_disc(const instance& inst) const
    {
        return const_disc_iterator(*this, distance(_fields.begin(),
                                                   begin_disc_fields()), inst);
    }

    const_disc_iterator end_disc(const instance& inst) const
    {
        return const_disc_iterator(*this, distance(_fields.begin(),
                                                   end_disc_fields()), inst);
    }

    disc_iterator begin_disc(instance& inst) const {
        return disc_iterator(*this, distance(_fields.begin(),
                                             begin_disc_fields()), inst);
    }

    disc_iterator end_disc(instance& inst) const {
        return disc_iterator(*this, distance(_fields.begin(),
                                             end_disc_fields()), inst);
    }

    const_contin_iterator begin_contin(const instance& inst) const {
        return const_contin_iterator(*this, 0, inst);
    }

    const_contin_iterator end_contin(const instance& inst) const {
        return const_contin_iterator(*this, _contin.size(), inst);
    }

    contin_iterator begin_contin(instance& inst) const {
        return contin_iterator(*this, 0, inst);
    }

    contin_iterator end_contin(instance& inst) const {
        return contin_iterator(*this, _contin.size(), inst);
    }

    const_onto_iterator begin_onto(const instance& inst) const {
        return const_onto_iterator(*this, 0, inst);
    }

    const_onto_iterator end_onto(const instance& inst) const {
        return const_onto_iterator(*this, _onto.size(), inst);
    }

    const_disc_iterator begin_raw(const instance& inst) const {
        return const_disc_iterator(*this, 0, inst);
    }
    const_disc_iterator end_raw(const instance& inst) const {
        return const_disc_iterator(*this, _fields.size(), inst);
    }
    disc_iterator begin_raw(instance& inst) const {
        return disc_iterator(*this, 0, inst);
    }
    disc_iterator end_raw(instance& inst) const {
        return disc_iterator(*this, _fields.size(), inst);
    }
};

template<>
inline disc_t field_set::iterator_base < field_set::disc_iterator,
disc_t >::reference::do_get() const
{
    return _it->_fs->get_raw(*_it->_inst, _idx);
}

template<>
inline void field_set::iterator_base < field_set::disc_iterator,
disc_t >::reference::do_set(disc_t x)
{
    _it->_fs->set_raw(*_it->_inst, _idx, x);
}

template<>
inline contin_t field_set::iterator_base < field_set::contin_iterator,
contin_t >::reference::do_get() const
{
    return _it->_fs->get_contin(*_it->_inst, _idx);
}

template<>
inline void field_set::iterator_base < field_set::contin_iterator,
contin_t >::reference::do_set(contin_t x)
{
    _it->_fs->set_contin(*_it->_inst, _idx, x);
}

// pack the data in [from,from+dof) according to our scheme, copy to out
template<typename It, typename Out>
Out field_set::pack(It from, Out out) const
{
    unsigned int offset = 0;

    foreach(const onto_spec& o, _onto) {
        size_t width = nbits_to_pack(o.branching);
        size_t total_width = size_t((width * o.depth - 1) /
                                    bits_per_packed_t + 1) * bits_per_packed_t;
        for (arity_t i = 0;i < o.depth;++i) {
            *out |= (*from++) << offset;
            offset += width;
            if (offset == bits_per_packed_t) {
                offset = 0;
                ++out;
            }
        }
        offset += total_width - (o.depth * width); //onto vars must pack evenly
        if (offset == bits_per_packed_t) {
            offset = 0;
            ++out;
        }
    }

    foreach(const contin_spec& c, _contin) {
        for (arity_t i = 0;i < c.depth;++i) {
            *out |= (*from++) << offset;
            offset += 2;
            if (offset == bits_per_packed_t) {
                offset = 0;
                ++out;
            }
        }
    }

    foreach(const disc_spec& d, _disc) {
        *out |= (*from++) << offset;
        offset += nbits_to_pack(d.arity);
        if (offset == bits_per_packed_t) {
            offset = 0;
            ++out;
        }
    }
    if (offset > 0) //so we always point to one-past-the-end
        ++out;
    return out;
}

} // ~namespace moses
} // ~namespace opencog

#endif
