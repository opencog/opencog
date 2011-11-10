#ifndef _OPENCOG_ALGORITHM_H
#define _OPENCOG_ALGORITHM_H

#include <algorithm>
#include <boost/lexical_cast.hpp>
#include <boost/bind.hpp>

#include <ext/algorithm>

#include "numeric.h"
#include "exceptions.h"

namespace opencog
{

//these needs to be changed for non-gcc
using __gnu_cxx::copy_n;
using __gnu_cxx::lexicographical_compare_3way;
using __gnu_cxx::random_sample_n;
using __gnu_cxx::random_sample;
using __gnu_cxx::is_heap;
using __gnu_cxx::is_sorted;

//binary and ternary and quaternary for_each
template<typename It1, typename It2, typename F>
F for_each(It1 from1, It1 to1, It2 from2, F f)
{
    for (;from1 != to1;++from1, ++from2)
        f(*from1, *from2);
    return f;
}
template<typename It1, typename It2, typename It3, typename F>
F for_each(It1 from1, It1 to1, It2 from2, It3 from3, F f)
{
    for (;from1 != to1;++from1, ++from2, ++from3)
        f(*from1, *from2, *from3);
    return f;
}
template<typename It1, typename It2, typename It3, typename It4, typename F>
F for_each(It1 from1, It1 to1, It2 from2, It3 from3, It4 from4, F f)
{
    for (;from1 != to1;++from1, ++from2, ++from3, ++from4)
        f(*from1, *from2, *from3, *from4);
    return f;
}

//accumulate over a range of containers
template <class iter, class T>
T accumulate2d(iter first, iter last, T init)
{
    for ( ;first != last;++first)
        init = std::accumulate(first->begin(), first->end(), init);
    return init;
}

// appends b at the end of a, that is a = a@b
template<class T>
void append(T& a, const T& b) {
    a.insert(a.end(), b.begin(), b.end());
}

//erase the intersection of sorted ranges [from1,to1) and [from2,to2) from c,
//leaving the difference
template<typename Erase, typename It1, typename It2, typename Comp>
void erase_set_intersection(Erase erase, It1 from1, It1 to1,
                            It2 from2, It2 to2, Comp comp)
{
    OC_ASSERT(__gnu_cxx::is_sorted(from1, to1, comp),
              "algorithm - from1 -> to1 aren't sorted (erase_set_intersection).");
    OC_ASSERT(__gnu_cxx::is_sorted(from2, to2, comp),
              "algorithm - from2 -> to2 aren't sorted (erase_set_intersection).");

    while (from1 != to1 && from2 != to2)
        if (comp(*from1, *from2)) {
            ++from1;
        } else if (comp(*from2, *from1)) {
            ++from2;
        } else {
            erase(from1++);
            ++from2;
        }
}

//erase the difference of sorted ranges [from1,to1) and [from2,to2) from c,
//leaving the intersection
template<typename Erase, typename It1, typename It2, typename Comp>
void erase_set_difference(Erase erase, It1 from1, It1 to1,
                          It2 from2, It2 to2, Comp comp)
{
    OC_ASSERT(__gnu_cxx::is_sorted(from1, to1, comp),
              "algorithm - from1 -> to1 aren't sorted (erase_set_difference).");
    OC_ASSERT(__gnu_cxx::is_sorted(from2, to2, comp),
              "algorithm - from2 -> to2 aren't sorted (erase_set_difference).");

    while (from1 != to1 && from2 != to2)
        if (comp(*from1, *from2)) {
            erase(from1++);
        } else if (comp(*from2, *from1)) {
            ++from2;
        } else {
            ++from1;
            ++from2;
        }
    while (from1 != to1)
        erase(from1++);
}

//insert [from2,to2)-[from1,to1) with inserter
//i.e., if insert inserts into the container holding [from1,to1),
//it will now hold the union
template<typename Insert, typename It1, typename It2, typename Comp>
void insert_set_complement(Insert insert, It1 from1, It1 to1,
                           It2 from2, It2 to2, Comp comp)
{
    cassert(TRACE_INFO, __gnu_cxx::is_sorted(from1, to1, comp),
            "algorithm - from1 -> to1 aren't sorted (insert_set_complement).");
    cassert(TRACE_INFO, __gnu_cxx::is_sorted(from2, to2, comp),
            "algorithm - from2 -> to2 aren't sorted (insert_set_complement).");

    while (from1 != to1 && from2 != to2)
        if (comp(*from1, *from2)) {
            ++from1;
        } else if (comp(*from2, *from1)) {
            insert(from1, *from2);
            ++from2;
        } else {
            ++from1;
            ++from2;
        }
    while (from2 != to2) {
        insert(from1, *from2);
        ++from2;
    }
}

template<typename It1, typename It2, typename Comp>
bool has_empty_intersection(It1 from1, It1 to1,
                            It2 from2, It2 to2, Comp comp)
{
    cassert(TRACE_INFO, __gnu_cxx::is_sorted(from1, to1, comp),
            "algorithm - from1 -> to1 aren't sorted (has_empty_intersection).");
    cassert(TRACE_INFO, __gnu_cxx::is_sorted(from2, to2, comp),
            "algorithm - from2 -> to2 aren't sorted (has_empty_intersection).");

    while (from1 != to1 && from2 != to2)
        if (comp(*from1, *from2))
            ++from1;
        else if (comp(*from2, *from1))
            ++from2;
        else
            return false;
    return true;
}

template<typename Set>
bool has_empty_intersection(const Set& ls, const Set& rs) {
    typedef typename Set::const_iterator const_iterator;
    typedef typename Set::key_compare key_compare;
    return has_empty_intersection(ls.begin(), ls.end(),
                                  rs.begin(), rs.end(),
                                  ls.key_comp());
}

/**
 * return a singleton set with the input given in it
 */
template<typename Set>
Set make_singleton_set(const typename Set::value_type& v) {
    Set ret;
    ret.insert(v);
    return ret;
}

/**
 * Return s1 union s2
 * s1 and s2 being std::set or similar concept
 */
template<typename Set>
Set set_union(const Set& s1, const Set& s2) {
    Set res(s1);
    res.insert(s2.begin(), s2.end());
    return res;
}

/**
 * Return s1 inter s2
 * s1 and s2 must be sorted
 */
template<typename Set>
Set set_intersection(const Set& s1, const Set& s2) {
    Set res;
    std::set_intersection(s1.begin(), s1.end(), s2.begin(), s2.end(),
                          std::inserter(res, res.end()));
    return res;
}

/**
 * Returns s1 - s2
 * s1 and s2 must be sorted
 */
template<typename Set>
Set set_difference(const Set& s1, const Set& s2) {
    Set res;
    std::set_difference(s1.begin(), s1.end(), s2.begin(), s2.end(),
                        std::inserter(res, res.end()));
    return res;
}

/**
 * overload sort for container
 */
template<typename Seq>
void sort(Seq& s) {
    std::sort(s.begin(), s.end());
}

// Predicate maps to the range [0, n)
// n-1 values (the pivots) are copied to out
template<typename It, typename Pred, typename Out>
Out n_way_partition(It begin, It end, const Pred p, int n, Out out)
{
    // could be made more efficient if needed
    for (int i = 0;i < n - 1;++i)
        *out++ = begin = std::partition(begin, end, boost::bind(p, _1) == i);
    return out;
}

/**
 * return the power set ps of s such that all elements of ps are
 * subsets of size n or below.
 *
 * @param s       the input set
 * @param n       the size of the largest subset of s
 * @param exact   if true then do not include subsets of size below n
 *
 * @return        the power set of s with subsets up to size n
 */
template<typename Set> std::set<Set> powerset(const Set& s, size_t n, bool exact = false)
{
    typedef typename Set::const_iterator SetCIt;
    typedef typename std::set<Set>::const_iterator PowerSetCIt;
    std::set<Set> res;
    if(n > 0) {
        std::set<Set> ps = powerset(s, n-1, exact);
        for(PowerSetCIt ss = ps.begin(); ss != ps.end(); ss++)
            for(SetCIt el = s.begin(); el != s.end(); el++) {
                Set subset(*ss);
                if(subset.find(*el) == subset.end()) {
                    subset.insert(*el);
                    res.insert(subset);
                }
            }
        if(!exact)
            res.insert(ps.begin(), ps.end());
    } else
        res.insert(Set());
    return res;
}
/**
 * return the power set of s.
 */
template<typename Set> std::set<Set> powerset(const Set& s)
{
    return powerset(s, s.size());
}

/**
 * overload find for container
 */
template<typename C>
typename C::const_iterator find(const C& c, const typename C::value_type& e)
{
    return std::find(c.begin(), c.end(), e);
}
template<typename C>
typename C::iterator find(C& c, const typename C::value_type& e)
{
    return std::find(c.begin(), c.end(), e);
}

} //~namespace opencog

#endif
