#ifndef _OPENCOG_UTIL_FOREACH_H
#define _OPENCOG_UTIL_FOREACH_H

#include <boost/iterator/counting_iterator.hpp>
#include <boost/foreach.hpp>

namespace boost {

//namespace foreach {
// template<>
//struct is_lightweight_proxy<int> : mpl::true_ { };
//} // namespace foreach

/// @todo check if any of this is actually used. Probably superseded
/// by Boost.Range
    
inline boost::counting_iterator<int> boost_range_begin(int) {
    return make_counting_iterator(0);
}
inline boost::counting_iterator<int> boost_range_end(int i) {
    return make_counting_iterator(i);
}
template<>
struct range_iterator<int> { typedef boost::counting_iterator<int> type; };
template<>
struct range_const_iterator<int> { typedef boost::counting_iterator<int> type; };

inline boost::counting_iterator<int> boost_range_begin(const std::pair<int,int>& i) {
    return make_counting_iterator(i.first);
}
inline boost::counting_iterator<int> boost_range_end(const std::pair<int,int>& i) {
    return make_counting_iterator(i.second);
}
template<>
struct range_iterator<std::pair<int,int> > { 
    typedef boost::counting_iterator<int> type; 
};
template<>
struct range_const_iterator<std::pair<int,int> > { 
    typedef boost::counting_iterator<int> type; 
};

} // namespace boost

namespace opencog { 
    inline std::pair<int,int> from_one(int i) { return std::pair<int,int>(1,i+1); }
} //~namespace opencog

#define foreach BOOST_FOREACH
#define reverse_foreach BOOST_REVERSE_FOREACH

#endif
