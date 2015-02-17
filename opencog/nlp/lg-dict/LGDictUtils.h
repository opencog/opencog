#ifndef _OPENCOG_LG_DICT_UTILS_H
#define _OPENCOG_LG_DICT_UTILS_H

namespace opencog
{

bool lg_conn_type_match(const Handle& hConn1, const Handle& hConn2);
bool lg_conn_linkable(const Handle& hConn1, const Handle& hConn2);

}

#endif // _OPENCOG_LG_DICT_UTILS_H
