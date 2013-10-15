/*
 * opencog/rest/JsonUtil.cc
 *
 * Copyright (C) 2010 by Singularity Institute for Artificial Intelligence
 * Copyright (C) 2010 by Joel Pitt
 * All Rights Reserved
 *
 * Written by Joel Pitt <joel@fruitionnz.com>
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

#include <opencog/atomspace/TruthValue.h>
#include <opencog/atomspace/IndefiniteTruthValue.h>
#include <opencog/atomspace/SimpleTruthValue.h>
#include <opencog/atomspace/CountTruthValue.h>
#include <opencog/atomspace/CompositeTruthValue.h>

#include <opencog/web/json_spirit/json_spirit.h>

using namespace json_spirit;

namespace opencog {

bool assertJSONTVCorrect(std::string expected, std::string actual,
        std::ostringstream& _output)
{
    if (expected != actual) {
        _output << "{\"error\":\"expected '" << expected << "' but got '"
            << actual << "', check JSON.\"}" << std::endl;
        return false;
    }
    return true;
}

bool assertJsonMapContains(const Object& o, std::vector<std::string> keys,
        std::ostringstream& _output)
{
    std::vector<bool> key_present;
    for (unsigned int i=0; i < keys.size(); ++i)
        key_present.push_back(false);
    // check all strings in keys are in json object
    for( Object::size_type i = 0; i < o.size(); ++i ) {
        // # keys checked should only be small amounts < 5
        // so just iterate.
        unsigned int j;
        for (j = 0; j < keys.size(); j++) {
            if (keys[j] == o[i].name_) {
                key_present[j] = true;
                break;
            }
        }
        if (j == keys.size()) {
            _output << "{\"error\":\"unknown truth value key '" << o[i].name_ <<
                "'\"}" << std::endl;
            return false; // unexpected key
        }
    }
    for (unsigned int i=0; i < keys.size(); ++i) {
        if (!key_present[i]) {
            _output << "{\"error\":\"missing truth value key '" << keys[i] <<
                "'\"}" << std::endl;
            return false;
        }
    }
    return true;
}

TruthValuePtr JSONToTV(const Value& v, std::ostringstream& _output )
{
    TruthValuePtr tv = NULL;
    const Object& tv_obj = v.get_obj();
    if (tv_obj.size() != 1) return NULL;
    const Pair& pair = tv_obj[0];
    std::string tv_type_str = pair.name_;
    if (tv_type_str == "simple") {
        const Object& stv_obj = pair.value_.get_obj();
        float str=0.0f, count=0.0f;
        std::vector<std::string> key_list;
        key_list.push_back("str");
        key_list.push_back("count");
        if (!assertJsonMapContains(stv_obj, key_list, _output)) return NULL;
        for( Object::size_type i = 0; i < stv_obj.size(); ++i ) {
            if (stv_obj[i].name_ == "str")
                str = stv_obj[i].value_.get_real();
            else if (stv_obj[i].name_ == "count")
                count = stv_obj[i].value_.get_real();
        }
        tv = SimpleTruthValue::createTV(str,count);
    } else if (tv_type_str == "count") {
        const Object& stv_obj = pair.value_.get_obj();
        float str=0.0f, conf=0.0f, count=0.0f;
        std::vector<std::string> key_list;
        key_list.push_back("str");
        key_list.push_back("conf");
        key_list.push_back("count");
        if (!assertJsonMapContains(stv_obj, key_list, _output)) return NULL;
        for( Object::size_type i = 0; i < stv_obj.size(); ++i ) {
            if (stv_obj[i].name_ == "str")
                str = stv_obj[i].value_.get_real();
            else if (stv_obj[i].name_ == "count")
                count = stv_obj[i].value_.get_real();
            else if (stv_obj[i].name_ == "conf")
                conf = stv_obj[i].value_.get_real();
        }
        tv = CountTruthValue::createTV(str,conf,count);
    } else if (tv_type_str == "indefinite") {
        //! @todo Allow asymmetric Indefinite TVs
        const Object& stv_obj = pair.value_.get_obj();
        float conf=0.0f, l=0.0f, u=0.0f;
        std::vector<std::string> key_list;
        key_list.push_back("l");
        key_list.push_back("u");
        key_list.push_back("conf");
        if (!assertJsonMapContains(stv_obj, key_list, _output)) return NULL;
        for( Object::size_type i = 0; i < stv_obj.size(); ++i ) {
            if (stv_obj[i].name_ == "l")
                l = stv_obj[i].value_.get_real();
            else if (stv_obj[i].name_ == "u")
                u = stv_obj[i].value_.get_real();
            else if (stv_obj[i].name_ == "conf")
                conf = stv_obj[i].value_.get_real();
        }
        tv = IndefiniteTruthValue::createTV(l, u, conf);
    } else if (tv_type_str == "composite") {
        const Object& stv_obj = pair.value_.get_obj();
        if (stv_obj.size() == 0) return NULL;
        if (!assertJSONTVCorrect("primary",stv_obj[0].name_, _output)) return NULL;
        TruthValuePtr primary_tv = JSONToTV(stv_obj[0].value_, _output);
        if (primary_tv == NULL) return NULL;
        CompositeTruthValuePtr ctv = CompositeTruthValue::createCTV(primary_tv,
                NULL_VERSION_HANDLE);
        for( Object::size_type i = 1; i < stv_obj.size(); ++i ) {
            TruthValuePtr ctxt_tv = NULL;
            std::string indicatorStr;
            try {
                const Pair& pair = stv_obj[i];
                indicatorStr = pair.name_; // 
                const Array& vharray = pair.value_.get_array();
                // should be [ contextual UUID, {TV} ]
                if (vharray.size() != 2) continue;
                Handle context = Handle(vharray[0].get_uint64());
                ctxt_tv = JSONToTV(vharray[1], _output);
                if (ctxt_tv == NULL) {
                    return NULL;
                }
                IndicatorType indicator =
                    VersionHandle::strToIndicator(indicatorStr.c_str());
                ctv->setVersionedTV(ctxt_tv, VersionHandle(indicator, context));
            } catch (InvalidParamException& e) {
                _output << "{\"error\":\"bad indicator for version handle: '" <<
                    indicatorStr << "'\"}" << std::endl;
                return NULL;
            } catch (std::runtime_error& e) {
                _output << "{\"error\":\"bad json in truth value\"}" << std::endl;
                return NULL;
            }
        }
        tv = ctv;
    }
    return tv;
}

} // namespace
