#include "OctoMapValue.h"

#include <opencog/spacetime/octomap/atom_types.h>
#include <opencog/util/exceptions.h>
#include <opencog/atoms/base/ClassServer.h>
#include <opencog/atoms/value/ValueFactory.h>

using namespace opencog;


void OctoValue::update() const
{
    auto it = (_om->get_locations_of_atom(_item)).rbegin(); //We need the last.
    _value = {it->x(), it->y(), it->z()};
}

void OctoValue::update(const std::vector<double>& loc) const
{
  _value = {loc[0], loc[1], loc[2]};
}

OctoValue::OctoValue(const HandleSeq& hseq) : FloatValue(OCTO_VALUE)
{

    if( hseq.size() != 2) {
        throw InvalidParamException(TRACE_INFO, 
                "Required 2 arguments. Provided %d.", hseq.size());
    }

    _item = hseq[0];
    _octo_node = hseq[1];
    _om = OctoMapNodeCast(_octo_node)->get_map();
    update();
}

bool OctoValue::operator==(const Value& other) const
{
    if (OCTO_VALUE != other.get_type()) return false;

    const OctoValue* fov = (const OctoValue*) &other;

    if (_octo_node == fov->_octo_node) return true;

    return false;
}

std::string OctoValue::to_string(const std::string& indent) const
{
    update(); // Update values
    std::string rv = indent + "(" + nameserver().getTypeName(_type);
    rv += ("\n " + _item->to_short_string() + " ");
    rv += (_octo_node->to_short_string() + " (");
    for (double v : _value) {
        char buf[40];
        snprintf(buf, 40, "%.17g", v);
        rv += std::string(" ") + buf;
    }
    rv += ")\n)\n";

    return rv;
}

// Adds factory when the library is loaded.
DEFINE_VALUE_FACTORY(OCTO_VALUE, createOctoValue, HandleSeq)
