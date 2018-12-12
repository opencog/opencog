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

OctoValue::OctoValue(const HandleSeq& hseq) : FloatValue(OCTO_VALUE)
{

    if( hseq.size() != 2) {
        throw InvalidParamException(TRACE_INFO, 
                "Required 2 arguments. Provided %d.", hseq.size());
    }

    _item = hseq[0];
    _octo_atom = hseq[1];
    _om = OctoMapNodeCast(_octo_atom)->get_map();
    // TODO start tracking _item by saying sth like _om->insert(_item)
}

bool OctoValue::operator==(const Value& other) const
{
    if (OCTO_VALUE != other.get_type()) return false;

    const OctoValue* fov = (const OctoValue*) &other;

    if (_octo_atom == fov->_octo_atom) return true;

    return false;
}

std::string OctoValue::to_string(const std::string& indent) const
{
    update(); // Update values
    std::string rv = indent + "(" + nameserver().getTypeName(_type);
    rv += ("\n " + _item->to_short_string() + " ");
    rv += (_octo_atom->to_short_string() + " (");
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
