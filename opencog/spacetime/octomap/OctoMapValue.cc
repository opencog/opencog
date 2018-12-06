#include "OctoMapValue.h"

#include <opencog/spacetime/octomap/atom_types.h>
#include <opencog/util/exceptions.h>
#include <opencog/atoms/base/ClassServer.h>
#include <opencog/atoms/value/ValueFactory.h>

using namespace opencog;


void OctoValue::update()
{
    //point3d p = (_om->get_locations_of_atom(_octo_atom)).front();
    //_value = {p.x(), p.y(), p.z()};
    _om->insert_atom(point3d(_value[0], _value[1], _value[2]), _item);
}

OctoValue::OctoValue(const HandleSeq& hseq,
                     std::vector<double> values) : FloatValue(OCTO_VALUE, values)
{

    if( hseq.size() != 2) {
        throw InvalidParamException(TRACE_INFO, 
                "Required 2 arguments. Provided %d.", hseq.size());
    }

    _item = hseq[0];
    _octo_atom = hseq[1];
    _om = OctoMapNodeCast(_octo_atom)->get_map();

    update();
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
static __attribute__ ((constructor)) void init(void)
{
    valueserver().addFactory(OCTO_VALUE,(ValueFactory)&createOctoValue,
                             std::vector<std::type_index> {
                               std::type_index(typeid(HandleSeq)),
                               std::type_index(typeid(std::vector<double>))
                             });
}

