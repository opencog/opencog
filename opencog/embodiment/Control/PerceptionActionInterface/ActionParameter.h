/*
 * opencog/embodiment/Control/PerceptionActionInterface/ActionParameter.h
 *
 * Copyright (C) 2002-2009 Novamente LLC
 * All Rights Reserved
 * Author(s): Welter Luigi
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

#ifndef ACTIONPARAMETER_H_
#define ACTIONPARAMETER_H_

#include <xercesc/dom/DOM.hpp>
#include <xercesc/dom/DOMElement.hpp>
#include <xercesc/dom/DOMDocument.hpp>
#include <xercesc/util/PlatformUtils.hpp>

#include <boost/variant.hpp>

#include <string>
#include <exception>

#include "ActionParamType.h"

namespace opencog { namespace pai {

using namespace std;
using namespace boost;


/**
 * XSD vector element struct.
 */
struct Vector {
public:
    double x;
    double y;
    double z;

    Vector() {
        x = -1; y = -1; z = -1;
    }
    Vector(double xx, double yy, double zz) {
        x = xx;
        y = yy;
        z = zz;
    }
};

/**
 * XSD rotation element struct.
 */
struct Rotation {
public:
    double pitch;
    double roll;
    double yaw;

    Rotation() {}
    Rotation(double pp, double rr, double yy) {
        pitch = pp;
        roll = rr;
        yaw = yy;
    }
};

/**
 * XSD entity element struct.
 */
struct Entity {
public:
    string id;
    string type;

    Entity() {}
    Entity(string ii, string tt) {
        id = ii;
        type = tt;
    }
};


typedef variant<string, Rotation, Vector, Entity > ParamValue;

/**
 * Boost visitor for checking real types inside ParamValue variants.
 */
class same_type_visitor : public static_visitor<bool>
{
public:

    template <typename T, typename U>
    bool operator()( const T &, const U & ) const {
        return false; // different types
    }

    template <typename T>
    bool operator()( const T & lhs, const T & rhs ) const {
        return true; // same types
    }
};

class ActionParameter
{

private:

    /**
     * Name of the parameter
     */
    string name;

    /**
     * Type of the parameter
     */
    ActionParamType type;

    /**
     * Value of the parameter
     */
    ParamValue value;

    static bool areFromSameType(const ParamValue& v1, const ParamValue& v2);

public:

    /**
     * Empty Constructor
     */
    ActionParameter();

    /**
     * Constructor
     */
    ActionParameter(const string& name, const ActionParamType& type, const ParamValue& value);

    /**
     * Destructor
     */
    ~ActionParameter();


    /**
     * Basic getters
     */
    const string& getName() const;
    const ActionParamType& getType() const;
    const ParamValue& getValue() const;

    /**
     * Methods for checking the real type of the value attribute
     */
    bool isStringValue() const;
    bool isRotationValue() const;
    bool isVectorValue() const;
    bool isEntityValue() const;

    /**
     * Methods for getting the real value of the value attribute.
     * Before calling one of thes methods, the is<Type>Value() method
     * must be called before for checking the right real value.
     */
    const string& getStringValue() const;
    const Rotation& getRotationValue() const;
    const Vector& getVectorValue() const;
    const Entity& getEntityValue() const;

    std::string stringRepresentation() const throw (opencog::RuntimeException, std::bad_exception);

    /**
     * Create a pvp xml element in the DOMDocument XML document. An action parameter element has the form:
     *   <param name="" type="" value=""/>
     * or
     *   <param name="" type="vector">
     *      <vector x="" y="" z=""/>
     *   </param>
     * or
     *   <param name="" type="rotation">
     *      <rotation pitch="" roll="" yaw=""/>
     *   </param>
     * or
     *   <param name="" type="entity">
     *      <entity id="" parent-id="" type=""/>
     *   </param>
     *
     * @param doc The XML document where the element will be inserted.
     * @param parent The parent element where the element will be created.
     * @return The DOMElement just created.
     */
    XERCES_CPP_NAMESPACE::DOMElement* createPVPXmlElement(XERCES_CPP_NAMESPACE::DOMDocument* doc,
            XERCES_CPP_NAMESPACE::DOMElement* parent) const;

};

} } // namespace opencog::pai

#endif /*ACTIONPARAMETER_H_*/
