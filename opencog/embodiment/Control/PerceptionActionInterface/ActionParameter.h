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
#include <opencog/atomspace/AtomSpace.h>

#include <boost/variant.hpp>

#include <string>
#include <exception>

#include "ActionParamType.h"

namespace opencog { namespace pai {

using namespace std;
using namespace boost;

#define MIN_DOUBLE 0.0001


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

    bool operator==(const Vector& other) const
    {
        if ((x - other.x < MIN_DOUBLE) &&
            (y - other.y < MIN_DOUBLE) &&
            (z - other.z < MIN_DOUBLE) )
            return true;
        else
            return false;

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

    bool operator==(const Rotation& other) const
    {
        if ((pitch - other.pitch < MIN_DOUBLE) &&
            (roll - other.roll < MIN_DOUBLE) &&
            (yaw - other.yaw < MIN_DOUBLE) )
            return true;
        else
            return false;

    }
};

/**
 * XSD entity element struct.
 */

struct Entity {
public:
    static Entity NON_Entity;
    string id;
    string type;

    Entity() {}
    Entity(string ii, string tt) {
        id = ii;
        type = tt;
    }

    bool operator==(const Entity& other) const
    {
       return ((id == other.id) && (type == other.type) );

    }

    std::string stringRepresentation() const
    {
        return "(" + id + "," + type + ")";
    }

};

struct FuzzyIntervalFloat
{
public:
    float bound_low;
    float bound_high;

    FuzzyIntervalFloat(){}
    FuzzyIntervalFloat(float low, float high)
    {
        bound_low = low;
        bound_high = high;
    }

    bool operator==(const FuzzyIntervalFloat& other) const
    {
        return ((bound_low == other.bound_low)&&(bound_high == other.bound_high));
    }

    bool isInsideMe(float f) const
    {
        if ((f <= bound_high) && (f >= bound_low))
           return true;
        else
           return false;
    }

    bool isInsideMe(const FuzzyIntervalFloat& other) const
    {
        return ((other.bound_high < bound_high) && (other.bound_low > bound_low));
    }
};

struct FuzzyIntervalInt
{
public:
    int bound_low;
    int bound_high;

    FuzzyIntervalInt(){}
    FuzzyIntervalInt(int low, int high)
    {
        bound_low = low;
        bound_high = high;
    }

    bool operator==(const FuzzyIntervalInt& other) const
    {
        return ((bound_low == other.bound_low)&&(bound_high == other.bound_high));
    }

    bool isInsideMe(int i)
    {
        if ((i <= bound_high) && (i >= bound_low))
           return true;
        else
           return false;

    }

    bool isInsideMe(FuzzyIntervalInt& other)
    {
        return ((other.bound_high < bound_high) && (other.bound_low > bound_low));
    }
};

typedef variant<string, Rotation, Vector, Entity,FuzzyIntervalInt,FuzzyIntervalFloat > ParamValue;

extern ParamValue UNDEFINED_VALUE;

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

    string valueString; // The string to represent value, for easy debugging

public:
    static bool areFromSameType(const ParamValue& v1, const ParamValue& v2);

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
    string getName() const;
    const ActionParamType& getType() const;
    ParamValue &getValue();

    void assignValue(const ParamValue& newValue);

    /**
     * Methods for checking the real type of the value attribute
     */
    bool isStringValue() const;
    bool isRotationValue() const;
    bool isVectorValue() const;
    bool isEntityValue() const;
    bool isFuzzyIntervalIntValue() const;
    bool isFuzzyIntervalFloatValue() const;

    static bool isStringValue(ParamValue value) ;
    static bool isRotationValue(ParamValue value) ;
    static bool isVectorValue(ParamValue value) ;
    static bool isEntityValue(ParamValue value) ;
    static bool isFuzzyIntervalIntValue(ParamValue value) ;
    static bool isFuzzyIntervalFloatValue(ParamValue value) ;

    /**
     * Methods for getting the real value of the value attribute.
     * Before calling one of thes methods, the is<Type>Value() method
     * must be called before for checking the right real value.
     */
    const string& getStringValue() const;
    const Rotation& getRotationValue() const;
    const Vector& getVectorValue() const;
    const Entity& getEntityValue() const;
    const FuzzyIntervalInt& getFuzzyIntervalIntValue() const;
    const FuzzyIntervalFloat& getFuzzyIntervalFloatValue() const;

    static const string& getStringValue(ParamValue value) ;
    static const Rotation& getRotationValue(ParamValue value) ;
    static const Vector& getVectorValue(ParamValue value) ;
    static const Entity& getEntityValue(ParamValue value) ;
    static const FuzzyIntervalInt& getFuzzyIntervalIntValue(ParamValue value) ;
    static const FuzzyIntervalFloat& getFuzzyIntervalFloatValue(ParamValue value) ;

    std::string stringRepresentation() const throw (opencog::RuntimeException, std::bad_exception);
    std::string static ParamValueToString(const ParamValue& paramVal);

    inline bool operator == (ActionParameter& other) const
    {
        if (name != other.getName())
            return false;

        if (type != other.getType())
            return false;

        if (!(value == other.getValue()))
            return false;

        return true;
    }

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
