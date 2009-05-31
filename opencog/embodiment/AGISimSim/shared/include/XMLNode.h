/*
 * opencog/embodiment/AGISimSim/shared/include/XMLNode.h
 *
 * Copyright (C) 2002-2009 Novamente LLC
 * All Rights Reserved
 * Author(s): Ari A. Heljakka
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


#ifndef __XMLNODE_H
#define __XMLNODE_H
//#define nocase_string string //Uncomment if case-sensitivity wanted.

//------------------------------------------------------------------------------------------------------------
/** @class XMLtag
 \brief A helper class to contain the data of an XMLNode. */
//------------------------------------------------------------------------------------------------------------
struct XMLtag {
    nocase_string       name;
    std::map<nocase_string, nocase_string>  arguments;
    std::string         textcontent;
};

//------------------------------------------------------------------------------------------------------------
/** @class XMLNode
 \brief An elementary XML parser.
 The class can only handle nested blocks with arguments.
 No actual XML special tags, such as "<? XML" are tolerated.
 Upon parse error, an exception string is thrown.
 Eg. this is a valid string:
 "<xml>
  <mytag> Hello, World! </mytag>
 </xml>"

 For the most part, you should use XML_ITERATE, XML_VALIDATE and XML_HANDLE
 macros defined at the end of this header.
*/
//------------------------------------------------------------------------------------------------------------
class XMLNode
{
public:
    typedef std::vector<XMLNode*> ChildNodes;

    /** The typical way of creating an XMLNode. @param data The XML string. */
    XMLNode (const std::string& data, XMLNode* _super = NULL);
    XMLNode (char* data, XMLNode* _super, int size);
    XMLNode (const XMLNode& rhs);
    XMLNode ();
    ~XMLNode();

    void   SetName   (std::string n);
    XMLNode*    Super     ();
    ChildNodes  StealSub();
    ChildNodes& UnSafeSub ();

    /** Returns the subnodes of this XML node. */
    const ChildNodes& Sub   ();
    /** Returns the name and content of this XML node. */
    const XMLtag&    TagData ();
    bool           Add     (const XMLNode& node);
    std::string         AsText  ();
    bool      ok;
private:
    int      bytesconsumed;
    XMLtag   tagdata;
    XMLNode* super;
    std::vector<XMLNode*> sub;

    static int   ExpandLevel(const std::string&  _source,
                             XMLtag&   _contents,
                             std::vector<XMLNode*>& _sub,
                             XMLNode*   _super); //true = meaninful expansion was done.
    static bool  ExpandArgument (char** ptr, std::map<nocase_string, nocase_string>& arg, bool* ends);
    static char* nextc          (char **ptr);
};

//------------------------------------------------------------------------------------------------------------
struct iXMLvalue {
    virtual std::string AsXML() const = 0;
    virtual ~iXMLvalue() {}
};

//------------------------------------------------------------------------------------------------------------
template<typename T>
struct XMLvalue : public iXMLvalue {
    std::string  name;
    T            value;

    XMLvalue (std::string _name, T _value);
    virtual ~XMLvalue ();
    std::string AsXML () const;
    virtual std::string Value2String () const = 0;
};

//------------------------------------------------------------------------------------------------------------
struct IntXMLValue : XMLvalue<int> {
    IntXMLValue (std::string name, int _value);
    std::string Value2String () const;
};

//------------------------------------------------------------------------------------------------------------
struct StringXMLValue : XMLvalue<std::string> {
    StringXMLValue (std::string name, std::string _value);
    std::string Value2String () const;
};

//------------------------------------------------------------------------------------------------------------
struct FloatXMLValue : XMLvalue<float> {
    FloatXMLValue (std::string name, float _value);
    std::string Value2String () const;
};

void CleanSpace    (std::string& s);

#define XMLVALIDATE(node, nodename, badnode_handling) \
    if ((node).TagData().name != (nodename))  \
        badnode_handling;

#define XMLITERATE(node, child) \
    for (XMLNode::ChildNodes::const_iterator  child = (node).Sub().begin();\
            child!= (node).Sub().end(); \
            child++)

#define XMLHANDLE(node, nodename) \
    if ((node).TagData().name == (nodename))

#endif
