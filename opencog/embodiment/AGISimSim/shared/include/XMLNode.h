/***************************************************************************
 *  Modest XML parser.        
 *
 *  Project: AgiSim
 *
 *  See implementation (.cc, .cpp) files for license details (GPL).
 *
 *  Fri Feb 18 11:35:16 2005
 *  Copyright  2005  Ari A. Heljakka / Novamente LLC
 *  Email [heljakka at iki dot fi]
 *
 *	19.01.06	FP	formatting 	
 ****************************************************************************/

#ifndef __XMLNODE_H
#define __XMLNODE_H
//#define nocase_string string //Uncomment if case-sensitivity wanted.

//------------------------------------------------------------------------------------------------------------
/** @class XMLtag
	\brief A helper class to contain the data of an XMLNode. */
//------------------------------------------------------------------------------------------------------------
struct XMLtag {
	nocase_string					  name;
	map<nocase_string,nocase_string>  arguments;
	string							  textcontent;
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
class XMLNode {
public:
	typedef vector<XMLNode*> ChildNodes;

	/** The typical way of creating an XMLNode. @param data The XML string.	*/
	XMLNode (const string& data, XMLNode* _super = NULL);
	XMLNode (char* data, XMLNode* _super, int size);
	XMLNode (const XMLNode& rhs);
	XMLNode ();
	~XMLNode();
	
	void 		SetName   (string n);
	XMLNode*    Super     ();
	ChildNodes  StealSub();
	ChildNodes& UnSafeSub ();

	/** Returns the subnodes of this XML node. */
	const ChildNodes& Sub	  ();
	/** Returns the name and content of this XML node. */
	const XMLtag& 	  TagData ();
	bool   		      Add     (const XMLNode& node);
	string 		      AsText  ();	
	bool 			  ok;
private:
	int      bytesconsumed;
	XMLtag   tagdata;
	XMLNode* super;
	vector<XMLNode*> sub;

	static int   ExpandLevel(const string&		_source,
				 			 XMLtag&			_contents,
							 vector<XMLNode*>&	_sub,
							 XMLNode*			_super); //true = meaninful expansion was done.
	static bool  ExpandArgument (char** ptr, map<nocase_string,nocase_string>& arg,bool* ends);
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

void CleanSpace    (string& s);

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
