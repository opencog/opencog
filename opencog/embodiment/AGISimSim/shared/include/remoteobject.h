/***************************************************************************
 *  RemoteObject class.
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

#ifndef REMOTE_OBJECT_H
#define REMOTE_OBJECT_H

//class Listener;
struct Listener;

//------------------------------------------------------------------------------------------------------------
/** @class RemoteObject
	\brief Subclasses will have the ability to store and retrieve
	(variable, value) pairs of multiple sorts. Variable names are
	case-insensitive. */
//------------------------------------------------------------------------------------------------------------
class RemoteObject {
protected:
	map<nocase_string, nocase_string> 	   	   properties;
	map<nocase_string, set<nocase_string> >    validValues;
	map<nocase_string, shared_ptr<Listener> >  listeners;
	set<nocase_string> 			   			   privateSetProperties;
	set<nocase_string> 	                  	   privateGetProperties;
	bool liberal;
public:
	/**	@param liberal If true, the oject does not check for the validity of the inserted property values.	*/
	RemoteObject (bool _liberal = true);
	~RemoteObject();
	
	/** Set the @param property to value @param value */
	bool Set(const nocase_string property, const nocase_string value);

	/** Set the @param property to value @param value */
	bool Set(const nocase_string property, const int value);

	/** Set the @param property to value @param value */
	bool Set(const nocase_string property, const float value);

	/** Store the value of the @param property into @param value */
	bool Get(const nocase_string property, nocase_string& value) const;

	/** Store the value of the @param property into @param value */
	bool Get(const nocase_string property, int& value) const;

	/** Store the value of the @param property into @param value */
	bool Get(const nocase_string property, float& value) const;
	
	/** Store the value of the @param property into @param value */
	nocase_string Get(const nocase_string property) const;
	
	/** Define which values are possible for the given property.
	NOTE: Does not <i>set</i> the value!
	@param property The name of the property variable
	@param value The set of valid values of the variable	*/
	void DefineValidValues(const nocase_string property,
						   const set<nocase_string> value);

	/** Add a single allowed value for the given property.
	NOTE: Does not <i>set</i> the value!
	@param property The name of the property variable
	@param value A valid value of the variable
	*/				
	void AddValidValue(const nocase_string property,
					   const nocase_string value);

	/** Set an object to be called whenever the property value changes. */
	void SetPropertyListener(const nocase_string property,
							 shared_ptr<Listener> listener);

	/** Disable the setting of the value of @param property outside the subclass */
	void MakePrivateSet(const nocase_string property);
	
	/** Disable the getting of the value of @param property outside the subclass */
	void MakePrivateGet(const nocase_string property);
					
	/** Print and return a dump of the stored variable values */
	string PrintList() const;
	
	/** Return an XML dump of the stored variable values */
	string AsXML() const;
};

#endif
