#ifndef ANDRULEARITYFREERULE_H
#define ANDRULEARITYFREERULE_H

namespace reasoning
{

/**
	@class ArityFreeANDRule
	Shouldn't be used directly. Use AndRule<number of arguments> instead.
*/

class ArityFreeANDRule : public Rule
{
protected:
	ArityFreeANDRule(iAtomTableWrapper *_destTable)
	: Rule(_destTable,true,true,"")
	{}
	SymmetricANDFormula fN;
	AsymmetricANDFormula f2;
public:
	bool validate2				(MPs& args) const { return true; }

	bool asymmetric(Handle* A, Handle* B) const;
	//Handle compute(Handle A, Handle B, Handle CX = NULL)  const; //std::vector<Handle> vh)
	BoundVertex computeSymmetric(const vector<Vertex>& premiseArray, Handle CX = NULL) const;
	void DistinguishNodes(const vector<Vertex>& premiseArray, set<Handle>& ANDlinks, set<Handle>& nodes) const;

	BoundVertex compute(const vector<Vertex>& premiseArray, Handle CX = NULL) const=0;
};

} // namespace reasoning
#endif // ANDRULEARITYFREERULE_H

