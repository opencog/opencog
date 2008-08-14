#ifndef PRINTRULE_H
#define PRINTRULE_H

namespace reasoning
{

#if 0
class PrintRule : public Rule
{
public:
	PrintRule(iAtomTableWrapper *_destTable)
	: Rule(_destTable,false,true,"PrintRule")
	{
		inputFilter.push_back(meta(
				new tree<Vertex>(mva((Handle)ATOM))));
		inputFilter.push_back(meta(
				new tree<Vertex>(mva((Handle)ATOM))));
	}

	Rule::setOfMPs o2iMetaExtra(meta outh, bool& overrideInputFilter) const
	{
		return Rule::setOfMPs();
	}

	NO_DIRECT_PRODUCTION;

  BoundVertex compute(const vector<Vertex>& premiseArray, Handle CX = NULL) const
  {
  	for (int i = 0; i < premiseArray.size(); i++)
  		printTree(premiseArray[i], 0, 0);
  }
};
#endif

} // namespace reasoning
#endif // PRINTRULE_H
