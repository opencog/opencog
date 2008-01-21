#include "utils2.h"

//this typedef is put is the .cc not the .h because that provokes a vtree clash
//with combo::vtree
typedef tree<Vertex> vtree;
typedef vtree::iterator pre_it;

vtree MakeVirtualAtom_slow(Handle T, const vtree& t1, const vtree& t2, const vtree& t3, const vtree& t4, const vtree& t5)
{
	try
	{
		vtree ret;
		ret.set_head(Vertex((Handle)T));
		pre_it head_it = ret.begin();
		ret.replace(ret.append_child(head_it), t1.begin());
		ret.replace(ret.append_child(head_it), t2.begin());
		ret.replace(ret.append_child(head_it), t3.begin());
		ret.replace(ret.append_child(head_it), t4.begin());
		ret.replace(ret.append_child(head_it), t5.begin());
		
		return ret;
						
	} catch(...) { puts("MakeVirtualAtom_slow exception."); getc(stdin); return vtree(); }
}
vtree MakeVirtualAtom_slow(Handle T, const vtree& t1, const vtree& t2, const vtree& t3, const vtree& t4)
{
	try
	{
		vtree ret;
		ret.set_head(Vertex((Handle)T));
		pre_it head_it = ret.begin();
		ret.replace(ret.append_child(head_it), t1.begin());
		ret.replace(ret.append_child(head_it), t2.begin());
		ret.replace(ret.append_child(head_it), t3.begin());
		ret.replace(ret.append_child(head_it), t4.begin());
		
		return ret;
						
	} catch(...) { puts("MakeVirtualAtom_slow exception."); getc(stdin); return tree<Vertex>(); }
}
vtree MakeVirtualAtom_slow(Handle T, const vtree& t1, const vtree& t2, const vtree& t3)
{
	try
	{
		vtree ret;
		ret.set_head(Vertex((Handle)T));
		pre_it head_it = ret.begin();
		ret.replace(ret.append_child(head_it), t1.begin());
		ret.replace(ret.append_child(head_it), t2.begin());
		ret.replace(ret.append_child(head_it), t3.begin());
		
		return ret;
						
	} catch(...) { puts("MakeVirtualAtom_slow exception."); getc(stdin);  return vtree(); }
}

vtree MakeVirtualAtom_slow(Handle T, const vtree& t1, const vtree& t2)
{
	try
	{
		vtree ret;
		ret.set_head(Vertex((Handle)T));
		pre_it head_it = ret.begin();
		ret.replace(ret.append_child(head_it), t1.begin());
		ret.replace(ret.append_child(head_it), t2.begin());
		
		return ret;
						
	} catch(...) { puts("MakeVirtualAtom_slow exception."); getc(stdin);  return tree<Vertex>(); }
}

vtree MakeVirtualAtom_slow(Handle T, const vtree& t1)
{
	try
	{
		vtree ret;
		ret.set_head(Vertex((Handle)T));
		pre_it head_it = ret.begin();
		ret.replace(ret.append_child(head_it), t1.begin());
		
		return ret;
						
	} catch(...) { puts("MakeVirtualAtom_slow exception."); getc(stdin);  return tree<Vertex>(); }
}

vtree MakeVirtualAtom_slow(Handle T)
{
	try
	{
		vtree ret;
		ret.set_head(Vertex((Handle)T));
		
		return ret;
						
	} catch(...) { puts("MakeVirtualAtom_slow exception."); getc(stdin);  return tree<Vertex>(); }
}
vtree MakeVirtualAtom_slow(Vertex T, const vtree& t1, const vtree& t2, const vtree& t3, const vtree& t4, const vtree& t5)
{
	try
	{
		vtree ret;
		ret.set_head(T);
		pre_it head_it = ret.begin();
		ret.replace(ret.append_child(head_it), t1.begin());
		ret.replace(ret.append_child(head_it), t2.begin());
		ret.replace(ret.append_child(head_it), t3.begin());
		ret.replace(ret.append_child(head_it), t4.begin());
		ret.replace(ret.append_child(head_it), t5.begin());
		
		return ret;
						
	} catch(...) { puts("MakeVirtualAtom_slow exception."); getc(stdin);  return tree<Vertex>(); }
}
vtree MakeVirtualAtom_slow(Vertex T, const vtree& t1, const vtree& t2, const vtree& t3, const vtree& t4)
{
	try
	{
		vtree ret;
		ret.set_head(T);
		pre_it head_it = ret.begin();
		ret.replace(ret.append_child(head_it), t1.begin());
		ret.replace(ret.append_child(head_it), t2.begin());
		ret.replace(ret.append_child(head_it), t3.begin());
		ret.replace(ret.append_child(head_it), t4.begin());
		
		return ret;
						
	} catch(...) { puts("MakeVirtualAtom_slow exception."); getc(stdin);  return tree<Vertex>(); }
}
vtree MakeVirtualAtom_slow(Vertex T, const vtree& t1, const vtree& t2, const vtree& t3)
{
	try
	{
		vtree ret;
		ret.set_head(T);
		pre_it head_it = ret.begin();
		ret.replace(ret.append_child(head_it), t1.begin());
		ret.replace(ret.append_child(head_it), t2.begin());
		ret.replace(ret.append_child(head_it), t3.begin());
		
		return ret;
						
	} catch(...) { puts("MakeVirtualAtom_slow exception."); getc(stdin);  return tree<Vertex>(); }
}

vtree MakeVirtualAtom_slow(Vertex T, const vtree& t1, const vtree& t2)
{
	try
	{
		vtree ret;
		ret.set_head(T);
		pre_it head_it = ret.begin();
		ret.replace(ret.append_child(head_it), t1.begin());
		ret.replace(ret.append_child(head_it), t2.begin());
		
		return ret;
						
	} catch(...) { puts("MakeVirtualAtom_slow exception."); getc(stdin);  return tree<Vertex>(); }
}

vtree MakeVirtualAtom_slow(Vertex T, const vtree& t1)
{
	try
	{
		vtree ret;
		ret.set_head(T);
		pre_it head_it = ret.begin();
		ret.replace(ret.append_child(head_it), t1.begin());
		
		return ret;
						
	} catch(...) { puts("MakeVirtualAtom_slow exception."); getc(stdin);  return tree<Vertex>(); }
}

vtree MakeVirtualAtom_slow(Vertex T)
{
	try
	{
		vtree ret;
		ret.set_head(T);
		
		return ret;
						
	} catch(...) { puts("MakeVirtualAtom_slow exception."); getc(stdin);  return tree<Vertex>(); }
}
