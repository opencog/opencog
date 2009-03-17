#ifndef PREDAVESE_H
#define PREDAVESE_H

//#include "PredaveseStdafx.h"
#include <boost/shared_ptr.hpp>
#include <boost/variant.hpp>
#include <string>
#include <vector>
#include <map>


namespace predavese
{

#define STLhas(cont, el) ((cont).find(el) != cont.end())

using namespace std;
using namespace boost;

const unsigned long recentPeriodOfTime = 60*10;  // what does mean recently? (simulated time in decimal of seconds)

class Atom;
typedef boost::shared_ptr<Atom> pAtom;
typedef unsigned int PatIDT;
typedef boost::variant<string, pAtom> c;
typedef vector<c> pat;
typedef pair<pat, pat> pp;
void print(pat p);

#define __p(elT) make_pat1(pAtom(new Atom(elT)))

class Atom 
{
public:
    PatIDT id;
    vector<c> out;

    Atom(PatIDT _id, vector<c> _out = vector<c>()) : id(_id), out(_out) {}

    string	toString() const;
	string	toPrettyString() const;
    string toIndentString(int indentLevel) const;
    void	print() const;
	void	prettyprint() const;
    void indentprint() const;
};

/// Treats Atoms identical if their IDs are identical. The outgoing set isn't compared here.
/// The idea is that you can't "patmap" Atoms to other Atoms.

struct less_atom
{
    less_atom() {}
    
    bool operator()(pAtom a, pAtom b) const
    {
        if (a->id < b->id)
            return true;
        else
            return false;
    }
};

/// Used by less_pat. Takes the atom comparison operator as a template arg

template<typename less_atomT>
struct less_c
{
    bool operator()(const c& a, const c& b) const
    {
        const string* a_s = get<string>(&a);
        pAtom* a_atom    = const_cast<pAtom*>(get<pAtom>(&a));
        const string* b_s = get<string>(&b);
        pAtom* b_atom    = const_cast<pAtom*>(get<pAtom>(&b));

        if (a_atom && !b_atom)
            return true;
        if (b_atom && !a_atom)
            return false;
        if (a_atom && b_atom)
        {
            if (less_atomT()(*a_atom, *b_atom))
                return true;
            if (less_atomT()(*b_atom, *a_atom))
                return false;
        }
        else
        {
            if (*a_s < *b_s)
                return true;
            if (*b_s < *a_s)
                return false;
        }

        return false;
    }
};

/// Pattern comparison functor. Takes the atom comparison operator as a template arg.

template<typename less_atomT>
struct less_pat
{
    bool operator()(const pat& a, const pat& b) const
    {
        if (a.size() < b.size())
            return true;
        if (a.size() > b.size())
            return false;

        for (pat::const_iterator ai = a.begin(), bi = b.begin();
                ai != a.end();
                ai++, bi++)
            if (less_c<less_atomT>()(*ai, *bi))
                return true;
            else if (less_c<less_atomT>()(*bi, *ai))
                return false;
        
        return false;
    }
};

/// Treats Atoms identical if their IDs and outgoing vectors are identical.

struct less_atom_by_structure
{
    bool operator()(pAtom a, pAtom b) const
    {
        if (a->id < b->id)
            return true;
        if (a->id > b->id)
            return false;

        if (a->out.size() < b->out.size())
            return true;
        if (a->out.size() > b->out.size())
            return false;
        if (less_pat<less_atom_by_structure>()(a->out, b->out))
            return true;
        if (less_pat<less_atom_by_structure>()(b->out, a->out))
            return false;

        return false;
    }
};

typedef map<pat, pat, less_pat<less_atom> > PatMap;


//#define isvalidtoken(ch) ((ch) == ')' || (ch)== '(' || (ch) == ',')	
/// Clean str from non-alphadigit and convert to lower case before passing here. (This function
/// is recursively called, so we can't clean up here.
/*struct clean_string : public string
{
    string val;

    clean_string(string rhs) { val = clean(rhs.c_str()); }
    clean_string(const char* rhs) { val = clean(rhs); }

    static string clean(const char* rhs) {
        string ret;
        int len = (int)strlen(rhs);
        bool started = false;
        for (int i=0; i < len; i++)
            if (isalpha(rhs[i]) || isdigit(rhs[i]) ||
				rhs[i] == '_' ||
				isvalidtoken(rhs[i]) ||
                (started && rhs[i] == ' ') )
            {
                ret.push_back( (rhs[i]>'Z' && isalpha(rhs[i])) ? (rhs[i]-('z'-'Z') ) : rhs[i]);
                started = true;
            }
        puts(ret.c_str());
        return ret;
    }
};
*/

struct promote
{
    c operator()(c _c)
    {
        //string* s = get<string>(&_c);
        pAtom* a = get<pAtom>(&_c);
        if (a && (*a)->out.size()==1)
            return promote()((*a)->out[0]);
        else
            return _c;
    }
};

int replace(c* start, c* end, c* buf_end, int &reduction, const map<pat, pat, less_pat<less_atom> >& patmap);

pat str2pat(string s, const PatMap& patmap);

//string make_upper(string s);

/*
class Parser
{
    const PatMap& patmap;

	friend class PredaveseParser;

public:
    Parser(const map<pat, pat, less_pat<less_atom> >& _patmap)
        : patmap(_patmap) {}

    template<typename OutIterT>
    void parse(string s, OutIterT ret)
    {
        pat p = str2pat(s,patmap); 
        return parse(p, ret, 0, 0);
    }

    template<typename OutIterT>
    void parse(const pat& strs, OutIterT ret, int depth, int original_offset)
    {
        //    print(strs);
        if (STLhas(i_patmap, strs) && original_offset==0) 
        {
            const pat& newStrs = i_patmap[strs];
            parse(newStrs, ret, depth+1, original_offset);
        }
        else if (STLhas(elliptic_prefix_patmap, strs) && original_offset==0)
        {
            pat newp(elliptic_prefix_patmap[strs]);
            transform(strs.begin(), strs.end(), back_inserter(newp), promote());
            parse(newp, ret, depth+1, original_offset);
        }
        else
        {
            //        map<pat, set<pAtom> >::iterator it = parse_cache.find(strs);
            //        if (it != parse_cache.end())
            //            copy(it->second.begin(), it->second.end(), ret);
            //        else
            //        {
            //    printf("\n***     %d     ***\n\n", depth);

            /// Parse all variations of strs created by mapping some subvector of strings into a pattern ID
	        for (unsigned int i=0; i < strs.size(); i++)
                for (unsigned int len=0; i+len <= strs.size(); len++)
                {
                    vector<c> mod_strs(strs);    
#ifdef _DEBUG
                    //print(mod_strs);
                    //printf("-------------------------------------------------------------------------\n");
#endif
                    int reduction=0;
		   	
		    //c* pc = &mod_strs[mod_strs.size()];
		    if (replace(&mod_strs[i], &mod_strs[i+len], &mod_strs[mod_strs.size()], reduction, patmap))
                    {
                        mod_strs.resize(mod_strs.size()-reduction);
                        parse(mod_strs, ret, depth+1, i);
                    }
		    
                    if (STLhas(terminal, mod_strs))
                    {
                        //                        print(mod_strs);
                        pAtom repl_atom = pAtom(new Atom(terminal[mod_strs]->id));
                        repl_atom->out = mod_strs;
                        *(ret++) = repl_atom;
                        //                        parse_cache[strs].insert(repl_atom);
                    }
                }
        }
    }
};
*/
} //namespace predavese
#endif
