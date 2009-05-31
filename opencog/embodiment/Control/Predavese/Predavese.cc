/*
 * opencog/embodiment/Control/Predavese/Predavese.cc
 *
 * Copyright (C) 2002-2009 Novamente LLC
 * All Rights Reserved
 * Author(s): Novamente team
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

/**
 *  Predavese.cpp: Predavese utility functions and support classes.
 */


#include <opencog/util/misc.h>

#include "PredaveseStdafx.h"
#include "Predavese.h"
#include "PredaveseActions.h"
#include "PredaveseParser.h"
#include "PredaveseDefinitions.h"



using namespace std;

/*
    TODO:
        - Fast Rules for mapping "You talk to me" into "talk to me". These are too slow:
            //    ok_tests.push_back("You give Fred the ball");
        - Support (Jill�s). Requires using � as a token, but not as a token-you-remove. This fails:
            //    ok_tests.push_back("Take Jill�s shirt off her.");
        - fail_tests.push_back("am a dog") no  longer fails because "am" is considered a command!
        - -ing is not supported. fnorbling = fnorble.
        - "I'm fnorbling" is not supported. "I'm fnorbling" = "I fnorble". "I'm done sitting"
        = "done sitting"
        - No support for Learning Meta-Commands
            Try to sit
            Like this
            [then push the pet�s butt down]
        - Try fnorble
        - Try to fnorble
        - Support unknown words
        - Choose the most specific interpretation
        - Reduce the search space? Choose the 1st interpretation 1st, and then when
        more resources are given, find more and more interpretations.
*/

namespace predavese
{
pat make_pat1(c _c);
pat make_pat2(c c1, c c2);
pat make_pat3(c c1, c c2, c c3);
pat make_pat4(c c1, c c2, c c3, c c4);
pat make_pat5(c c1, c c2, c c3, c c4, c c5);
pat make_pat6(c c1, c c2, c c3, c c4, c c5, c c6);
pat add_pat(c c1, c c2, PatIDT id, PatMap& patmap);
pat add_pat(c c1, c c2, c c3, PatIDT id, PatMap& patmap);
pat add_pat(c c1, c c2, c c3, c c4, PatIDT id, PatMap& patmap);
//    pAtom add_terminal(const pat& p, PatMap& patmap);
bool add_pat(const pat& p, PatIDT id, PatMap& patmap);
bool add_pat(string s, PatIDT id, PatMap& patmap);

/* string make_upper(string s)
 {
  string ret;
  foreach(char _c, s)
   ret.push_back((_c>'Z' && isalpha(_c))
    ? (_c-('z'-'Z') )
    : _c);

  return ret;
 }
*/

const bool AddUnknownWordsAsElems = true;

vector<c> make_pat1(c _c)
{
    vector<c> ret;
    ret.push_back(_c);
    return ret;
}

string Atom::toString() const
{
    string ret("[" + PredaveseParser::id2str[id] + (out.empty() ? "]" : ("] ( ")));

    BOOST_FOREACH(c _c, out) {
        string* s = get<string>(&_c);
        pAtom* a = get<pAtom>(&_c);
        if (s)
            ret += *s + " ";
        else
            ret += (*a)->toString() + " ";
    }
    return ret + (out.empty() ? "" : " )");
}

string Atom::toPrettyString() const
{
    string ret;
    bool istop = true;

    if (out.empty()) {
        ret += (!PredaveseParser::id2str[id].empty() ? (":" + PredaveseParser::id2str[id] + " ") : string(":[untyped] "));
        return ret;
    }

    BOOST_FOREACH(c _c, out)
    if (istop) {
        istop = false;

        string* s = get<string>(&_c);
        pAtom* a = get<pAtom>(&_c);
        if (s)
            ret += *s;
        else
            ret += (*a)->toPrettyString() + (!PredaveseParser::id2str[(*a)->id].empty() ? (":" + PredaveseParser::id2str[(*a)->id] + " ") : string(":[untyped] "));
    } else {
        string* s = get<string>(&_c);
        pAtom* a = get<pAtom>(&_c);
        if (s)
            ret += "\n\t" + *s ;
        else
            ret += "\n\t" + (*a)->toPrettyString() + (!PredaveseParser::id2str[(*a)->id].empty() ? (":" + PredaveseParser::id2str[(*a)->id] + " ") : string(":[untyped] "));
    }

    if (PredaveseParser::id2str[id].empty()) {
        puts("");
    }

    return ret;
}

string Atom::toIndentString(int indentLevel) const
{
    string ret;
    for (int i = 0; i < indentLevel; i++) ret += " ";
    ret += "[" + (PredaveseParser::id2str[id].empty() ? "untyped" : PredaveseParser::id2str[id])  + "]";

    BOOST_FOREACH(c _c, out) {
        string* s = get<string>(&_c);
        pAtom* a = get<pAtom>(&_c);
        if (s)
            ret += " (" + *s + ")";
        else
            ret += "\n" + (*a)->toIndentString(indentLevel + 1);
    }
    if (out.empty()) ret += "\n";
    return ret;
}

void Atom::print() const
{
    printf("%s\n", toString().c_str());
}

void Atom::prettyprint() const
{
    printf("%s\n", toPrettyString().c_str());
}

void Atom::indentprint() const
{
    printf("%s\n", toIndentString(0).c_str());
}

#if 0

/// This inheritance system based on primes can be taken into use later, to maximize speed.

inline bool inherits(PatIDT sub, PatIDT super)
{
    return ! (sub % super);
}
/*
template<typename IterT>
void types_of(const vector<c>& s, IterT outIt)
{
    if (STLhas(patmap, s))
    {
        *(outIt++) = patmap[s];
        types_of(patmap[s], OutIt);
    }
}
*/

#endif

void print(pat p)
{
    foreach(c _c, p) {
        string* s = get<string>(&_c);
        pAtom* a = get<pAtom>(&_c);
        if (s)
            printf("String: %s ", s->c_str());
        else
            printf("Atom: %s ", ((*a)->toString() + " ").c_str());
    }
    puts("");
}
bool add_pat(string s, PatIDT id, PatMap& patmap)
{
    pat tpat;
    tpat.push_back(s);

    if (STLhas(patmap, tpat))
        return false;

    patmap[make_pat1(s)] = __p(id);
    return true;
}


pat str2pat(string s, const PatMap& patmap)
{
    pat strs;
    //opencog::tokenize(s, std::back_inserter(strs), " "); // English only
    opencog::tokenize(s, std::back_inserter(strs), " (),"); //English + Predavese
//#ifdef _DEBUG
// print(strs);
//#endif
    return strs;
}

string clean(string s)
{
    return s;
}

bool add_pat(const pat& p, PatIDT id, PatMap& patmap)
{
    if (STLhas(patmap, p)) {
//        puts("Warning! Double-declaration of a pattern has occurred!");
        return false;
    }

    patmap[p] = __p(id);

    return true;
}

pat make_pat2(c c1, c c2)
{
    pat tpat;
    tpat.push_back(c1);
    tpat.push_back(c2);

    return tpat;
}

pat make_pat3(c c1, c c2, c c3)
{
    pat tpat;
    tpat.push_back(c1);
    tpat.push_back(c2);
    tpat.push_back(c3);

    return tpat;
}


pat make_pat4(c c1, c c2, c c3, c c4)
{
    pat tpat;
    tpat.push_back(c1);
    tpat.push_back(c2);
    tpat.push_back(c3);
    tpat.push_back(c4);

    return tpat;
}

pat make_pat5(c c1, c c2, c c3, c c4, c c5)
{
    pat tpat;
    tpat.push_back(c1);
    tpat.push_back(c2);
    tpat.push_back(c3);
    tpat.push_back(c4);
    tpat.push_back(c5);

    return tpat;
}

pat make_pat6(c c1, c c2, c c3, c c4, c c5, c c6)
{
    pat tpat;
    tpat.push_back(c1);
    tpat.push_back(c2);
    tpat.push_back(c3);
    tpat.push_back(c4);
    tpat.push_back(c5);
    tpat.push_back(c6);

    return tpat;
}

pat add_pat(c c1, c c2, PatIDT id, PatMap& patmap)
{
    pat tpat;
    tpat.push_back(c1);
    tpat.push_back(c2);

    if (add_pat(tpat, id, patmap))
        return tpat;
    else
        return pat();
}

pat add_pat(c c1, c c2, c c3, PatIDT id, PatMap& patmap)
{
    pat tpat;
    tpat.push_back(c1);
    tpat.push_back(c2);
    tpat.push_back(c3);

    if (add_pat(tpat, id, patmap))
        return tpat;
    else
        return pat();
}

pat add_pat(c c1, c c2, c c3, c c4, PatIDT id, PatMap& patmap)
{
    pat tpat;
    tpat.push_back(c1);
    tpat.push_back(c2);
    tpat.push_back(c3);
    tpat.push_back(c4);

    if (add_pat(tpat, id, patmap))
        return tpat;
    else
        return pat();
}

}
