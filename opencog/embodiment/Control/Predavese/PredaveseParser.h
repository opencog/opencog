/*
 * opencog/embodiment/Control/Predavese/PredaveseParser.h
 *
 * Copyright (C) 2007-2008 TO_COMPLETE
 * All Rights Reserved
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
#ifndef PREDAVESE_PARSER_H
#define PREDAVESE_PARSER_H

#include <map>
#include <set>

#include "util/Logger.h"
#include "util/StringManipulator.h"

#include "Predavese.h"
#include "PetInterface.h"
#include "EmbodimentConfig.h"
#include "PredaveseActions.h"

namespace predavese
{

using namespace std;

typedef pair<action*, pAtom> ActionAtomPair;

class PredaveseParser
{
public:
    /// Run this before using the Parser!
    void Create();

    PredaveseParser(Control::PetInterface&);

    virtual ~PredaveseParser( );

    /// Parses a string and returns the set of (pat, action) pairs that hence are due.
    template<typename OutputIterT> void Parse(string s, unsigned long timestamp, OutputIterT actions) const {
        set<pAtom, less_atom_by_structure> res;

        parse(opencog::StringManipulator::clean(s), inserter(res, res.begin()));

        for (set<pAtom, less_atom_by_structure>::const_iterator itr = res.begin(); itr != res.end(); itr++) {
            pAtom a = *itr;
            if (STLhas(pat2action, a)) {
                *(actions++) = ActionAtomPair(pat2action[a], a);
            } // if
        } // for
    }

    void testEng();

    void testPre();

    // TODO: The given avatar is just a hack, in future version predavese
    // should infer this id
    int processInstruction(const string& instruction, unsigned long timestamp, const string& avatarId = "");

    static map<PatIDT, string> id2str;

protected:

    Control::PetInterface& petInterface;
    std::string vocabularyFilename;

    /// patmap is a mapping from A to B so that len(A) >= len(B)
    /// i(nterjection)_patmap is a mapping from A to B so that len(A) and len(B) are independent

    map<pat, pat, less_pat<less_atom> > patmap;
    PatMap i_patmap, elliptic_prefix_patmap;
    map<pat, pAtom, less_pat<less_atom> > terminal;

    map<pAtom, action*, less_atom> pat2action;

    /**
     * Load predavese parser vocabulary from a file. This vocabulary grows
     * dinamically as the system learns new tricks.
     *
     * return True if the file was correctly loaded, false otherwise.
     */
    bool loadVocabulary();

    /**
     * Save predavese parser vocabulary in a file. This method should be called
     * within class destructor.
     *
     * return True if the file was created, false otherwise.
     */
    bool saveVocabulary();

    void basic_init();
    void initEng();
    void initPre();

    pAtom add_terminal(const pat& p);

    template<typename OutIterT> void parse(string s, OutIterT ret) const {
        //logger().log( opencog::Logger::DEBUG, "PredaveseParser - Parsing [%s]", s.c_str( ) );
        pat p = str2pat(s, patmap);
        return parse(p, ret, 0, 0);
    }


    bool isStopWord(const std::string& word);

    std::string petaveseToPredavese(const std::string& petaveseInstruction);

    template<typename OutIterT> void parse(const pat& strs, OutIterT ret, int depth, int original_offset) const {
        // to find expressions that should be converted into I AM AMAZED
        PatMap::const_iterator ip = i_patmap.find(strs);

        //logger().log( opencog::Logger::DEBUG, "PredaveseParser - Starting parse expression. Strings[%d]", strs.size( ) );

        // if is such an expression should be parsed here
        if (ip != i_patmap.end() && original_offset == 0 ) {
            const pat& newStrs = ip->second;
            parse(newStrs, ret, depth + 1, original_offset);
            //logger().log( opencog::Logger::DEBUG, "PredaveseParser - newStrs size[%d] next depth[%d]", newStrs.size( ), depth+1 );
        } else { // no such expression, do further tests
            PatMap::const_iterator ep = elliptic_prefix_patmap.find(strs);

            if (ep != elliptic_prefix_patmap.end() && original_offset == 0) {
                pat newp(ep->second);
                transform(strs.begin(), strs.end(), back_inserter(newp), promote());

                //logger().log( opencog::Logger::DEBUG, "PredaveseParser - newp size[%d] next depth[%d]", newp.size( ), depth+1 );
                parse(newp, ret, depth + 1, original_offset);
            } else {

                /// Parse all variations of strs created by mapping some subvector of strings into a pattern ID
                for (unsigned int i = 0; i < strs.size(); i++) {
                    for (unsigned int len = 0; i + len <= strs.size(); len++) {
                        vector<c> mod_strs(strs);
                        int reduction = 0;

                        if (replace(&mod_strs[i], &mod_strs[i+len], &mod_strs[mod_strs.size()], reduction, patmap)) {
                            mod_strs.resize(mod_strs.size() - reduction);
                            //logger().log( opencog::Logger::DEBUG, "PredaveseParser - mod_strs after reduction[%d] reduction[%d]", mod_strs.size( ), reduction );
                            parse(mod_strs, ret, depth + 1, i);
                        } // if

                        map<pat, pAtom, less_pat<less_atom> >::const_iterator tp = terminal.find(mod_strs);

                        if (tp != terminal.end()) {
                            pAtom repl_atom = pAtom(new Atom(tp->second->id));
                            repl_atom->out = mod_strs;
                            *(ret++) = repl_atom;
                        } // if

                    } // for
                } // for

            } // else

        } // else

    } // parse()

}; // class PredaveseParser

} // namespace

#endif
