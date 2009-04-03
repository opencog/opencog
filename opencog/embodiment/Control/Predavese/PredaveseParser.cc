/*
 * opencog/embodiment/Control/Predavese/PredaveseParser.cc
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
#include <fstream>
#include "util/files.h"
#include "PredaveseStdafx.h"
#include "Predavese.h"
#include "PredaveseParser.h"
#include "PredaveseDefinitions.h"
#include "util/StringTokenizer.h"
#include "util/PorterStemmer.h"

#include <boost/regex.hpp>
#include <boost/algorithm/string/case_conv.hpp>

#define ACCEPT_GENERIC_PET_COMMANDS 1
#define ACCEPT_DEVELOPER_META_COMMANDS 1

namespace predavese
{

map<PatIDT, string> PredaveseParser::id2str;

vector<c> make_pat1(c _c);
pat make_pat2(c c1, c c2);
pat make_pat3(c c1, c c2, c c3);
pat make_pat4(c c1, c c2, c c3, c c4);
pat make_pat5(c c1, c c2, c c3, c c4, c c5);
pat make_pat6(c c1, c c2, c c3, c c4, c c5, c c6);
pat add_pat(c c1, c c2, PatIDT id, PatMap& patmap);
pat add_pat(c c1, c c2, c c3, PatIDT id, PatMap& patmap);
pat add_pat(c c1, c c2, c c3, c c4, PatIDT id, PatMap& patmap);
pAtom add_terminal(const pat& p, PatMap& patmap);
bool add_pat(const pat& p, PatIDT id, PatMap& patmap);
bool add_pat(string s, PatIDT id, PatMap& patmap);


void annotate(pat& replacement_pat_with_annotations, const pat& oldrange)
{
    pAtom* first_repl_atom = get<pAtom>(&replacement_pat_with_annotations[0]);
    if (first_repl_atom) { // If the first entry on the repl pat is an Atom, annotate it with the old string
        /*     if (replacement_pat_with_annotations.size() != 1)
               {
               printf("size %d\n", (replacement_pat_with_annotations.size()));
               print(replacement_pat_with_annotations);

               }*/

        //        cassert(TRACE_INFO, replacement_pat_with_annotations.size()==1);
        pAtom new_atom(new Atom((*first_repl_atom)->id, oldrange));
        replacement_pat_with_annotations[0] = new_atom;
        //        (*first_repl_atom)->out = oldrange;
    }
}


int replace(c* start, c* end, c* buf_end, int &reduction, const map<pat, pat, less_pat<less_atom> >& patmap)
{
    // create a copy from the strs based on start/end iterators
    pat oldrange(start, end);

    /*
    // debug
    int t = 0;
    if (t == 1) {
      //printf("Old range: ");
      //print(oldrange);

      foreach(pp _pp, patmap) {
    printf("PP first: ");
    print(_pp.first);
    printf("PP second: ");
    print(_pp.second);
    //if (_pp.first == oldrange)
    //{
    //    printf("Equals: ");
    //    print(_pp.first);
    //puts("eq!");
    // }
      }
    }
    */

    //printf("Old range: ");
    //print(oldrange);
    map<pat, pat, less_pat<less_atom> >::const_iterator o = patmap.find(oldrange);

    if (o != patmap.end() ) {
        pat replacement_pat_with_annotations(o->second.begin(), o->second.end());
        //printf("Replacement pat with annotations: ");
        //print(replacement_pat_with_annotations);

        annotate(replacement_pat_with_annotations, oldrange);

        copy(replacement_pat_with_annotations.begin(),
             replacement_pat_with_annotations.end(), start);


        int newsize = (int)o->second.size();
        reduction = (int)oldrange.size() - newsize;

        if (reduction) {
            copy( start + oldrange.size(), buf_end, start + newsize);
        }
        return true;

    } else {

        if (oldrange.size() == 1) {

            std::string *s = boost::get<std::string>(&oldrange[0]);
            if (s) {
                //printf("String found: %s\n", s->c_str());
                //std::string dudu("dudu");
                if (*s != "A") {
                    add_pat((std::string)*s, (predavese::PatIDT)Verb, (predavese::PatMap &)patmap);
                }
            }
            //printf("OldRange size: %d, Possivel verbo: ", oldrange.size());
            //print(oldrange);
        }

    }
    //printf("Returning false\n");
    return false;
}

//map<pat, set<pAtom>, less_pat<less_atom> > parse_cache;

int next_free_terminal_id = NEXT_FREE_TERMINAL_ID;

PredaveseParser::PredaveseParser(Control::PetInterface& _petInterface,
                                 Control::SystemParameters& _systemParameters) :
        petInterface(_petInterface)
{
    vocabularyFilename = _systemParameters.get(std::string("VOCABULARY_FILE"));
}

PredaveseParser::~PredaveseParser()
{
    saveVocabulary();

    map<pAtom, action*, less_atom>::iterator it;
    for (it = pat2action.begin(); it != pat2action.end(); it++) {
        delete (*it).second;
    }
}

pAtom PredaveseParser::add_terminal(const pat& p)
{
    terminal[p] = pAtom(new Atom(next_free_terminal_id, p));
    patmap[__p(next_free_terminal_id++)] = __p(Elem);
    return terminal[p];
}

//map<pAtom, action*, less_atom> pat2action;

/**

/// RelationShipID which begin with an _
_obj(verb,N1)        indicating that N is the object of V,
"verb N1"
"verb N1 N2"
_subj(V, N)        indicating that N is the subject of V,
"Term verb"
_subj-a(ADJ, N)    indicating that ADJ modifies N
"Term Term"
_obj2(verb,N2)    indicating that N is the secondary object of V (used for ditransitive verbs).
"verb N1 N2"
_poss(O,Name)    indicating the possessive form of N
"Name's O"
"O of Name"

_subj-r(ADV, X)    indicating that ADV modifies X,
???

_to(V,O)        "V to O"
_to-do(V1, V2)
indicating an instruction to do V to N.
"V1 V2[-1]-ing"
*/


void PredaveseParser::initPre()
{
    //        basic_init();
    initEng();

    pat2action[add_terminal(
                   make_pat3( opencog::StringManipulator::toUpper(id2str[(Relation_obj        )]),
                              pAtom(new Atom(Verb)),
                              pAtom(new Atom(Name)))
               )] = new Relation_obj_action(petInterface);
    pat2action[add_terminal(
                   make_pat3( opencog::StringManipulator::toUpper(id2str[(Relation_subj        )]),
                              pAtom(new Atom(Verb)),
                              pAtom(new Atom(Name)))
               )] = new Relation_subj_action(petInterface);
    pat2action[add_terminal(
                   make_pat3( opencog::StringManipulator::toUpper(id2str[(Relation_obj2        )]),
                              pAtom(new Atom(Verb)),
                              pAtom(new Atom(Elem)))
               )] = new Relation_obj2_action(petInterface);
    pat2action[add_terminal(
                   make_pat2( opencog::StringManipulator::toUpper(id2str[(Relation_poss        )]),
                              pAtom(new Atom(Name)))
               )] = new Relation_poss_action(petInterface);
    pat2action[add_terminal(
                   make_pat3( opencog::StringManipulator::toUpper(id2str[(Relation_subj_r        )]),
                              pAtom(new Atom(Elem)),
                              pAtom(new Atom(Elem)))
               )] = new Relation_subj_r_action(petInterface);

    pat2action[add_terminal(
                   make_pat3( opencog::StringManipulator::toUpper(id2str[(Relation_to_do        )]),
                              pAtom(new Atom(Verb)),
                              pAtom(new Atom(Verb)))
               )] = new Relation_to_do_action(petInterface);

    add_pat(id2str[(Relation_obj           )], Elem, patmap);
    add_pat(id2str[(Relation_subj          )], Elem, patmap);
    add_pat(id2str[(Relation_obj2   )], Elem, patmap);
    add_pat(id2str[(Relation_poss          )], Elem, patmap);
    add_pat(id2str[(Relation_subj_r        )], Elem, patmap);
    add_pat(id2str[(Relation_to            )], Elem, patmap);
    add_pat(id2str[(Relation_to_do         )], Elem, patmap);
}

bool PredaveseParser::saveVocabulary()
{
    std::ofstream out(vocabularyFilename.c_str(), ios_base::trunc);

//        printf("patmap.size %d", patmap.size());
    foreach(pp _pp, patmap) {

        // only pat with size one are considered (no recursion)
        if (_pp.first.size()  == 1 &&
                _pp.second.size() == 1) {

            pat first = _pp.first;
            pat second = _pp.second;

            string *word = get<string>(&first[0]);
            pAtom *type = get<pAtom>(&second[0]);

            if (word && type) {
//                    printf("word: %s, type: %d\n", (*word).c_str(), (*type)->id);
                out << (*word) << "\t" << (*type)->id << "\n";
            }
        }
    }
    out.close();
    return true;
}

bool PredaveseParser::loadVocabulary()
{
    if (!fileExists(vocabularyFilename.c_str())) {
        logger().log(opencog::Logger::INFO,
                     "PredaveseParser - file %s does not exist. It will be created at shutdown.",
                     vocabularyFilename.c_str());
        return false;
    }

    char line[256];
    std::ifstream fin(vocabularyFilename.c_str());

    std::string name;
    PatIDT type;

    while (!fin.eof()) {
        fin.getline(line, 256);

        // not a comentary or an empty line
        if (line[0] != '#' && line[0] != 0x00) {
            std::istringstream in ((std::string)line);
            in >> name;
            in >> type;

            add_pat(name, type, patmap);
        }
    }

    fin.close();
    return true;
}

void PredaveseParser::initEng()
{
    basic_init();

    pat pat1, pat2, pat3, pat4;

    pat1.push_back(pAtom(new Atom(Elem)));
    pat1.push_back(pAtom(new Atom(Verb)));
    pat1.push_back(pAtom(new Atom(Elem)));
    pat2action[add_terminal(pat1)] = new predicate_action(petInterface, 3);

    pat2action[add_terminal(
                   make_pat2(pAtom(new Atom(Pron)),
                             pAtom(new Atom(Verb)))
               )] = new predicate_action(petInterface, 2);

    pat2action[add_terminal(
                   make_pat4(pAtom(new Atom(Pron)), pAtom(new Atom(Verb)), "THE", pAtom(new Atom(Name))  )
               )] = new predicate_action(petInterface, 4);

    pat2action[add_terminal(
                   make_pat2(pAtom(new Atom(Pron)), pAtom(new Atom(Name))  )
               )] = new predicate_action(petInterface, 2);

    //pat2.push_back("A");
    //pat2.push_back(pAtom(new Atom(Name)));
    //patmap[pat2] = __p(Elem);

    add_pat("A",
            pAtom(pAtom(new Atom(Name))),
            Elem, patmap);
    add_pat("THE",
            pAtom(pAtom(new Atom(Name))),
            Elem, patmap);
    add_pat(pAtom(new Atom(Adj)),
            pAtom(pAtom(new Atom(Name))),
            Elem, patmap);
    add_pat("THE",
            pAtom(new Atom(Adj)),
            pAtom(pAtom(new Atom(Name))),
            Elem, patmap);
    /*        patmap[make_pat5("YOU", pAtom(new Atom(Verb)), pAtom(new Atom(Elem)), pAtom(new Atom(Elem)), pAtom(new Atom(Elem)))] =
              make_pat4(pAtom(new Atom(Verb)), pAtom(new Atom(Elem)), pAtom(new Atom(Elem)), pAtom(new Atom(Elem)));
              patmap[make_pat4("YOU", pAtom(new Atom(Verb)), pAtom(new Atom(Elem)), pAtom(new Atom(Elem)))] =
              make_pat3(pAtom(new Atom(Verb)), pAtom(new Atom(Elem)), pAtom(new Atom(Elem)));
              patmap[make_pat3("YOU", pAtom(new Atom(Verb)), pAtom(new Atom(Elem)))] =
              make_pat2(pAtom(new Atom(Verb)), pAtom(new Atom(Elem)));
              patmap[make_pat2("YOU", pAtom(new Atom(Verb)))] =
              make_pat1(pAtom(new Atom(Verb)));
              */
    i_patmap[make_pat1("WOW")] = str2pat("I AM AMAZED", patmap);
    i_patmap[make_pat1("HOT DAMN")] = str2pat("I AM AMAZED", patmap);
    i_patmap[make_pat1("NICE")] = str2pat("I AM AMAZED", patmap);
    //i_patmap[make_pat1("GREAT")] = str2pat("I AM AMAZED",patmap);

    //pat4.push_back(pAtom(new Atom(Poss)));
    //pat4.push_back(pAtom(new Atom(Name)));
    //patmap[pat4] = __p(Elem);

    add_pat(pAtom(new Atom(Poss)), pAtom(new Atom(Name)), Elem, patmap);

    pat2action[add_terminal(
                   str2pat("I AM AMAZED",patmap)
               )] = new reward_action(petInterface);
    pat2action[add_terminal(
                   str2pat("GOOD",patmap)
               )] = new reward_action(petInterface);
    pat2action[add_terminal(
                   str2pat("GREAT",patmap)
               )] = new reward_action(petInterface);

    pat2action[add_terminal(
                   str2pat("BAD",patmap)
               )] = new punish_action(petInterface);
    pat2action[add_terminal(
                   str2pat("TERRIBLE",patmap)
               )] = new punish_action(petInterface);

    pat2action[add_terminal(
                   make_pat2("I", pAtom(new Atom(Verb)))
               )] = new exemplar_start_command(petInterface, 2);
    pat2action[add_terminal(
                   make_pat3("I", pAtom(new Atom(Verb)), pAtom(new Atom(Elem)))
               )] = new exemplar_start_command(petInterface, 2);

    pat2action[add_terminal(
                   make_pat3("I", pAtom(new Atom(Adj)), pAtom(new Atom(Elem)))
               )] = new exemplar_start_command(petInterface, 2);

    pat2action[add_terminal(
                   make_pat4("I", pAtom(new Atom(Verb)), pAtom(new Atom(Elem)), pAtom(new Atom(Elem)))
               )] = new exemplar_start_command(petInterface, 2);



    pat2action[add_terminal(
                   make_pat4("I", "WILL", "SHOW", "YOU")
               )] = new exemplar_start_command(petInterface, 5);

    pat2action[add_terminal(
                   make_pat2(pAtom(new Atom(ProperName)), pAtom(new Atom(Verb)))
               )] = new exemplar_start_command(petInterface, 2);
    pat2action[add_terminal(
                   make_pat3(pAtom(new Atom(ProperName)), pAtom(new Atom(Verb)), pAtom(new Atom(Elem)))
               )] = new exemplar_start_command(petInterface, 2);
    pat2action[add_terminal(
                   make_pat4(pAtom(new Atom(ProperName)), pAtom(new Atom(Verb)), pAtom(new Atom(Elem)), pAtom(new Atom(Elem)))
               )] = new exemplar_start_command(petInterface, 2);

    pat2action[add_terminal(
                   make_pat4(pAtom(new Atom(ProperName)), "WILL", "SHOW", "YOU")
               )] = new exemplar_start_command(petInterface, 5);

    pat2action[add_terminal(
                   make_pat2("DONE", pAtom(new Atom(Verb)))
               )] = new exemplar_end_command(petInterface, 2);
    pat2action[add_terminal(
                   make_pat3("DONE", pAtom(new Atom(Verb)), pAtom(new Atom(Elem)))
               )] = new exemplar_end_command(petInterface, 2);
    pat2action[add_terminal(
                   make_pat4("DONE", pAtom(new Atom(Verb)), pAtom(new Atom(Elem)), pAtom(new Atom(Elem)))
               )] = new exemplar_end_command(petInterface, 2);

    pat2action[add_terminal(
                   make_pat3("DONE", "SHOW", "YOU")
               )] = new exemplar_end_command(petInterface, 4);

    pat2action[add_terminal(
                   make_pat5("YOU", "ARE", "A", "GOOD", pAtom(new Atom(PetName)))
               )] = new reward_action(petInterface);

    pat2action[add_terminal(
                   make_pat5("YOU", "ARE", "A", "BAD", pAtom(new Atom(PetName)))
               )] = new punish_action(petInterface);

#if ACCEPT_GENERIC_PET_COMMANDS
    pat2action[add_terminal(
                   make_pat1(pAtom(new Atom(Verb)))
               )] = new pet_command(petInterface, 1);
    pat2action[add_terminal(
                   make_pat2(pAtom(new Atom(Verb)), pAtom(new Atom(Elem)))
               )] = new pet_command(petInterface, 2);
    pat2action[add_terminal(
                   make_pat3(pAtom(new Atom(Verb)), pAtom(new Atom(Elem)), pAtom(new Atom(Elem)))
               )] = new pet_command(petInterface, 3);
    pat2action[add_terminal(
                   make_pat4(pAtom(new Atom(Verb)), pAtom(new Atom(Elem)), pAtom(new Atom(Elem)), pAtom(new Atom(Elem)))
               )] = new pet_command(petInterface, 4);
    pat2action[add_terminal(
                   make_pat5(pAtom(new Atom(Verb)), pAtom(new Atom(Elem)), pAtom(new Atom(Elem)), pAtom(new Atom(Elem)), pAtom(new Atom(Elem)))
               )] = new pet_command(petInterface, 5);
    pat2action[add_terminal(
                   make_pat6(pAtom(new Atom(Verb)), pAtom(new Atom(Elem)), pAtom(new Atom(Elem)), pAtom(new Atom(Elem)), pAtom(new Atom(Elem)), pAtom(new Atom(Elem)) )
               )] = new pet_command(petInterface, 6);
#endif

#if ACCEPT_DEVELOPER_META_COMMANDS
    pat2action[add_terminal(
                   make_pat1(pAtom(new Atom(MetaCommand)))
               )] = new pet_dev_meta_command(petInterface, 1);
#endif

    pat2action[add_terminal(
                   make_pat3("GIVE", pAtom(new Atom(Elem)), pAtom(new Atom(Elem)))
               )] = new pet_command(petInterface, 2);

    pat2action[add_terminal(
                   make_pat2("FETCH", pAtom(new Atom(Name)))
               )] = new pet_command(petInterface, 2);

    /*    pat2action[add_terminal(
          make_pat2("DO", pAtom(new Atom(Verb)))
          )] = new pet_command(petInterface, 2);
    pat2action[add_terminal(
            make_pat1(pAtom(new Atom(Verb)))
            )] = new pet_command(petInterface, 1);
          */
    pat2action[add_terminal(
                   make_pat1("TRY")
               )] = new try_schema_command(petInterface, 2);

    pat2action[add_terminal(
                   make_pat2("TRY", pAtom(new Atom(Elem)))
               )] = new try_schema_command(petInterface, 2);
    pat2action[add_terminal(
                   make_pat3("TRY", pAtom(new Atom(Elem)), pAtom(new Atom(Elem)))
               )] = new try_schema_command(petInterface, 2);
    pat2action[add_terminal(
                   make_pat4("TRY", pAtom(new Atom(Elem)), pAtom(new Atom(Elem)), pAtom(new Atom(Elem)))
               )] = new try_schema_command(petInterface, 2);
    /*
            pat2action[add_terminal(
                    make_pat3("LEARN", "TO", pAtom(new Atom(Elem)))
                    )] = new pet_learn_command(petInterface, 3);
    */
    pat2action[add_terminal(
                   make_pat2("LEARN", pAtom(new Atom(Elem)) )
               )] = new pet_learn_command(petInterface, 2);
    pat2action[add_terminal(
                   make_pat3("LEARN", pAtom(new Atom(Elem)), pAtom(new Atom(Elem)))
               )] = new pet_learn_command(petInterface, 2);
    pat2action[add_terminal(
                   make_pat4("LEARN", pAtom(new Atom(Elem)), pAtom(new Atom(Elem)), pAtom(new Atom(Elem)))
               )] = new pet_learn_command(petInterface, 2);
    /*
            pat2action[add_terminal(
                    make_pat4("LEARN", pAtom(new Atom(Elem)), "WITH", pAtom(new Atom(ProperName)) )
                    )] = new pet_learn_command(petInterface, 2);
    */
    pat2action[add_terminal(
                   make_pat5("LEARN", pAtom(new Atom(Elem)), pAtom(new Atom(Elem)), "WITH", pAtom(new Atom(ProperName)) )
               )] = new pet_learn_command(petInterface, 2);
    pat2action[add_terminal(
                   make_pat6("LEARN", pAtom(new Atom(Elem)), pAtom(new Atom(Elem)), pAtom(new Atom(Elem)), "WITH", pAtom(new Atom(ProperName)) )
               )] = new pet_learn_command(petInterface, 2);

    /*
            pat2action[add_terminal(
                    make_pat3("STOP", "LEARNING", pAtom(new Atom(Elem)))
                    )] = new pet_stop_learn_command(petInterface, 3);
    */
    pat2action[add_terminal(
                   make_pat3("STOP", "LEARN", pAtom(new Atom(Elem)))
               )] = new pet_stop_learn_command(petInterface, 3);
    pat2action[add_terminal(
                   make_pat4("STOP", "LEARN", pAtom(new Atom(Elem)), pAtom(new Atom(Elem)))
               )] = new pet_stop_learn_command(petInterface, 3);
    pat2action[add_terminal(
                   make_pat5("STOP", "LEARN", pAtom(new Atom(Elem)), pAtom(new Atom(Elem)), pAtom(new Atom(Elem)))
               )] = new pet_stop_learn_command(petInterface, 3);
    /*
            pat2action[add_terminal(
                    make_pat4("STOP", "LEARNING", "TO", pAtom(new Atom(Elem)))
                    )] = new pet_stop_learn_command(petInterface, 4);
    */
    pat2action[add_terminal(
                   make_pat2("STOP", pAtom(new Atom(Verb)))
               )] = new pet_stop_command(petInterface, 2);
    pat2action[add_terminal(
                   make_pat3("STOP", pAtom(new Atom(Verb)), pAtom(new Atom(Elem)))
               )] = new pet_stop_command(petInterface, 2);
    pat2action[add_terminal(
                   make_pat4("STOP", pAtom(new Atom(Verb)), pAtom(new Atom(Elem)), pAtom(new Atom(Elem)))
               )] = new pet_stop_command(petInterface, 2);

    pat2action[add_terminal(
                   make_pat3("LIKE", "THIS", pAtom(new Atom(Verb)))
               )] = new pet_meta_command(petInterface, 3);
    pat2action[add_terminal(
                   make_pat4("LIKE", "THIS", pAtom(new Atom(Verb)), pAtom(new Atom(Elem)))
               )] = new pet_meta_command(petInterface, 3);
    pat2action[add_terminal(
                   make_pat5("LIKE", "THIS", pAtom(new Atom(Verb)), pAtom(new Atom(Elem)), pAtom(new Atom(Elem)))
               )] = new pet_meta_command(petInterface, 3);

    pat2action[add_terminal(
                   make_pat2("WHERE", pAtom(new Atom(Elem)))
               )] = new pet_interrogative_command(petInterface, 2);

    pat2action[add_terminal(
                   make_pat2("THAT", pAtom(new Atom(Elem)))
               )] = new pet_declarative_command(petInterface, 2);
    //    i_patmap[make_pat2(new Atom(Adj), new Atom(PetName))] =
    //        make_pat5("YOU", "ARE", "A", new Atom(Adj), new Atom(PetName));

    elliptic_prefix_patmap[make_pat2(pAtom(new Atom(Adj)), pAtom(new Atom(PetName)))] =
        make_pat3("YOU", "ARE", "A");

    add_pat("WOW", Elem, patmap);

    add_pat("AMAZED", Adj, patmap);

    add_pat("YOU", Pron, patmap);
    add_pat("HER", Pron, patmap);
    add_pat("I", Pron, patmap);
    add_pat("ME", Pron, patmap);
    add_pat("THIS", Pron, patmap);
    add_pat("HER", Poss, patmap);
    add_pat("THAT", Pron, patmap);
    add_pat("�S", Poss, patmap);
    add_pat("YOUR", Poss, patmap);

    add_pat("GOOD", Adj, patmap);
    add_pat("BAD", Adj, patmap);
    add_pat("RED", Adj, patmap);
    add_pat("REALLY", Adj, patmap);
    add_pat("BIG", Adj, patmap);
    add_pat("GREAT", Adj, patmap);
    add_pat("TERRIBLE", Adj, patmap);

    add_pat("RED", Adj, patmap);
    add_pat("BLUE", Adj, patmap);

    add_pat("DOG", PetName, patmap);
    add_pat("PET", PetName, patmap);
    add_pat("BOY", PetName, patmap);
    add_pat("SHIRT", Name, patmap);
    add_pat("BALL", Name, patmap);
    add_pat("LOG", Name, patmap);
    add_pat("STICK", Name, patmap);
    add_pat("MAN", Name, patmap);
    add_pat("SCAVENGER", Name, patmap);
    add_pat("HUNT", Name, patmap);
    add_pat("SH", Name, patmap); // Scavenger Hunt
    add_pat("TREASURE", Name, patmap);
    add_pat("TREASURES", Name, patmap);
    add_pat("TEAM", Name, patmap);
    add_pat("AREA", Name, patmap);

    add_pat("ARE", Verb, patmap);
    add_pat("AM", Verb, patmap);
    add_pat("IS", Verb, patmap);

    add_pat("ENJOY", Verb, patmap);
    add_pat("GETJIGGYWITHIT", Verb, patmap);
    add_pat("DO", Verb, patmap);
    add_pat("KICK", Verb, patmap);
    add_pat("FETCH", Verb, patmap);
    add_pat("GIVE", Verb, patmap);
    add_pat("TAKE", Verb, patmap);
    add_pat("GIVE", Verb, patmap);
    add_pat("LEARN", KeyVerb, patmap);
    add_pat("LEARNING", KeyVerb, patmap);
    add_pat("SHOW", KeyVerb, patmap);
    add_pat("WHERE", KeyVerb, patmap);
    add_pat("STOP", KeyVerb, patmap);
    add_pat("WILL", KeyVerb, patmap);
    add_pat("�M", Verb, patmap);

    add_pat("LETS", Verb, patmap);
    add_pat("PLAY", Verb, patmap);
    add_pat("PLAYING", Verb, patmap);
    add_pat("FIND", Verb, patmap);
    add_pat("GO", Verb, patmap);
    add_pat("TRY", KeyVerb, patmap);
    add_pat("FOLLOW", Verb, patmap);
    add_pat("COLLECT", Verb, patmap);

    add_pat("COME", Verb, patmap);
    add_pat("WAIT", Verb, patmap);
    add_pat("REGROUP", Verb, patmap);
    add_pat("EXPLORE", Verb, patmap);
    add_pat("SPY", Verb, patmap);

    add_pat("HERE", Elem, patmap);

    add_pat("SAVE_MAP", MetaCommand, patmap);
    add_pat("SAVE_VISMAP", MetaCommand, patmap);
    // buitlin actions supported according to BasicSchemaFeeder
    // Carlos Lopes - 02/10/07
    add_pat("GRAB", Verb, patmap);
    add_pat("JUMP", Verb, patmap);
    add_pat("DROP", Verb, patmap);
    add_pat("SNIFF", Verb, patmap);
    add_pat("BARK", Verb, patmap);
    add_pat("DROP", Verb, patmap);
    add_pat("LOOK_UP_TURN_HEAD", Verb, patmap);
    add_pat("BARETEETH", Verb, patmap);
    add_pat("WAGTAIL", Verb, patmap);
    add_pat("STRETCH", Verb, patmap);
    add_pat("SIT", Verb, patmap);
    add_pat("BEG", Verb, patmap);
    add_pat("HEEL", Verb, patmap);

    add_pat("WOW", Elem, patmap);
    add_pat("FNORBLE", Verb, patmap);
    //    add_pat("FNORBLE", Elem, patmap);
    add_pat("FRED", ProperName, patmap);
    add_pat("BOB", ProperName, patmap);
    add_pat("JILL", ProperName, patmap);
    add_pat("BEN", ProperName, patmap);
    add_pat("WYNX", ProperName, patmap);
    add_pat("SALLY", ProperName, patmap);
    add_pat("JOHN", ProperName, patmap);
    add_pat("BIL", ProperName, patmap);
    add_pat("MICK", ProperName, patmap);

    add_pat("OFF", Elem, patmap);
    add_pat("TO", Elem, patmap);
    add_pat("LIKE", KeyVerb, patmap);
    add_pat("WITH", Elem, patmap);

    add_pat("GOING", Elem, patmap);
    add_pat("NOW", Elem, patmap);

    add_pat("DONE", Elem, patmap);
}

void PredaveseParser::basic_init()
{
    patmap[__p(Relation)] = __p(Elem);
    patmap[__p(Term    )] = __p(Elem);

    patmap[__p(Relation_obj        )] = __p(Relation);
    patmap[__p(Relation_subj    )] = __p(Relation);
    patmap[__p(Relation_obj2    )] = __p(Relation);
    patmap[__p(Relation_poss    )] = __p(Relation);
    patmap[__p(Relation_subj_r    )] = __p(Relation);
    patmap[__p(Relation_to        )] = __p(Relation);
    patmap[__p(Relation_to_do    )] = __p(Relation);

    patmap[__p(    BasicTerm          )]    = __p(Term);
    patmap[__p( QualifiedBasicTerm)] = __p(Term);

    patmap[__p( KeyVerb          )]    = __p(Elem);
    patmap[__p( MetaCommand      )]    = __p(Elem);

    //patmap[__p(Pron    )] = __p(BasicTerm);
    //patmap[__p(Poss    )] = __p(BasicTerm);
    //patmap[__p(    Verb               )]    = __p(BasicTerm);
    //patmap[__p(    GeneralWord          )]    = __p(BasicTerm);
    //patmap[__p(    Name               )]    = __p(BasicTerm);
    //patmap[__p(    Adj                )]    = __p(BasicTerm);

    patmap[__p(    ProperName               )]    = __p(Name);
    patmap[__p(    TrickName               )]    = __p(Name);

    patmap[__p(    PetName               )]    = __p(Name);

    // Simplification:

    patmap[__p(Pron    )] = __p(Elem);
    patmap[__p(Poss    )] = __p(Elem);
    patmap[__p(    Verb               )]    = __p(Elem);
    patmap[__p(    GeneralWord          )]    = __p(Elem);
    patmap[__p(    Name               )]    = __p(Elem);
    patmap[__p(    Adj                )]    = __p(Elem);

    id2str[Elem]        = "Elem";
    id2str[(Relation)]    = "Relation";
    id2str[(Term    )]    = "Term";

    id2str[(Relation_obj        )] = "_obj";
    id2str[(Relation_subj    )]  = "_subj";
    id2str[(Relation_obj2    )]  = "_obj2";
    id2str[(Relation_poss    )]  = "_poss";
    id2str[(Relation_subj_r    )] = "_subj_r";
    id2str[(Relation_to_do    )] = "_to_do";
    id2str[(Relation_to        )] = "_to";

    id2str[(    BasicTerm          )]    = "BasicTerm";
    id2str[( QualifiedBasicTerm)] = "QualifiedBasicTerm";
    id2str[(    KeyVerb          )]    = "KeyVerb";
    id2str[(    MetaCommand      )]    = "MetaCommand";

    id2str[(    Verb               )]    = "Verb";
    id2str[(    GeneralWord          )]    = "GeneralWord";
    id2str[(    Name               )]    = "Name";
    id2str[(    Adj                )]    = "Adj";

    id2str[(    ProperName               )]    = "ProperName";
    id2str[(    TrickName               )]    = "TrickName";
    id2str[(    PetName               )]    = "PetName";
}

bool PredaveseParser::isStopWord(const std::string& word)
{
    set<std::string> stopWords;
    stopWords.insert("A");
    stopWords.insert("AN");
    stopWords.insert("DO");
    stopWords.insert("FROM");
    stopWords.insert("IS");
    stopWords.insert("PLAY");
    stopWords.insert("THE");
    stopWords.insert("TO");
    //stopWords.insert("WITH");

    return ( stopWords.find(word) != stopWords.end() );
}


std::string PredaveseParser::petaveseToPredavese(const std::string& petaveseInstruction)
{
    logger().log(opencog::Logger::INFO, "PredaveseParser - PetaveseToPredavese.");

    std::string predaveseInstruction("");
    std::string strToken("");

    opencog::StringTokenizer strTokenizer(petaveseInstruction, " ");
    while ( (strToken = strTokenizer.nextToken()) != "") {
        logger().log(opencog::Logger::INFO, "PredaveseParser - PetaveseToPredavese - token: '%s'.", strToken.c_str());
        opencog::StringTokenizer strTokenizerContraction(strToken, "'");
        std::string strTokenContraction("");
        while ( (strTokenContraction = strTokenizerContraction.nextToken()) != "") {
            if ( strTokenContraction == "LL" )
                strTokenContraction = "WILL";
            else if ( strTokenContraction == "S" )
                strTokenContraction = "IS";

            if ( !isStopWord(strTokenContraction) ) {
                if ( strTokenContraction.rfind("ING", strTokenContraction.length() - 1) != string::npos && strTokenContraction.length() > 3 ) {
                    strTokenContraction = opencog::StringManipulator::toUpper(opencog::PorterStemmer::getStem(opencog::StringManipulator::toLower(strTokenContraction)));
                }
                predaveseInstruction = predaveseInstruction + strTokenContraction + " ";
            }
        }
    }
    return predaveseInstruction;
}

int PredaveseParser::processInstruction(const string& instruction, unsigned long timestamp, const string& avatarId)
{

    logger().log(opencog::Logger::INFO, "PredaveseParser - Parse '%s' instruction given by '%s'.",
                 instruction.c_str(), avatarId.c_str());
    patmap[__p( KeyVerb          )]    = __p(Elem);

    string clean_instruction = opencog::StringManipulator::clean(instruction);
    logger().log(opencog::Logger::DEBUG, "PredaveseParser - clean_instruction '%s'.", clean_instruction.c_str());


    // TODO: maybe computing the distance to the speaker can be better to
    // determine whether the agent 'listen' or not the frase
    // TODO: This is a HACK and must be removed when a better solution was planned

    std::string agentName = petInterface.getName( );
    boost::algorithm::to_upper( agentName );

    std::stringstream patternStr;
    patternStr << "(I\\s(RED|BLUE)\\sTEAM)";
    patternStr << "|(GO (FIND|AFTER) TREASURES? " << agentName << ")";
    patternStr << "|(FOLLOW ME " << agentName << ")";
    patternStr << "|(FOLLOW ME COLLECT " << agentName << ")";
    patternStr << "|(COME HERE " << agentName << ")";
    patternStr << "|(EXPLORE AREA \\d " << agentName << ")";
    patternStr << "|(WAIT " << agentName << ")";
    patternStr << "|(REGROUP TEAM (RED|BLUE))";
    patternStr << "|(SPY \\w+ " << agentName << ")";

    boost::regex avatarFrasesPattern( patternStr.str() );

    if ( avatarId != petInterface.getOwnerId( ) && !boost::regex_match( clean_instruction, avatarFrasesPattern ) ) {
        logger().log(opencog::Logger::DEBUG, "PredaveseParser - Invalid instruction from non-owner. It can only say 'I RED TEAM' or 'I BLUE TEAM' but '%s' was said by '%s'",  instruction.c_str(), avatarId.c_str());
        return 0;
    } // if


    string processedPetaveseInstruction = petaveseToPredavese(clean_instruction);
    logger().log(opencog::Logger::DEBUG, "PredaveseParser - processedPetaveseInstruction '%s'.", processedPetaveseInstruction.c_str());

    set<pAtom, less_atom_by_structure> res;
    parse(processedPetaveseInstruction, inserter(res, res.begin()));

    foreach(pAtom a, res) {
        if (STLhas(pat2action, a)) {
            logger().log(opencog::Logger::DEBUG, "PredaveseParser - Parse successfull. Taking actions.");

            //printf("Calling () operator for action\n");
            if (!(*pat2action[a])(a, timestamp, avatarId )) {

                // the action tried wasn't sucessfull so remove it from the
                // list o possible verbs (probabily it hasn't been learned
                // yet)
                c _c = (a->out).back();
                std::string *s = get<std::string>(&_c);
                if (s) {
                    PatMap::iterator patIter = patmap.find(make_pat1(*s));
                    if (patIter != patmap.end()) {
                        patmap.erase(patIter);
                    }
                }
            }
            // TODO: What if there are multiple results. How to select the right one?
        }
    }
    return res.size();
}


void PredaveseParser::Create()
{
    if (!loadVocabulary()) {
        logger().log(opencog::Logger::WARN, "PredaveseParser - Cannot load vocabulary.");
    }
    initEng();
}

} //namespace predavese;
