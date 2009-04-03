#include "util/StringManipulator.h"

#include "PredaveseStdafx.h"
#include "PredaveseParser.h"
#include "ActionPlanSenderMock.h"
#include "PredaveseDefinitions.h"

const static std::string avatar = "avatar";

namespace predavese
{

vector<c> make_pat1(c _c);
pat make_pat2(c c1, c c2);
pat make_pat3(c c1, c c2, c c3);
pat make_pat4(c c1, c c2, c c3, c c4);
pat make_pat5(c c1, c c2, c c3, c c4, c c5);
pat add_pat(c c1, c c2, PatIDT id, PatMap& patmap);
pat add_pat(c c1, c c2, c c3, PatIDT id, PatMap& patmap);
pat add_pat(c c1, c c2, c c3, c c4, PatIDT id, PatMap& patmap);
pAtom add_terminal(const pat& p, PatMap& patmap);
bool add_pat(const pat& p, PatIDT id, PatMap& patmap);
bool add_pat(string s, PatIDT id, PatMap& patmap);

void parse_error()
{
    puts("PARSE ERROR!");
#ifdef USE_ASSERT_OR_EXIT
    exit(0);
#endif
}

/**

verb = am give
adj = good, bad

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

void PredaveseParser::testPre()
{
    vector<string> ok_tests, fail_tests;

    fail_tests.push_back(id2str[(Relation_obj        )] + "(walk, walk)");
    ok_tests.push_back(id2str[(Relation_obj        )] + "(fetch, stick)");
    ok_tests.push_back(id2str[(Relation_subj    )] + "(fetch, stick)");
    ok_tests.push_back(id2str[(Relation_obj2    )] + "(fetch, stick)");
    ok_tests.push_back(id2str[(Relation_poss    )] + "(Ben)");
    ok_tests.push_back(id2str[(Relation_subj_r    )] + "(good, stick)");
    ok_tests.push_back(id2str[(Relation_to_do    )] + "(fetch, enjoy)");
//    ok_tests.push_back("_subj_a(big, stick)");

    set<pAtom, less_atom_by_structure> res;

    foreach(string s, ok_tests) {
//set<pAtom, less_atom_by_structure> res;
        unsigned int original_res_size = (unsigned int)res.size();
//        Parser parser(patmap);
//        string clean_s = clean_string::clean(s.c_str());
        string clean_s = opencog::StringManipulator::clean(s);
        parse(clean_s, inserter(res, res.begin()));//->print();
#ifdef USE_ASSERT_OR_EXIT
        assert(res.size() > original_res_size);
#else
        if (res.size() <= original_res_size) {
            printf("OK TEST FAILED\n");
        }
#endif

        printf(" (Res.size = %d (vs. %d before)) \n", res.size(), original_res_size);

        if (!(res.size() > original_res_size))
            parse_error();
    }

    foreach(string s, fail_tests) {
        unsigned int original_res_size = (unsigned int)res.size();
        //parse(clean_string::clean(s.c_str()), inserter(res, res.begin())); //->print();
        parse(opencog::StringManipulator::clean(s), inserter(res, res.begin())); //->print();
#ifdef USE_ASSERT_OR_EXIT
        assert(res.size() == original_res_size);
#else
        if (res.size() != original_res_size) {
            printf("FAIL TEST FAILED\n");
        }
#endif
        if (res.size() != original_res_size)
            parse_error();
    }

    puts("Results:");

    foreach(pAtom a, res) {
        a->print();
        puts("");
    }

    puts("\nTaking actions...");

    foreach(pAtom a, res) {
        if (STLhas(pat2action, a))
            (*pat2action[a])(a, 0, "Sarah");
    }


}

void PredaveseParser::testEng()
{
    vector<string> ok_tests, fail_tests;

    fail_tests.push_back("a dog");
    fail_tests.push_back("I a dog");
    fail_tests.push_back("good ben");

    //ok_tests.push_back("I am a dog");
    //ok_tests.push_back("I am a Ben");
    ok_tests.push_back("wow");
    //ok_tests.push_back("boy is your dog"); //slow?
    //ok_tests.push_back("Your boy is a dog");

    ok_tests.push_back("good boy");
    ok_tests.push_back("boy is a dog");
    ok_tests.push_back("good dog");
    ok_tests.push_back("bad pet");


    ok_tests.push_back("bad dog");
    ok_tests.push_back("You fnorble");
    ok_tests.push_back("You fnorble bob");

    /*    ok_tests.push_back("Jump");
    ok_tests.push_back("Walk");
    ok_tests.push_back("GetJiggyWithIt");
    ok_tests.push_back("Do wow");
    //ok_tests.push_back("learn");
    ok_tests.push_back("Fetch the ball");
    ok_tests.push_back("Kick the man.");
    ok_tests.push_back("Fnorble the red ball!");
    */
    // Broken tests
    //ok_tests.push_back("Give Fred the ball.");

    //ok_tests.push_back("Give her the ball");

    ok_tests.push_back("learn fnorble");
    ok_tests.push_back("learn to fnorble");
    ok_tests.push_back("stop learning to fnorble");
    ok_tests.push_back("stop fetch");

    ok_tests.push_back("I fetch");
    ok_tests.push_back("done fetch");
    ok_tests.push_back("Sit");

    ok_tests.push_back("learn to dig");
    ok_tests.push_back("i dig");
    ok_tests.push_back("done dig");
    ok_tests.push_back("stop learning to dig");
    ok_tests.push_back("do dig");
    ok_tests.push_back("stop dig");

    //ok_tests.push_back("You are a good dog. ");
    //ok_tests.push_back("I am fnorbling.");
    //ok_tests.push_back("I am going to fnorbl now.");
    //ok_tests.push_back("I am done fnorbling.");
    //ok_tests.push_back("Try fnorbling");
    //ok_tests.push_back("Try to fnorbl");
    //ok_tests.push_back("Try to sit   ");
    //ok_tests.push_back("Like this    ");
    //ok_tests.push_back("That is a big stick");
    //ok_tests.push_back("That is a log  ");
    //ok_tests.push_back("That is a really big stick");
    //ok_tests.push_back("There is your log. ");
    //ok_tests.push_back("That is Bobs stick. ");

    //vector<pAtom> res;
    //foreach(string s, tests)
    //    parse(clean_string::clean(s.c_str()), back_inserter(res)); //->print();
    set<pAtom, less_atom_by_structure> res;

    foreach(string s, ok_tests) {
//set<pAtom, less_atom_by_structure> res;
        unsigned int original_res_size = (unsigned int)res.size();

        map<pAtom, action*, less_atom> _pat2action = pat2action;
        map<pat, pAtom, less_pat<less_atom> > _terminal = terminal;
        map<pat, pat, less_pat<less_atom> > _patmap = patmap, _i_patmap = i_patmap,
                _elliptic_prefix_patmap = elliptic_prefix_patmap;

        if (_patmap != patmap) puts("!!!!! 3");

        //string clean_s = clean_string::clean(s.c_str());
        string clean_s = opencog::StringManipulator::clean(s);
        parse(clean_s, inserter(res, res.begin())); //->print();
#ifdef USE_ASSERT_OR_EXIT
        assert(res.size() > original_res_size);
#else
        if (res.size() <= original_res_size) {
            printf("OK TEST FAILED\n");
        }
#endif
        if (_terminal != terminal) puts("1");
        if (_pat2action != pat2action) puts("2");
        if (_patmap != patmap) puts("3");
        if (_i_patmap != i_patmap) puts("4");
        if (_elliptic_prefix_patmap != elliptic_prefix_patmap) puts("5");

        printf(" (Res.size = %d (vs. %d before)) \n", res.size(), original_res_size);

        if (!(res.size() > original_res_size)
                || (_terminal != terminal)
                || (_pat2action != pat2action)
                || (_patmap != patmap)
                || (_i_patmap != i_patmap)
                || (_elliptic_prefix_patmap != elliptic_prefix_patmap))
            parse_error();
    }

    foreach(string s, fail_tests) {
        unsigned int original_res_size = (unsigned int)res.size();
        //parse(clean_string::clean(s.c_str()), inserter(res, res.begin())); //->print();
        parse(opencog::StringManipulator::clean(s), inserter(res, res.begin())); //->print();
#ifdef USE_ASSERT_OR_EXIT
        assert(res.size() == original_res_size);
#else
        if (res.size() != original_res_size) {
            printf("FAIL TEST FAILED\n");
        }
#endif
        if (res.size() != original_res_size)
            parse_error();
    }

    puts("Results:");

    foreach(pAtom a, res) {
        a->print();
        puts("");
    }

    puts("\nTaking actions...");

    unsigned long timestamp = 1;
    foreach(pAtom a, res) {
        if (STLhas(pat2action, a))
            (*pat2action[a])(a, timestamp++, "Sarah");
    }
}

} //namespace predavese;

using namespace predavese;

#include "PetInterfaceMock.h"

#ifdef WIN32
int _tmain(int argc, _TCHAR* argv[])
#else
int main(int argc, char* argv[])
#endif
{
    /*    map<pat, pat, less_pat<less_atom> > patmap;
        //initPre(patmap);
        //testPre(patmap);

        initEng(patmap);
     testEng(patmap);*/


    AtomSpace atomSpace;
    SpaceServer spaceServer(atomSpace);

    Control::SystemParameters parameters;
    PetInterfaceMock petInterface;
    FailureActionPlanSender sender;
    PerceptionActionInterface::PAI pai(spaceServer, sender, petInterface, parameters);
    petInterface.setPAI(&pai);
    PredaveseParser *p;
    p = pai.getPredaveseParser();
//    p.Create();

    if (argc > 1) {
        // process the instructions passed as program arguments, as the example bellow:
        //   ./PredaveseTest "I AM AMAZED" "FETCH THE RED BALL"
        for (int i = 1; i < argc; i++) {
            printf("\nGot instruction: %s\n", argv[i]);
            printf("Got %d results\n", p->processInstruction(argv[i], 100));
        }
    } else {
        p->testEng();
    }

    /*    set<ActionAtomPair> s;
        p.Parse("Learn to fetch!", inserter(s, s.begin()));
        p.Parse("Stop learning to fetch!", inserter(s, s.begin()));

        foreach(const ActionAtomPair& a, s)
        {
            (*a.first)(a.second);
        }
    */
    return 0;
}
