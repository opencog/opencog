#include "tree.h"
#include <boost/spirit/include/classic_core.hpp>

namespace {
using namespace boost::spirit::classic;
using std::string;
using namespace opencog;

tree<string> tr;
tree<string>::iterator at=tr.begin();

void begin_internal(const char* from, const char* to) {
    at= tr.empty() 
        ? tr.insert(at,string(from,to-1))
        : tr.append_child(at,string(from,to-1));
}
void end_internal(const char) {
    at=tr.parent(at);
}
void add_leaf(const char* from, const char* to) {
    if (tr.empty())
        at=tr.insert(at,string(from,to));
    else
        tr.append_child(at,string(from,to));
}

struct TreeGrammar : public grammar<TreeGrammar> {
    std::vector<string> v;

    template<typename ScannerT>
    struct definition {
        definition(const TreeGrammar&) {
            term= 
                lexeme_d[//or a message M with the syntax message:"M"
                         //added this to parse correctly has_said perceptions
                         ( str_p("message:") >> ch_p('"') 
                           >> *(anychar_p - ch_p('"')) >> ch_p('"'))
                         | (+( anychar_p - ch_p('(') - ch_p(')') - space_p))]
                [&add_leaf];
            beg=
                lexeme_d[(+( anychar_p - ch_p('(') - ch_p(')') - space_p)) >> '('];
            expr=
                (beg[&begin_internal] >> +expr >> ch_p(')')[&end_internal]) |
                term;
            //expr=term | (term >> '(' >> +expr >> ')');
        }
        rule<ScannerT> expr,beg,term;
  
        const rule<ScannerT>& start() const { return expr; }
    };
};

tree<std::string> parse_string_tree(const std::string& str) {
    TreeGrammar tg;
    tr.clear();
    at=tr.begin();
    parse(str.c_str(),tg,space_p);

    tree<std::string> tmp(tr);
    tr.clear();
    return tmp;
}

} // ~namespace

namespace std {

std::istream& operator>>(std::istream& in,opencog::tree<std::string>& t) throw (InconsistenceException, std::bad_exception) {
    t.clear();
    std::string str,tmp;
    int nparen=0;
    do {
        //replaced by getline so that a message i.e. "yo  man" isn't
        //replaced by "yo man" (where a space is missing)
        //Other assumption : no '(' and ')' between " "
        std::getline(in, tmp);
        nparen+=count(tmp.begin(),tmp.end(),'(')-count(tmp.begin(),tmp.end(),')');
        str+=tmp+' ';
    } while (in.good() && nparen>0);
    if (nparen!=0) {
        std::stringstream stream (std::stringstream::out);
        stream << "Paren mismatch parsing tree: '" << str << "'" << std::endl;
        throw InconsistenceException(TRACE_INFO, "tree - %s.",
                                     stream.str().c_str());
    }
    t=parse_string_tree(str);
    return in;
}

} // ~namespace opencog
