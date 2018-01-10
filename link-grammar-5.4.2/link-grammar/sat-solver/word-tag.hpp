#ifndef __WORD_TAG_HPP__
#define __WORD_TAG_HPP__

#include <vector>
#include <map>
#include <set>

extern "C" {
#include "connectors.h"
#include "dict-common/dict-common.h"
};

#include "variables.hpp"

struct PositionConnector
{
  PositionConnector(Exp* e, Connector* c, char d, int w, int p, 
                    double cst, double pcst, bool lr, bool ll,
                    const std::vector<int>& er, const std::vector<int>& el, const X_node *w_xnode)
    : exp(e), dir(d), word(w), position(p),
      cost(cst), parent_cost(pcst),
      leading_right(lr), leading_left(ll),
      eps_right(er), eps_left(el), word_xnode(w_xnode)
  {
    // Initialize some fields in the connector struct.
    connector.string = c->string;
    connector.multi = c->multi;
    connector.length_limit = c->length_limit;

    if (word_xnode == NULL) {
       cerr << "Internal error: Word" << w << ": " << "; connector: '" << c->string << "'; X_node: " << (word_xnode?word_xnode->string: "(null)") << endl;
    }

    /*
    cout << c->string << " : ." << w << ". : ." << p << ". ";
    if (leading_right) {
      cout << "lr: ";
      copy(er.begin(), er.end(), ostream_iterator<int>(cout, " "));
    }
    if (leading_left) {
      cout << "ll: ";
      copy(el.begin(), el.end(), ostream_iterator<int>(cout, " "));
    }
    cout << endl;
    */
  }

  // Added only to suppress the warning:
  // warning: inlining failed in call to ‘PositionConnector::~PositionConnector() noexcept’: call is unlikely and code size would grow [-Winline]
  // See: https://gcc.gnu.org/bugzilla/show_bug.cgi?id=70328
  // Can be removed when this GCC problem is fixed.
  ~PositionConnector() {};

  // Original expression that this connector came from
  Exp* exp;

  // Connector itself
  Connector connector;
  // Direction
  char dir;
  // word in a sentence that this connector belongs to
  size_t word;
  // position in the word tag
  int position;
  // cost of the connector
  double cost;
  // parent cost
  double parent_cost;

  bool leading_right;
  bool leading_left;
  std::vector<int> eps_right;
  std::vector<int> eps_left;


  // The corresponding X_node - chosen-disjuncts[]
  const X_node *word_xnode;

  // Matches with other words
  std::vector<PositionConnector*> matches;

};


// XXX TODO: Hash connectors for faster matching

class WordTag
{
private:
  std::vector<PositionConnector> _left_connectors;
  std::vector<PositionConnector> _right_connectors;

  std::vector<char> _dir;
  std::vector<int> _position;

  int _word;
  Variables* _variables;

  Sentence _sent;
  Parse_Options _opts;

  // Could this word tag match a connector (wi, pi)?
  // For each word wi I keep a set of positions pi that can be matched
  std::vector< std::set<int> > _match_possible;
  void set_match_possible(int wj, int pj) {
    _match_possible[wj].insert(pj);
  }

public:
  WordTag(int word, const char* name, Variables* variables, Sentence sent, Parse_Options opts)
    : _word(word), _variables(variables), _sent(sent), _opts(opts) {
    _match_possible.resize(_sent->length);

    // The SAT word variables are set to be equal to the word numbers.
    Var var = _variables->string(name);
    assert(word == var);

    verbosity = opts->verbosity;
    debug = opts->debug;
    test = opts->test;
  }

  // Added only to suppress the warning:
  // warning: inlining failed in call to ‘WordTag::~WordTag() noexcept’: call is unlikely and code size would grow [-Winline]
  // See: https://gcc.gnu.org/bugzilla/show_bug.cgi?id=70328
  // Can be removed when this GCC problem is fixed.
  ~WordTag() {};

  const std::vector<PositionConnector>& get_left_connectors() const {
    return _left_connectors;
  }

  const std::vector<PositionConnector>& get_right_connectors() const {
    return _right_connectors;
  }

  PositionConnector* get(int dfs_position)
  {
    switch (_dir[dfs_position - 1]) {
    case '+':
      return &_right_connectors[_position[dfs_position - 1]];
    case '-':
      return &_left_connectors[_position[dfs_position - 1]];
    }
    return NULL;
  }

  void set_connector_length_limit(Connector* c)
  {
    unsigned int len = _opts->short_length;
    Connector_set * conset = _sent->dict->unlimited_connector_set;

    if (len > UNLIMITED_LEN) len = UNLIMITED_LEN;

    if (_opts->all_short ||
        (conset != NULL && !match_in_connector_set(conset, c)))
    {
      c->length_limit = len;
    }
  }

  bool match(int w1, Connector& cntr1, char dir, int w2, Connector& cntr2)
  {
      int dist = w2 - w1;
      assert(0 < dist, "match() did not receive words in the natural order.");
      if (dist > cntr1.length_limit || dist > cntr2.length_limit) return false;
      return easy_match(cntr1.string, cntr2.string);
  }

  void insert_connectors(Exp* exp, int& dfs_position,
                         bool& leading_right, bool& leading_left,
                         std::vector<int>& eps_right,
                         std::vector<int>& eps_left,
                         char* var, bool root, double parent_cost,
                         Exp* parent, const X_node *word_xnode);

  // Caches information about the found matches to the _matches vector, and also
  // updates the _matches vector of all connectors in the given tag.
  // In order to have all possible matches correctly cached, the function assumes that it is
  // iteratively called for all words in the sentence, where the tag is on the right side of
  // this word
  void add_matches_with_word(WordTag& tag);

  // Find matches in this word tag with the connector (name, dir).
  void find_matches(int w, const char* C, char dir,  std::vector<PositionConnector*>& matches);

  // A simpler function: Can any connector in this word match a connector wi, pi?
  // It is assumed that
  bool match_possible(int wi, int pi)
  {
    return _match_possible[wi].find(pi) != _match_possible[wi].end();
  }

private:
  int verbosity;
  const char *debug;
  const char *test;
};

#endif
