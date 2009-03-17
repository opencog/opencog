#include "LocalSpaceMap2DUtil.h"
#include "TangentBug.h"
#include <list>
#include <boost/lexical_cast.hpp>
#include <boost/tuple/tuple.hpp>
#include "util/mt19937ar.h"

#ifdef TB_PRINT_NCURSES
#include <ncurses.h>
#endif

int main(int argc, char * argv[]) {
  using namespace Spatial;
  using namespace TangentBugBits;
  using namespace std;

  int seed;
  if (argc > 1)
    try {
      seed = boost::lexical_cast<int, char*>(argv[1]);
    }
    catch(boost::bad_lexical_cast &) {
      cerr << "Usage:\n";
      cerr << "If ncurses printing is enabled, you had better redirect stderr:\n";
      cerr << argv[0] << " [integral random seed] 2>log.txt\n";
      cerr << "Otherwise:\n";
      cerr << argv[0] << " [integral random seed]\n";
      exit(1);
    }
  else
    seed = time(NULL);

  cout << "Random seed is " << seed << endl;
  opencog::MT19937RandGen rng(seed);

  //typedef LocalSpaceMap2D<Handle, double, hashHandle, Spatial::ObjMetaData > LSM;
  typedef LocalSpaceMap2D LSM;
  typedef TangentBug TB;

  LSM lsm(500, 999, 200,
          500, 999, 200,
          //5);
          6);

  TB::CalculatedPath action_plan;
  TB tb(lsm, action_plan, rng);
  tb.init_ncurses();

  // Randomly generate a map:
  populateRandom(rng, lsm, 20, tb.getCenter());
  //populateRandom<Handle,double,hashHandle,Spatial::ObjMetaData>(lsm, 20, tb.getCenter());
  tb.place_pet_randomly(tb.getCenter());
  tb.place_goal_randomly(tb.getCenter());

  // Just aesthetic, does nothing with NCurses off:
  tb.trace_path(tb.getCenter(), tb.getGoal());
  tb.trace_path(tb.getCenter(), tb.getCurrPos());

  bool success = tb.seek_goal();
  //tb.print_action_plan();
  tb.print();
  //call to getch commented since its implementation is empty, anyway...
  //getch();
  tb.stop_ncurses();
  return (success ? 0 : 1);
}
