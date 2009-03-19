#include "comboreduct/combo/vertex.h"
#include "ComboInterpreter.h"
#include "ComboProcedureRepository.h"
#include "ShellWorldWrapper.h"
#include <fstream>
#include <iostream>
#include "util/mt19937ar.h"
#include "PetComboVocabulary.h"


int main(int argc,char** argv) {
  using namespace std;
  using namespace PetCombo;
  using namespace Procedure;

  //instantiate a Logger that does not print messages to not interfere with
  //standard IO
  //opencog::Logger* NoIOLogger = new opencog::Logger();
  opencog::logger().setPrintToStdoutFlag(false);
  //opencog::Logger::initMainLogger(NoIOLogger);    

  ComboProcedureRepository cpr;
  
  if (argc>2) {
    cerr << "usage: " << argv[0] << " [function-file]" << endl;
    cerr << "some trivial sample functions are in scripts/funcs.combo" << endl;
    exit(1);
  } else if (argc==2) {
    ifstream fin(argv[1]);
    int n=cpr.loadFromStream(fin);
    std::cout << "loaded " << n << " combo functions from " << argv[1] << std::endl;
  }

  opencog::MT19937RandGen rng(0);

  combo_tree tr;
  WorldWrapper::ShellWorldWrapper sww;
  while (cin.good()) {
    cout << "> ";
    if (cin.peek()==' ' ||
	cin.peek()=='\n' ||
	cin.peek()=='\t')
      cin.get();
    while (cin.peek()==' ' ||
	   cin.peek()=='\n' ||
	   cin.peek()=='\t') {
      if (cin.peek()=='\n')
	cout << "> ";
      cin.get();
    }
    if (cin.peek()=='#') { //a comment line
      char tmp[1024];
      cin.getline(tmp,1024);
      continue;
    }
    cin >> tr;
    if (!cin.good())
      break;
    cout << "running " << tr << endl;
    try {
      cpr.instantiateProcedureCalls(tr,true);
      std::vector<combo::vertex> empty_arguments; 
      RunningComboProcedure rp(sww, tr, rng, empty_arguments);
      while (!rp.isFinished()) {
	rp.cycle();
      } 
      if (rp.isFailed()) {
	cout << "execution failed!" << endl;
      } else {
	if (rp.getResult()!=id::action_success)
	  cout << "result: " << rp.getResult() << endl;
	cout << "execution succeeded!" << endl;
      }
    } catch(...) {
      cout << "execution failed (threw exception)" << endl;
    }
  }
  cout << endl;
  return 0;
}
