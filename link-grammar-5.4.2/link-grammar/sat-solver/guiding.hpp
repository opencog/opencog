#ifndef __GUIDING_HPP__
#define __GUIDING_HPP__

#include <vector>
#include <core/Solver.h>
#undef assert
#include "util.hpp"

extern "C"
{
#include "api-structures.h" // For definition of Sentence
}

#ifndef HAVE_SETPOLARITY_BOOL // setPolarity has lbool argument, not bool
#define setPolarity(v, b) setPolarity(v, toLbool(b))
#endif

using namespace Minisat;

// This class represents different guiding strategies of LinkParser SAT search
class Guiding {
public:
  struct SATParameters {
    /* Should the variable with the given number be used as a decision
       variable during the SAT search? */
    bool isDecision;
    /* What is the decision priority of the variable with the given number
       during the SAT search? */
    double priority;
    /* What is the preferred polarity of the variable with the given number
       during the SAT search? */
    double polarity;
  };

  Guiding(Sentence sent)
    : _sent(sent) {
  }

  virtual ~Guiding() {}

  /* Abstract functions that calculate params for each type of variable */

  /* string variables */
  virtual void setStringParameters  (int var, const char* str)
  {
    bool isDecision = false;
    setParameters(var, isDecision, 0.0, 0.0);
  }
  virtual void setStringParameters  (int var, const char* str, double cost) = 0;

  /* epsilon variables */
  virtual void setEpsilonParameters (int var)
  {
    bool isDecision = false;
    setParameters(var, isDecision, 0.0, 0.0);
  }

  /* link_cc variables */
  virtual void setLinkParameters    (int var, int wi, const char* ci, int wj, const char* cj, const char* label) = 0;
  virtual void setLinkParameters    (int var, int wi, const char* ci, int wj, const char* cj, const char* label,
                                     double cost) = 0;

  /* linked_variables */
  virtual void setLinkedParameters  (int var, int wi, int wj)
  {
    bool isDecision = false;
    setParameters(var, isDecision, 0.0, 0.0);
  }

  virtual void setLinkedMinMaxParameters  (int var, int wi, int wj)
  {
    bool isDecision = false;
    setParameters(var, isDecision, 0.0, 0.0);
  }

  /* link_cw_variables */
  virtual void setLinkCWParameters  (int var, int wi, int wj, const char* cj) {
    bool isDecision = false;
    setParameters(var, isDecision, 0.0, 0.0);
  }

  /* link_top_cw variables */
  virtual void setLinkTopCWParameters  (int var, int wi, int wj, const char* cj) {
    bool isDecision = false;
    setParameters(var, isDecision, 0.0, 0.0);
  }

  /* link_top_ww variables */
  virtual void setLinkTopWWParameters  (int var, int wi, int wj) {
    bool isDecision = false;
    setParameters(var, isDecision, 0.0, 0.0);
  }

#if 0
  /* thin_link variables */
  virtual void setThinLinkParameters (int var, int wi, int wj) = 0;
#endif

  /* Pass SAT search parameters to the MiniSAT solver */
  void passParametersToSolver(Solver* solver) {
    for (size_t v = 0; v < _parameters.size(); v++) {
      solver->setDecisionVar(v, _parameters[v].isDecision);
      if (_parameters[v].isDecision) {
        /* The following setActivity() call may significantly damage
         * the performance (timeit -r 6 -n 6):
         * -30% for en/4.0.fixes.batch.
         * -5% for en/4.0.batch.
         * ~0% for ru/4.0.fixes.batch (i.e. it doesn't matter).
         * It still damages the performance also when producing all the
         * linkages (tested with !limit=10000).
         * The link cost influences the priority (set_cost()), but the
         * produced linkages using it are not according to cost order anyway.
         * Most probably it should be removed (its is not supported
         * anyway in Minisat >= 2.2). */
        //solver->setActivity(v, _parameters[v].priority);
        /* TODO: make polarity double instead of boolean*/
        solver->setPolarity(v, _parameters[v].polarity > 0.0);
      }
    }
  }

protected:
  /* Set the parameters for the given variable to given values */
  void setParameters(size_t var, bool isDecision, double priority, double polarity) {
    if (var >= _parameters.size())
      _parameters.resize(var + 1);
    _parameters[var].isDecision = isDecision;
    _parameters[var].priority = priority;
    _parameters[var].polarity = polarity;

    //printf("Set: %zu %s .%g. .%g.\n", var, isDecision ? "true" : "false", priority, polarity);
  }


  /* Sentence that is being parsed */
  Sentence _sent;

  /* Parameters for each variable */
  std::vector<SATParameters> _parameters;
};

////////////////////////////////////////////////////////////////////////////
class CostDistanceGuiding : public Guiding
{
public:
  double cost2priority(double cost) const {
    return cost == 0.0 ? 0.0 : ((double)(_sent->length) + cost);
  }

  CostDistanceGuiding(Sentence sent)
    : Guiding(sent) {
  }

  virtual void setStringParameters(int var, const char* str, double cost)
  {
    bool isDecision = cost > 0.0;
    double priority = cost2priority(cost);
    double polarity = 0.0;
    setParameters(var, isDecision, priority, polarity);
  }

  virtual void setLinkParameters(int var, int wi, const char* ci, int wj, const char* cj, const char* label)
  {
    bool isDecision = true;
    double priority = 0.0;
    double polarity = 0.0;
    setParameters(var, isDecision, priority, polarity);
  }

  virtual void setLinkParameters(int var, int i, const char* ci, int j, const char* cj, const char* label,
                                 double cost)
  {
    bool isDecision = true;
    double priority = cost2priority(cost);
    double polarity = 0.0;
    setParameters(var, isDecision, priority, polarity);
  }

#if 0
  void setThinLinkParameters(int var, int i, int j)
  {
    bool isDecision = true;

    double priority = (double)(j - i);

    // XXX this is wrong, isEndingInterpunction is just wrong, and
    // must be killed FIXME I think that this is just looking for the
    // one single long link from left-wall to right-punctuation.
    // seems like a hack, why is such an exception needed???
    double polarity = j - i == 1 ? 1.0 : 0.0;
    if (i == 0 && j == (int) _sent->length - 2 &&
       isEndingInterpunction(_sent->word[j].alternatives[0])) {
      polarity = 1.0;
    }
    if (i == 0 && j == (int) _sent->length - 1 &&
      !isEndingInterpunction(_sent->word[j - 1].alternatives[0])) {
      polarity = 1.0;
    }

    setParameters(var, isDecision, priority, polarity);
  }
#endif
};



////////////////////////////////////////////////////////////////////////////
class CostDistanceGuidingOnlyLink : public Guiding
{
public:
  double cost2priority(int cost) const {
    return cost == 0 ? 0.0 : (double)(_sent->length + cost);
  }

  CostDistanceGuidingOnlyLink(Sentence sent)
    : Guiding(sent) {
  }

  virtual void setStringParameters  (int var, const char* str, double cost)
  {
    bool isDecision = cost > 0.0;
    double priority = cost2priority(cost);
    double polarity = 0.0;
    setParameters(var, isDecision, priority, polarity);
  }

  virtual void setLinkParameters    (int var, int wi, const char* ci, int wj, const char* cj, const char* label)
  {
    bool isDecision = true;
    double priority = _sent->length - (wj - wi);
    double polarity = wj - wi <= 3 ? 1.0 : 0.0;;
    setParameters(var, isDecision, priority, polarity);
  }

  virtual void setLinkParameters(int var, int wi, const char* ci, int wj, const char* cj, const char* label,
                                 double cost)
  {
    bool isDecision = true;
    double priority = cost > 0.0 ? cost2priority(cost) : wj - wi;
    double polarity = wj - wi <= 3 ? 1.0 : 0.0;
    setParameters(var, isDecision, priority, polarity);
  }

#if 0
  void setThinLinkParameters(int var, int i, int j) {
    bool isDecision = false;
    setParameters(var, isDecision, 0.0, 0.0);
  }
#endif
};
#endif
