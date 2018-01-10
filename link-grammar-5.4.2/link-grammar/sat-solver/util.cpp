#include "util.hpp"

extern "C" {
#include "api-structures.h"
#include "disjunct-utils.h"
#include "linkage/linkage.h"
#include "utilities.h"
};

/**
 * Free all the connectors and disjuncts of a specific linkage.
 */
void free_linkage_connectors_and_disjuncts(Linkage lkg)
{
  // Free the connectors
  for(size_t i = 0; i < lkg->lasz; i++) {
    free(lkg->link_array[i].rc);
    free(lkg->link_array[i].lc);
  }
  // Free the disjuncts
  for (size_t i = 0; i < lkg->cdsz; i++) {
    free_disjuncts(lkg->chosen_disjuncts[i]);
  }
}

/**
 * Free all the connectors and disjuncts of all the linkages.
 */
void sat_free_linkages(Sentence sent, LinkageIdx next_linkage_index)
{
  Linkage lkgs = sent->lnkages;

  for (LinkageIdx li = 0; li < next_linkage_index; li++) {
    free_linkage_connectors_and_disjuncts(&lkgs[li]);
    free_linkage(&lkgs[li]);
  }
  free(lkgs);
  sent->lnkages = NULL;
  sent->num_linkages_alloced = 0;
}

Exp* null_exp()
{
  static Exp e;

  if (e.type) return &e;
  e.u.l = NULL;
  e.type = AND_type;
  return &e;
}

void add_anded_exp(Exp*& orig, Exp* addit)
{
    if (orig == NULL)
    {
      orig = addit;
    } else {
      // flist is orig
      E_list* flist = (E_list*)xalloc(sizeof(E_list));
      flist->e = orig;
      flist->next = NULL;

      // elist is addit, orig
      E_list* elist = (E_list*)xalloc(sizeof(E_list));
      elist->next = flist;
      elist->e = addit;

      // The updated orig is addit & orig
      orig = (Exp*)xalloc(sizeof(Exp));
      orig->type = AND_type;
      orig->cost = 0.0;
      orig->u.l = elist;
    }
}

#if 0
bool isEndingInterpunction(const char* str)
{
  return strcmp(str, ".") == 0 ||
    strcmp(str, "?") == 0 ||
    strcmp(str, "!") == 0;
}
#endif
