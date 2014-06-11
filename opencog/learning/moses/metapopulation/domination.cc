/** domination.cc ---
 *
 * Copyright (C) 2010 Novemente LLC
 * Copyright (C) 2012 Poulin Holdings LLC
 *
 * Authors: Nil Geisweiller, Moshe Looks, Linas Vepstas
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

#include <math.h>
#include <future>

#include <boost/range/algorithm/sort.hpp>

#include <opencog/util/oc_omp.h>

#include "metapopulation.h"

namespace opencog {
namespace moses {

using namespace std;
using boost::logic::tribool;
using boost::logic::indeterminate;
using namespace combo;


// ==============================================================
// Gene domination code
// I beleive that the dominated-merge, dominated remove code
// is no longer used anywhere (as of 2012). This is for several
// reasons: 
// -- Computing the bscores needed for domination is slowww.
// -- Removing dominated demes destroys a lot of diversity in the
//    metapopulation, causing learning to stagnate.  There's an 
//    entire diary entry exploring and explaining this phenomenon.
//    In short: from standard evolutionary theory, specialization
//    can only arise out of damage. Eliminating the weak, damaged
//    genes prevents them from being able to discover optimal 
//    solutions.  If only the strong survive, they get trapped in a
//    local maximum, and can never jump out.
//
// We're going to keep this code around for a while, there are some
// useful-seeming sub-algorithms in it ...
//
void metapopulation::remove_dominated(scored_combo_tree_set& bcs, unsigned jobs)
{
    // get the nondominated candidates
    scored_combo_tree_ptr_vec bcv = random_access_view(bcs);
    scored_combo_tree_ptr_vec res = get_nondominated_rec(bcv, jobs);
    // get the dominated by set difference
    boost::sort(bcv); boost::sort(res);
    scored_combo_tree_ptr_vec dif = set_difference(bcv, res);
    // remove the dominated ones
    for (const scored_combo_tree* cnd_ptr : dif)
        bcs.erase(*cnd_ptr);
}

scored_combo_tree_set
metapopulation::get_nondominated_iter(const scored_combo_tree_set& bcs)
{
    typedef std::list<scored_combo_tree> scored_combo_tree_list;
    typedef scored_combo_tree_list::iterator scored_combo_tree_list_it;
    scored_combo_tree_list mcl(bcs.begin(), bcs.end());
    // remove all dominated candidates from the list
    for (scored_combo_tree_list_it it1 = mcl.begin(); it1 != mcl.end();) {
        scored_combo_tree_list_it it2 = it1;
        ++it2;
        if (it2 != mcl.end())
            for (; it2 != mcl.end();) {
                tribool dom = dominates(it1->get_bscore(), it2->get_bscore());
                if (dom)
                    it2 = mcl.erase(it2);
                else if (!dom) {
                    it1 = mcl.erase(it1);
                    it2 = mcl.end();
                } else
                    ++it2;
                if (it2 == mcl.end())
                    ++it1;
            }
        else
            ++it1;
    }
    return scored_combo_tree_set(mcl.begin(), mcl.end());
}

typedef std::pair<scored_combo_tree_set,
                  scored_combo_tree_set> scored_combo_tree_set_pair;

typedef std::vector<const scored_combo_tree*> scored_combo_tree_ptr_vec;
typedef scored_combo_tree_ptr_vec::iterator scored_combo_tree_ptr_vec_it;
typedef scored_combo_tree_ptr_vec::const_iterator scored_combo_tree_ptr_vec_cit;
typedef std::pair<scored_combo_tree_ptr_vec,
                  scored_combo_tree_ptr_vec> scored_combo_tree_ptr_vec_pair;


scored_combo_tree_ptr_vec
metapopulation::get_nondominated_rec(const scored_combo_tree_ptr_vec& bcv,
                     unsigned jobs)
{
    ///////////////
    // base case //
    ///////////////
    if (bcv.size() < 2) {
        return bcv;
    }
    //////////////
    // rec case //
    //////////////
//  The names in enum std::launch have not yet been standardized.
#if defined(__GNUC__) && (__GNUC__ == 4) && (__GNUC_MINOR__ >= 5) && (__GNUC_MINOR__ < 7)
 #define LAUNCH_SYNC std::launch::sync
#else
 #define LAUNCH_SYNC std::launch::deferred
#endif
    scored_combo_tree_ptr_vec_pair bcv_p = split(bcv);
    if (jobs > 1) { // multi-threaded
        auto s_jobs = split_jobs(jobs); // pair
        // recursive calls
        std::future<scored_combo_tree_ptr_vec> task =
            std::async(jobs > 1 ? std::launch::async : LAUNCH_SYNC,
                       bind(&metapopulation::get_nondominated_rec, this,
                            bcv_p.first, s_jobs.first));
        scored_combo_tree_ptr_vec bcv2_nd =
            get_nondominated_rec(bcv_p.second, s_jobs.second);
        scored_combo_tree_ptr_vec_pair res_p =
            get_nondominated_disjoint_rec(task.get(), bcv2_nd, jobs);
        // union and return
        append(res_p.first, res_p.second);
        return res_p.first;
    } else { // single-threaded
        // recursive calls
        scored_combo_tree_ptr_vec
            bcv1_nd = get_nondominated_rec(bcv_p.first),
            bcv2_nd = get_nondominated_rec(bcv_p.second);
        scored_combo_tree_ptr_vec_pair
            res_p = get_nondominated_disjoint_rec(bcv1_nd, bcv2_nd);
        // union and return
        append(res_p.first, res_p.second);
        return res_p.first;
    }
}

// reciprocal of random_access_view
scored_combo_tree_set
metapopulation::to_set(const scored_combo_tree_ptr_vec& bcv)
{
    scored_combo_tree_set res;
    for (const scored_combo_tree* cnd : bcv)
        res.insert(*cnd);
    return res;
}

scored_combo_tree_set_pair
metapopulation::get_nondominated_disjoint(const scored_combo_tree_set& bcs1,
                          const scored_combo_tree_set& bcs2,
                          unsigned jobs)
{
    scored_combo_tree_ptr_vec_pair res_p =
        get_nondominated_disjoint_rec(random_access_view(bcs1),
                                      random_access_view(bcs2),
                                      jobs);
    return std::make_pair(to_set(res_p.first), to_set(res_p.second));
}

scored_combo_tree_ptr_vec_pair
metapopulation::get_nondominated_disjoint_rec(const scored_combo_tree_ptr_vec& bcv1,
                              const scored_combo_tree_ptr_vec& bcv2,
                              unsigned jobs)
{
    ///////////////
    // base case //
    ///////////////
    if (bcv1.empty() || bcv2.empty())
        return std::make_pair(bcv1, bcv2);
    else if (bcv1.size() == 1) {
        scored_combo_tree_ptr_vec bcv_res1, bcv_res2;
        scored_combo_tree_ptr_vec_cit it1 = bcv1.begin(),
            it2 = bcv2.begin();
        bool it1_insert = true; // whether *it1 is to be inserted
                                // in bcv_res1
        for (; it2 != bcv2.end(); ++it2) {
            tribool dom = dominates((*it1)->get_bscore(), (*it2)->get_bscore());
            if (!dom) {
                it1_insert = false;
                bcv_res2.insert(bcv_res2.end(), it2, bcv2.end());
                break;
            } else if (indeterminate(dom))
                bcv_res2.push_back(*it2);
        }
        if (it1_insert)
            bcv_res1.push_back(*it1);
        return std::make_pair(bcv_res1, bcv_res2);
    }
    //////////////
    // rec case //
    //////////////
    // split bcs1 in 2
    scored_combo_tree_ptr_vec_pair bcv1_p = split(bcv1);
    if(jobs > 1) { // multi-threaded
        unsigned jobs1 = jobs / 2;
        unsigned jobs2 = std::max(1U, jobs - jobs1);
        std::future<scored_combo_tree_ptr_vec_pair> task =
            std::async(std::launch::async,
                       bind(&metapopulation::get_nondominated_disjoint_rec, this,
                            bcv1_p.first, bcv2, jobs1));
        scored_combo_tree_ptr_vec_pair bcv_m2 =
            get_nondominated_disjoint_rec(bcv1_p.second, bcv2, jobs2);
        scored_combo_tree_ptr_vec_pair bcv_m1 = task.get();
        // merge results
        append(bcv_m1.first, bcv_m2.first);
        boost::sort(bcv_m1.second); boost::sort(bcv_m2.second);
        scored_combo_tree_ptr_vec bcv_m2_inter =
            set_intersection(bcv_m1.second, bcv_m2.second);
        return std::make_pair(bcv_m1.first, bcv_m2_inter);
    } else { // single-threaded
        scored_combo_tree_ptr_vec_pair
            bcv_m1 = get_nondominated_disjoint_rec(bcv1_p.first, bcv2),
            bcv_m2 = get_nondominated_disjoint_rec(bcv1_p.second,
                                                   bcv_m1.second);
        // merge results
        append(bcv_m1.first, bcv_m2.first);
        return std::make_pair(bcv_m1.first, bcv_m2.second);
    }
}

// merge nondominated candidate to the metapopulation assuming
// that bcs contains no dominated candidates within itself
void metapopulation::merge_nondominated(const scored_combo_tree_set& bcs, unsigned jobs)
{
    scored_combo_tree_ptr_vec bcv = random_access_view(bcs);
    scored_combo_tree_ptr_vec bcv_mp;
    for (const scored_combo_tree& cnd : *this)
        bcv_mp.push_back(&cnd);
    scored_combo_tree_ptr_vec_pair bcv_p =
        get_nondominated_disjoint_rec(bcv, bcv_mp, jobs);

    // remove the dominated ones from the metapopulation
    boost::sort(bcv_mp);
    boost::sort(bcv_p.second);
    scored_combo_tree_ptr_vec diff_bcv_mp =
        set_difference(bcv_mp, bcv_p.second);
    for (const scored_combo_tree* cnd : diff_bcv_mp)
        _scored_trees.erase(*cnd);

    // add the nondominated ones from bsc
    for (const scored_combo_tree* cnd : bcv_p.first)
        _scored_trees.insert(new scored_combo_tree(*cnd));
}

} // ~namespace moses
} // ~namespace opencog

