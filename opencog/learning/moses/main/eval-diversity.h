/** eval-diversity.h --- 
 *
 * Copyright (C) 2012 OpenCog Foundation
 *
 * Author: Nil Geisweiller <ngeiswei@gmail.com>
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

#include <vector>

#include <boost/range/algorithm/transform.hpp>
#include <boost/accumulators/accumulators.hpp>
#include <boost/accumulators/statistics/stats.hpp>
#include <boost/accumulators/statistics/count.hpp>
#include <boost/accumulators/statistics/mean.hpp>
#include <boost/accumulators/statistics/variance.hpp>
#include <boost/accumulators/statistics/min.hpp>
#include <boost/accumulators/statistics/max.hpp>

#include <opencog/util/iostreamContainer.h>

namespace opencog { namespace moses {

using namespace boost::accumulators;

// structure holding the options
struct eval_diversity_params
{
    // IO
    std::vector<std::string> input_files,
        moses_files;
    std::string output_file,
        target_feature;
    bool display_stats,
        display_values;
    // parameters
    std::string diversity_dst;
    double diversity_p_norm;
};

// diversity distance types
static const std::string p_norm = "p_norm";
static const std::string tanimoto = "tanimoto";
static const std::string angular = "angular";
        
// define accumulator to gather stats
typedef accumulator_set<double, stats<tag::count,
                                     tag::mean,
                                     tag::variance,
                                     tag::min,
                                     tag::max>> accumulator_t;

template<typename Out>
Out& ostream_results(Out& out, const eval_diversity_params& edp,
                     const std::vector<score_t>& dsts)
{
    if (edp.display_values)
        ostreamContainer(out, dsts, "\n") << std::endl;

    if (edp.display_stats) {
        // compute the statistics
        accumulator_t acc;
        for (score_t f : dsts) acc(f);

        // display the statistics
        out << "count: " << count(acc) << std::endl;
        out << "mean: " << mean(acc) << std::endl;
        out << "std dev: " << sqrt(variance(acc)) << std::endl;
        out << "min: " << boost::accumulators::min(acc) << std::endl;
        out << "max: " << boost::accumulators::max(acc) << std::endl;
    }
    return out;
}

} // ~namespace moses
} // ~namespace opencog

