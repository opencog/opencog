/** geometric_mean.h ---
 *
 * Copyright (C) 2014 OpenCog Foundation
 *
 * Author: Nil Geisweiller
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

#ifndef OPENCOG_UTIL_GEOMETRIC_MEAN
#define OPENCOG_UTIL_GEOMETRIC_MEAN

#include <boost/mpl/placeholders.hpp>
#include <boost/accumulators/framework/accumulator_base.hpp>
#include <boost/accumulators/framework/extractor.hpp>
#include <boost/accumulators/numeric/functional.hpp>
#include <boost/accumulators/framework/parameters/sample.hpp>
#include <boost/accumulators/framework/parameters/weight.hpp>
#include <boost/accumulators/framework/accumulators/external_accumulator.hpp>
#include <boost/accumulators/framework/depends_on.hpp>
#include <boost/accumulators/statistics_fwd.hpp>
#include <boost/accumulators/statistics/count.hpp>

// Geometric mean boost accumulator extension. That is formally
//
// sqrt(x1 * .. * xn)

namespace boost { namespace accumulators
{

namespace impl
{
    ///////////////////////////////////////////////////////////////////////////////
    // geometric_mean_impl
    template<typename Sample, typename Tag>
    struct geometric_mean_impl
      : accumulator_base
    {
        // for boost::result_of
        typedef Sample result_type;

        template<typename Args>
        geometric_mean_impl(Args const &args)
          : prod(1.0)
        {
        }

        template<typename Args>
        void operator ()(Args const &args)
        {
            // what about overflow?
            this->prod *= args[parameter::keyword<Tag>::get()];
        }

        template<typename Args>
        result_type result(Args const &args) const
        {
            return std::pow(this->prod, 1.0 / count(args));
        }

    private:

        Sample prod;
    };

} // namespace impl

///////////////////////////////////////////////////////////////////////////////
// tag::geometric_mean
//
namespace tag
{
    struct geometric_mean
      : depends_on<count>
    {
        /// INTERNAL ONLY
        ///
        typedef accumulators::impl::geometric_mean_impl<mpl::_1, tag::sample> impl;
    };

}

///////////////////////////////////////////////////////////////////////////////
// extract::geometric_mean
//
namespace extract
{
    extractor<tag::geometric_mean> const geometric_mean = {};

    BOOST_ACCUMULATORS_IGNORE_GLOBAL(geometric_mean)
}

using extract::geometric_mean;

}} // namespace boost::accumulators

#endif
