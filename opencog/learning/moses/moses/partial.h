/** partial.h ---
 *
 * Copyright (C) 2012 Poulin Holdings
 *
 * Author: Linas Vepstas <linasvepstas@gmail.com>
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
#include <opencog/comboreduct/combo/table.h>
#include <opencog/comboreduct/combo/type_tree.h>
#include <opencog/comboreduct/combo/vertex.h>
#include "../main/moses_exec.h"

namespace opencog { namespace moses {

using namespace combo;

/// Implements the "leave well-enough alone" algorithm.
class partial_solver
{
    public:
        partial_solver(const std::vector<CTable> &,
                       int,
                       float,
                       const type_tree&,
                       const std::vector<combo_tree>&,
                       const rule&,
                       const optim_parameters&,
                       const metapop_parameters&,
                       const moses_parameters&,
                       const metapop_print_parameters&);
        void solve();

        void candidate (const combo_tree& cond);

    private:
        std::vector<CTable> _ctables;
        int _alf_sz;
        float _noise;
        type_tree _table_tt;
        std::vector<combo_tree> _exemplars;
        const rule& _contin_reduct;
        optim_parameters _opt_params;
        metapop_parameters _meta_params;
        moses_parameters _moses_params;
        metapop_print_parameters _mmr_pa;
};

};};
