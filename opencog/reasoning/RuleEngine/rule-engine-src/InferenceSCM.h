/*
 * InferenceSCM.h
 *
 * Copyright (C) 2014 Misgana Bayetta
 *
 * Author: Misgana Bayetta <misgana.bayetta@gmail.com>  Sept 2014
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
#ifndef CHAINERSCM_H_
#define CHAINERSCM_H_

#include <opencog/atomspace/Handle.h>

namespace opencog{

class InferenceSCM {
private:
	static InferenceSCM* _inst;
	void init(void);
	Handle do_forward_chaining(Handle h);
public:
	InferenceSCM();
	virtual ~InferenceSCM();
};
} /*end of namespace opencog*/
#endif /* CHAINERSCM_H_ */
