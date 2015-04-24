/*
 * PLNModule.h
 *
 * Copyright (C) 2015 by OpenCog Foundation
 * Written by Misgana Bayetta <misgana.bayetta@gmail.com>
 * All Rights Reserved
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

#ifndef PLNAGENT_H_
#define PLNAGENT_H_

#include <opencog/server/Module.h>

namespace opencog
{
/**
 * PLN module for backward and forward chaining triggered by atomspace
 * events.  Whenever atoms are created or enter to the attentional
 * focus, the rule engine forward/backward chainer will be triggered
 * (Reactive rule engine ). Having this functionality will also help
 * us experiment with the dynamics of ECAN together with PLN
 * reasoning.
 */
class CogServer;
class AtomSpace;
class PLNModule: public Module {
private:
    AtomSpace * as_;
    boost::signals2::connection add_af_connection_; //!atom entering to AF
    boost::signals2::connection add_atom_connection_; //!atom creation
public:
    PLNModule(CogServer&);
    virtual ~PLNModule();
    virtual void run();
    const char * id(void);
    virtual void init(void);

    //! Attentional focus events
    void add_af_signal(const Handle&, const AttentionValuePtr&,
                       const AttentionValuePtr&);
    void add_af_signal_handler(const Handle&, const AttentionValuePtr&,
                               const AttentionValuePtr&);
    //! Atom added events
    void add_atom_signal(const Handle&);
    void add_atom_signal_handler(const Handle&);

};
} /* namespace opencog*/
#endif /* PLNAGENT_H_ */
