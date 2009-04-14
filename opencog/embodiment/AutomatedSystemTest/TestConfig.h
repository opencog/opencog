/*
 * opencog/embodiment/AutomatedSystemTest/TestConfig.h
 *
 * Copyright (C) 2002-2008 Novamente LLC
 * All Rights Reserved
 *
 * Written by Welter Luigi
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

#ifndef _TEST_PARAMETERS_H_
#define _TEST_PARAMETERS_H_

#include <opencog/embodiment/Control/EmbodimentConfig.h>

namespace AutomatedSystemTest
{

class TestConfig : public Control::EmbodimentConfig
{

public:

    TestConfig();
    ~TestConfig();

    // Returns a new TestConfig instance
    static Config* testCreateInstance(void);

}; // class
}  // namespace

#endif
