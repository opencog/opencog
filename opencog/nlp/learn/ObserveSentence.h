/*
 * opencog/nlp/learn/ObserveSentence.h
 *
 * Copyright (C) 2017 by OpenCog Foundation
 * All Rights Reserved
 *
 * Written by Curtis Faith
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

#ifndef _OBSERVE_SENTENCE_H
#define _OBSERVE_SENTENCE_H

#define NO_PAIR_DISTANCE_LIMIT 0

namespace opencog
{
    class AtomSpace;

    typedef std::vector<std::string> WordVector;

    void break_sentence_into_words( const std::string& sentence,
                                    WordVector& words);

    void observe_sentence(AtomSpace*      atomspace,
                          std::string&    sentence,
                          int             pair_distance_limit = NO_PAIR_DISTANCE_LIMIT );
}

#endif // _OBSERVE_SENTENCE_H
