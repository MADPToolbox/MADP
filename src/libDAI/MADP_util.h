/**\file MADP_util.h
 *
 * Authors:
 * Frans Oliehoek <fao@csail.mit.edu>
 * Matthijs Spaan <mtjspaan@isr.ist.utl.pt>
 *
 * Copyright 2010 Massachusetts Institute of Technology, 
 *  Instituto Superior Tecnico
 *
 * This file is part of MultiAgentDecisionProcess.
 *
 * MultiAgentDecisionProcess is free software: you can redistribute it
 * and/or modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation, either version 3 of
 * the License, or (at your option) any later version.
 *
 * MultiAgentDecisionProcess is distributed in the hope that it will
 * be useful, but WITHOUT ANY WARRANTY; without even the implied
 * warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with MultiAgentDecisionProcess.  If not, see
 * <http://www.gnu.org/licenses/>.
 *
 * $Id$
 *
 *
 * This file contains utility definitions that were made to make interfacing
 * with MADP easier.
 */

/* Only include this header file once. */
#ifndef _MADP_UTIL_H_
#define _MADP_UTIL_H_ 1

#include <utility>
#include <vector>
#include <cstddef>

//the difference we require a solution to be better ('V' for value)
#define V_EPS 1e-9

namespace libDAI {
    namespace MADP_util{
    bool EqualValue(double r1, double r2);
    typedef std::pair< double, std::vector<size_t> > valConf;
    }
}

#endif /* !_MADP_UTIL_H_ */

// Local Variables: ***
// mode:c++ ***
// End: ***
