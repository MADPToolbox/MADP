/**\file MADP_util.cpp
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
 */

#include "MADP_util.h"
#include <stdlib.h>
#include <math.h>

using namespace std;

namespace libDAI {

namespace MADP_util{
    bool EqualValue(double r1, double r2)
    {
        double absdiff= fabs(r1-r2);
        bool  equal = (absdiff < V_EPS );
        //cout << "[absdiff="<<absdiff<<",equal="<<equal<<"]";
        return ( equal ) ;
    }

}

}
