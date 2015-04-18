/* This file is part of the Multiagent Decision Process (MADP) Toolbox v0.3. 
 *
 * The majority of MADP is free software released under GNUP GPL v.3. However,
 * some of the included libraries are released under a different license. For 
 * more information, see the included COPYING file. For other information, 
 * please refer to the included README file.
 *
 * This file has been written and/or modified by the following people:
 *
 * Frans Oliehoek 
 * Matthijs Spaan 
 *
 * For contact information please see the included AUTHORS file.
 */

/* Only include this header file once. */
#ifndef _VECTORSET_H_
#define _VECTORSET_H_ 1

/* the include directives */
#include <iostream>
#include "Globals.h"

#include "boost/numeric/ublas/matrix_sparse.hpp"
#include "boost/numeric/ublas/matrix.hpp"



/// Represents a set of (alpha) vectors.
typedef boost::numeric::ublas::matrix<double> VectorSet;

typedef VectorSet::const_iterator1 VScit1;
typedef VectorSet::iterator1 VSit1;
typedef VectorSet::const_iterator2 VScit2;
typedef VectorSet::iterator2 VSit2;

#endif /* !_VECTORSET_H_ */

// Local Variables: ***
// mode:c++ ***
// End: ***
