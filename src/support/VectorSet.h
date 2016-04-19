/* REPLACE_MADP_HEADER */
/* REPLACE_CONTRIBUTING_AUTHORS_START
 * Frans Oliehoek 
 * Matthijs Spaan 
 * REPLACE_CONTRIBUTING_AUTHORS_END
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
