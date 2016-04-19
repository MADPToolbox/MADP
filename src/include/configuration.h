/* REPLACE_MADP_HEADER */
/* REPLACE_CONTRIBUTING_AUTHORS_START
 * Frans Oliehoek 
 * Matthijs Spaan 
 * REPLACE_CONTRIBUTING_AUTHORS_END
 */

// This file contains some project-wide defines that indicate the
// availability of certain software libraries. Enabling something here
// usually also requires modifying Makefile.custom.

// Indicates whether we use libgmp to represent LIndex.
#define USE_ARBITRARY_PRECISION_INDEX 0

// Indicates whether to use the pomdp-solve library, or call an external program.
#if !DARWIN
#define USE_POMDPSOLVE_LIBRARY 1
#else // on OSX the pomdpsolve library doesn't get compiled in, so this
      // should always be set to 0
#define USE_POMDPSOLVE_LIBRARY 0
#endif
