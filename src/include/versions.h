/* REPLACE_MADP_HEADER */
/* REPLACE_CONTRIBUTING_AUTHORS_START
 * Frans Oliehoek 
 * Matthijs Spaan 
 * REPLACE_CONTRIBUTING_AUTHORS_END
 */

#include "boost/version.hpp"

#if BOOST_VERSION < 103300
#define BOOST_1_32_OR_LOWER 1
#else
#define BOOST_1_32_OR_LOWER 0
#endif

#if BOOST_VERSION >= 103600
#define BOOST_1_36_OR_HIGHER 1
#else
#define BOOST_1_36_OR_HIGHER 0
#endif

#if BOOST_VERSION >= 103800
// Spirit V2 has been merged in the Boost, so now we need to use the
// "classic" version
#define USE_BOOST_SPIRIT_CLASSIC 1
#else
#define USE_BOOST_SPIRIT_CLASSIC 0
#endif

