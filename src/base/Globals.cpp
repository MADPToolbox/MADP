/* REPLACE_MADP_HEADER */
/* REPLACE_CONTRIBUTING_AUTHORS_START
 * Frans Oliehoek 
 * Matthijs Spaan 
 * REPLACE_CONTRIBUTING_AUTHORS_END
 */

#include "Globals.h"
#include "EOverflow.h"
#include <sstream>

using namespace std;

bool Globals::EqualProbability(double p1, double p2)
{
    return ( abs(p1-p2) < PROB_PRECISION ) ;
}

bool Globals::EqualReward(double r1, double r2)
{
    return ( abs(r1-r2) < REWARD_PRECISION ) ;
}

#if USE_ARBITRARY_PRECISION_INDEX
Index Globals::CastLIndexToIndex(LIndex i)
{
    Index j=0;
    if(i.fits_ulong_p())
        j=i.get_ui();
    else
    {
        stringstream ss;
        ss << "LIndex with value "
           << i
           << " does not fit in an Index";
        throw(EOverflow(ss));
    }
    return(j);
}
#else
Index Globals::CastLIndexToIndex(LIndex i) {  return(i); }
#endif

#if USE_ARBITRARY_PRECISION_INDEX
double Globals::CastLIndexToDouble(LIndex i)
{
    mpf_t y;
    mpf_init(y);
    mpf_set_z(y,i.get_mpz_t());
    return mpf_get_d(y);
}
#else
double Globals::CastLIndexToDouble(LIndex i) {  return(static_cast<double>(i)); }
#endif
