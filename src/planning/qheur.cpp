/* REPLACE_MADP_HEADER */
/* REPLACE_CONTRIBUTING_AUTHORS_START
 * Frans Oliehoek
 * Matthijs Spaan
 * REPLACE_CONTRIBUTING_AUTHORS_END
 */


using namespace std;

#include "qheur.h"
#include "E.h"

std::string qheur::SoftPrint(Qheur_t Qheur)
{
    switch(Qheur)
    {
    case(eQMDP):
        return("QMDP");
    case(eQPOMDP):
        return("QPOMDP");
    case(eQBG):
        return("QBG");
    case(eQMDPc):
        return("QMDPc");
    case(eQPOMDPav):
        return("QPOMDPav");
    case(eQBGav):
        return("QBGav");
    case eQHybrid:
        return("QHybrid");
    case eQPOMDPhybrid:
        return("QPOMDPhybrid");
    case eQBGhybrid:
        return("QBGhybrid");
    case eQBGTreeIncPrune:
        return("QBGTreeIncPrune");
    case eQBGTreeIncPruneBnB:
        return("QBGBnBIncPrune");
    case eQheurUndefined:
        return("INVALIDQHEUR");
    }

    throw(E("qheur::SoftPrint invalid type"));

    return("INVALIDQHEUR");
}


