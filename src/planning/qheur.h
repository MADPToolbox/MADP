/* REPLACE_MADP_HEADER */
/* REPLACE_CONTRIBUTING_AUTHORS_START
 * Frans Oliehoek
 * Matthijs Spaan
 * REPLACE_CONTRIBUTING_AUTHORS_END
 */

#ifndef  _QHEUR_H_
#define  _QHEUR_H_ 1

#include <string>

namespace qheur {
    
enum Qheur_t {eQMDP, eQPOMDP, eQBG, eQMDPc, eQPOMDPav, eQBGav, 
              eQHybrid, eQPOMDPhybrid, eQBGhybrid, eQBGTreeIncPrune,
              eQBGTreeIncPruneBnB, eQheurUndefined};
std::string SoftPrint(Qheur_t Qheur);

}

#endif /* !_QHEUR_H_*/
