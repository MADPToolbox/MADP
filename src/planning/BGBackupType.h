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

#ifndef _BGBACKUPTYPE_H_
#define _BGBACKUPTYPE_H_ 1

enum BGBackupType { EXHAUSTIVE_ONLYKEEPMAX,
                    EXHAUSTIVE_STOREALL,
                    BGIP_SOLVER_EXHAUSTIVE,
                    BGIP_SOLVER_ALTERNATINGMAXIMIZATION,
                    BGIP_SOLVER_ALTERNATINGMAXIMIZATION_100STARTS,
                    BGIP_SOLVER_BRANCH_AND_BOUND
};

#endif /* !_BGBACKUPTYPE_H_ */
