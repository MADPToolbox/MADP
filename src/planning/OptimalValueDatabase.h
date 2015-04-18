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
#ifndef _OPTIMALVALUEDATABASE_H_
#define _OPTIMALVALUEDATABASE_H_ 1

/* the include directives */
#include "Globals.h"
#include <map>

class PlanningUnitDecPOMDPDiscrete;

/** \brief OptimalValueDatabase provides values of optimal policies
 * for problems, so to be used for verification purposes.  */
class OptimalValueDatabase 
{
private:    
    typedef std::pair<std::string,double> nameDiscountT;
    typedef std::pair<nameDiscountT, size_t> nameDiscountHorizonT;

    /// The database
    std::map<nameDiscountHorizonT, double> _m_optimalValues;

    const PlanningUnitDecPOMDPDiscrete* _m_pu;

    /// Add an entry to the database.
    void AddEntry(const std::string &problemName,
                  double discount,
                  size_t horizon,
                  double value);
    /// Get an entry from the database.
    double GetEntry(const std::string &problemName,
                    double discount,
                    size_t horizon) const;
    /// Check if the problem is in the database already.
    bool IsInDatabase(const std::string &problemName,
                      double discount,
                      size_t horizon) const;
    /// Check if the value corresponds with a known optimal value.
    bool IsOptimal(const std::string &problemName,
                   double discount,
                   size_t horizon,
                   double value) const;

    /// Load the database from disk.
    void Load();

    /// Store the database to disk.
    void Save() const;

protected:
    
public:
    // Constructor, destructor and copy assignment.
    /// (default) Constructor
    OptimalValueDatabase(const PlanningUnitDecPOMDPDiscrete *pu);

    /// Add an entry to the database. Saves it to disk.
    void SetOptimalValue(double value);

    /// Get the optimal value (if known, otherwise expection is thrown).
    double GetOptimalValue() const;

    /// Check if the value is optimal.
    bool IsOptimal(double value) const;

    /// Check if the particular (problem, discount, horizon) tuple is stored.
    bool IsInDatabase() const;

    /// A string representation of the database.
    std::string SoftPrint() const;
};


#endif /* !_OPTIMALVALUEDATABASE_H_ */

// Local Variables: ***
// mode:c++ ***
// End: ***
