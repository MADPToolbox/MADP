/* REPLACE_MADP_HEADER */
/* REPLACE_CONTRIBUTING_AUTHORS_START
 * Frans Oliehoek 
 * Matthijs Spaan 
 * REPLACE_CONTRIBUTING_AUTHORS_END
 */

/* Only include this header file once. */
#ifndef _MONAHANPLANNER_H_
#define _MONAHANPLANNER_H_ 1

/* the include directives */
#include "Globals.h"
#include "AlphaVectorPlanning.h"

class PlanningUnitDecPOMDPDiscrete;

/** \brief MonahanPlanner provides shared functionality for
 * MonahanPOMDPPlanner and MonahanBGPlanner.
 **/
class MonahanPlanner : public AlphaVectorPlanning
{
private:    

    int _m_timeStep;

protected:

    bool _m_doIncPrune;

    // Vector of value functions for each time step.
    QFunctionsDiscreteNonStationary _m_qFunction;

    bool _m_initialized;
    bool _m_alreadyComputed;

    std::vector<size_t> _m_maxNrAlphas;

    std::string _m_resultsFilename;

    void CheckMaxNrVectors(size_t maxNrAlphas, size_t nrAlphas) const;

    /// Compute a backup stage.
    virtual QFunctionsDiscrete
    BackupStage(const QFunctionsDiscrete& Q, size_t maxNrAlphas=0) = 0;

    int GetTimeStep() const { return(_m_timeStep); }

public:
    // Constructor, destructor and copy assignment.
    /// (default) Constructor
    MonahanPlanner(const PlanningUnitDecPOMDPDiscrete* pu,
                   bool doIncPrune=true);
    MonahanPlanner(const boost::shared_ptr<const PlanningUnitDecPOMDPDiscrete> &pu,
                   bool doIncPrune=true);

    /// Destructor.
    virtual ~MonahanPlanner();

    void SetMaxNrAlphas(const std::vector<size_t> &maxNrAlphas)
        { _m_maxNrAlphas=maxNrAlphas; }

    virtual void Initialize() = 0;

    virtual void Plan();

    virtual void PlanWithCache(const std::string &filenameCache, 
                               bool computeIfNotCached=true);

    double GetQ(Index jaohI, Index jaI) const;
    double GetQ(const JointBeliefInterface &b, Index jaI) const;
    double GetQ(const JointBeliefInterface &b, Index t, Index jaI) const;

    void Save(const std::string &filename) const;

    void Load(const std::string &filename);

    /// Returns the total number of alpha-vectors stored (summed over all time steps).
    size_t GetNrVectors() const;

    void SetResultsFilename(const std::string &filename);

    virtual std::string SoftPrintBrief() const = 0;

};


#endif /* !_MONAHANPLANNER_H_ */

// Local Variables: ***
// mode:c++ ***
// End: ***
