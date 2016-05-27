/* REPLACE_MADP_HEADER */
/* REPLACE_CONTRIBUTING_AUTHORS_START
 * Frans Oliehoek 
 * Matthijs Spaan 
 * REPLACE_CONTRIBUTING_AUTHORS_END
 */

/* Only include this header file once. */
#ifndef _FSDIST_COF_H_
#define _FSDIST_COF_H_ 1

/* the include directives */
#include "Globals.h"
#include "FactoredStateDistribution.h"

class MADPComponentFactoredStates;
class MultiAgentDecisionProcessDiscreteFactoredStatesInterface;
class Scope;

/** \brief FSDist_COF is a class that represents a completely factored state
 * distribution. I.e., a distribution represented as the product of marginal
 * state factor probabilities.
 * */
class FSDist_COF : public FactoredStateDistribution
{
private:   

    ///The number of state factors
    size_t _m_nrStateFactors;

    ///Vector with size of the domain of each state factor (the nr. values)
    /**This is used to compute state indices.*/
    std::vector<size_t> _m_sfacDomainSizes;
    ///Array caching the stepsize - used for computing indices.
    /**Computed during initialization.*/
    size_t* _m_stepSize;

    ///_m_probs[sfacI][valI] contains probability of valI for SF sfacI.
    std::vector< std::vector<double> > _m_probs;
    
protected:
    
public:
    // Constructor, destructor and copy assignment.
    /// Constructor without arguments, needed for serialization.
    FSDist_COF();
    FSDist_COF(const MADPComponentFactoredStates& a);
    FSDist_COF(const MultiAgentDecisionProcessDiscreteFactoredStatesInterface& a);
    /// Copy constructor.
    FSDist_COF(const FSDist_COF& a);
    /// Destructor.
    virtual ~FSDist_COF();
    
    /// Copy assignment operator
    FSDist_COF& operator= (const FSDist_COF& o);

    //operators:

    //data manipulation (set) functions:
    virtual void SetZero();
    virtual void SetUniform();
        
    //get (data) functions:
    double& GetReferrence(Index sfacI, Index sfacValueI)
        { return _m_probs[sfacI][sfacValueI]; }
    
    virtual double GetProbability( Index sI) const;
    double GetProbability(const std::vector<Index>& sfacValues) const;
    double GetProbability(const Scope& sfSc, 
                          const std::vector<Index>& sfacValues) const;
    std::vector<Index> SampleState() const;

    void SetProbability(Index sfacI, Index valI, double prob)
        {
            _m_probs.at(sfacI).at(valI)=prob;
        }

    virtual std::string SoftPrint() const;
        
    virtual std::vector<double> ToVectorOfDoubles() const;

    virtual size_t GetNrStates() const;
        
    /// Returns a pointer to a copy of this class.
    virtual FSDist_COF* Clone() const
        { return new FSDist_COF(*this); }
    
    void SanityCheck();

    //Normalization of the distribution of a given state factor. The normalization constant can be an input, if known.
    virtual void Normalize(Index sfacI);
    void Normalize(Index sfacI, double sum);
};


#endif /* !_FSDIST_COF_H_ */

// Local Variables: ***
// mode:c++ ***
// End: ***
