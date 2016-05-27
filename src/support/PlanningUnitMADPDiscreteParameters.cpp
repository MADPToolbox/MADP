/* REPLACE_MADP_HEADER */
/* REPLACE_CONTRIBUTING_AUTHORS_START
 * Frans Oliehoek 
 * Matthijs Spaan 
 * REPLACE_CONTRIBUTING_AUTHORS_END
 */

#include "PlanningUnitMADPDiscreteParameters.h"

using namespace std;

//Default constructor
PlanningUnitMADPDiscreteParameters::PlanningUnitMADPDiscreteParameters()
{
    _m_individualObservationHistories=true;
    _m_individualActionHistories=true;
    _m_individualActionObservationHistories=true;
    _m_jointObservationHistories=true;
    _m_jointActionHistories=true;
    _m_jointActionObservationHistories=true;
    _m_JointBeliefs=true;
    _m_useSparseBeliefs=false;
    _m_eventObservability=false;
}

//Destructor
PlanningUnitMADPDiscreteParameters::~PlanningUnitMADPDiscreteParameters()
{
}

void PlanningUnitMADPDiscreteParameters::SetComputeAllJointHistories(bool val)
{
    SetComputeJointObservationHistories(val);
    SetComputeJointActionHistories(val);
    SetComputeJointActionObservationHistories(val);
}

void
PlanningUnitMADPDiscreteParameters::SetComputeAllIndividualHistories(bool val)
{
    SetComputeIndividualObservationHistories(val);
    SetComputeIndividualActionHistories(val);
    SetComputeIndividualActionObservationHistories(val);
}

void PlanningUnitMADPDiscreteParameters::SetComputeAll(bool val)
{
    SetComputeAllIndividualHistories(val);

    // order is important because of the sanity check
    if(val)
    {
        SetComputeAllJointHistories(val);
        SetComputeJointBeliefs(val);
    }
    else
    {
        SetComputeJointBeliefs(val);
        SetComputeAllJointHistories(val);
    }
}

void PlanningUnitMADPDiscreteParameters::Print() const
{
    cout << "ComputeIndividualObservationHistories: "
         << GetComputeIndividualObservationHistories() << endl;

    cout << "ComputeIndividualActionHistories: "
         << GetComputeIndividualActionHistories() << endl;

    cout << "ComputeIndividualActionObservationHistories: "
         << GetComputeIndividualActionObservationHistories() << endl;

    cout << "ComputeJointObservationHistories: "
         << GetComputeJointObservationHistories() << endl;

    cout << "ComputeJointActionHistories: "
         << GetComputeJointActionHistories() << endl;

    cout << "ComputeJointActionObservationHistories: "
         << GetComputeJointActionObservationHistories() << endl;

    cout << "ComputeJointBeliefs: "
         << GetComputeJointBeliefs() << endl;

    cout << "UseSparseJointBeliefs: "
         << GetUseSparseJointBeliefs() << endl;
}

void PlanningUnitMADPDiscreteParameters::SanityCheck() const
{
    if(GetComputeJointBeliefs() && !GetComputeJointActionObservationHistories())
    {
        Print();
        throw(E("PlanningUnitMADPDiscreteParameters::SanityCheck error, in order to compute joint beliefs all joint action observation histories also need to be generated"));
    }
}
