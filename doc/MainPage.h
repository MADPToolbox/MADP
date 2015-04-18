// $Id: MainPage.h 9056 2014-04-30 08:43:26Z faolieho $

// This file contains the text of the first page of the
// Doxygen-generated documentation. The file is called .h so doxygen
// will process it.

/**
\mainpage MultiAgentDecisionProcess Reference Documentation

\section intro Introduction

%MultiAgentDecisionProcess (MADP) is a toolbox for scientific research
in decision-theoretic planning and learning in multiagent systems.  It
is designed to be rather general, but most effort has been put in
planning algorithms for discrete Dec-POMDPs.

The PDF \c doc/MADPToolbox.pdf provides more general background about
MADP models, and documents general design principles and details about
indices and history representations.

Authors: Frans Oliehoek, Matthijs Spaan, Philipp Robbel, João Messias

\section libs MADP Libraries

The framework consists of several parts, grouped in different
libraries. The base library (libMADPBase) contains:

\li A representation of the basic elements in a decision process such
as states, (joint) actions and observations. See State, StateDiscrete,
Action, ActionDiscrete, Observation, ObservationDiscrete, JointAction,
JointActionDiscrete, JointObservation, JointObservationDiscrete,
Agent.

\li A representation of the transition, observation and reward models
in a multiagent decision process. These models can also be stored in a
sparse fashion. See TransitionModelMapping,
TransitionModelMappingSparse, ObservationModelMapping,
ObservationModelMappingSparse, RewardModelMapping.

\li A uniform representation for MADP problems, which provides an
interface to a problem's model parameters. See
MultiAgentDecisionProcessInterface, MultiAgentDecisionProcessDiscreteInterface,
DecPOMDPDiscreteInterface, POSGDiscreteInterface, TransitionObservationIndependentMADPDiscrete, TOIDecMDPDiscrete, TOIDecPOMDPDiscrete, TOIFactoredRewardDecPOMDPDiscrete, TOICompactRewardDecPOMDPDiscrete.

\li Auxiliary functionality regarding manipulating indices, exception
handling and printing: E, IndexTools, PrintTools. Some project-wide
definitions are stored in the Globals namespace.

The parser library (libMADPParser) only requires the base library, and
contains:

\li A parser for \c dpomdp problem specifications, which is a
fileformat for discrete Dec-POMDPs. A set of benchmark problem files
can be found in the \c problems/ directory, and the \c dpomdp syntax
is documented in \link example.dpomdp \endlink. The format is based on
Tony's POMDP file format, and the formal specification is found in
dpomdp.spirit. The parser uses the Boost Spirit library. See
MADPParser.

\li Parsers ParserTOIDecMDPDiscrete, ParserTOIDecPOMDPDiscrete,
ParserTOIFactoredRewardDecPOMDPDiscrete, and
ParserTOICompactRewardDecPOMDPDiscrete, which are all derived from the
.dpomdp parser. These can be accessed in a uniform way by using
MADPParser.

The support library (libMADPSupport) contains basic data types and
support useful for planning:

\li A representation for (joint) histories, for storing and
manipulating observation, action and action-observation histories. See
ActionHistory, ObservationHistory, ActionObservationHistory,
JointActionHistory, JointObservationHistory,
JointActionObservationHistory.

\li A representation for (joint) beliefs, both stored as a full vector
as well as a sparse one. See JointBeliefInterface, JointBelief,
JointBeliefSparse.

\li Functionality for representing (joint) policies, as mappings from
histories to actions. See JointPolicyPureVector,
JointPolicyDiscretePure, JointPolicyDiscrete.

\li An implementation of the DecTiger problem which does not use \link
dectiger.dpomdp \endlink, see ProblemDecTiger. Also an implementation of ProblemFireFighting.

\li Shared functionality for discrete MADP planning algorithms,
collect in PlanningUnitMADPDiscrete and
PlanningUnitDecPOMDPDiscrete. Computes (joint) history trees, joint
beliefs, and value functions.

\li Functionality for handling command-line arguments is provided by
ArgumentHandlers.

Finally, the planning library (libMADPplanning) contains functionality
for planning algorithms, as well as some solution methods.

\li Dec-POMDP solution algorithms: BruteForceSearchPlanner,
JESPExhaustivePlanner, JESPDynamicProgrammingPlanner, DICEPSPlanner,
GMAA_kGMAA, and GMAA_MAAstar.

\li POMDP solution techniques: Perseus.

\li Functionality for building and solving Bayesian Games: see
BayesianGame and BayesianGameIdenticalPayoffInterface.

\li Heuristic Q-functions: QMDP, QPOMDP, and QBG.

\li A simple simulator to empirically test the control quality of a
solution, see SimulationDecPOMDPDiscrete.

\section programs Programs using the MADP libraries

In the \c src/examples/ and \c src/utils/ directories are a number
of programs included that use the MADP libraries. Running each binary
with as argument \c --help will display a short summary of usage.

\li \c JESP runs the JESPDynamicProgrammingPlanner on a \c dpomdp problem
specification, for instance 
\verbatim 
JESP -h 3 <PATH_TO>/dectiger.dpomdp
\endverbatim
or
\verbatim 
JESP -h 3 DT
\endverbatim
runs JESP for horizon 3 on the DecTiger problem. First one parses the
dectiger.dpomdp file, the second one uses the ProblemDecTiger
class. Many more problem files are provided in the \c problems/
directory.

\li \c BFS runs the BruteForceSearchPlanner, \c JESP the JESPExhaustivePlanner or JESPDynamicProgrammingPlanner, \c DICEPS the DICEPSPlanner, and \c GMAA runs the GMAA variations (MAAstar or Forward Search %Policy Sweep).

\li \c %Perseus runs the Perseus POMDP or BG planner.

\li \c printProblem loads a \c dpomdp problem description and prints
it to standard out. \c printJointPolicyPureVector prints out a
particular joint policy given its index.

\li \c evaluateJointPolicyPureVector simulates a particular joint
policy for a problem. \c evaluateRandomPolicy uses a policy that
chooses actions uniformly at random.

\li \c analyzeRewardResults and \c getAvgReward
print information about the expected reward of simulation runs, saved
using SimulationResult::Save().

\section acks Author Info and Acknowledgments

\htmlonly

\endhtmlonly
\verbinclude "AUTHORS"
*/
