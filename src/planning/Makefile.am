include $(top_srcdir)/src/Makefile.custom
include $(top_srcdir)/config/Make-rules

##############
# Includedirs, libdirs, libs and cflags for all programs 
INCLUDEPATH		= -I$(top_srcdir)/src/base\
 -I$(top_srcdir)/src/include -I$(top_srcdir)/src/support\
 -I$(top_srcdir)/src/planning\
 -I$(top_srcdir)/src/parser \
 $(MADP_INCLUDES) 
AM_LDFLAGS		= 
AM_CPPFLAGS		= -Wall -pipe $(MARCH_FLAGS) $(INCLUDEPATH)
if DARWIN
AM_CPPFLAGS += -DDARWIN
endif
AM_CXXFLAGS 		= $(CSTANDARD)

SIMULATION_CPPFILES=SimulationResult.cpp\
 SimulationDecPOMDPDiscrete.cpp \
 SimulationTOIDecPOMDPDiscrete.cpp \
 SimulationFactoredDecPOMDPDiscrete.cpp \
 AgentPOMDP.cpp\
 AgentBG.cpp\
 AgentQMDP.cpp\
 AgentRandom.cpp\
 AgentMDP.cpp\
 AgentOnlinePlanningMDP.cpp\
 AgentQLearner.cpp

SIMULATION_HFILES=$(SIMULATION_CPPFILES:.cpp=.h) \
 Simulation.h\
 SimulationAgent.h \
 AgentDecPOMDPDiscrete.h\
 AgentDelayedSharedObservations.h\
 AgentFullyObservable.h\
 AgentSharedObservations.h\
 AgentLocalObservations.h\
 AgentTOIFullyObservableSyncedSpecialReward.h\
 AgentTOIFullyObservableSynced.h


GMAA_CPPFILES=\
 GeneralizedMAAStarPlanner.cpp \
 GeneralizedMAAStarPlannerForDecPOMDPDiscrete.cpp \
 GeneralizedMAAStarPlannerForFactoredDecPOMDPDiscrete.cpp \
 GMAA_MAAstar.cpp\
 GMAA_MAAstarClassic.cpp\
 GMAA_MAAstarCluster.cpp\
 GMAA_kGMAACluster.cpp\
 PolicyPoolJPolValPair.cpp\
 PolicyPoolPartialJPolValPair.cpp\
 JPPVValuePair.cpp \
 JPPVIndexValuePair.cpp\
 PartialJPDPValuePair.cpp\
 PartialJPPVIndexValuePair.cpp\
 GMAA_kGMAA.cpp \
 GMAA_MAA_ELSI.cpp 

GMAA_HFILES=$(GMAA_CPPFILES:.cpp=.h)\
 PolicyPoolInterface.h\
 PartialPolicyPoolInterface.h\
 PartialPolicyPoolItemInterface.h\
 PolicyPoolItemInterface.h


POMDP_CPPFILES=\
 BeliefValue.cpp\
 BeliefSetNonStationary.cpp\
 AlphaVector.cpp \
 AlphaVectorPlanning.cpp\
 AlphaVectorPruning.cpp\
 Perseus.cpp \
 AlphaVectorPOMDP.cpp\
 AlphaVectorConstrainedPOMDP.cpp\
 PerseusPOMDPPlanner.cpp\
 PerseusConstrainedPOMDPPlanner.cpp\
 MonahanPOMDPPlanner.cpp \
 AlphaVectorBG.cpp\
 MonahanBGPlanner.cpp\
 MonahanPlanner.cpp \
 QMonahanPOMDP.cpp \
 PerseusBGPlanner.cpp \
 QMonahanBG.cpp \
 QAlphaVector.cpp \
 TreeIncPruneBGPlanner.cpp\
 QBGPlanner_TreeIncPruneBnB.cpp\
 QTreeIncPruneBG.cpp\
 PerseusBGPOMDPPlanner.cpp \
 AlphaVectorWeighted.cpp\
 PerseusWeightedPlanner.cpp \
 PerseusQFunctionPlanner.cpp\
 PerseusStationary.cpp\
 PerseusNonStationary.cpp\
 PerseusNonStationaryQPlanner.cpp\
 PerseusBGNSPlanner.cpp


POMDP_HFILES=$(POMDP_CPPFILES:.cpp=.h)\
 ValueFunctionPOMDPDiscrete.h\
 BGBackupType.h\
 PerseusBackupType.h\
 BeliefSet.h \
 QAV.h QAVParameters.h

PLANNING_CPPFILES=\
 BruteForceSearchPlanner.cpp\
 JESPExhaustivePlanner.cpp\
 JESPDynamicProgrammingPlanner.cpp\
 DICEPSPlanner.cpp\
 QBG.cpp QPOMDP.cpp QMDP.cpp\
 MDPSolver.cpp\
 MDPValueIteration.cpp \
 MaxPlusSolver.cpp\
 MaxPlusSolverForBGs.cpp\
 BGIP_SolverBruteForceSearch.cpp\
 BGIP_SolverBFSNonIncremental.cpp\
 BGIP_SolverBranchAndBound.cpp\
 BGIP_BnB_Node.cpp\
 BnB_JointTypeOrdering.cpp\
 BayesianGameIdenticalPayoffSolver.cpp\
 BayesianGameIdenticalPayoffSolver_T.cpp\
 BGIP_SolverAlternatingMaximization.cpp \
 BGIP_SolverCE.cpp \
 BGIP_SolverMaxPlus.cpp \
 BGIP_SolverRandom.cpp \
 BGIP_SolverCreator_MP.cpp \
 BGCG_Solver.cpp\
 BGCG_SolverNonserialDynamicProgramming.cpp\
 BGCG_SolverMaxPlus.cpp\
 BGCG_SolverRandom.cpp\
 BGCG_SolverCreator.cpp \
 BGCG_SolverFG.cpp\
 BG_FactorGraphCreator.cpp\
 FG_Solver.cpp\
 FG_SolverMaxPlus.cpp\
 FG_SolverNDP.cpp\
 QFunction.cpp\
 QFunctionForDecPOMDP.cpp\
 QFunctionForFactoredDecPOMDP.cpp\
 QFunctionJAOHInterface.cpp\
 QFunctionJAOH.cpp\
 QFunctionJAOHTree.cpp\
 QFunctionJointBelief.cpp \
 QHybrid.cpp \
 qheur.cpp\
 gmaatype.cpp\
 JESPtype.cpp\
 BGIP_SolverType.cpp\
 BayesianGameBase.cpp \
 BayesianGame.cpp \
 BayesianGameIdenticalPayoff.cpp\
 BayesianGameCollaborativeGraphical.cpp\
 BayesianGameForDecPOMDPStage.cpp\
 BGforStageCreation.cpp\
 BayesianGameForDecPOMDPStageInterface.cpp\
 BayesianGameWithClusterInfo.cpp\
 JointPolicyPureVectorForClusteredBG.cpp\
 TypeCluster.cpp\
 Type.cpp\
 Type_PointerTuple.cpp\
 Type_AOHIndex.cpp\
 LocalBGValueFunctionVector.cpp\
 LocalBGValueFunctionInterface.cpp\
 LocalBGValueFunctionBGCGWrapper.cpp\
 BGIPSolution.cpp\
 ValueFunctionDecPOMDPDiscrete.cpp\
 FactoredQLastTimeStepOrElse.cpp\
 FactoredQLastTimeStepOrQMDP.cpp \
 FactoredQLastTimeStepOrQPOMDP.cpp\
 FactoredQLastTimeStepOrQBG.cpp \
 OptimalValueDatabase.cpp\
 argumentHandlers.cpp\
 argumentUtils.cpp\
 directories.cpp 

PLANNING_HFILES=$(PLANNING_CPPFILES:.cpp=.h) \
 argumentHandlersPostChild.h\
 JointPolicyValuePair.h\
 PartialJointPolicyValuePair.h\
 ValueFunction.h\
 QFunctionInterface.h\
 QFunctionForDecPOMDPInterface.h\
 QFunctionForFactoredDecPOMDP.h\
 QFunctionForFactoredDecPOMDPInterface.h\
 QFunctionJAOHInterface.h\
 QFunctionJointBeliefInterface.h\
 FactoredQFunctionStateJAOHInterface.h\
 BGIP_SolverCreatorInterface_T.h\
 BGIP_IncrementalSolverCreatorInterface_T.h\
 BGIP_SolverCreatorInterface.h\
 BGIP_SolverCreator_AM.h\
 BGIP_SolverCreator_MP.h\
 BGIP_SolverCreator_CE.h\
 BGIP_SolverCreator_BFS.h\
 BGIP_SolverCreator_BFSNonInc.h\
 BGIP_SolverCreator_BnB.h\
 BGIP_SolverCreator_Random.h\
 BGCG_SolverCreator_NDP.h \
 BGCG_SolverCreator_MP.h \
 BGCG_SolverCreator_FG.h \
 BGCG_SolverCreator_Random.h \
 BayesianGameIdenticalPayoffInterface.h \
 BGIP_IncrementalSolverInterface_T.h \
 BGIP_IncrementalSolverInterface.h\
 OnlineMDPPlanner.h

PLANNING_FILES=$(PLANNING_CPPFILES) $(PLANNING_HFILES)\
 $(SIMULATION_CPPFILES) $(SIMULATION_HFILES)\
 $(POMDP_CPPFILES) $(POMDP_HFILES)\
 $(GMAA_CPPFILES) $(GMAA_HFILES)

###############
# All DYNAMIC libraries
# the LTLIBRARIES (LibTool-libraries)

# the library to be build...
lib_LTLIBRARIES = 
noinst_LTLIBRARIES = libMADPPlanning.la

# sources for the library:
libMADPPlanning_la_SOURCES = $(PLANNING_FILES)
libMADPPlanning_la_CPPFLAGS = $(AM_CPPFLAGS) $(CPP_OPTIMIZATION_FLAGS)
libMADPPlanning_la_LDFLAGS = $(AM_LDFLAGS)
libMADPPlanning_la_LIBADD = 
# for installing the headers
libMADPPlanning_ladir = $(pkgincludedir)#/planning
libMADPPlanning_la_HEADERS = $(PLANNING_HFILES) $(SIMULATION_HFILES)\
 $(POMDP_HFILES) $(GMAA_HFILES)
