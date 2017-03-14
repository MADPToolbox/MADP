include $(top_srcdir)/src/Makefile.custom
include $(top_srcdir)/config/Make-rules

##############
# Includedirs, libdirs, libs and cflags for all programs 
INCLUDEPATH		= -I$(top_srcdir)/src/base\
 -I$(top_srcdir)/src/include -I$(top_srcdir)/src/support \
 $(MADP_INCLUDES) 
AM_LDFLAGS		= 
LDADD			= 
AM_CPPFLAGS		= -pipe $(MARCH_FLAGS) $(INCLUDEPATH)

if DARWIN
AM_CPPFLAGS += -DDARWIN
endif
AM_CXXFLAGS 		= $(CSTANDARD)

HISTORY_CPPFILES= ObservationHistory.cpp ObservationHistoryTree.cpp\
 JointObservationHistory.cpp \
 ActionHistory.cpp JointActionHistory.cpp\
 JointActionObservationHistory.cpp JointActionObservationHistoryTree.cpp\
 ActionObservationHistory.cpp ActionObservationHistoryTree.cpp
HISTORY_HFILES=$(HISTORY_CPPFILES:.cpp=.h) TreeNode.h ActionHistoryTree.h\
 JointActionHistoryTree.h History.h IndividualHistory.h JointHistory.h\
 JointObservationHistoryTree.h

POLICY_CPPFILES= \
 PolicyDiscrete.cpp PolicyDiscretePure.cpp PolicyPureVector.cpp \
 JointPolicyDiscrete.cpp JointPolicyDiscretePure.cpp\
 PartialJointPolicy.cpp \
 PartialJointPolicyDiscretePure.cpp \
 PartialJointPolicyPureVector.cpp \
 JointPolicyPureVector.cpp \
 JPolComponent_VectorImplementation.cpp

POLICY_HFILES=$(POLICY_CPPFILES:.cpp=.h) Interface_ProblemToPolicyDiscrete.h \
 Interface_ProblemToPolicyDiscretePure.h Policy.h \
 JointPolicy.h PolicyGlobals.h

PROBLEM_CPPFILES= \
 ProblemType.cpp\
 ProblemDecTiger.cpp\
 ProblemDecTigerWithCreaks.cpp\
 ProblemFireFighting.cpp\
 ProblemFireFightingFactored.cpp\
 ProblemFireFightingGraph.cpp\
 ProblemFOBSFireFightingFactored.cpp\
 ProblemFOBSFireFightingGraph.cpp\
 ProblemAloha.cpp

PROBLEM_HFILES=$(PROBLEM_CPPFILES:.cpp=.h)

BELIEF_CPPFILES=Belief.cpp JointBelief.cpp JointBeliefEventDriven.cpp\
 BeliefSparse.cpp JointBeliefSparse.cpp \
 FactoredStateAOHDistribution.cpp\
 IndividualBeliefJESP.cpp\
 FSAOHDist_NECOF.cpp 

BELIEF_HFILES=$(BELIEF_CPPFILES:.cpp=.h)\
 JointBeliefInterface.h BeliefInterface.h BeliefIteratorInterface.h\
 BeliefIterator.h BeliefIteratorSparse.h BeliefIteratorGeneric.h

PLANNINGUNIT_CPPFILES=\
 PlanningUnit.h \
 PlanningUnitMADPDiscrete.cpp\
 PlanningUnitDecPOMDPDiscrete.cpp PlanningUnitMADPDiscreteParameters.cpp\
 PlanningUnitFactoredDecPOMDPDiscrete.cpp\
 PlanningUnitTOIDecPOMDPDiscrete.cpp \
 NullPlanner.cpp NullPlannerTOI.cpp NullPlannerFactored.cpp
PLANNINGUNIT_HFILES=$(PLANNINGUNIT_CPPFILES:.cpp=.h)

MISC_CPPFILES= Timing.cpp TimedAlgorithm.cpp \
 FixedCapacityPriorityQueue.cpp \
 MADPDiscreteStatistics.cpp\
 QTable.cpp
MISC_HFILES=$(MISC_CPPFILES:.cpp=.h) VectorSet.h

SUPPORT_FILES=$(POLICY_CPPFILES) $(POLICY_HFILES) \
 $(PROBLEM_CPPFILES) $(PROBLEM_HFILES)\
 $(HISTORY_CPPFILES) $(HISTORY_HFILES) \
 $(BELIEF_CPPFILES) $(BELIEF_HFILES)\
 $(PLANNINGUNIT_CPPFILES) $(PLANNINGUNIT_HFILES)\
 $(MISC_CPPFILES) $(MISC_HFILES)

###############
# All DYNAMIC libraries
# the LTLIBRARIES (LibTool-libraries)

# the library to be build...
lib_LTLIBRARIES = 
noinst_LTLIBRARIES =   libMADPSupport.la

# sources for the library:
libMADPSupport_la_SOURCES = $(SUPPORT_FILES) 
libMADPSupport_la_CPPFLAGS = $(AM_CPPFLAGS) $(CPP_OPTIMIZATION_FLAGS)
libMADPSupport_la_LDFLAGS = $(AM_LDFLAGS) 
# for installing the headers
libMADPSupport_ladir = $(pkgincludedir)#/support
libMADPSupport_la_HEADERS = $(POLICY_HFILES) $(PROBLEM_HFILES)\
 $(HISTORY_HFILES) $(BELIEF_HFILES) $(PLANNINGUNIT_HFILES) $(MISC_HFILES)
