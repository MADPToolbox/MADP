# $Id: Makefile.am 9372 2015-03-10 18:27:08Z mtjspaan $

include $(top_srcdir)/src/Makefile.custom
include $(top_srcdir)/config/Make-rules

##############
# Includedirs, libdirs, libs and cflags for all programs 
INCLUDES		= -I$(top_srcdir)/src/base\
 -I$(top_srcdir)/src/include\
 $(MADP_INCLUDES) 
AM_LDFLAGS		= 
LDADD			= 
AM_CPPFLAGS		= -Wall -pipe $(MARCH_FLAGS)
if DARWIN
AM_CPPFLAGS += -DDARWIN
endif
AM_CXXFLAGS 		= $(CSTANDARD)


GENERAL_CPPFILES=\
 NamedDescribedEntity.cpp\
 IndexTools.cpp\
 Globals.cpp \
 VectorTools.cpp\
 TimeTools.cpp\
 StringTools.cpp
GENERAL_HFILES=$(GENERAL_CPPFILES:.cpp=.h)\
 PrintTools.h\
 EInvalidIndex.h\
 ENotCached.h\
 ENoSubScope.h\
 EParse.h\
 EOverflow.h\
 E.h\
 EDeadline.h\
 Referrer.h\
 DiscreteEntity.h 

#the MADP files should be the more basic data types.
MADP_CPPFILES=JointActionDiscrete.cpp JointObservationDiscrete.cpp\
 TransitionModelDiscrete.cpp ObservationModelDiscrete.cpp\
 TransitionModelMapping.cpp ObservationModelMapping.cpp\
 RewardModelMapping.cpp RewardModelMappingSparse.cpp\
 RewardModelTOISparse.cpp \
 RewardModelMappingSparseMapped.cpp \
 MultiAgentDecisionProcess.cpp\
 MultiAgentDecisionProcessDiscrete.cpp\
 TransitionObservationIndependentMADPDiscrete.cpp TOIDecPOMDPDiscrete.cpp\
 TOIDecMDPDiscrete.cpp TOIFactoredRewardDecPOMDPDiscrete.cpp\
 TOICompactRewardDecPOMDPDiscrete.cpp\
 MADPComponentDiscreteActions.cpp MADPComponentDiscreteObservations.cpp\
 MADPComponentDiscreteStates.cpp DecPOMDP.cpp DecPOMDPDiscrete.cpp\
 POMDPDiscrete.cpp \
 POSG.cpp POSGDiscrete.cpp\
 FactoredDecPOMDPDiscrete.cpp \
 FactoredMMDPDiscrete.cpp \
 MultiAgentDecisionProcessDiscreteFactoredStates.cpp \
 TwoStageDynamicBayesianNetwork.cpp \
 MADPComponentFactoredStates.cpp \
 StateFactorDiscrete.cpp \
 TransitionModelMappingSparse.cpp\
 ObservationModelMappingSparse.cpp \
 CPT.cpp\
 Scope.cpp\
 FactoredQFunctionScopeForStage.cpp\
 FactoredStateDistribution.cpp FSDist_COF.cpp\
 EventObservationModelMapping.cpp\
 EventObservationModelMappingSparse.cpp

MADP_HFILES=$(MADP_CPPFILES:.cpp=.h) Action.h Observation.h State.h Agent.h\
 ActionDiscrete.h ObservationDiscrete.h StateDiscrete.h\
 JointAction.h JointObservation.h\
 MultiAgentDecisionProcessInterface.h\
 MultiAgentDecisionProcessDiscreteInterface.h \
 DecPOMDPInterface.h\
 DecPOMDPDiscreteInterface.h\
 RewardModelDiscreteInterface.h\
 TransitionModelDiscreteInterface.h\
 ObservationModelDiscreteInterface.h\
 POSGInterface.h POSGDiscreteInterface.h\
 RewardModel.h ObservationModel.h TransitionModel.h\
 MultiAgentDecisionProcessDiscreteFactoredStatesInterface.h\
 FactoredDecPOMDPDiscreteInterface.h\
 CPDDiscreteInterface.h\
 CPDKroneckerDelta.h\
 PDDiscreteInterface.h\
 QTableInterface.h\
 StateDistribution.h StateDistributionVector.h\
 TGet.h RGet.h OGet.h

BASE_FILES=$(MADP_CPPFILES) $(MADP_HFILES)\
 $(GENERAL_CPPFILES) $(GENERAL_HFILES)

###############
# All DYNAMIC libraries
# the LTLIBRARIES (LibTool-libraries)

# the library to be build...
lib_LTLIBRARIES =  
noinst_LTLIBRARIES =  libMADPBase.la

# sources for the library:
libMADPBase_la_SOURCES = $(BASE_FILES) 
libMADPBase_la_CPPFLAGS = $(AM_CPPFLAGS) $(CPP_OPTIMIZATION_FLAGS)
libMADPBase_la_LDFLAGS = $(AM_LDFLAGS) 
# next two lines are need to install the headers files
libMADPBase_ladir = $(pkgincludedir)#/base
libMADPBase_la_HEADERS = $(MADP_HFILES) $(GENERAL_HFILES)

