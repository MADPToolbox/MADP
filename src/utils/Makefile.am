include $(top_srcdir)/src/Makefile.custom
include $(top_srcdir)/config/Make-rules

# Which programs to build. Only PROGRAMS_NORMAL will be installed.
PROGRAMS_NORMAL =  \
 analyzeTimings\
 evaluatePerseusPolicy\
 analyzeRewardResults\
 getAvgReward\
 printJointPolicyPureVector\
 evaluateRandomPolicy\
 printPolicyPureVector\
 policyPureVectorToDotGraph\
 evaluateJointPolicyPureVector\
 calculateQheuristic\
 printProblem\
 printProblemStats

##############
# Includedirs, libdirs, libs and cflags for all programs 
INCLUDEPATH		= -I..\
 -I$(top_srcdir)/src/base -I$(top_srcdir)/src/include\
 -I$(top_srcdir)/src/support -I$(top_srcdir)/src/planning\
 -I$(top_srcdir)/src/parser \
 $(MADP_INCLUDES)
AM_LDFLAGS		= 
LDADD			= 
AM_CPPFLAGS		= -Wall -pipe $(MARCH_FLAGS) $(INCLUDEPATH)
if DARWIN
AM_CPPFLAGS += -DDARWIN
endif

noinst_PROGRAMS = 
bin_PROGRAMS = $(PROGRAMS_NORMAL)

additional_sources = 


# Build analyzeTimings
analyzeTimings_SOURCES = analyzeTimings.cpp
analyzeTimings_LDADD = $(MADPLIBS_NORMAL_NOPARSER) $(MADP_LD)
analyzeTimings_LDFLAGS = $(AM_LDFLAGS) 
analyzeTimings_DEPENDENCIES = $(MADPLIBS_NORMAL_NOPARSER)
analyzeTimings_CPPFLAGS= $(AM_CPPFLAGS) $(CPP_OPTIMIZATION_FLAGS)
analyzeTimings_CXXFLAGS= $(CSTANDARD)
analyzeTimings_CFLAGS=

# Build evaluatePerseusPolicy
evaluatePerseusPolicy_SOURCES = evaluatePerseusPolicy.cpp
evaluatePerseusPolicy_LDADD = $(MADPLIBS_NORMAL) $(MADP_LD)
evaluatePerseusPolicy_LDFLAGS = $(AM_LDFLAGS) 
evaluatePerseusPolicy_DEPENDENCIES = $(MADPLIBS_NORMAL)
evaluatePerseusPolicy_CPPFLAGS= $(AM_CPPFLAGS) $(CPP_OPTIMIZATION_FLAGS)
evaluatePerseusPolicy_CXXFLAGS= $(CSTANDARD)
evaluatePerseusPolicy_CFLAGS=
# Build analyzeRewardResults
analyzeRewardResults_SOURCES = analyzeRewardResults.cpp
analyzeRewardResults_LDADD = $(MADPLIBS_NORMAL) $(MADP_LD)
analyzeRewardResults_LDFLAGS = $(AM_LDFLAGS) 
analyzeRewardResults_DEPENDENCIES = $(MADPLIBS_NORMAL_NOPARSER)
analyzeRewardResults_CPPFLAGS= $(AM_CPPFLAGS) $(CPP_OPTIMIZATION_FLAGS)
analyzeRewardResults_CXXFLAGS= $(CSTANDARD)
analyzeRewardResults_CFLAGS=

# Build getAvgReward
getAvgReward_SOURCES = getAvgReward.cpp
getAvgReward_LDADD = $(MADPLIBS_NORMAL) $(MADP_LD)
getAvgReward_LDFLAGS = $(AM_LDFLAGS) 
getAvgReward_DEPENDENCIES = $(MADPLIBS_NORMAL)
getAvgReward_CPPFLAGS= $(AM_CPPFLAGS) $(CPP_OPTIMIZATION_FLAGS)
getAvgReward_CXXFLAGS= $(CSTANDARD)
getAvgReward_CFLAGS=

# Build printJointPolicyPureVector
printJointPolicyPureVector_SOURCES = printJointPolicyPureVector.cpp
printJointPolicyPureVector_LDADD = $(MADPLIBS_NORMAL) $(MADP_LD)
printJointPolicyPureVector_LDFLAGS = $(AM_LDFLAGS) 
printJointPolicyPureVector_DEPENDENCIES = $(MADPLIBS_NORMAL)
printJointPolicyPureVector_CPPFLAGS= $(AM_CPPFLAGS) $(CPP_OPTIMIZATION_FLAGS)
printJointPolicyPureVector_CXXFLAGS= $(CSTANDARD)
printJointPolicyPureVector_CFLAGS=

# Build evaluateRandomPolicy
evaluateRandomPolicy_SOURCES = evaluateRandomPolicy.cpp
evaluateRandomPolicy_LDADD = $(MADPLIBS_NORMAL) $(MADP_LD)
evaluateRandomPolicy_LDFLAGS = $(AM_LDFLAGS) 
evaluateRandomPolicy_DEPENDENCIES = $(MADPLIBS_NORMAL)
evaluateRandomPolicy_CPPFLAGS= $(AM_CPPFLAGS) $(CPP_OPTIMIZATION_FLAGS)
evaluateRandomPolicy_CXXFLAGS= $(CSTANDARD)
evaluateRandomPolicy_CFLAGS=

# Build printPolicyPureVector
printPolicyPureVector_SOURCES = printPolicyPureVector.cpp
printPolicyPureVector_LDADD = $(MADPLIBS_NORMAL) $(MADP_LD)
printPolicyPureVector_LDFLAGS = $(AM_LDFLAGS) 
printPolicyPureVector_DEPENDENCIES = $(MADPLIBS_NORMAL)
printPolicyPureVector_CPPFLAGS= $(AM_CPPFLAGS) $(CPP_OPTIMIZATION_FLAGS)
printPolicyPureVector_CXXFLAGS= $(CSTANDARD)
printPolicyPureVector_CFLAGS=

# Build policyPureVectorToDotGraph
policyPureVectorToDotGraph_SOURCES = policyPureVectorToDotGraph.cpp
policyPureVectorToDotGraph_LDADD = $(MADPLIBS_NORMAL) $(MADP_LD)
policyPureVectorToDotGraph_LDFLAGS = $(AM_LDFLAGS) 
policyPureVectorToDotGraph_DEPENDENCIES = $(MADPLIBS_NORMAL)
policyPureVectorToDotGraph_CPPFLAGS= $(AM_CPPFLAGS) $(CPP_OPTIMIZATION_FLAGS)
policyPureVectorToDotGraph_CXXFLAGS= $(CSTANDARD)
policyPureVectorToDotGraph_CFLAGS=

# Build evaluateJointPolicyPureVector
evaluateJointPolicyPureVector_SOURCES = evaluateJointPolicyPureVector.cpp
evaluateJointPolicyPureVector_LDADD = $(MADPLIBS_NORMAL) $(MADP_LD)
evaluateJointPolicyPureVector_LDFLAGS = $(AM_LDFLAGS) 
evaluateJointPolicyPureVector_DEPENDENCIES = $(MADPLIBS_NORMAL)
evaluateJointPolicyPureVector_CPPFLAGS= $(AM_CPPFLAGS) $(CPP_OPTIMIZATION_FLAGS)
evaluateJointPolicyPureVector_CXXFLAGS= $(CSTANDARD)
evaluateJointPolicyPureVector_CFLAGS=

# Build calculateQheuristic
calculateQheuristic_SOURCES = calculateQheuristic.cpp
calculateQheuristic_LDADD = $(MADPLIBS_NORMAL) $(MADP_LD)
calculateQheuristic_LDFLAGS = $(AM_LDFLAGS) 
calculateQheuristic_DEPENDENCIES = $(MADPLIBS_NORMAL)
calculateQheuristic_CPPFLAGS= $(AM_CPPFLAGS) $(CPP_OPTIMIZATION_FLAGS)
calculateQheuristic_CXXFLAGS= $(CSTANDARD)
calculateQheuristic_CFLAGS=

# Build printProblem
printProblem_SOURCES = printProblem.cpp
printProblem_LDADD = $(MADPLIBS_NORMAL) $(MADP_LD)
printProblem_LDFLAGS = $(AM_LDFLAGS) 
printProblem_DEPENDENCIES = $(MADPLIBS_NORMAL)
printProblem_CPPFLAGS= $(AM_CPPFLAGS) $(CPP_OPTIMIZATION_FLAGS)
printProblem_CXXFLAGS= $(CSTANDARD)
printProblem_CFLAGS=

# Build printProblemStats
printProblemStats_SOURCES = printProblemStats.cpp
printProblemStats_LDADD = $(MADPLIBS_NORMAL) $(MADP_LD)
printProblemStats_LDFLAGS = $(AM_LDFLAGS) 
printProblemStats_DEPENDENCIES = $(MADPLIBS_NORMAL)
printProblemStats_CPPFLAGS= $(AM_CPPFLAGS) $(CPP_OPTIMIZATION_FLAGS)
printProblemStats_CXXFLAGS= $(CSTANDARD)
printProblemStats_CFLAGS=

