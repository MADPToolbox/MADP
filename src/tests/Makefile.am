# $Id$

# Lists all the subdirs where 'make' has to look
# When changing the subdirs, also change AC_CONFIG_FILES in configure.in
SUBDIRS =

include ../Makefile.custom
include $(top_srcdir)/config/Make-rules

# lib of the Boost Test Library
BOOSTTEST_LD=-lboost_unit_test_framework

# Which programs to build.
PROGRAMS_NORMAL = \
 tst_parse\
 tst_np\
 tst_bgipsolving\
 tst_GMAA_MAAstar\
 tst_GMAA_MAAstarCluster\
 tst_GMAA_MAA_ELSI\
 tst_ProblemFireFightingGraph\
 tst_BGIP_Solvers\
 tst_BGIP_IncrementalSolvers\
 tst_jpol_index\
 tst_serialize\
 tst_CGBG_FF\
 tst_OptimalValue\
 tst_pomdp\
 tst_sim

###########
# All test programs which will be run by 'make check'
check_PROGRAMS =\
 tst_jpol_index\
 tst_serialize\
 tst_OptimalValue\
 tst_jpol_index\
 tst_serialize
dist_check_SCRIPTS =\
 runGMAA-GMAAstarClassic-QQMDP_DecTiger.sh\
 runGMAA-GMAAstar-BBFS-QQPOMDP-Aloha.sh\
 runGMAA-IC-GMAAstarClassic-GBnB-QQBG-FFG.sh\
 runGMAA-IC-GMAAstar-BBnb-QQMDPc-FFF.sh\
 runGMAA_ICE-GMAAstar-BBFS-QQPOMDPhybrid-GridSmall.sh\
 runGMAA-ICE-GMAAstar-BBnB-QQBGav-2generals.sh\
 runGMAA-GMAAstarClassic-QQBG-dectiger_skewed.sh\
 runGMAA-GMAAstarClassic-QQBGhybrid-recycling.sh\
 runGMAA-GMAAstarClassic-QQBGTreeIncPrune-relay4.sh\
 runGMAA-IC-GMAAstar-QQBGTreeIncPrune-dectiger.sh\
 runGMAA-IC-GMAAstar-QQBGav-prisoners.sh\
 runGMAA-GMAAstar-BBFS-QQMDPc-broadcastChannel.sh\
 runGMAA-ELSI-QQBG-FFG.sh\
 runGMAA-ICE-GMAAstar-DTPGMX.sh\
 runGMAA-ICE-GMAAstar-FFG343.sh\
 runGMAA-GFSPC-BMP-ALoha.sh\
 runPerseus-b0-DT.sh\
 runPerseus-b1-DT.sh\
 runDICEPS-twoCorridors.sh\
 runMMDP_QLearner_DT.sh\
 runBFS-DT.sh\
 tst_parse.sh\
 tst_np.sh\
 tst_BGIP_Solvers.sh

TESTS = $(check_PROGRAMS) $(dist_check_SCRIPTS)

EXTRA_DIST = logs optimalValueDatabase $(dist_check_SCRIPTS) # add to distribution for test purposes

PROGRAMS_DEBUG =

PROGRAMS_PROFILE =

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

# Don't install any of the test programs
noinst_PROGRAMS = $(PROGRAMS_NORMAL) $(PROGRAMS_DEBUG) $(PROGRAMS_PROFILE)
bin_PROGRAMS = 

additional_test_sources =

# Build tst
tst_parse_SOURCES =  test_parse.cpp $(additional_test_sources) test_common_functions.h
tst_parse_LDADD = $(MADPLIBS_NORMAL) $(MADP_LD)
tst_parse_DEPENDENCIES = $(MADPLIBS_NORMAL)
tst_parse_CPPFLAGS= $(AM_CPPFLAGS) $(CPP_OPTIMIZATION_FLAGS) 
tst_parse_CXXFLAGS= $(CSTANDARD)
tst_parse_CFLAGS=

tst_BGIP_Solvers_SOURCES =   test_BGIP_Solvers.cpp $(additional_test_sources)
tst_BGIP_Solvers_LDADD = $(MADPLIBS_NORMAL) $(MADP_LD)
tst_BGIP_Solvers_DEPENDENCIES = $(MADPLIBS_NORMAL)
tst_BGIP_Solvers_CPPFLAGS= $(AM_CPPFLAGS) $(CPP_OPTIMIZATION_FLAGS)
tst_BGIP_Solvers_CXXFLAGS= $(CSTANDARD)
tst_BGIP_Solvers_CFLAGS=

tst_CGBG_FF_SOURCES =   test_CGBG_FF.cpp $(additional_test_sources)
tst_CGBG_FF_LDADD = $(MADPLIBS_NORMAL) $(MADP_LD)
tst_CGBG_FF_DEPENDENCIES = $(MADPLIBS_NORMAL)
tst_CGBG_FF_CPPFLAGS= $(AM_CPPFLAGS) $(CPP_OPTIMIZATION_FLAGS)
tst_CGBG_FF_CXXFLAGS= $(CSTANDARD)
tst_CGBG_FF_CFLAGS=

# Create tst without the parsing
tst_np_SOURCES = test_no_parse.cpp $(additional_test_sources) test_common_functions.h
tst_np_LDADD = $(MADPLIBS_NORMAL_NOPARSER) $(MADP_LD)
tst_np_DEPENDENCIES = $(MADPLIBS_NORMAL_NOPARSER)
tst_np_CPPFLAGS= $(AM_CPPFLAGS) $(CPP_OPTIMIZATION_FLAGS)
tst_np_CXXFLAGS= $(CSTANDARD)
tst_np_CFLAGS=

# Build tst_sim
tst_sim_SOURCES =  test_simulation.cpp $(additional_test_sources)
tst_sim_LDADD = $(MADPLIBS_NORMAL) $(MADP_LD)
tst_sim_DEPENDENCIES = $(MADPLIBS_NORMAL)
tst_sim_CPPFLAGS= $(AM_CPPFLAGS) $(CPP_OPTIMIZATION_FLAGS)
tst_sim_CXXFLAGS= $(CSTANDARD)
tst_sim_CFLAGS=

# Build tst_pomdp
tst_pomdp_SOURCES =  test_pomdp.cpp $(additional_test_sources)
tst_pomdp_LDADD = $(MADPLIBS_NORMAL) $(MADP_LD)
tst_pomdp_DEPENDENCIES = $(MADPLIBS_NORMAL)
tst_pomdp_CPPFLAGS= $(AM_CPPFLAGS) $(CPP_OPTIMIZATION_FLAGS)
tst_pomdp_CXXFLAGS= $(CSTANDARD)
tst_pomdp_CFLAGS=

# Build tst_serialize
tst_serialize_SOURCES = test_serialize.cpp
tst_serialize_LDADD = -lboost_serialization 
tst_serialize_LDFLAGS = $(AM_LDFLAGS)
tst_serialize_DEPENDENCIES = 
tst_serialize_CPPFLAGS= $(AM_CPPFLAGS) $(CPP_OPTIMIZATION_FLAGS)
tst_serialize_CXXFLAGS= $(CSTANDARD)
tst_serialize_CFLAGS=

# tst_GMAA_MAAstarCluster
tst_GMAA_MAAstarCluster_SOURCES =  test_GMAA_MAAstarCluster.cpp $(additional_test_sources)
tst_GMAA_MAAstarCluster_LDADD = $(MADPLIBS_NORMAL) $(MADP_LD)
tst_GMAA_MAAstarCluster_LDFLAGS = $(AM_LDFLAGS)
tst_GMAA_MAAstarCluster_DEPENDENCIES = $(MADPLIBS_NORMAL)
tst_GMAA_MAAstarCluster_CPPFLAGS= $(AM_CPPFLAGS) -O0 
tst_GMAA_MAAstarCluster_CXXFLAGS= $(CSTANDARD)
tst_GMAA_MAAstarCluster_CFLAGS=

# tst_GMAA_MAAstar
tst_GMAA_MAAstar_SOURCES =  test_GMAA_MAAstar.cpp $(additional_test_sources)
tst_GMAA_MAAstar_LDADD = $(MADPLIBS_NORMAL) $(MADP_LD)
tst_GMAA_MAAstar_LDFLAGS = $(AM_LDFLAGS)
tst_GMAA_MAAstar_DEPENDENCIES = $(MADPLIBS_NORMAL)
tst_GMAA_MAAstar_CPPFLAGS= $(AM_CPPFLAGS) -O0
tst_GMAA_MAAstar_CXXFLAGS= $(CSTANDARD)
tst_GMAA_MAAstar_CFLAGS=

# tst_OptimalValue
tst_OptimalValue_SOURCES =  test_OptimalValue.cpp $(additional_test_sources)
tst_OptimalValue_LDADD = $(MADPLIBS_NORMAL) $(MADP_LD)
tst_OptimalValue_LDFLAGS = $(AM_LDFLAGS)
tst_OptimalValue_DEPENDENCIES = $(MADPLIBS_NORMAL)
tst_OptimalValue_CPPFLAGS= $(AM_CPPFLAGS) -O0
tst_OptimalValue_CXXFLAGS= $(CSTANDARD)
tst_OptimalValue_CFLAGS=

# tst_GMAA_MAA_ELSI
tst_GMAA_MAA_ELSI_SOURCES =  test_GMAA_MAA_ELSI.cpp $(additional_test_sources)
tst_GMAA_MAA_ELSI_LDADD = $(MADPLIBS_NORMAL) $(MADP_LD)
tst_GMAA_MAA_ELSI_LDFLAGS = $(AM_LDFLAGS) 
tst_GMAA_MAA_ELSI_DEPENDENCIES = $(MADPLIBS_NORMAL)
tst_GMAA_MAA_ELSI_CPPFLAGS= $(AM_CPPFLAGS) $(CPP_OPTIMIZATION_FLAGS)
tst_GMAA_MAA_ELSI_CXXFLAGS= $(CSTANDARD)
tst_GMAA_MAA_ELSI_CFLAGS=

# test program for bgip solving
tst_bgipsolving_SOURCES =  test_bgipsolving.cpp $(additional_test_sources)
tst_bgipsolving_LDADD = $(MADPLIBS_NORMAL) $(MADP_LD)
tst_bgipsolving_DEPENDENCIES = $(MADPLIBS_NORMAL)
tst_bgipsolving_CPPFLAGS= $(AM_CPPFLAGS) $(CPP_OPTIMIZATION_FLAGS)
tst_bgipsolving_CXXFLAGS= $(CSTANDARD)
tst_bgipsolving_CFLAGS=

# test program for ProblemFireFightingGraph
tst_ProblemFireFightingGraph_SOURCES =  test_ProblemFireFightingGraph.cpp
tst_ProblemFireFightingGraph_LDADD = $(MADPLIBS_NORMAL) $(MADP_LD)
tst_ProblemFireFightingGraph_DEPENDENCIES = $(MADPLIBS_NORMAL)
tst_ProblemFireFightingGraph_CPPFLAGS= $(AM_CPPFLAGS) $(CPP_OPTIMIZATION_FLAGS)
tst_ProblemFireFightingGraph_CXXFLAGS= $(CSTANDARD)
tst_ProblemFireFightingGraph_CFLAGS=

tst_BGIP_IncrementalSolvers_SOURCES =   test_BGIP_IncrementalSolvers.cpp $(additional_test_sources)
tst_BGIP_IncrementalSolvers_LDADD = $(MADPLIBS_NORMAL) $(MADP_LD)
tst_BGIP_IncrementalSolvers_DEPENDENCIES = $(MADPLIBS_NORMAL)
tst_BGIP_IncrementalSolvers_CPPFLAGS= $(AM_CPPFLAGS) $(CPP_OPTIMIZATION_FLAGS)
tst_BGIP_IncrementalSolvers_CXXFLAGS= $(CSTANDARD)
tst_BGIP_IncrementalSolvers_CFLAGS=

tst_jpol_index_SOURCES =   test_jpol_index.cpp $(additional_test_sources)
tst_jpol_index_LDADD = $(MADPLIBS_NORMAL) $(MADP_LD)
tst_jpol_index_DEPENDENCIES = $(MADPLIBS_NORMAL)
tst_jpol_index_CPPFLAGS= $(AM_CPPFLAGS) $(CPP_OPTIMIZATION_FLAGS)
tst_jpol_index_CXXFLAGS= $(CSTANDARD)
tst_jpol_index_CFLAGS=

###############
# All DYNAMIC libraries
# the LTLIBRARIES (LibTool-libraries)

# the library to be build...
lib_LTLIBRARIES =
noinst_LTLIBRARIES =

# sources for the library:
###############
# All STATIC libraries
lib_LIBRARIES	=

debug:	$(PROGRAMS_DEBUG)

normal:	$(PROGRAMS_NORMAL)	

profile:$(PROGRAMS_PROFILE)	
