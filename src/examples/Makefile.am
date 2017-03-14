# $Id: Makefile.am 3635 2009-08-27 13:33:33Z mtjspaan $

include $(top_srcdir)/src/Makefile.custom
include $(top_srcdir)/config/Make-rules

# Which programs to build. These example programs are not installed.
PROGRAMS_NORMAL = \
example_JESP \
example_BFS \
example_decTigerJESP \
example_DICEPS \
example_MMDP_SolveAndSimulate \
example_RandomBGs \
example_FOBS_SPUDD_export \
example_MMDP_OnlineSolve \
example_GMAA \
example_Perseus



##############
# Includedirs, libdirs, libs and cflags for all programs 
INCLUDE_PATH		= -I..\
 -I$(top_srcdir)/src/base -I$(top_srcdir)/src/include\
 -I$(top_srcdir)/src/support -I$(top_srcdir)/src/planning\
 -I$(top_srcdir)/src/parser \
 $(MADP_INCLUDES) 
AM_LDFLAGS		= 
LDADD			= 
AM_CPPFLAGS		= -pipe $(MARCH_FLAGS) $(INCLUDE_PATH)

if DARWIN
AM_CPPFLAGS += -DDARWIN
endif

noinst_PROGRAMS = $(PROGRAMS_NORMAL)
bin_PROGRAMS = 

# Build example_JESP
example_JESP_SOURCES = example_JESP.cpp
example_JESP_LDADD = $(MADPLIBS_NORMAL) $(MADP_LD)
example_JESP_LDFLAGS = $(AM_LDFLAGS) 
example_JESP_DEPENDENCIES = $(MADPLIBS_NORMAL)
example_JESP_CPPFLAGS= $(AM_CPPFLAGS) $(CPP_OPTIMIZATION_FLAGS)
example_JESP_CXXFLAGS= $(CSTANDARD)
example_JESP_CFLAGS=

# Build example_BFS
example_BFS_SOURCES = example_BFS.cpp
example_BFS_LDADD = $(MADPLIBS_NORMAL) $(MADP_LD)
example_BFS_LDFLAGS = $(AM_LDFLAGS) 
example_BFS_DEPENDENCIES = $(MADPLIBS_NORMAL)
example_BFS_CPPFLAGS= $(AM_CPPFLAGS) $(CPP_OPTIMIZATION_FLAGS)
example_BFS_CXXFLAGS= $(CSTANDARD)
example_BFS_CFLAGS=

# Build example_decTigerJESP
example_decTigerJESP_SOURCES = example_decTigerJESP.cpp
example_decTigerJESP_LDADD = $(MADPLIBS_NORMAL) $(MADP_LD)
example_decTigerJESP_LDFLAGS = $(AM_LDFLAGS) 
example_decTigerJESP_DEPENDENCIES = $(MADPLIBS_NORMAL)
example_decTigerJESP_CPPFLAGS= $(AM_CPPFLAGS) $(CPP_OPTIMIZATION_FLAGS)
example_decTigerJESP_CXXFLAGS= $(CSTANDARD)
example_decTigerJESP_CFLAGS=

# Build example_GMAA
example_GMAA_SOURCES = example_GMAA.cpp
example_GMAA_LDADD = $(MADPLIBS_NORMAL) $(MADP_LD)
example_GMAA_LDFLAGS = $(AM_LDFLAGS) 
example_GMAA_DEPENDENCIES = $(MADPLIBS_NORMAL)
example_GMAA_CPPFLAGS= $(AM_CPPFLAGS) $(CPP_OPTIMIZATION_FLAGS)
example_GMAA_CXXFLAGS= $(CSTANDARD)
example_GMAA_CFLAGS=

# Build example_DICEPS
example_DICEPS_SOURCES = example_DICEPS.cpp
example_DICEPS_LDADD = $(MADPLIBS_NORMAL) $(MADP_LD)
example_DICEPS_LDFLAGS = $(AM_LDFLAGS) 
example_DICEPS_DEPENDENCIES = $(MADPLIBS_NORMAL)
example_DICEPS_CPPFLAGS= $(AM_CPPFLAGS) $(CPP_OPTIMIZATION_FLAGS)
example_DICEPS_CXXFLAGS= $(CSTANDARD)
example_DICEPS_CFLAGS=

# Build example_Perseus
example_Perseus_SOURCES = example_Perseus.cpp
example_Perseus_LDADD = $(MADPLIBS_NORMAL) $(MADP_LD)
example_Perseus_LDFLAGS = $(AM_LDFLAGS) 
example_Perseus_DEPENDENCIES = $(MADPLIBS_NORMAL)
example_Perseus_CPPFLAGS= $(AM_CPPFLAGS) $(CPP_OPTIMIZATION_FLAGS)
example_Perseus_CXXFLAGS= $(CSTANDARD)
example_Perseus_CFLAGS=

# Build example_MMDP_SolveAndSimulate
example_MMDP_SolveAndSimulate_SOURCES = example_MMDP_SolveAndSimulate.cpp
example_MMDP_SolveAndSimulate_LDADD = $(MADPLIBS_NORMAL) $(MADP_LD)
example_MMDP_SolveAndSimulate_LDFLAGS = $(AM_LDFLAGS) 
example_MMDP_SolveAndSimulate_DEPENDENCIES = $(MADPLIBS_NORMAL)
example_MMDP_SolveAndSimulate_CPPFLAGS= $(AM_CPPFLAGS) $(CPP_OPTIMIZATION_FLAGS)
example_MMDP_SolveAndSimulate_CXXFLAGS= $(CSTANDARD)
example_MMDP_SolveAndSimulate_CFLAGS=

# Build example_MMDP_OnlineSolve
example_MMDP_OnlineSolve_SOURCES = example_MMDP_OnlineSolve.cpp
example_MMDP_OnlineSolve_LDADD = $(MADPLIBS_NORMAL) $(MADP_LD)
example_MMDP_OnlineSolve_LDFLAGS = $(AM_LDFLAGS) 
example_MMDP_OnlineSolve_DEPENDENCIES = $(MADPLIBS_NORMAL)
example_MMDP_OnlineSolve_CPPFLAGS= $(AM_CPPFLAGS) $(CPP_OPTIMIZATION_FLAGS)
example_MMDP_OnlineSolve_CXXFLAGS= $(CSTANDARD)
example_MMDP_OnlineSolve_CFLAGS=

# Build example_FOBS_SPUDD_export
example_FOBS_SPUDD_export_SOURCES = example_FOBS_SPUDD_export.cpp
example_FOBS_SPUDD_export_LDADD = $(MADPLIBS_NORMAL) $(MADP_LD)
example_FOBS_SPUDD_export_LDFLAGS = $(AM_LDFLAGS) 
example_FOBS_SPUDD_export_DEPENDENCIES = $(MADPLIBS_NORMAL)
example_FOBS_SPUDD_export_CPPFLAGS= $(AM_CPPFLAGS) $(CPP_OPTIMIZATION_FLAGS)
example_FOBS_SPUDD_export_CXXFLAGS= $(CSTANDARD)
example_FOBS_SPUDD_export_CFLAGS=

# Build example_RandomBGs
example_RandomBGs_SOURCES = example_RandomBGs.cpp
example_RandomBGs_LDADD = $(MADPLIBS_NORMAL) $(MADP_LD)
example_RandomBGs_LDFLAGS = $(AM_LDFLAGS) 
example_RandomBGs_DEPENDENCIES = $(MADPLIBS_NORMAL)
example_RandomBGs_CPPFLAGS= $(AM_CPPFLAGS) $(CPP_OPTIMIZATION_FLAGS)
example_RandomBGs_CXXFLAGS= $(CSTANDARD)
example_RandomBGs_CFLAGS=

###############
