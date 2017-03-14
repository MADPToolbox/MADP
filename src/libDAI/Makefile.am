# $Id: Makefile.am 2415 2008-03-04 18:20:04Z faolieho $

# Don't build the directories with binaries by default, they need
# an extra boost lib (program_options9
#SUBDIRS = #utils tests MADP-integr-tests

include $(top_srcdir)/src/Makefile.custom
include $(top_srcdir)/config/Make-rules


#-------------------------------------------------------
# Excerpt from original libDAI Makefile - automake incompatible

#WITH_BP = true
#WITH_TRW = true
#WITH_MF = true
#WITH_HAK = true
#WITH_LC = true
#WITH_LCLIN = true
#WITH_TREEEP = true
#WITH_JTREE = true
#WITH_IJGP = true
#WITH_LCBPLIN = 
#WITH_LCBP1 = 
#WITH_LCBP2 = 
#WITH_LCBP3 = 
#WITH_LCBP4 = 
#WITH_LCBP5 = 
#WITH_MR =
#WITH_GMR =
#DEBUG =
#NEW_MATLAB = true
#WITH_MATLAB = true

#CCFLAGS = -Wall -fpic -I. -I./experimental/ #-pg -pedantic -Wextra
#BOOSTFLAGS = -lboost_program_options
#GSLFLAGS = -lgsl -lgslcblas

#ifdef DEBUG
#CCFLAGS := $(CCFLAGS) -g -DDEBUG
#else
#CCFLAGS := $(CCFLAGS) -O3
#endif

#OBJECTS := exact.o
#ifdef WITH_BP
#CCFLAGS := $(CCFLAGS) -DWITH_BP
#OBJECTS := $(OBJECTS) bp.o
#endif
#ifdef WITH_TRW
#CCFLAGS := $(CCFLAGS) -DWITH_TRW
#OBJECTS := $(OBJECTS) trw.o
#endif
#ifdef WITH_MF
#CCFLAGS := $(CCFLAGS) -DWITH_MF
#OBJECTS := $(OBJECTS) mf.o
#endif
#ifdef WITH_HAK
#CCFLAGS := $(CCFLAGS) -DWITH_HAK
#OBJECTS := $(OBJECTS) hak.o
#endif
#ifdef WITH_LC
#CCFLAGS := $(CCFLAGS) -DWITH_LC
#OBJECTS := $(OBJECTS) lc.o
#endif
#ifdef WITH_LCLIN
#CCFLAGS := $(CCFLAGS) -DWITH_LCLIN
#OBJECTS := $(OBJECTS) lclin.o
#endif
#ifdef WITH_TREEEP
#CCFLAGS := $(CCFLAGS) -DWITH_TREEEP
#OBJECTS := $(OBJECTS) treeep.o
#endif
#ifdef WITH_JTREE
#CCFLAGS := $(CCFLAGS) -DWITH_JTREE
#OBJECTS := $(OBJECTS) jtree.o
#endif
#ifdef WITH_IJGP
#CCFLAGS := $(CCFLAGS) -DWITH_IJGP
#OBJECTS := $(OBJECTS) ijgp.o
#endif
#ifdef WITH_LCBPLIN
#CCFLAGS := $(CCFLAGS) -DWITH_LCBPLIN
#OBJECTS := $(OBJECTS) lcbplin.o
#endif
#ifdef WITH_LCBP1
#CCFLAGS := $(CCFLAGS) -DWITH_LCBP1
#OBJECTS := $(OBJECTS) lcbp1.o
#endif
#ifdef WITH_LCBP2
#CCFLAGS := $(CCFLAGS) -DWITH_LCBP2
#OBJECTS := $(OBJECTS) lcbp2.o
#endif
#ifdef WITH_LCBP3
#CCFLAGS := $(CCFLAGS) -DWITH_LCBP3
#OBJECTS := $(OBJECTS) lcbp3.o
#endif
#ifdef WITH_LCBP4
#CCFLAGS := $(CCFLAGS) -DWITH_LCBP4
#OBJECTS := $(OBJECTS) lcbp4.o
#endif
#ifdef WITH_LCBP5
#CCFLAGS := $(CCFLAGS) -DWITH_LCBP5
#OBJECTS := $(OBJECTS) lcbp5.o
#endif
#ifdef WITH_MR
#CCFLAGS := $(CCFLAGS) -DWITH_MR
#OBJECTS := $(OBJECTS) mr.o
#endif
#ifdef WITH_GMR
#CCFLAGS := $(CCFLAGS) -DWITH_GMR
#OBJECTS := $(OBJECTS) gmr.o
#endif
#-------------------------------------------------------

#This also causes automake to complain:
#CCFLAGS=$(CCFLAGS) -DWITH_BP
#CCFLAGS=$(CCFLAGS) -DWITH_TRW
#CCFLAGS=$(CCFLAGS) -DWITH_MF
#CCFLAGS=$(CCFLAGS) -DWITH_HAK
#CCFLAGS=$(CCFLAGS) -DWITH_LC
#CCFLAGS=$(CCFLAGS) -DWITH_LCLIN
#CCFLAGS=$(CCFLAGS) -DWITH_TREEEP
#CCFLAGS=$(CCFLAGS) -DWITH_JTREE
#CCFLAGS=$(CCFLAGS) -DWITH_IJGP
#CCFLAGS=$(CCFLAGS) -DWITH_LCBPLIN
#CCFLAGS=$(CCFLAGS) -DWITH_LCBP1
#CCFLAGS=$(CCFLAGS) -DWITH_LCBP2
#CCFLAGS=$(CCFLAGS) -DWITH_LCBP3
#CCFLAGS=$(CCFLAGS) -DWITH_LCBP4
#CCFLAGS=$(CCFLAGS) -DWITH_LCBP5
#CCFLAGS=$(CCFLAGS) -DWITH_MR
#CCFLAGS=$(CCFLAGS) -DWITH_GMR

#CCFLAGS= -DWITH_BP -DWITH_TRW -DWITH_MF -DWITH_HAK -DWITH_LC -DWITH_LCLIN -DWITH_TREEEP -DWITH_JTREE -DWITH_IJGP -DWITH_LCBPLIN -DWITH_LCBP1 -DWITH_LCBP2 -DWITH_LCBP3 -DWITH_LCBP4 -DWITH_LCBP5 -DWITH_MR -DWITH_GMR

#not all files are included
CCFLAGS= -DWITH_BP -DWITH_TRW -DWITH_MF -DWITH_HAK -DWITH_LC -DWITH_LCLIN -DWITH_TREEEP -DWITH_JTREE -DWITH_IJGP -DWITH_MAXPLUS


##############
# Includedirs, libdirs, libs and cflags for all programs 
INCLUDEPATH		= -I. $(MADP_INCLUDES)
AM_LDFLAGS		= 
LDADD			= 
AM_CPPFLAGS		= -pipe $(MARCH_FLAGS) $(CCFLAGS) $(INCLUDEPATH)
AM_CXXFLAGS 		= $(CSTANDARD)


DAI_CPPFILES=alldai.cpp bounds.cpp boxbp.cpp box.cpp bp.cpp clustergraph.cpp daialg.cpp exact.cpp exceptions.cpp factorgraph.cpp hak.cpp ijgp.cpp jtree.cpp lc.cpp lclin.cpp mf.cpp properties.cpp regiongraph.cpp treeep.cpp trw.cpp util.cpp weightedgraph.cpp \
	     maxplus.cpp ndp.cpp MADP_util.cpp
#clustergraph_old.cpp

#example.cpp

#old (v 0.2)
#alldai.cpp bp.cpp clustergraph.cpp daialg.cpp example.cpp factorgraph.cpp hak.cpp jtree.cpp lc.cpp mf.cpp mr.cpp properties.cpp regiongraph.cpp treeep.cpp util.cpp weightedgraph.cpp x2x.cpp

DAI_HFILES=$(DAI_CPPFILES:.cpp=.h) var.h varset.h prob.h index.h graphicalmodel.h generic.h factor.h enum.h diffs.h bipgraph.h

DAI_FILES=$(DAI_CPPFILES) $(DAI_HFILES)

EXTRA_DIST = COPYING FILEFORMAT README tree 

###############
# All DYNAMIC libraries
# the LTLIBRARIES (LibTool-libraries)

# the library to be build...
lib_LTLIBRARIES = 
noinst_LTLIBRARIES =  libDAI.la

# sources for the library:
libDAI_la_SOURCES = $(DAI_FILES) 
libDAI_la_CPPFLAGS = $(AM_CPPFLAGS) $(CPP_OPTIMIZATION_FLAGS)
libDAI_la_LDFLAGS = $(AM_LDFLAGS) 
libDAI_ladir = $(pkgincludedir)#/libDAI
libDAI_la_HEADERS = $(DAI_HFILES)


# this gives dependency problems... (it happens that example is compiled before
# the library, resulting in :
# "No rule to make target `../../src/libDAI/libDAI.la', needed by `example'. "
# )
#
#PROGRAMS_NORMAL = example
#PROGRAMS_DEBUG = 
##PROGRAMS_PROFILE = 
#
##programs to be installed
#bin_PROGRAMS = 
##programs not to be installed
#noinst_PROGRAMS = $(PROGRAMS_NORMAL) $(PROGRAMS_DEBUG) 
#    #$(PROGRAMS_PROFILE)
#
#
##build example
#example_SOURCES = example.cpp
#example_LDADD = $(LIBDAI_NORMAL)
#example_DEPENDENCIES =  $(LIBDAI_NORMAL)
#example_CPPFLAGS= $(AM_CPPFLAGS) $(CPP_OPTIMIZATION_FLAGS) 
#example_CXXFLAGS=
#example_CFLAGS=
#


