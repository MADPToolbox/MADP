#because lp-solve-old contains a file called debug.c, it is not possible to 'make debug' in that dir...
NODEBUG_SUBDIRS = #lp-solve-old
DEBUG_SUBDIRS = #mdp #laspack 
SUBDIRS =  $(NODEBUG_SUBDIRS) $(DEBUG_SUBDIRS)

include $(top_srcdir)/src/Makefile.custom
include $(top_srcdir)/config/Make-rules

##############
# Includedirs, libdirs, libs and cflags for all programs 
INCLUDES		=   $(MADP_INCLUDES)  \
			    -I$(top_srcdir)/src/include \
			    -I$(top_srcdir)/src/libpomdp-solve/src \
			    -I$(top_srcdir)/src/libpomdp-solve/src/mdp -I$(top_srcdir)/src/libpomdp-solve/src/lp-solve-old 
# -I$(top_srcdir)/src/libpomdp-solve/src/laspack
#-I$(top_srcdir)/src/base\
#-I$(top_srcdir)/src/support \
#-I$(top_srcdir)/src/planning\
#-I$(top_srcdir)/src/parser 

AM_LDFLAGS		= 
AM_CPPFLAGS		= -Wall -pipe $(MARCH_FLAGS)

# Make sure to generate the header file, since the scanner will need it.
AM_YFLAGS = -d

# Assumed location of dmalloc.h

#src/libpomdp-solve/src/Makefile.am:34: `LDFLAGS' is a user variable, you should not override it;
#src/libpomdp-solve/src/Makefile.am:34: use `AM_LDFLAGS' instead.
#LDFLAGS = -L$(top_builddir)/src/libpomdp-solve/src/mdp -L$(top_builddir)/src/libpomdp-solve/src/lp-solve-old -L$(top_builddir)/src/libpomdp-solve/src/laspack

COMMON_LIBS = @LEXLIB@
#$(top_builddir)/src/libpomdp-solve/src/mdp/libmdp.a $(top_builddir)/src/libpomdp-solve/src/lp-solve-old/liblpk.a 

COMMON_SRC = alpha.h alpha.c belief.h belief.c\
 cross-sum.h cross-sum.c \
 global.h global.c \
 lp-interface.h lp-interface.c neighbor.h\
 neighbor.c params.h params.c\
 parsimonious.h parsimonious.c pg.h pg.c pomdp.h pomdp.c \
 projection.h projection.c random.h random.c region.h region.c\
 stats.h stats.c timing.h timing.c\
 xalloc.h xalloc.c \
 associative-array.h associative-array.c \
 config-file.h config-file.c\
 utils.c utils.h\
 pomdp-solve-options.c pomdp-solve-options.h\
 program-options.c program-options.h\
 command-line.c command-line.h \
 cmd-line.c cmd-line.h\
 common.c common.h\
 mdp/mdp-common.h mdp/imm-reward.c mdp/imm-reward.h mdp/mdp.c mdp/mdp.h mdp/parse_constant.h mdp/parse_err.c mdp/parse_err.h mdp/parse_hash.c mdp/parse_hash.h mdp/parser.y mdp/scanner.l mdp/sparse-matrix.c mdp/sparse-matrix.h\
 lp-solve-old/debug.c lp-solve-old/debug.h lp-solve-old/hash.c lp-solve-old/hash.h lp-solve-old/lpglob.h lp-solve-old/lpkit.c lp-solve-old/lpkit.h lp-solve-old/presolve.c lp-solve-old/solve.c
# lp-solve-old/debug.c lp-solve-old/debug.h lp-solve-old/hash.c lp-solve-old/hash.h lp-solve-old/lp.y lp-solve-old/lex.l lp-solve-old/lpglob.h lp-solve-old/lpkit.c lp-solve-old/lpkit.h lp-solve-old/presolve.c lp-solve-old/read.c lp-solve-old/read.h lp-solve-old/solve.c


#POMDP_SOLVE_SRC = $(COMMON_SRC) main.c

#POMDP_TOOLS_SRC = $(COMMON_SRC) utils.h utils.c pg-eval.h pg-eval.c laspack-util.h laspack-util.c pomdp-tools-options.h pomdp-tools-options.c pomdp-tools-main.c

#POMDP_FG_SRC = $(COMMON_SRC) pomdp-fg-main.c pomdp-fg.h pomdp-fg.c index-list.h index-list.h pomdp-fg-options.h pomdp-fg-options.c fg-params.h fg-params.c value-function.h value-function.c belief-state.h belief-state.c double-vector.h double-vector.c 


bin_PROGRAMS = #pomdp-solve pomdp-tools pomdp-fg
##### pomdp-solve
#pomdp_solve_SOURCES =  $(POMDP_SOLVE_SRC)

##### pomdp-tools
#pomdp_tools_SOURCES = $(POMDP_TOOLS_SRC)

##### pomdp-fg
#pomdp_fg_SOURCES = $(POMDP_FG_SRC)

# For testing via 'make check'
##check_PROGRAMS = test-pomdp-solve
##test_pomdp_solve_SOURCES = ...


noinst_LIBRARIES = 
#adding libDAI here doesn't work. _LIBADD is for adding libtool objects (.lo 
#or .la) not static debug libraries. -> debug binaries will have to add 
#_LDADD =  $(LIBDAI_DEBUG) 
#
#_a_LIBADD =

# the library to be build...
lib_LTLIBRARIES =  
noinst_LTLIBRARIES =  libPOMDPSolve.la


# sources for the library:
libPOMDPSolve_la_SOURCES = $(COMMON_SRC)
libPOMDPSolve_la_CPPFLAGS = $(AM_CPPFLAGS) $(CPP_OPTIMIZATION_FLAGS)
libPOMDPSolve_la_LDFLAGS = $(AM_LDFLAGS) 
libPOMDPSolve_la_LIBADD = $(LPSOLVEOLD_LIB) 
# for installing the headers
libPOMDPSolve_ladir = $(pkgincludedir)#/libpomdp-solve
libPOMDPSolve_la_HEADERS = $(POMDP_headers)

