include $(top_srcdir)/src/Makefile.custom
include $(top_srcdir)/config/Make-rules

##############
# Includedirs, libdirs, libs and cflags for all programs 
INCLUDEPATH		=  -I$(top_srcdir)/src/base\
 -I$(top_srcdir)/src/include\
 $(MADP_INCLUDES) 
AM_LDFLAGS		= 
LDADD			=
AM_CPPFLAGS		= $(MADP_CPPFLAGS) $(INCLUDEPATH) 
AM_CXXFLAGS 		= $(MADP_CXXFLAGS)
AM_CFLAGS 		= $(MADP_CFLAGS)
if DARWIN
AM_CPPFLAGS += -DDARWIN
endif

PARSER_CPPFILES=\
 ParserDPOMDPFormat_Spirit.cpp\
 ParserPOMDPFormat_Spirit.cpp\
 ParserTOIDecPOMDPDiscrete.cpp\
 ParserTOIDecMDPDiscrete.cpp\
 ParserTOIFactoredRewardDecPOMDPDiscrete.cpp\
 ParserTOICompactRewardDecPOMDPDiscrete.cpp\
 ParserProbModelXML.cpp\
 ParserPOMDPDiscrete.cpp\
 MADPParser.cpp 

PARSER_HFILES=$(PARSER_CPPFILES:.cpp=.h) ParserInterface.h\
 CommentOrBlankParser.h\
 CommentBlankOrNewlineParser.h

PARSER_FILES=$(PARSER_CPPFILES) $(PARSER_HFILES)

###############
# All DYNAMIC libraries
# the LTLIBRARIES (LibTool-libraries)

# the library to be build...
lib_LTLIBRARIES =  
noinst_LTLIBRARIES =  libMADPParser.la

# sources for the library:
libMADPParser_la_SOURCES = $(PARSER_FILES) 
libMADPParser_la_CPPFLAGS = $(AM_CPPFLAGS) $(CPP_OPTIMIZATION_FLAGS)
libMADPParser_la_LIBADD = 
libMADPParser_la_DEPENDENCIES =
# for installing the headers
libMADPParser_ladir = $(pkgincludedir)#/parser
libMADPParser_la_HEADERS = $(PARSER_HFILES)

