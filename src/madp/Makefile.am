# $Id$

include $(top_srcdir)/src/Makefile.custom
include $(top_srcdir)/config/Make-rules

lib_LTLIBRARIES = libMADP.la
libMADP_la_SOURCES =
# Dummy C++ source to cause C++ linking.
#nodist_EXTRA_libMADP_la_SOURCES = dummy.cxx
libMADP_la_LIBADD = \
  ../planning/libMADPPlanning.la \
  ../support/libMADPSupport.la \
  ../parser/libMADPParser.la \
  ../base/libMADPBase.la \
  ../libDAI/libDAI.la 
#if !DARWIN
#libMADP_la_LIBADD += ../libpomdp-solve/src/libPOMDPSolve.la
#endif

libMADP_la_LDFLAGS = -release $(PACKAGE_VERSION)

