#                                               -*- Autoconf -*-
# Process this file with autoconf to produce a configure script.

AC_PREREQ(2.61)
AC_INIT([MADP],[0.4.1],[madp-users@omni.isr.ist.utl.pt])
# subdir-objects is used in src/libpomdp-solve
AM_INIT_AUTOMAKE([1.9 tar-pax subdir-objects])
AC_CONFIG_SRCDIR([config.h.in])
AC_CONFIG_HEADER([config.h])
AC_CONFIG_MACRO_DIR([m4])

# Checks for programs.
AC_PROG_CXX
AC_PROG_CC
AC_PROG_CPP
AC_PROG_INSTALL
AC_PROG_LN_S
AC_PROG_MAKE_SET

# whether to use serialization:
#AC_ARG_ENABLE([serialization],
#              AS_HELP_STRING([--enable-serialization], [Enable boost serialization (Experimental, requires c++11 standard)]))
#AM_CONDITIONAL([USE_SERIALIZATION], [test x"$enable_serialization" = x"yes"])

#CSTANDARD:
#See https://gcc.gnu.org/onlinedocs/gcc/C-Dialect-Options.html#C-Dialect-Options
#
#'c++11'
#The 2011 ISO C++ standard plus amendments. The name ‘c++0x’ is deprecated.
#Important: GCC's support for C++11 is still experimental. Some features were implemented based on early proposals, and no attempt will be made to maintain backward compatibility when they are updated to match the final C++11 standard.
CSTD11="-std=c++11"
AC_SUBST(CSTD11)
#--leads to problems compiling boost 1.49
#  (problems occur when including the file needed for the ERF function...)
#
#'gnu++11'
#GNU dialect of -std=c++11. The name ‘gnu++0x’ is deprecated. 
CSTDGNU11="-std=gnu++11"
AC_SUBST(CSTDGNU11)
#
#‘gnu++98’, ‘gnu++03’
#GNU dialect of -std=c++98. This is the default for C++ code.  
#(i.e. default, if no C++ language dialect options are given, is -std=gnu++98.) 
CSTDGNU="-std=gnu++03" 
AC_SUBST(CSTDGNU)
#
#‘c++98’, ‘c++03’
#The 1998 ISO C++ standard plus the 2003 technical corrigendum and some additional defect reports. Same as -ansi for C++ code. 
#CSTANDARD="-std=c++03" 
CSTD="-std=c++98"
AC_SUBST(CSTD)
#
#overall standard we want to use:
#AC_SUBST(CSTANDARD,"-std=c++98")
#AC_SUBST(CSTANDARD,"-std=c++11")



#specified the flags in src/Makefile.am
# You can turn off shared libraries during beta-testing, since they
# make the build process take too long.
# AC_DISABLE_SHARED
# AC_DISABLE_STATIC
AC_PROG_LIBTOOL

#CHECK for standard c++ libs (and include the linkind directives...)
AC_CHECK_LIB([stdc++],[__cxa_free_exception])

# defining the libs doesn't work...
# Don't check nor include lp-solve anymore until Tony Cassandra's code
# has been updated for using the system-wide lp-solve
#AC_CHECK_HEADER(
#        [lpsolve/lp_types.h],
#        [AC_DEFINE([HAVE_LPSOLVE], [],[lpsolve available])])
#        [AC_DEFINE([HAVE_LPSOLVE], [],[lpsolve available]) LPSOLVE_LIB="/usr/lib/liblpsolve55_pic.a" LPSOLVE_LD="-lcolamd -ldl"])

# IMPORTANT; check for math lib before lpsolve, otherwise lpsolve will fail (it needs '-lm')
AC_CHECK_LIB([m], [main])
# check for lpsolve
AC_CHECK_HEADER(
        [lpsolve/lp_lib.h],
        [],
        [AC_MSG_WARN([lpsolve not installed, required for pruning POMDP value functions])])
AC_CHECK_LIB(dl, dlopen)
AC_CHECK_LIB(colamd, colamd)
AC_CHECK_LIB([lpsolve55_pic], [make_lp],
        [],
        [AC_MSG_WARN([lpsolve not installed, required for pruning POMDP value functions])])


# Checks for header files.
AC_HEADER_STDC
AC_HEADER_STDBOOL
AC_CHECK_HEADERS([float.h limits.h stdlib.h])
# Checks for typedefs, structures, and compiler characteristics.
AC_C_CONST
AC_C_INLINE
AC_TYPE_SIZE_T


#check for cplex:
AC_CPLEX

#check for cuda:
AX_CHECK_CUDA

# Checks for library functions.
AC_FUNC_ERROR_AT_LINE
AC_CHECK_FUNCS([pow strtoull])

# Allow the defined vars to come from a file instead of a slew of '-D'
# compiler flags.
#AM_CONFIG_HEADER([config.h])

## Option for using compile -g flag for debugging symbols
##
AM_CONDITIONAL(DEBUG, true)
AC_ARG_ENABLE(debug,
        [  --enable-debug      enable debugging symbols],
        [
                if test "x$enableval" = "xyes" ; then
                        AM_CONDITIONAL(DEBUG, true)
                fi
        ]
)
## Option for using compile -pg flag for profiling
##
AM_CONDITIONAL(PROFILE, false)
AC_ARG_ENABLE(profile,
        [  --enable-profile      enable debugging symbols and profiling],
        [
                if test "x$enableval" = "xyes" ; then
                        AM_CONDITIONAL(PROFILE, true)
                fi
        ]
)
## Option for using dmalloc memory leak checking library
##
AM_CONDITIONAL(DMALLOC, false)
AC_ARG_ENABLE(dmalloc,
        [  --enable-dmalloc      enable using dmalloc memory checker],
        [
                if test "x$enableval" = "xyes" ; then
                        AM_CONDITIONAL(DMALLOC, true)
                fi
        ]
)
# Checks for programs.
AC_PROG_CC
#AC_PROG_YACC
#AM_PROG_LEX


# Checks for libraries.
# FIXME: Replace `main' with a function in `-lcplex':
#AC_CHECK_LIB([cplex], [main])
# FIXME: Replace `main' with a function in `-lm':
AC_CHECK_LIB([m], [main])

# Checks for header files.
AC_HEADER_STDC
AC_CHECK_HEADERS([float.h limits.h stddef.h stdlib.h string.h sys/time.h unistd.h])

# Checks for typedefs, structures, and compiler characteristics.
AC_C_CONST
AC_TYPE_SIZE_T
AC_HEADER_TIME

# Checks for library functions.
AC_FUNC_ERROR_AT_LINE
AC_FUNC_MALLOC
AC_FUNC_REALLOC
AC_TYPE_SIGNAL
AC_FUNC_STRTOD
AC_FUNC_VPRINTF
AC_CHECK_FUNCS([bzero floor gettimeofday memset pow sqrt strstr])



#For OS-dependent compilation
AC_CANONICAL_HOST

case $host_os in
*darwin*)
	HOST_OS=darwin
	AC_MSG_WARN("Compiling on OSX. Some functionalities will not be available.")
	;;
#Other os-specific instructions can go here.
esac

AM_CONDITIONAL([DARWIN], [test x$HOST_OS = xdarwin])

#Small check for OSX systems where glibc ports may leave argp_parse undefined
AC_SEARCH_LIBS([argp_parse], [argp], [], [
  AC_MSG_ERROR([unable to find the argp_parse() function])
])

#Check for libxml2 (required for ParserProbModelXML)
AC_CHECK_LIB([xml2], [xmlInitParser])
AC_CHECK_HEADER([libxml2/libxml/xmlexports.h])
AM_CONDITIONAL(FOUND_LIBXML2, [test "x$ac_cv_header_libxml2_libxml_xmlexports_h" == xyes])

AC_CONFIG_FILES([Makefile
                 src/Makefile
                 src/parser/Makefile
                 src/base/Makefile
                 src/support/Makefile
                 src/planning/Makefile
                 src/madp/Makefile
                 src/utils/Makefile
                 src/examples/Makefile
                 src/libDAI/Makefile
                 src/solvers/Makefile
                 src/tests/Makefile
                 ])

#avoid ": compiling `test_gsl.c' with per-target flags requires `AM_PROG_CC_C_O' in `configure.ac'"
AM_PROG_CC_C_O

AC_OUTPUT

