dnl @synopsis AC_CPLEX([ACTION-IF-FOUND[, ACTION-IF-NOT-FOUND]])
dnl
dnl This macro looks for a library that implements the CPLEX
dnl optimization library (see http://www.ilog.com/products/cplex/).  On 
dnl success, it sets the CPLEX_LIBS output variable to hold the requisite 
dnl library linkages.
dnl
dnl The user may use the following configuration options to setup
dnl cplex:
dnl 	--with-cplex-includes=<include-dir> 
dnl 	--with-cplex-libs=<link-libraries> 
dnl 	--with-cplex-ldflags=<link-flags> 
dnl
dnl ACTION-IF-FOUND is a list of shell commands to run if a CPLEX
dnl library is found, and ACTION-IF-NOT-FOUND is a list of commands
dnl to run it if it is not found.  If ACTION-IF-FOUND is not specified,
dnl the default action will define HAVE_CPLEX.
dnl

AC_DEFUN([AC_CPLEX], [

dnl default value:
ac_cplex_ok=no

dnl specify arguments:
dnl --from lx_check_cplex.m4--
  AC_ARG_WITH([cplex],
dnl AS_HELP_STRING([--with-cplex@<:@=PREFIX@:>@], [search for CPLEX under PREFIX or under the default search paths if PREFIX is not given @<:@default@:>@])
AS_HELP_STRING([--without-cplex], [disable checking for CPLEX]),
              [], [with_cplex=yes])

dnl --NOT USED--
dnl  AC_ARG_WITH([cplex-includedir],
dnl  AS_HELP_STRING([--with-cplex-includedir=DIR], [search for CPLEX headers in DIR]),
dnl              [], [with_cplex_includedir=no])
dnl
dnl  AC_ARG_WITH([cplex-libdir],
dnl  AS_HELP_STRING([--with-cplex-libdir=DIR], [search for CPLEX libraries in DIR]),
dnl              [], [with_cplex_libdir=no])

dnl --original arguments---
AC_ARG_WITH(cplex-includes,
        AS_HELP_STRING([--with-cplex-includes=<include-flags>], [include flags for CPLEX library headers. E.g., "-I$CPLEX_LOCATION/cplex/include/ -I$CPLEX_LOCATION/concert/include/" ])
        )
AC_ARG_WITH(cplex-libs,
        AS_HELP_STRING([--with-cplex-libs=<link-libraries>], [link with these CPLEX libraries, E.g., "-lilocplex -lconcert -lcplex" ])
        )
AC_ARG_WITH(cplex-ldflags,
        AS_HELP_STRING([--with-cplex-ldflags=<link-flags>], [link with these CPLEX flags. E.g., "-L$CPLEX_LOCATION/cplex/lib/x86-64_sles10_4.1/static_pic -L$CPLEX_LOCATION/concert/lib/x86-64_sles10_4.1/static_pic" ]))


if test x"$with_cplex" != x"no"; then
    AC_MSG_CHECKING([for CPLEX])


    dnl --- Note, doing something like the following would be convenient (i.e., specifying one dir and then testing for /include, /lib
    dnl     but since we need cplex and concert, it will become a bit messy... Therefore just let the user specify INCLUDES, LIBS, and LD_FLAGS
    dnl    if test x"$with_cplex_includedir" != x"no"; then
    dnl      CPLEX_CFLAGS="-I$with_cplex_includedir"
    dnl    elif test x"$with_cplex" != x"yes"; then
    dnl      CPLEX_CFLAGS="-I$with_cplex/include"
    dnl    elif test x"$CPLEX_INCLUDEDIR" != x; then
    dnl      CPLEX_CFLAGS="-I$CPLEX_INCLUDEDIR"
    dnl    fi
    dnl    if test x"$with_cplex_libdir" != x"no"; then
    dnl      CPLEX_LDFLAGS="-L$with_cplex_libdir"
    dnl    elif test x"$with_cplex" != x"yes"; then
    dnl      CPLEX_LDFLAGS="-L$with_cplex/lib"
    dnl    elif test x"$CPLEX_LIBDIR" != x; then
    dnl      CPLEX_LDFLAGS="-L$CPLEX_LIBDIR"
    dnl    fi
    dnl
    dnl CPLEX_LIBS="-lcplex -lm -lpthread"
    dnl FAO: we use "-lilocplex -lconcert -lcplex"
    dnl --end parse new arguments--


    dnl --INCLUDES--
    case $with_cplex_includes in
            yes | "") ;;
            no) ac_cplex_ok=disable ;;
            *) CPLEX_INCLUDES="$with_cplex_includes" ;;
    esac
    dnl --location of LIBS ---
    case $with_cplex_libs in
            yes | "") ;;
            no) ac_cplex_ok=disable ;;
            *) CPLEX_LIBS="$with_cplex_libs" ;;
    esac
    dnl --LD_FLAGS--
    dnl NOTE: need to include -lm -lpthread too
    case $with_cplex_ldflags in
            yes | "") ;;
            no) ac_cplex_ok=disable ;;
            *) CPLEX_LDFLAGS="-lm -lpthread $with_cplex_ldflags" ;;
    esac





dnl# First, check CPLEX_LIBS environment variable
dnl    if test "x$CPLEX_LIBS" = x; then :; else
dnl            save_LIBS="$LIBS"; LIBS="$CPLEX_LIBS $LIBS"
dnl            AC_MSG_CHECKING([for $cplex in $CPLEX_LIBS])
dnl            AC_TRY_LINK_FUNC($cplex, [ac_cplex_ok=yes], [CPLEX_LIBS=""])
dnl            AC_MSG_RESULT($ac_cplex_ok)
dnl            LIBS="$save_LIBS"
dnl            if test ac_cplex_ok = no; then
dnl                    CPLEX_INCLUDES=""
dnl                    CPLEX_LIBS=""
dnl                    CPLEX_LDFLAGS=""
dnl            fi
dnl    fi

    dnl later?
    dnl AC_SUBST(CPLEX_INCLUDES)
    dnl AC_SUBST(CPLEX_LIBS)
    dnl AC_SUBST(CPLEX_LDFLAGS)


    #---compile test program---
    lx_save_cxxflags="$CXXFLAGS"
    lx_save_ldflags="$LDFLAGS"
    lx_save_libs="$LIBS"
    dnl CXXFLAGS="$CPLEX_CFLAGS"
    CXXFLAGS="$CXXFLAGS $CPLEX_INCLUDES"
    LDFLAGS="$CPLEX_LDFLAGS"
    LIBS="$CPLEX_LIBS"

    cplex_test_prog='

      //C version:
      extern "C" {
      #include <ilcplex/cplex.h>    
      }

      //C++ version
      #include <iostream>
      #define IL_STD                //<- tell CPLEX to use STL
      #include <ilcplex/ilocplex.h> //<- fao: this is what we use
      ILOSTLBEGIN

      int main(int argc, char** argv)
      {
        //C version:
        CPXENVptr env = NULL;

        //C++ version
        //fao: some stuff we use in our code
        IloEnv env2;
        IloModel lpModel (env2);

        return 0;
      }'

    AC_LANG_PUSH(C++)
    AC_LINK_IFELSE([AC_LANG_SOURCE([$cplex_test_prog])], [cplex_found=yes], [cplex_found=no])
    AC_LANG_POP(C++)

    CXXFLAGS="$lx_save_cxxflags"
    LDFLAGS="$lx_save_ldflags"
    LIBS="$lx_save_libs"
    #---end compile test program---

    # Finally, execute ACTION-IF-FOUND/ACTION-IF-NOT-FOUND:
    if test x"$cplex_found" = x"yes"; then
            #cplex is found...        
            AC_DEFINE(HAVE_CPLEX,1,[Define if you have CPLEX library.])            
            AC_MSG_RESULT([yes])
    else
            #cplex is not found.
            AC_MSG_RESULT([no])
            AC_MSG_NOTICE([WARNING: CPLEX not found, will not be able to use certain functionality!])
    fi

fi dnl END if test x"$with_cplex" != x"no"; then

AC_SUBST(CPLEX_INCLUDES)
AC_SUBST(CPLEX_LIBS)
AC_SUBST(CPLEX_LDFLAGS)

AM_CONDITIONAL([HAVE_CPLEX], [test x"$lx_cplex_found" = x"yes"])

])dnl ACX_CPLEX
