#!/bin/sh
#
# cleans up old style headers. Run from /src.
#
find base/ planning/ support/ solvers/ parser/ include/ examples/ utils/ -regextype sed -regex "\(.*\.cpp\|.*\.h\)" -print0 | xargs -0 sed -i '
    1,/#/ {  #only search up to the first preprocessor command
      /\( *\|\/*\)/ { #limit to comment lines
#replace the first bit, up until 'Authors', with:
        1,/Authors/ c\
/* REPLACE_MADP_HEADER */\n/* REPLACE_CONTRIBUTING_AUTHORS_START
      }
      {
#replace the last bit:
        /*$/,/*\// c\
 * REPLACE_CONTRIBUTING_AUTHORS_END\n */
      }
    }
'
