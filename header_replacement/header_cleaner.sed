#only search up to the first preprocessor command  --- "1,/#/"   means search from line 1 to a line that matches '#'
1,/#/ { 
    #match (limit to) comment lines
    /\( *\|\/*\)/ { 
        #replace the first bit, up until 'Authors'
        1,/Authors/ c\
/* REPLACE_MADP_HEADER */\n/* REPLACE_CONTRIBUTING_AUTHORS_START
        #match email addresses:
        s/<[^@]*@[^>]*>//
    }
    {
        #replace the last bit:
        /*$/,/*\// c\
 * REPLACE_CONTRIBUTING_AUTHORS_END\n */
    }
}
