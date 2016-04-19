# sed script that replaces already replaced header files back to template replacements:
#
#only search up to the first preprocessor command  --- "1,/#/"   means search from line 1 to a line that matches '#'
1,/#/ { 
    #match (limit to) comment lines
    /\( *\|\/*\)/{ 
        #replace the first bit, up until 'the following people:'
        1,/ * This file has been written and\/or modified by the following people:/ c\
/* REPLACE_MADP_HEADER */\n/* REPLACE_CONTRIBUTING_AUTHORS_START
        #match email addresses:
        #s/<[^@]*@[^>]*>//
    }
    {
        / \*$/d
    }
    {
        #replace the last bit:
        / \* For contact information please see the included AUTHORS file./ c\
 * REPLACE_CONTRIBUTING_AUTHORS_END
    }
}
