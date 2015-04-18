/* This file is part of the Multiagent Decision Process (MADP) Toolbox v0.3. 
 *
 * The majority of MADP is free software released under GNUP GPL v.3. However,
 * some of the included libraries are released under a different license. For 
 * more information, see the included COPYING file. For other information, 
 * please refer to the included README file.
 *
 * This file has been written and/or modified by the following people:
 *
 * Frans Oliehoek 
 * Matthijs Spaan 
 *
 * For contact information please see the included AUTHORS file.
 */


namespace ArgumentHandlers {

static struct argp_option options_main[] = {
    { 0 }
};
static error_t
parse_main (int key, char *arg, struct argp_state *state)
{
    struct arguments *theArgumentsStruc = (struct arguments*) state->input;
    switch (key)
    {
        case ARGP_KEY_INIT:
            //give child_parsers access to the arguments structure on 
            //initialization.
            for(int i = 0; i < nrChildParsers; i++)
                state->child_inputs[i] = theArgumentsStruc;
            break;
        default:
            return ARGP_ERR_UNKNOWN;
    }
    return 0;
}
/* Our argp parser. */
static argp theArgpStruc = {options_main, parse_main, 0, doc, childVector };

}
