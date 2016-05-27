/* REPLACE_MADP_HEADER */
/* REPLACE_CONTRIBUTING_AUTHORS_START
 * Frans Oliehoek 
 * Matthijs Spaan 
 * REPLACE_CONTRIBUTING_AUTHORS_END
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
