/* REPLACE_MADP_HEADER */
/* REPLACE_CONTRIBUTING_AUTHORS_START
 * Frans Oliehoek 
 * Matthijs Spaan 
 * REPLACE_CONTRIBUTING_AUTHORS_END
 */

        (
            str_p("agents")[DebugOutput("agentstok_parser (sub_grammar_defs):")]
        )
    {
    }
    start_t start;
};
//static agentstok_parser AGENTSTOK;

struct discounttok_parser : public sub_grammar<discounttok_parser>
{
    typedef
//int        
        boost::spirit::action<boost::spirit::strlit<const char*>, ParserDPOMDPFormat_Spirit::DebugOutput>
    start_t;
    discounttok_parser()
    :         start        (str_p("discount")[DebugOutput("discounttok_parser:")])
    {}
    start_t start;
};
//static discounttok_parser DISCOUNTTOK;

struct valuestok_parser : public sub_grammar<valuestok_parser>
{
    typedef //int        
        boost::spirit::action<boost::spirit::strlit<const char*>, ParserDPOMDPFormat_Spirit::DebugOutput>
    start_t;
    valuestok_parser()
    :         start        (str_p("values")[DebugOutput("valuestok_parser:")])
    {}
    start_t start;
};
//static valuestok_parser VALUESTOK;

struct statestok_parser : public sub_grammar<statestok_parser>
{
    typedef //int        
        boost::spirit::action<boost::spirit::strlit<const char*>, ParserDPOMDPFormat_Spirit::DebugOutput>
    start_t;
    statestok_parser()
    :         start        (str_p("states")[DebugOutput("statestok_parser:")])
    {}
    start_t start;
};
//static statestok_parser STATESTOK;

struct actionstok_parser : public sub_grammar<actionstok_parser>
{
    typedef //int        
        boost::spirit::action<boost::spirit::strlit<const char*>, ParserDPOMDPFormat_Spirit::DebugOutput>
    start_t;
    actionstok_parser()
    :         start        (str_p("actions")[DebugOutput("actionstok_parser:")])
    {}
    start_t start;
};
//static actionstok_parser ACTIONSTOK;

struct observationstok_parser : public sub_grammar<observationstok_parser>
{
    typedef //int        
        boost::spirit::action<boost::spirit::strlit<const char*>, ParserDPOMDPFormat_Spirit::DebugOutput>
    start_t;
    observationstok_parser()
    :         start        (str_p("observations")[DebugOutput("observationstok_parser:")])
    {}
    start_t start;
};
//static observationstok_parser OBSERVATIONSTOK;
struct ttok_parser : public sub_grammar<ttok_parser>
{
    typedef
        boost::spirit::action<boost::spirit::strlit<const char*>, ParserDPOMDPFormat_Spirit::DebugOutput>
    start_t;
    ttok_parser()
    :         start
        (
            str_p("T")[DebugOutput("ttok_parser:")]
        )
    {
    }
    start_t start;
};
struct otok_parser : public sub_grammar<otok_parser>
{
    typedef
        boost::spirit::action<boost::spirit::strlit<const char*>, ParserDPOMDPFormat_Spirit::DebugOutput>
    start_t;
    otok_parser()
    :         start
        (
            str_p("O")[DebugOutput("otok_parser:")]
        )
    {
    }
    start_t start;
};
struct rtok_parser : public sub_grammar<rtok_parser>
{
    typedef
        boost::spirit::action<boost::spirit::strlit<const char*>, ParserDPOMDPFormat_Spirit::DebugOutput>
    start_t;
    rtok_parser()
    :         start
        (
            str_p("R")[DebugOutput("rtok_parser:")]
        )
    {
    }
    start_t start;
};

struct uniformtok_parser : public sub_grammar<uniformtok_parser>
{
    ParserDPOMDPFormat_Spirit* _m_po;
    typedef
//int                
        boost::spirit::sequence<boost::spirit::action<boost::spirit::eol_parser, ParserDPOMDPFormat_Spirit::DebugOutput>, boost::spirit::action<boost::spirit::action<boost::spirit::strlit<const char*>, ParserDPOMDPFormat_Spirit::DebugOutput>, ParserDPOMDPFormat_Spirit::SetLastParsedType> >
    start_t;
    uniformtok_parser(ParserDPOMDPFormat_Spirit* p)
    :         _m_po(p),
        start
        (
            eol_p [DebugOutput("uniformtok_parser(sub_grammar_defs): \"eol \"")]
            >> str_p("uniform")[DebugOutput("uniformtok_parser(sub_grammar_defs): \"uniform\"")]
            [SetLastParsedType(_m_po, UNIFORM)]
        )
    {}
    start_t start;
};

struct inttok_parser : public sub_grammar<inttok_parser>
{
    ParserDPOMDPFormat_Spirit* _m_po;
    typedef 
//        int
        boost::spirit::action<boost::spirit::action<boost::spirit::uint_parser<unsigned int, 10, 1u, -0x000000001>, ParserDPOMDPFormat_Spirit::StoreLastParsedElement>, ParserDPOMDPFormat_Spirit::DebugOutputNoParsed>
    start_t;
    inttok_parser(ParserDPOMDPFormat_Spirit* p)
    :         _m_po(p),
        start
        (
            uint_p[StoreLastParsedElement(_m_po)][DebugOutputNoParsed("inttok_parser")]
        )
    {}
    start_t start;
};

 * REPLACE_CONTRIBUTING_AUTHORS_END
 */
