/* REPLACE_MADP_HEADER */
/* REPLACE_CONTRIBUTING_AUTHORS_START
 * Frans Oliehoek 
 * Matthijs Spaan 
 * REPLACE_CONTRIBUTING_AUTHORS_END
 */

/* Only include this header file once. */
#ifndef _PARSERINTERFACE_H_
#define _PARSERINTERFACE_H_ 1

/* the include directives */
#include <iostream>

/// ParserInterface is an interface for parsers.
class ParserInterface 
{
private:
    
protected:
    
public:
    // Constructor, destructor and copy assignment.
    /// (default) Constructor
    ParserInterface(){};

    virtual ~ParserInterface(){};
    
    virtual void Parse() = 0;

};


#endif /* !_PARSERINTERFACE_H_ */

// Local Variables: ***
// mode:c++ ***
// End: ***
