/* REPLACE_MADP_HEADER */
/* REPLACE_CONTRIBUTING_AUTHORS_START
 * Frans Oliehoek 
 * Matthijs Spaan 
 * REPLACE_CONTRIBUTING_AUTHORS_END
 */

/* Only include this header file once. */
#ifndef _E_H_
#define _E_H_ 1

/* the include directives */
#include <iostream>
#include <string>
#include <sstream>

/// E is a class that represents a basic exception.
class E 
{
    private:    
    
    protected:
    
    public:
    
    /// The error message for this particular exception.
    std::string _m_error;

    // Constructor, destructor and copy assignment.

    /// Constructor with a C-style string
    E(const char* arg): _m_error(arg) {}
    /// Constructor with an STL string
    E(std::string arg) : _m_error(arg) {}
    /// Constructor with an STL stringstream
    E(const std::stringstream& arg) : _m_error(arg.str()) {}

    /// Destructor
    virtual ~E(){};

    /// Returns the error message.
    std::string SoftPrint() const {return _m_error;}
    /// Prints the error message to cout.
    void Print() const 
        { std::cout << "ERROR: "<< _m_error << std::endl;} 
};


#endif /* !_E_H_ */


// Local Variables: ***
// mode:c++ ***
// End: ***
