/* REPLACE_MADP_HEADER */
/* REPLACE_CONTRIBUTING_AUTHORS_START
 * Frans Oliehoek 
 * Matthijs Spaan 
 * REPLACE_CONTRIBUTING_AUTHORS_END
 */

/* Only include this header file once. */
#ifndef _TGET_H_
#define _TGET_H_ 1

/* the include directives */
#include "Globals.h"

#include "TransitionModelMapping.h"
#include "TransitionModelMappingSparse.h"

/** \brief TGet can be used for direct access to the transition model.  */
class TGet 
{
public:
    virtual ~TGet() = 0;
    //get (data) functions:
    virtual double Get(Index sI, Index jaI, Index sucSI) const = 0;
};

//http://www.parashift.com/c++-faq-lite/pointers-to-members.html
//says "defined even though it's pure virtual; it's faster this way; trust me"
inline TGet::~TGet() {}

/** \brief TGet_TransitionModelMapping can be used for direct access
 * to a TransitionModelMapping.  */
class TGet_TransitionModelMapping : public TGet
{
 
private:
    std::vector<TransitionModelMapping::Matrix* > _m_T;
public:
    TGet_TransitionModelMapping( TransitionModelMapping* tm)
    {
        _m_T = tm->_m_T;
    };

    virtual double Get(Index sI, Index jaI, Index sucSI) const
    {  { return((*_m_T[jaI])(sI,sucSI)); } }

};

/** \brief TGet_TransitionModelMappingSparse can be used for direct
 * access to a TransitionModelMappingSparse.  */
class TGet_TransitionModelMappingSparse : public TGet
{
 
private:
    std::vector<TransitionModelMappingSparse::SparseMatrix* > _m_T;
public:
    TGet_TransitionModelMappingSparse( TransitionModelMappingSparse* tm)
    {
        _m_T = tm->_m_T;
    };

    virtual double Get(Index sI, Index jaI, Index sucSI) const
    {  { return((*_m_T[jaI])(sI,sucSI)); } }

};

#endif /* !_TGET_H_ */

// Local Variables: ***
// mode:c++ ***
// End: ***
