/* REPLACE_MADP_HEADER */
/* REPLACE_CONTRIBUTING_AUTHORS_START
 * Frans Oliehoek 
 * Matthijs Spaan 
 * REPLACE_CONTRIBUTING_AUTHORS_END
 */

/* Only include this header file once. */
#ifndef _REFERRER_H_
#define _REFERRER_H_ 1

/* the include directives */
#include <iostream>



/// Referrer is a template class that represents objects that refer another.
/** I.e. it containt a pointer to type T  */
template <class T>
class Referrer 
{
    private:
        T* _m_referred;
    
    protected:
    
    public:
        // Constructor, destructor and copy assignment.
        //    Referrer() {_m_referred = 0;}
        /// (default) Constructor
        Referrer(T* t_p = 0) : _m_referred(t_p){};
        ///Alternative constructor.
        Referrer(T& t) : _m_referred(&t) {};
        /// Copy constructor.
        Referrer(const Referrer& a) : _m_referred(a._m_referred){};
        /// Destructor.
        virtual ~Referrer(){};

        //operators:

        //data manipulation (set) functions:
        ///Change the referred thing...
        void SetReferred(T* t_p)  { _m_referred = t_p; }

        
        //get (data) functions:
        ///Return the referred thing...
        T* GetReferred() const { return(_m_referred); }
};


#endif /* !_REFERRER_H_ */

// Local Variables: ***
// mode:c++ ***
// End: ***
