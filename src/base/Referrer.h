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
