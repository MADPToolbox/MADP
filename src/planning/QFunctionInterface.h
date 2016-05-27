/* REPLACE_MADP_HEADER */
/* REPLACE_CONTRIBUTING_AUTHORS_START
 * Frans Oliehoek 
 * Matthijs Spaan 
 * REPLACE_CONTRIBUTING_AUTHORS_END
 */

/* Only include this header file once. */
#ifndef _QFUNCTIONINTERFACE_H_
#define _QFUNCTIONINTERFACE_H_ 1

/* the include directives */
#include <iostream>
#include "Globals.h"

/**\brief QFunctionInterface is an abstract class for all Q-Functions.
 *
*/
class QFunctionInterface 
{
private:
    
protected:
    
public:
    virtual ~QFunctionInterface(){};

    ///Compute the heuristic.
    virtual void Compute() = 0;

    /// Compute Qvalue function, while caching the Qvalues to disk.
    /** Before computing them, the function checks whether the Qvalues
     * have been computed before, and if so, loads them from disk. If
     * not, it computes them, and afterwards saves them for
     * re-use. This behavior can be changed by settings
     * computeIfNotCached to false, in which case an Exception will be
     * thrown if the Q function has not been previously stored on
     * disk.
     */
    virtual void ComputeWithCachedQValues(bool computeIfNotCached=true) = 0;

    /// Load the Qvalues from disk.
    virtual void Load() { Load(GetCacheFilename()); }
    
    /// Load the Qvalues from disk from a file named \a filename.
    virtual void Load(const std::string &filename) = 0;
    
    /// Stores the Qvalues to disk.
    virtual void Save() const { Save(GetCacheFilename()); }

    /// Stores the Qvalues to disk in file named \a filename.
    virtual void Save(const std::string &filename) const = 0;

    /// Gets the filename where the Qvalues should be stored.
    virtual std::string GetCacheFilename() const = 0;

    /**\brief Returns a short description of the heuristic, can be
    * used for constructing filenames. */ 
    virtual std::string SoftPrintBrief() const = 0;

};


#endif /* !_QFUNCTIONINTERFACE_H_ */

// Local Variables: ***
// mode:c++ ***
// End: ***
