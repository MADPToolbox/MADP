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
#ifndef _QTABLE_H_
#define _QTABLE_H_ 1

/* the include directives */
#include "Globals.h"
#include "QTableInterface.h"
#include "boost/numeric/ublas/matrix.hpp"
#include "boost/numeric/ublas/matrix_proxy.hpp"

typedef boost::numeric::ublas::matrix<double> matrix_t;
typedef boost::numeric::ublas::matrix_row<matrix_t const> row_t;

class QTable;
/// A Q-table for each time step.
typedef std::vector<QTable> QTables;

/** \brief QTable implements QTableInterface using a full matrix. */
class QTable : 
    public QTableInterface,
    public matrix_t
{
private:    
    
protected:
    
public:
    // Constructor, destructor and copy assignment.
    /// (default) Constructor
    QTable(){}
    QTable(size_t S, size_t A) : matrix_t(S,A){}
    QTable(size_t S, size_t A, double init) : matrix_t(S,A,init){}
    
    /// Copy constructor.
    QTable(const QTable& a)
        :   matrix_t(a)
        {}

    virtual double Get(Index s_i, Index ja_i) const
        { return this->operator() (s_i, ja_i); }

    row_t GetRow(Index s_i) const
        { return row_t(*this, s_i); }

    size_t GetNrStates() const
        { return this->size1(); }

    size_t GetNrActions() const
        { return this->size2(); }

    virtual void Set(Index s_i, Index ja_i, double rew)
        { this->operator() (s_i, ja_i) = rew; }
    
    virtual void SetToZero();

    /// Returns a pointer to a copy of this class.
    virtual QTable* Clone() const
        { return new QTable(*this); }

    /// Load a QTable from disk, resulting QTable is stored in Q argument.
    static void Load(const std::string &filename,
                     size_t nrRows,
                     size_t nrColumns,
                     QTable &Q);

    /// Loads QTables from disk, resulting QTables is stored in Qs argument.
    static void Load(const std::string &filename,
                     size_t nrRows,
                     size_t nrColumns,
                     size_t nrTables,
                     QTables &Qs);

    /// Save QTable Q to disk.
    static void Save(const QTable &Q, const std::string &filename);

    /// Save QTables Qs to disk.
    static void Save(const QTables &Qs, const std::string &filename);

};

#endif /* !_QTABLE_H_ */

// Local Variables: ***
// mode:c++ ***
// End: ***
