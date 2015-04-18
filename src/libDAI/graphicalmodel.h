/*  Copyright (C) 2006,2007  Joris Mooij  [joris at jorismooij dot nl]
    Radboud University Nijmegen, The Netherlands
    
    This file is part of libDAI.

    libDAI is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    (at your option) any later version.

    libDAI is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with libDAI; if not, write to the Free Software
    Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
*/


#ifndef __defined_libdai_graphicalmodel_h
#define __defined_libdai_graphicalmodel_h


#include "prob.h"


namespace libDAI {


    /// GraphicalModel is an abstract base class that defines the
    /// interface to graphical models in the sense of probability
    /// distributions over a finite number of discrete valued random
    /// variables with a probability mass function that factorizes
    /// as a product of factors, where each factor is a non-negative
    /// function of a subset of the random variables.
    /// FactorGraph and RegionGraph derive from GraphicalModel.
    class GraphicalModel {
        private:
            /// Normalisation used in this graphical model.
            /// Usually, this is the Prob::NormType::NORMLINF norm,
            /// which ensures that the probability distribution is
            /// properly normalized.
            Prob::NormType _normType;

        public:
            /// Default constructor
            GraphicalModel() : _normType(Prob::NORMPROB) {}
            
            /// Copy constructor
            GraphicalModel( const GraphicalModel & x ) : _normType(x._normType) {}

            /// Assignment operator
            GraphicalModel & operator=( const GraphicalModel & x ) {
                if( this != &x ) {
                    _normType = x._normType;
                }
                return *this;
            }
            
            /// Virtual desctructor
            virtual ~GraphicalModel() {}

            /// Create (virtual default constructor)
            virtual GraphicalModel* create() const = 0;

            /// Clone (virtual copy constructor)
            virtual GraphicalModel* clone() const = 0;


            /// Get connectedness of underlying factor graph
            virtual bool isConnected() const = 0;

            /// Get number of variables
            virtual size_t nrVars() const = 0;

            /// Get const reference to i'th variable
            virtual const Var & var( size_t i ) const = 0;

            /// Get const reference to all variables
            virtual const std::vector<Var> & vars() const = 0;

            /// Get index of variable n
            virtual size_t findVar( const Var & n ) const = 0;

            /// Get number of factors
            virtual size_t nrFactors() const = 0;

            /// Get const reference to I'th factor
            virtual const Factor & factor( size_t I ) const = 0;

            /// Get const reference to all factors
            virtual const std::vector<Factor> & factors() const = 0;

            /// Get index of first factor involving ns
            virtual size_t findFactor( const VarSet &ns ) const = 0;

            /// Return all variables that occur in a factor involving variable n, n itself included
            virtual VarSet Delta( const Var & n ) const = 0;

            /// Return all variables that occur in a factor involving variable n, n itself excluded
            virtual VarSet delta( const Var & n ) const = 0;

            /// Return all variables that occur in a factor involving some variable in ns, ns itself included
            virtual VarSet Delta( const VarSet & ns ) const = 0;

            /// Return all variables that occur in a factor involving some variable in ns, ns itself excluded
            virtual VarSet delta( const VarSet & ns ) const = 0;

            /// Set the content of the I'th factor and make a backup of its old content if backup == true
            virtual void setFactor( size_t I, const Factor &newFactor, bool backup = false ) = 0;

            /// Set the contents of all factors as specified by facs and make a backup of the old contents if backup == true
            virtual void setFactors( const std::map<size_t, Factor> & facs, bool backup = false ) = 0;

            /// Clamp variable n to value i (i.e. multiply with a Kronecker delta \f$\delta_{x_n, i}\f$);
            /// If backup == true, make a backup of all factors that are changed
            virtual void clamp( const Var & n, size_t i, bool backup = false ) = 0;

            /// Set all factors interacting with variable n to 1
            virtual void makeCavity( const Var & n ) = 0;

            /// Backup the factors specified by indices in facs
            virtual void backupFactors( const std::set<size_t> & facs ) = 0;

            /// Restore all factors to the backup copies
            virtual void restoreFactors() = 0;

            /// Return the norm type (usually, this would be Prob::ProbType::NORMLINF)
            Prob::NormType normType() const { return _normType; }
    };


}


#endif
