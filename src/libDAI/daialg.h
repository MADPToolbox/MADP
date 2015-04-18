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


#ifndef __defined_libdai_daialg_h
#define __defined_libdai_daialg_h


#include <string>
#include <iostream>
#include <vector>
#include "factorgraph.h"
#include "regiongraph.h"
#include "properties.h"
#include "exceptions.h"


namespace libDAI {


    /// The InferenceAlgorithm class is the common denominator of the various approximate inference algorithms.
    /// A InferenceAlgorithm object represents a discrete factorized probability distribution over multiple variables 
    /// together with an inference algorithm.
    class InferenceAlgorithm {
        private:
            /// Properties of the algorithm (replaces _tol, _maxiter, _verbose)
            Properties              _properties;

            /// Maximum difference encountered so far
            double                  _maxdiff;

            /// Number of iterations needed
            size_t                  _iterations;


        public:
            /// Default constructor
            InferenceAlgorithm() : _properties(), _maxdiff(0.0), _iterations(0UL) {}
            
            /// Constructor with options
            InferenceAlgorithm( const Properties &opts ) : _properties(opts), _maxdiff(0.0), _iterations(0UL) {}
            
            /// Copy constructor
            InferenceAlgorithm( const InferenceAlgorithm & x ) : _properties(x._properties), _maxdiff(x._maxdiff), _iterations(x._iterations) {}

            /// Assignment operator
            InferenceAlgorithm & operator=( const InferenceAlgorithm & x ) {
                if( this != &x ) {
                    _properties = x._properties;
                    _maxdiff    = x._maxdiff;
                    _iterations = x._iterations;
                }
                return *this;
            }

            /// Clone (virtual copy constructor)
            virtual InferenceAlgorithm* clone() const = 0;

            /// Create (virtual constructor)
            virtual InferenceAlgorithm* create() const = 0;
            
            /// Virtual desctructor
            virtual ~InferenceAlgorithm() {}
            

            /// Return const reference to graphical model
            virtual const GraphicalModel & grm() const = 0;
            
            /// Return reference to graphical model
            virtual GraphicalModel & grm() = 0;

            /// Returns true if a property with the given key is present
            bool HasProperty(const PropertyKey &key) const { return _properties.hasKey(key); }

            /// Gets a property
            const PropertyValue & GetProperty(const PropertyKey &key) const { return _properties.Get(key); }
     
            /// Gets a property, casted as ValueType
            template<typename ValueType>
            ValueType GetPropertyAs(const PropertyKey &key) const { return _properties.GetAs<ValueType>(key); }

            /// Gets a property, converting from string to ValueType
            template<typename ValueType>
            ValueType FromStringTo(const PropertyKey &key) const { return _properties.FromStringTo<ValueType>(key); }

            /// Sets a property 
            void SetProperty(const PropertyKey &key, const PropertyValue &val) { _properties[key] = val; }

            /// Converts a property from string to ValueType, if necessary
            template<typename ValueType>
            void ConvertPropertyTo(const PropertyKey &key) { _properties.ConvertTo<ValueType>(key); }

            /// Gets all properties
            const Properties & GetProperties() const { return _properties; }

            /// Sets properties
            void SetProperties(const Properties &p) { _properties = p; }

/*            /// Sets tolerance
            void Tol( double tol ) { SetProperty("tol", tol); }
            /// Gets tolerance
            double Tol() const { return GetPropertyAs<double>("tol"); }

            /// Sets damping
            void Damping( double damping ) { SetProperty("damping", damping); }
            /// Gets damping
            double Damping() const { return GetPropertyAs<double>("damping"); }

            /// Sets maximum number of iterations
            void MaxIter( size_t maxiter ) { SetProperty("maxiter", maxiter); }
            /// Gets maximum number of iterations
            size_t MaxIter() const { return GetPropertyAs<size_t>("maxiter"); }

            /// Sets verbosity
            void Verbose( size_t verbose ) { SetProperty("verbose", verbose); }
            /// Gets verbosity
            size_t Verbose() const { return GetPropertyAs<size_t>("verbose"); }

            /// Sets maximum difference encountered so far
            void MaxDiff( double maxdiff ) { _maxdiff = maxdiff; }
            /// Gets maximum difference encountered so far
            double MaxDiff() const { return _maxdiff; }
            /// Updates maximum difference encountered so far
            void updateMaxDiff( double maxdiff ) { if( maxdiff > _maxdiff ) _maxdiff = maxdiff; }
            /// Sets maximum difference encountered so far to zero
            void clearMaxDiff() { _maxdiff = 0.0; }

            /// Sets number of iterations
            void Iterations( size_t iter ) { _iterations = iter; }
            /// Gets number of iterations
            size_t Iterations() const { return _iterations; }
*/
            /// Return verbosity level
            virtual size_t Verbose() const = 0;

            /// Return number of passes over the factorgraph
            virtual size_t Iterations() const = 0;

            /// Return maximum difference between single node 
            /// beliefs for two consecutive iterations
            virtual double maxDiff() const = 0;

            /// Identifies itself for logging purposes
            virtual std::string identify() const = 0;

            /// Get single node belief
            virtual Factor belief( const Var &n ) const = 0;

            /// Get general belief
            virtual Factor belief( const VarSet &n ) const = 0;

            /// Get all beliefs
            virtual std::vector<Factor> beliefs() const = 0;

            /// Get log partition sum
            virtual Complex logZ() const = 0;

            /// Clear messages and beliefs
            virtual void init() = 0;

            /// Clear messages and beliefs corresponding to the nodes in ns
            virtual void init( const VarSet &ns ) = 0;

            /// The actual approximate inference algorithm
            virtual double run() = 0;

            /// Checks whether all necessary properties have been set
            /// and casts string-valued properties to other values if necessary
            virtual bool initProps() = 0;
    };


    template <class T>
    class DAIAlg : public InferenceAlgorithm {
        private:
            /// The graphical model
            T _grm;

        public:
            /// Default constructor
            DAIAlg() : InferenceAlgorithm(), _grm() {}
            
            /// Construct DAIAlg with empty _grm but using the specified properties
            DAIAlg( const Properties &opts ) : InferenceAlgorithm( opts ), _grm() {}

            /// Construct DAIAlg using the specified properties
            DAIAlg( const T & t, const Properties &opts ) : InferenceAlgorithm( opts ), _grm(t) {}
            
            /// Copy constructor
            DAIAlg( const DAIAlg & x ) : InferenceAlgorithm(x), _grm(x.grm()) {}

            /// Return const reference to graphical model
            virtual const T & grm() const { return _grm; }
            
            /// Return reference to graphical model
            virtual T & grm() { return _grm; }
    };


    typedef DAIAlg<FactorGraph> DAIAlgFG;
    typedef DAIAlg<RegionGraph> DAIAlgRG;


    /// Calculate the marginal of obj on ns by clamping 
    /// all variables in ns and calculating logZ for each joined state
    Factor calcMarginal( const InferenceAlgorithm & obj, const VarSet & ns, bool reInit );


    /// Calculate beliefs of all pairs in ns (by clamping
    /// nodes in ns and calculating logZ and the beliefs for each state)
    std::vector<Factor> calcPairBeliefs( const InferenceAlgorithm & obj, const VarSet& ns, bool reInit );


    /// Calculate beliefs of all pairs in ns (by clamping
    /// pairs in ns and calculating logZ for each joined state)
    std::vector<Factor> calcPairBeliefsNew( const InferenceAlgorithm & obj, const VarSet& ns, bool reInit );


    /// Calculate 2nd order interactions of the marginal of obj on ns
    Factor calcMarginal2ndO( const InferenceAlgorithm & obj, const VarSet& ns, bool reInit );


}


#endif
