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


#ifndef __defined_libdai_ijgp_h
#define __defined_libdai_ijgp_h


#include "daialg.h"
#include "regiongraph.h"
#include "enum.h"


namespace libDAI {


    class IJGP : public DAIAlgRG {

        public:
            ENUM(UpdateType,SEQFIX,SEQRND) //,SEQMAX,PARALL)

        protected:
            struct {
                UpdateType updates;
                double     tol;
                size_t     maxiter;
                size_t     verbose;
                double     damping;
                size_t     i;
            } Props;
            /// Maximum difference encountered so far
            double                       _maxdiff;
            /// Number of iterations needed
            size_t                       _iterations;

            std::vector<Factor>          _Qa;
            std::vector<Factor>          _Qb;
            std::vector<Factor>          _mes;


// DAIAlgRG interface 

        public:
            /// Default constructor
            IJGP() : DAIAlgRG(), Props(), _maxdiff(0.0), _iterations(0UL), _Qa(), _Qb(), _mes() {};
            
            /// Construct IJGP object using the specified properties
            IJGP( const FactorGraph & fg, const Properties &opts );
            
            /// Copy constructor
            IJGP( const IJGP & x ) : DAIAlgRG(x), Props(x.Props), _maxdiff(x._maxdiff), _iterations(x._iterations), _Qa(x._Qa), _Qb(x._Qb), _mes(x._mes) {};

            /// Assignment operator
            IJGP & operator=( const IJGP & x ) {
                if( this != &x ) {
                    DAIAlgRG::operator=( x );
                    Props       = x.Props;
                    _maxdiff    = x._maxdiff;
                    _iterations = x._iterations;
                    _Qa         = x._Qa;
                    _Qb         = x._Qb;
                    _mes        = x._mes;
                }
                return *this;
            }

            /// Clone (virtual copy constructor)
            virtual IJGP* clone() const {
                return new IJGP(*this);
            }

            /// Create (virtual constructor)
            virtual IJGP* create() const {
                return new IJGP();
            }
            
            /// Return verbosity level
            virtual size_t Verbose() const {
                return Props.verbose;
            }

            /// Return number of passes over the factorgraph
            virtual size_t Iterations() const {
                return _iterations;
            }

            /// Return maximum difference between single node 
            /// beliefs for two consecutive iterations
            virtual double maxDiff() const {
                return _maxdiff;
            }

            /// Identifies itself for logging purposes
            virtual std::string identify() const;

            /// Get single node belief
            virtual Factor belief( const Var &n ) const;

            /// Get general belief
            virtual Factor belief( const VarSet &n ) const;

            /// Get all beliefs
            virtual std::vector<Factor> beliefs() const;

            /// Get log partition sum
            virtual Complex logZ() const;

            /// Clear messages and beliefs
            virtual void init();

            /// Clear messages and beliefs corresponding to the nodes in ns
            virtual void init( const VarSet &ns );

            /// The actual approximate inference algorithm
            virtual double run();

            /// Checks whether all necessary properties have been set
            /// and casts string-valued properties to other values if necessary
            virtual bool initProps();

            /// Name of this inference method
            static const char *Name;


// IJGP specific stuff

        protected:
            /// A BucketEntry is a function contained in a bucket
            /// It either corresponds with a factor (isFactor == true) with index pointsTo
            /// or with a message (isFactor == false) originating from minibucket with index pointsTo
            class BucketEntry : public VarSet {
                public:
                    bool   isFactor;
                    size_t pointsTo;
                    BucketEntry( VarSet vars, bool isFactor, size_t pointsTo ) : VarSet(vars), isFactor(isFactor), pointsTo(pointsTo) {}
            };
            /// A Bucket is a vector of BucketEntry's corresponding to a variable index
            class Bucket : public std::vector<BucketEntry> {
                public:
                    size_t var;
                    Bucket( size_t var ) : std::vector<BucketEntry>(), var(var) {}
                    VarSet vars() {
                        VarSet result;
                        for( size_t i = 0; i < size(); i++ )
                            result |= operator[](i);
                        return result;
                    }
                    friend std::ostream & operator << ( std::ostream & os, const Bucket & b ) {
                        os << "Bucket[var = " << b.var << "; factors = {";
                        size_t count = 0;
                        for( size_t bE = 0; bE < b.size(); bE++ )
                            if( b[bE].isFactor )
                                os << ((count++) ? "," : "") << b[bE];
                        os << "}; messages = {";
                        count = 0;
                        for( size_t bE = 0; bE < b.size(); bE++ )
                            if( !b[bE].isFactor )
                                os << ((count++) ? "," : "") << b[bE];
                        os << "}]";

                        return(os);
                    }
            };

        public:
            Factor & message(size_t i1, size_t i2) { return( _mes[grm().ORIR2E(i1,i2)] ); }   
            const Factor & message(size_t i1, size_t i2) const { return( _mes[grm().ORIR2E(i1,i2)] ); }   
            
            /// Returns the lowest index in ElimOrder[start:] that corresponds with a variable in vars
            size_t findElimOrderIndex( const std::vector<Var> & ElimOrder, const VarSet & vars, size_t start=0 );

            /// Partition a bucket into mini-buckets of i variables at most
            std::vector<Bucket> partitionBucket( const Bucket & b, size_t i ); 

            void updateMessage(size_t e);
    };


}


#endif
