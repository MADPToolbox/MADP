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


#ifndef __defined_libdai_treeep_h
#define __defined_libdai_treeep_h


#include <vector>
#include "daialg.h"
#include "varset.h"
#include "regiongraph.h"
#include "factorgraph.h"
#include "clustergraph.h"
#include "weightedgraph.h"
#include "jtree.h"
#include "enum.h"


namespace libDAI {


    class TreeEP : public JTree {

        public:
            ENUM(TypeType,ORG,ALT)

        protected:
            struct {
                TypeType type;
                double   tol;
                size_t   maxiter;
                size_t   verbose;
            } Props2;
            /// Maximum difference encountered so far
            double                  _maxdiff;
            /// Number of iterations needed
            size_t                  _iterations;

            class TreeEPSubTree {
                protected:
                    std::vector<Factor>  _Qa;
                    std::vector<Factor>  _Qb;
                    DEdgeVec        _RTree;
                    std::vector<size_t>  _a;             // _Qa[alpha]  <->  superTree._Qa[_a[alpha]]
                    std::vector<size_t>  _b;             // _Qb[beta]   <->  superTree._Qb[_b[beta]]
                                                    // _Qb[beta]   <->  _RTree[beta]    
                    const Factor *  _I;
                    VarSet          _ns;
                    VarSet          _nsrem;
                    double          _logZ;
                    
                    
                public:
                    TreeEPSubTree() : _Qa(), _Qb(), _RTree(), _a(), _b(), _I(NULL), _ns(), _nsrem(), _logZ(0.0) {}
                    TreeEPSubTree( const TreeEPSubTree &x) : _Qa(x._Qa), _Qb(x._Qb), _RTree(x._RTree), _a(x._a), _b(x._b), _I(x._I), _ns(x._ns), _nsrem(x._nsrem), _logZ(x._logZ) {}
                    TreeEPSubTree & operator=( const TreeEPSubTree& x ) {
                        if( this != &x ) {
                            _Qa         = x._Qa;
                            _Qb         = x._Qb;
                            _RTree      = x._RTree;
                            _a          = x._a;
                            _b          = x._b;
                            _I          = x._I;
                            _ns         = x._ns;
                            _nsrem      = x._nsrem;
                            _logZ       = x._logZ;
                        }
                        return *this;
                    }

                    TreeEPSubTree( const DEdgeVec &subRTree, const DEdgeVec &jt_RTree, const std::vector<Factor> &jt_Qa, const std::vector<Factor> &jt_Qb, const Factor *I );
                    void init();
                    void InvertAndMultiply( const std::vector<Factor> &Qa, const std::vector<Factor> &Qb );
                    void HUGIN_with_I( std::vector<Factor> &Qa, std::vector<Factor> &Qb );
                    double logZ( const std::vector<Factor> &Qa, const std::vector<Factor> &Qb ) const;
                    const Factor *& I() { return _I; }
            };

            std::map<size_t, TreeEPSubTree>  _Q;

// JTree interface 

        public:
            /// Default constructor
            TreeEP() : JTree(), Props2(), _maxdiff(0.0), _iterations(0UL), _Q() {};
            
            /// Construct TreeEP object using the specified properties
            TreeEP( const FactorGraph & fg, const Properties &opts );
            
            /// Copy constructor
            TreeEP( const TreeEP & x ) : JTree(x), Props2(x.Props2), _maxdiff(x._maxdiff), _iterations(x._iterations), _Q(x._Q) {
                for( size_t I = 0; I < grm().nrFactors(); I++ )
                    if( offtree( I ) )
                        _Q[I].I() = &grm().factor(I);
            }

            /// Assignment operator
            TreeEP & operator=( const TreeEP & x ) {
                if( this != &x ) {
                    JTree::operator=( x );
                    Props2       = x.Props2;
                    _maxdiff     = x._maxdiff;
                    _iterations  = x._iterations;
                    _Q           = x._Q;
                    for( size_t I = 0; I < grm().nrFactors(); I++ )
                        if( offtree( I ) )
                            _Q[I].I() = &grm().factor(I);
                }
                return *this;
            }

            /// Clone (virtual copy constructor)
            virtual TreeEP* clone() const {
                return new TreeEP(*this);
            }

            /// Create (virtual constructor)
            virtual TreeEP* create() const {
                return new TreeEP();
            }
            
            /// Return verbosity level
            virtual size_t Verbose() const {
                return Props2.verbose;
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
//            virtual Factor belief( const Var &n ) const;

            /// Get general belief
//            virtual Factor belief( const VarSet &n ) const;

            /// Get all beliefs
//            virtual std::vector<Factor> beliefs() const;

            /// Get log partition sum
            virtual Complex logZ() const;

            /// Clear messages and beliefs
            virtual void init();

            /// Clear messages and beliefs corresponding to the nodes in ns
            virtual void init( const VarSet &ns ) {
                init();
            }

            /// The actual approximate inference algorithm
            virtual double run();

            /// Checks whether all necessary properties have been set
            /// and casts string-valued properties to other values if necessary
            virtual bool initProps2();

            /// Name of this inference method
            static const char *Name;


// TreeEP specific stuff


        public:
            void ConstructRG( const DEdgeVec &tree );

            bool offtree(size_t I) const { return !grm().nrFac2OR(I); }
    };


}


#endif
