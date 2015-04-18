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


#ifndef __defined_libdai_bp_h
#define __defined_libdai_bp_h


#include "daialg.h"
#include "factorgraph.h"
#include "enum.h"


namespace libDAI {


    class BP : public DAIAlgFG {
        public:
            ENUM(UpdateType,SEQFIX,SEQRND,SEQMAX,PARALL)


        protected:
            struct {
                UpdateType updates;
                double     tol;
                size_t     maxiter;
                size_t     verbose;
                double     damping;
            } Props;
            /// Maximum difference encountered so far
            double                       _maxdiff;
            /// Number of iterations needed
            size_t                       _iterations;

            typedef std::vector<size_t>  _ind_t;
            std::vector<_ind_t>          _indices;
            std::vector<Prob>            _messages, _newmessages;    //vector of probability vectors (Prob = TProb< Real > and Real = double)


// DAIAlgFG interface 

        public:
            /// Default constructor
            BP() : DAIAlgFG(), Props(), _maxdiff(0.0), _iterations(0UL), _indices(), _messages(), _newmessages() {};
            
            /// Construct BP object using the specified properties
            BP( const FactorGraph & fg, const Properties &opts );
            
            /// Copy constructor
            BP( const BP & x ) : DAIAlgFG(x), Props(x.Props), _maxdiff(x._maxdiff), _iterations(x._iterations), _indices(x._indices), _messages(x._messages), _newmessages(x._newmessages) {};

            /// Assignment operator
            BP & operator=( const BP & x ) {
                if( this != &x ) {
                    DAIAlgFG::operator=( x );
                    Props        = x.Props;
                    _maxdiff     = x._maxdiff;
                    _iterations  = x._iterations;
                    _indices     = x._indices;
                    _messages    = x._messages;
                    _newmessages = x._newmessages;
                }
                return *this;
            }

            /// Clone (virtual copy constructor)
            virtual BP* clone() const {
                return new BP(*this);
            }

            /// Create (virtual constructor)
            virtual BP* create() const {
                return new BP();
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


// BP specific stuff

            ///FRANS: I think this function computes the new message for the egde(?) with index iI...
            void calcNewMessage(size_t iI);
            void updateMessage(size_t iI) {
                if( Props.damping == 0.0 )
                    _messages[iI] = _newmessages[iI];
                else
                    _messages[iI] = (_messages[iI] ^ Props.damping) * (_newmessages[iI] ^ (1.0 - Props.damping));
            }
            Factor beliefV (size_t i) const;
            Factor beliefF (size_t I) const;
            const Prob & message(size_t i, size_t I) const { return( _messages[grm().edge(i,I)] ); }  

        private:
            Prob & message(size_t i, size_t I) { return( _messages[grm().edge(i,I)] ); }  
            Prob & newMessage(size_t i, size_t I) { return( _newmessages[grm().edge(i,I)] ); }    
            const Prob & newMessage(size_t i, size_t I) const { return( _newmessages[grm().edge(i,I)] ); }    
            _ind_t & index(size_t i, size_t I) { return( _indices[grm().edge(i,I)] ); }
            const _ind_t & index(size_t i, size_t I) const { return( _indices[grm().edge(i,I)] ); }
    };


}


#endif
