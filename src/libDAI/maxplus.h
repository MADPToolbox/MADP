/*  Copyright (C) 2008  Frans Oliehoek  [F.A.Oliehoek@uva.nl], Joris Mooij  [joris at jorismooij dot nl]
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


#ifndef __defined_libdai_maxplus_h
#define __defined_libdai_maxplus_h

#include <list>            
#include <fstream>
#include <cstdlib>
#include <float.h>
#include "daialg.h"
#include "factorgraph.h"
#include "enum.h"
#include "MADP_util.h"

//the difference we require a solution to be better ('V' for value)
#define V_EPS 1e-9



namespace libDAI {


    class MaxPlus : public DAIAlgFG {
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
            MaxPlus() : 
                DAIAlgFG(), Props(), _maxdiff(0.0), _iterations(0UL), _indices(),
                _messages(), _newmessages(), _g(NULL),_k(1),
                _k_th_Val(-DBL_MAX),  writeAnyTimeResults(false),  
                results_f(NULL), timings_f(NULL){};
            
            /// Construct MaxPlus object using the specified properties
            MaxPlus( const FactorGraph & fg, const Properties &opts, size_t k=1 );
            
            /// Copy constructor
            MaxPlus( const MaxPlus & x ) : DAIAlgFG(x), Props(x.Props), _maxdiff(x._maxdiff), _iterations(x._iterations), _indices(x._indices), _messages(x._messages), _newmessages(x._newmessages), _g(x._g) {};

            /// Assignment operator
            MaxPlus & operator=( const MaxPlus & x ) {
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
            virtual MaxPlus* clone() const {
                return new MaxPlus(*this);
            }

            /// Create (virtual constructor)
            virtual MaxPlus* create() const {
                return new MaxPlus();
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
            
            ///Turns Anytime results on and of 
            /**When turning on, valid ofstream pointers must be provided for the
             * results and timings file.
             */
            void SetAnyTimeResults(bool turn_on, std::ofstream* results, 
                    std::ofstream* timings);

// MaxPlus specific stuff
//
//
            ///Returns the best configuration 
            ///(the vector with the best value ('state') for each variable )
            ///Should be called after MaxPlus has run()
            const std::vector<size_t> & GetBestConfiguration(){return bestConfiguration;}
            std::list< MADP_util::valConf > & 
                GetBestConfigurations(){return _valConfs;}
            ///Returns the value of the best configuration 
            ///Should be called after MaxPlus has run()
            double GetBestValue() {return bestValue;}

            ///FRANS: this function computes the new message sent from factor I to variable i
            void calcNewMessage(size_t iI);
            void updateMessage(size_t iI);
           /* moved to cpp
            * {
                if( Props.damping <= 0.00001 )
                    _messages[iI] = _newmessages[iI];
                else
                    //this does not work correctly for arbitrary numbers
                    //_messages[iI] = (_messages[iI] ^ Props.damping) * (_newmessages[iI] ^ (1.0 - Props.damping));
                    //rather use regular pointwise multiplication
                    _messages[iI] = (_messages[iI] * Props.damping) * (_newmessages[iI] * (1.0 - Props.damping));
            }*/
            Factor beliefV (size_t i) const;
            Factor beliefF (size_t I) const;


            //const Prob & message(size_t i, size_t I) const { return( _messages[grm().edge(i,I)] ); }
            const Prob & message(size_t i, size_t I) const { return( _messages[_g->edge(i,I)] ); }
            ///this should be less costly:
            const FactorGraph& grm() const {return *_g;}

        private:
            Prob & message(size_t i, size_t I) { return( _messages[_g->edge(i,I)] ); }  
            Prob & newMessage(size_t i, size_t I) { return( _newmessages[_g->edge(i,I)] ); }    
            const Prob & newMessage(size_t i, size_t I) const { return( _newmessages[_g->edge(i,I)] ); }    
            _ind_t & index(size_t i, size_t I) { return( _indices[_g->edge(i,I)] ); }
            const _ind_t & index(size_t i, size_t I) const { return( _indices[_g->edge(i,I)] ); }
        
            ///calling grm() from superclass over and over is costly!, we store a pointer here and implement
            ///a local version of this function
            const FactorGraph * _g;// = grm();


            ///a vector that stores the best configuration found
            ///(for each variable it stores the index of the best value ('state') )
            std::vector<size_t> bestConfiguration;
            ///stores the value of the best configuration found so far.
            double bestValue;

            //the number of best configurations
            size_t _k;
            ///the list of _k best configurations, and their values
            /** .front() gives the valConf with k-th highest value, 
             *  .back() is the best found valConf
             */
            std::list< MADP_util::valConf > _valConfs;
            ///the value of the k-th first entry in _valConfs)
            double _k_th_Val;

            ///boolean that indicates whether writeAnyTimeResults results should be written
            bool writeAnyTimeResults;
            ///the file to which writes the results are written
            std::ofstream* results_f;
            ///the file to which writes the timings of the results are written
            std::ofstream* timings_f;


    };


}


#endif
