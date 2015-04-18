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


#include <iostream>
#include <fstream>
#include <vector>
#include "jtree.h"
#include "treeep.h"
#include "util.h"
#include "diffs.h"
#include "exceptions.h"


namespace libDAI {


    using namespace std;


    const char *TreeEP::Name = "TREEEP";


    bool TreeEP::initProps2() {
        if( !HasProperty("type") )
            return false;
        if( !HasProperty("tol") )
            return false;
        if( !HasProperty("maxiter") )
            return false;
        if( !HasProperty("verbose") )
            return false;
        
        Props2.type    = FromStringTo<TypeType>("type");
        Props2.tol     = FromStringTo<double>("tol");
        Props2.maxiter = FromStringTo<size_t>("maxiter");
        Props2.verbose = FromStringTo<size_t>("verbose");
        if( HasProperty("damping") )
            DAI_THROW( NOT_IMPLEMENTED );

        return true;
    }


    TreeEP::TreeEPSubTree::TreeEPSubTree( const DEdgeVec &subRTree, const DEdgeVec &jt_RTree, const std::vector<Factor> &jt_Qa, const std::vector<Factor> &jt_Qb, const Factor *I ) : _Qa(), _Qb(), _RTree(), _a(), _b(), _I(I), _ns(), _nsrem(), _logZ(0.0) {
        _ns = _I->vars();

        // Make _Qa, _Qb, _a and _b corresponding to the subtree
        _b.reserve( subRTree.size() );
        _Qb.reserve( subRTree.size() );
        _RTree.reserve( subRTree.size() );
        for( size_t i = 0; i < subRTree.size(); i++ ) {
            size_t alpha1 = subRTree[i].n1;     // old index 1
            size_t alpha2 = subRTree[i].n2;     // old index 2
            size_t beta;                        // old sep index
            for( beta = 0; beta < jt_RTree.size(); beta++ )
                if( UEdge( jt_RTree[beta].n1, jt_RTree[beta].n2 ) == UEdge( alpha1, alpha2 ) )
                    break;
            assert( beta != jt_RTree.size() );

            size_t newalpha1 = find(_a.begin(), _a.end(), alpha1) - _a.begin();
            if( newalpha1 == _a.size() ) {
                _Qa.push_back( Factor( jt_Qa[alpha1].vars(), 1.0 ) );
                _a.push_back( alpha1 );         // save old index in index conversion table
            } 

            size_t newalpha2 = find(_a.begin(), _a.end(), alpha2) - _a.begin();
            if( newalpha2 == _a.size() ) {
                _Qa.push_back( Factor( jt_Qa[alpha2].vars(), 1.0 ) );
                _a.push_back( alpha2 );         // save old index in index conversion table
            }
            
            _RTree.push_back( DEdge( newalpha1, newalpha2 ) );
            _Qb.push_back( Factor( jt_Qb[beta].vars(), 1.0 ) );
            _b.push_back( beta );
        }

        // Find remaining variables (which are not in the new root)
        _nsrem = _ns / _Qa[0].vars();
    }


    void TreeEP::TreeEPSubTree::init() { 
        for( size_t alpha = 0; alpha < _Qa.size(); alpha++ )
            _Qa[alpha].fill( 1.0 );
        for( size_t beta = 0; beta < _Qb.size(); beta++ )
            _Qb[beta].fill( 1.0 );
    }


    void TreeEP::TreeEPSubTree::InvertAndMultiply( const std::vector<Factor> &Qa, const std::vector<Factor> &Qb ) {
        for( size_t alpha = 0; alpha < _Qa.size(); alpha++ )
            _Qa[alpha] = Qa[_a[alpha]].divided_by( _Qa[alpha] );

        for( size_t beta = 0; beta < _Qb.size(); beta++ )
            _Qb[beta] = Qb[_b[beta]].divided_by( _Qb[beta] );
    }


    void TreeEP::TreeEPSubTree::HUGIN_with_I( std::vector<Factor> &Qa, std::vector<Factor> &Qb ) {
        multind mi( _nsrem );

        // Backup _Qa and _Qb
        vector<Factor> _Qa_old(_Qa);
        vector<Factor> _Qb_old(_Qb);

        // Clear Qa and Qb
        for( size_t alpha = 0; alpha < _Qa.size(); alpha++ )
            Qa[_a[alpha]].fill( 0.0 );
        for( size_t beta = 0; beta < _Qb.size(); beta++ )
            Qb[_b[beta]].fill( 0.0 );
        
        // For all states of _nsrem
        for( size_t j = 0; j < mi.max(); j++ ) {
            vector<size_t> vi = mi.vi( j );
            
            // Multiply root with slice of I
            _Qa[0] *= _I->slice( _nsrem, j );

            // CollectEvidence
            for( size_t i = _RTree.size(); (i--) != 0; ) {
                // clamp variables in nsrem
                size_t k = 0;
                for( VarSet::const_iterator n = _nsrem.begin(); n != _nsrem.end(); n++, k++ )
                    if( _Qa[_RTree[i].n2].vars() >> *n ) {
                        Factor delta( *n, 0.0 );
                        delta[vi[k]] = 1.0;
                        _Qa[_RTree[i].n2] *= delta;
                    }
                Factor new_Qb = _Qa[_RTree[i].n2].partSum( _Qb[i].vars() );
                _Qa[_RTree[i].n1] *= new_Qb.divided_by( _Qb[i] ); 
                _Qb[i] = new_Qb;
            }

            // DistributeEvidence
            for( size_t i = 0; i < _RTree.size(); i++ ) {
                Factor new_Qb = _Qa[_RTree[i].n1].partSum( _Qb[i].vars() );
                _Qa[_RTree[i].n2] *= new_Qb.divided_by( _Qb[i] ); 
                _Qb[i] = new_Qb;
            }

            // Store Qa's and Qb's
            for( size_t alpha = 0; alpha < _Qa.size(); alpha++ )
                Qa[_a[alpha]].p() += _Qa[alpha].p();
            for( size_t beta = 0; beta < _Qb.size(); beta++ )
                Qb[_b[beta]].p() += _Qb[beta].p();

            // Restore _Qa and _Qb
            _Qa = _Qa_old;
            _Qb = _Qb_old;
        }

        // Normalize Qa and Qb
        _logZ = 0.0;
        for( size_t alpha = 0; alpha < _Qa.size(); alpha++ ) {
            _logZ += log(Qa[_a[alpha]].totalSum());
            Qa[_a[alpha]].normalize();
        }
        for( size_t beta = 0; beta < _Qb.size(); beta++ ) {
            _logZ -= log(Qb[_b[beta]].totalSum());
            Qb[_b[beta]].normalize();
        }
    }


    double TreeEP::TreeEPSubTree::logZ( const std::vector<Factor> &Qa, const std::vector<Factor> &Qb ) const {
        double sum = 0.0;
        for( size_t alpha = 0; alpha < _Qa.size(); alpha++ )
            sum += (Qa[_a[alpha]] * _Qa[alpha].log0()).totalSum();
        for( size_t beta = 0; beta < _Qb.size(); beta++ )
            sum -= (Qb[_b[beta]] * _Qb[beta].log0()).totalSum();
        return sum + _logZ;
    }


    TreeEP::TreeEP( const FactorGraph &fg, const Properties &opts ) : JTree(fg, opts("updates",string("HUGIN")), false), Props2(), _maxdiff(0.0), _iterations(0UL), _Q() {
        if( !initProps2() )
            DAI_THROW(NOT_ALL_PROPERTIES_SPECIFIED);

        assert( fg.isConnected() );

        if( opts.hasKey("tree") ) {
            ConstructRG( opts.GetAs<DEdgeVec>("tree") );
        } else {
            if( Props2.type == TypeType::ORG || Props2.type == TypeType::ALT ) {
                // ORG: construct weighted graph with as weights a crude estimate of the
                // mutual information between the nodes
                // ALT: construct weighted graph with as weights an upper bound on the
                // effective interaction strength between pairs of nodes
                WeightedGraph<double> wg;
                double maxWeight = 0.0;
                for( vector<Var>::const_iterator i = grm().vars().begin(); i != grm().vars().end(); i++ ) {
                    VarSet di = grm().delta(*i);
                    for( VarSet::const_iterator j = di.begin(); j != di.end(); j++ )
                        if( *i < *j ) {
                            Factor piet;
                            for( size_t I = 0; I < grm().nrFactors(); I++ ) {
                                VarSet Ivars = grm().factor(I).vars();
                                if( Props2.type == TypeType::ORG ) {
                                    if( (Ivars == *i) || (Ivars == *j) )
                                        piet *= grm().factor(I);
                                    else if( Ivars >> (*i | *j) )
                                        piet *= grm().factor(I).marginal( *i | *j );
                                } else {
                                    if( Ivars >> (*i | *j) )
                                        piet *= grm().factor(I);
                                }
                            }
                            if( Props2.type == TypeType::ORG ) {
                                if( piet.vars() >> (*i | *j) ) {
                                    piet = piet.marginal( *i | *j );
                                    Factor pietf = piet.marginal(*i) * piet.marginal(*j);
                                    double w = real( KL_dist( piet, pietf ) );
                                    if( w > maxWeight )
                                        maxWeight = w;
                                    wg[UEdge(grm().findVar(*i),grm().findVar(*j))] = -w;
                                } else
                                    wg[UEdge(grm().findVar(*i),grm().findVar(*j))] = 0.0;
                            } else {
                                double w = piet.strength(*i, *j);
                                if( w > maxWeight )
                                    maxWeight = w;
                                wg[UEdge(grm().findVar(*i),grm().findVar(*j))] = -w;
                            }
                        }
                }
                // add constant because BGL implementation of Prim's algorithm cannot handle negative weights
                for( WeightedGraph<double>::iterator e = wg.begin(); e != wg.end(); e++ )
                    e->second += maxWeight;

                // find minimal spanning tree
                ConstructRG( MinSpanningTreePrims( wg ) );
            } else
                DAI_THROW(UNKNOWN_ENUM_VALUE);
        }
    }


    void TreeEP::ConstructRG( const DEdgeVec &tree ) {
        vector<VarSet> Cliques;
        for( size_t i = 0; i < tree.size(); i++ )
            Cliques.push_back( grm().var(tree[i].n1) | grm().var(tree[i].n2) );
        
        // Construct a weighted graph (each edge is weighted with the cardinality 
        // of the intersection of the nodes, where the nodes are the elements of
        // Cliques).
        WeightedGraph<int> JuncGraph;
        size_t N = grm().nrVars();
        for( size_t i = 0; i < Cliques.size(); i++ )
            for( size_t j = i+1; j < Cliques.size(); j++ ) {
                size_t w = (Cliques[i] & Cliques[j]).size();
                if( w ) 
                    JuncGraph[UEdge(i,j)] = N - w;
            }
        
        // Construct maximal spanning tree using Prim's algorithm
        _RTree = MinSpanningTreePrims( JuncGraph );

        // Construct corresponding region graph

        // Create outer regions
        grm().ORs().reserve( Cliques.size() );
        for( size_t i = 0; i < Cliques.size(); i++ )
            grm().ORs().push_back( FRegion( Factor(Cliques[i], 1.0), 1.0 ) );

        // For each factor, find an outer region that subsumes that factor.
        // Then, multiply the outer region with that factor.
        // If no outer region can be found subsuming that factor, label the
        // factor as off-tree.
        for( size_t I = 0; I < grm().nrFactors(); I++ ) {
            size_t alpha;
            for( alpha = 0; alpha < grm().nr_ORs(); alpha++ )
                if( grm().OR(alpha).vars() >> grm().factor(I).vars() ) {
                    grm().setFac2OR(I, alpha);
                    break;
                }
        // DIFF WITH JTree::GenerateJT:      assert
        }
        grm().RecomputeORs();

        // Create inner regions and edges
        grm().IRs().reserve( _RTree.size() );
        grm().Redges().reserve( 2 * _RTree.size() );
        for( size_t i = 0; i < _RTree.size(); i++ ) {
            grm().Redges().push_back( RegionGraph::R_edge_t( _RTree[i].n1, grm().IRs().size() ) );
            grm().Redges().push_back( RegionGraph::R_edge_t( _RTree[i].n2, grm().IRs().size() ) );
            // inner clusters have counting number -1
            grm().IRs().push_back( Region( Cliques[_RTree[i].n1] & Cliques[_RTree[i].n2], -1.0 ) );
        }

        // Regenerate BipartiteGraph internals
        grm().Regenerate();

        // Check counting numbers
        grm().Check_Counting_Numbers();
        
        // Create messages and beliefs
        _Qa.clear();
        _Qa.reserve( grm().nr_ORs() );
        for( size_t alpha = 0; alpha < grm().nr_ORs(); alpha++ )
            _Qa.push_back( grm().OR(alpha) );

        _Qb.clear();
        _Qb.reserve( grm().nr_IRs() );
        for( size_t beta = 0; beta < grm().nr_IRs(); beta++ ) 
            _Qb.push_back( Factor( grm().IR(beta), 1.0 ) );

        // DIFF with JTree::GenerateJT:  no messages
        
        // DIFF with JTree::GenerateJT:
        // Create factor approximations
        _Q.clear();
        size_t PreviousRoot = (size_t)-1;
        for( size_t I = 0; I < grm().nrFactors(); I++ )
            if( offtree(I) ) {
                // find efficient subtree
                DEdgeVec subTree;
                size_t subTreeSize = findEfficientTree( grm().factor(I).vars(), subTree, PreviousRoot );
                PreviousRoot = subTree[0].n1;
                //subTree.resize( subTreeSize );  // FIXME
    //          cout << "subtree " << I << " has size " << subTreeSize << endl;

    /*
                char fn[30];
                sprintf( fn, "/tmp/subtree_%d.dot", I );
                std::ofstream dots(fn);
                dots << "graph G {" << endl;
                dots << "graph[size=\"9,9\"];" << endl;
                dots << "node[shape=circle,width=0.4,fixedsize=true];" << endl;
                for( size_t i = 0; i < grm().nrVars(); i++ )
                    dots << "\tx" << grm().var(i).label() << ((grm().factor(I).vars() >> grm().var(i)) ? "[color=blue];" : ";") << endl;
                dots << "node[shape=box,style=filled,color=lightgrey,width=0.3,height=0.3,fixedsize=true];" << endl;
                for( size_t J = 0; J < grm().nrFactors(); J++ )
                    dots << "\tp" << J << ";" << endl;
                for( size_t iI = 0; iI < FactorGraph::nrEdges(); iI++ )
                    dots << "\tx" << grm().var(FactorGraph::edge(iI).first).label() << " -- p" << FactorGraph::edge(iI).second << ";" << endl;
                for( size_t a = 0; a < tree.size(); a++ )
                    dots << "\tx" << grm().var(tree[a].n1).label() << " -- x" << grm().var(tree[a].n2).label() << " [color=red];" << endl;
                dots << "}" << endl;
                dots.close();
    */
                
                TreeEPSubTree QI( subTree, _RTree, _Qa, _Qb, &grm().factor(I) );
                _Q[I] = QI;
            }
        // Previous root of first off-tree factor should be the root of the last off-tree factor
        for( size_t I = 0; I < grm().nrFactors(); I++ )
            if( offtree(I) ) {
                DEdgeVec subTree;
                size_t subTreeSize = findEfficientTree( grm().factor(I).vars(), subTree, PreviousRoot );
                PreviousRoot = subTree[0].n1;
                //subTree.resize( subTreeSize ); // FIXME
    //          cout << "subtree " << I << " has size " << subTreeSize << endl;

                TreeEPSubTree QI( subTree, _RTree, _Qa, _Qb, &grm().factor(I) );
                _Q[I] = QI;
                break;
            }

        if( Verbose() >= 3 ) {
            cout << "Resulting regiongraph: " << grm() << endl;
        }
    }


    string TreeEP::identify() const { 
        stringstream result (stringstream::out);
        result << Name << GetProperties();
        return result.str();
    }


    void TreeEP::init() {
        if( !initProps2() )
            DAI_THROW(NOT_ALL_PROPERTIES_SPECIFIED);

        runHUGIN();

        // Init factor approximations
        for( size_t I = 0; I < grm().nrFactors(); I++ )
            if( offtree(I) )
                _Q[I].init();
    }


    double TreeEP::run() {
        if( Verbose() >= 1 )
            cout << "Starting " << identify() << "...";
        if( Verbose() >= 3)
            cout << endl;

        clock_t tic = toc();
        Diffs diffs(grm().nrVars(), 1.0);

        vector<Factor> old_beliefs;
        old_beliefs.reserve( grm().nrVars() );
        for( size_t i = 0; i < grm().nrVars(); i++ )
            old_beliefs.push_back(belief(grm().var(i)));

        // do several passes over the network until maximum number of iterations has
        // been reached or until the maximum belief difference is smaller than tolerance
        for( _iterations = 0; _iterations < Props2.maxiter && diffs.max() > Props2.tol; _iterations++ ) {
            for( size_t I = 0; I < grm().nrFactors(); I++ )
                if( offtree(I) ) {  
                    _Q[I].InvertAndMultiply( _Qa, _Qb );
                    _Q[I].HUGIN_with_I( _Qa, _Qb );
                    _Q[I].InvertAndMultiply( _Qa, _Qb );
                }

            // calculate new beliefs and compare with old ones
            for( size_t i = 0; i < grm().nrVars(); i++ ) {
                Factor nb( belief(grm().var(i)) );
                diffs.push( dist( nb, old_beliefs[i], Prob::DISTLINF ) );
                old_beliefs[i] = nb;
            }

            if( Verbose() >= 3 )
                cout << "TreeEP::run:  maxdiff " << diffs.max() << " after " << _iterations+1 << " passes" << endl;
        }

        if( diffs.max() > _maxdiff )
            _maxdiff = diffs.max();

        if( Verbose() >= 1 ) {
            if( diffs.max() > Props2.tol ) {
                if( Verbose() == 1 )
                    cout << endl;
                cout << "TreeEP::run:  WARNING: not converged within " << Props2.maxiter << " passes (" << toc() - tic << " clocks)...final maxdiff:" << diffs.max() << endl;
            } else {
                if( Verbose() >= 3 )
                    cout << "TreeEP::run:  ";
                cout << "converged in " << _iterations << " passes (" << toc() - tic << " clocks)." << endl;
            }
        }

        return diffs.max();
    }


    Complex TreeEP::logZ() const {
        double sum = 0.0;

        // entropy of the tree
        for( size_t beta = 0; beta < grm().nr_IRs(); beta++ )
            sum -= real(_Qb[beta].entropy());
        for( size_t alpha = 0; alpha < grm().nr_ORs(); alpha++ )
            sum += real(_Qa[alpha].entropy());

        // energy of the on-tree factors
        for( size_t alpha = 0; alpha < grm().nr_ORs(); alpha++ )
            sum += (grm().OR(alpha).log0() * _Qa[alpha]).totalSum();

        // energy of the off-tree factors
        for( size_t I = 0; I < grm().nrFactors(); I++ )
            if( offtree(I) )
                sum += (_Q.find(I))->second.logZ( _Qa, _Qb );
        
        return sum;
    }


}
