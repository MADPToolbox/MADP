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
#include "jtree.h"
#include "clustergraph.h"
#include "exceptions.h"


namespace libDAI {


    using namespace std;


    const char *JTree::Name = "JTREE";


    bool JTree::initProps() {
        if( !HasProperty("updates") )
            return false;
        if( !HasProperty("verbose") )
            return false;
        
        Props.updates = FromStringTo<UpdateType>("updates");
        Props.verbose = FromStringTo<size_t>("verbose");

        return true;
    }


    JTree::JTree( const FactorGraph &fg, const Properties &opts, bool automatic ) : DAIAlgRG(fg, opts), Props(), _RTree(), _Qa(), _Qb(), _mes(), _logZ() {
        if( !initProps() )
            DAI_THROW(NOT_ALL_PROPERTIES_SPECIFIED);

        if( !grm().isConnected() ) 
           DAI_THROW(FACTORGRAPH_NOT_CONNECTED); 

        if( automatic ) {
            // Create ClusterGraph which contains factors as clusters
            vector<VarSet> cl;
            for( size_t I = 0; I < grm().nrFactors(); I++ )
                cl.push_back( grm().factor(I).vars() );
            ClusterGraph _cg(cl);

            if( Verbose() >= 3 ) {
                cout << "Initial clusters: " << _cg << endl;
            }

            // Retain only maximal clusters
            _cg.eraseNonMaximal();
            if( Verbose() >= 3 )
                cout << "Maximal clusters: " << _cg << endl;

            vector<VarSet> ElimVec = _cg.VarElim_MinFill().eraseNonMaximal().toVector();
            if( Verbose() >= 3 ) {
                cout << "VarElim_MinFill result: {" << endl;
                for( size_t i = 0; i < ElimVec.size(); i++ ) {
                    if( i != 0 )
                        cout << ", ";
                    cout << ElimVec[i];
                }
                cout << "}" << endl;
            }

            GenerateJT( ElimVec );
        }
    }


    void JTree::GenerateJT( const std::vector<VarSet> &Cliques ) {
        // Construct a weighted graph (each edge is weighted with the cardinality 
        // of the intersection of the nodes, where the nodes are the elements of
        // Cliques).
        size_t N = grm().nrVars();
        WeightedGraph<int> JuncGraph;
        for( size_t i = 0; i < Cliques.size(); i++ )
            for( size_t j = i+1; j < Cliques.size(); j++ ) {
                size_t w = (Cliques[i] & Cliques[j]).size();
                if( w ) 
                    JuncGraph[UEdge(i,j)] = N - w;
            }
        
        // Construct minimal spanning tree using Prim's algorithm
        _RTree = MinSpanningTreePrims( JuncGraph);

        // Construct corresponding region graph

        // Create outer regions
        grm().ORs().reserve( Cliques.size() );
        for( size_t i = 0; i < Cliques.size(); i++ )
            grm().ORs().push_back( FRegion( Factor(Cliques[i], 1.0), 1.0 ) );

        // For each factor, find an outer region that subsumes that factor.
        // Then, multiply the outer region with that factor.
        for( size_t I = 0; I < grm().nrFactors(); I++ ) {
            size_t alpha;
            for( alpha = 0; alpha < grm().nr_ORs(); alpha++ )
                if( grm().OR(alpha).vars() >> grm().factor(I).vars() ) {
    //              grm().OR(alpha) *= grm().factor(I);
                    grm().setFac2OR(I, alpha);
                    break;
                }
            assert( alpha != grm().nr_ORs() );
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

        // Create messages and beliefs
        _Qa.clear();
        _Qa.reserve( grm().nr_ORs() );
        for( size_t alpha = 0; alpha < grm().nr_ORs(); alpha++ )
            _Qa.push_back( grm().OR(alpha) );

        _Qb.clear();
        _Qb.reserve( grm().nr_IRs() );
        for( size_t beta = 0; beta < grm().nr_IRs(); beta++ ) 
            _Qb.push_back( Factor( grm().IR(beta), 1.0 ) );

        _mes.clear();
        _mes.reserve( grm().nr_Redges() );
        for( size_t e = 0; e < grm().nr_Redges(); e++ )
            _mes.push_back( Factor( grm().IR(grm().Redge(e).second), 1.0 ) );

        // Check counting numbers
        grm().Check_Counting_Numbers();

        if( Verbose() >= 3 ) {
            cout << "Resulting regiongraph: " << grm() << endl;
        }
    }


    void JTree::init() {
        if( !initProps() )
            DAI_THROW(NOT_ALL_PROPERTIES_SPECIFIED);
    }


    string JTree::identify() const {
        stringstream result (stringstream::out);
        result << Name << GetProperties();
        return result.str();
    }


    Factor JTree::belief( const VarSet &ns ) const {
        vector<Factor>::const_iterator beta;
        for( beta = _Qb.begin(); beta != _Qb.end(); beta++ )
            if( beta->vars() >> ns )
                break;
        if( beta != _Qb.end() )
            return( beta->marginal(ns) );
        else {
            vector<Factor>::const_iterator alpha;
            for( alpha = _Qa.begin(); alpha != _Qa.end(); alpha++ )
                if( alpha->vars() >> ns )
                    break;
            assert( alpha != _Qa.end() );
            return( alpha->marginal(ns) );
        }
    }


    vector<Factor> JTree::beliefs() const {
        vector<Factor> result;
        for( size_t beta = 0; beta < grm().nr_IRs(); beta++ )
            result.push_back( _Qb[beta] );
        for( size_t alpha = 0; alpha < grm().nr_ORs(); alpha++ )
            result.push_back( _Qa[alpha] );
        return result;
    }


    Factor JTree::belief( const Var &n ) const {
        return belief( (VarSet)n );
    }


    // Needs no init
    void JTree::runHUGIN() {
        for( size_t alpha = 0; alpha < grm().nr_ORs(); alpha++ )
            _Qa[alpha] = grm().OR(alpha);

        for( size_t beta = 0; beta < grm().nr_IRs(); beta++ )
            _Qb[beta].fill( 1.0 );

        // CollectEvidence
        _logZ = 0.0;
        for( size_t i = _RTree.size(); (i--) != 0; ) {
    //      Make outer region _RTree[i].n1 consistent with outer region _RTree[i].n2
    //      IR(i) = seperator OR(_RTree[i].n1) && OR(_RTree[i].n2)
            Factor new_Qb = _Qa[_RTree[i].n2].partSum( grm().IR( i ) );
            _logZ += log(new_Qb.normalize());
            _Qa[_RTree[i].n1] *= new_Qb.divided_by( _Qb[i] ); 
            _Qb[i] = new_Qb;
        }
        if( _RTree.empty() )
            _logZ += log(_Qa[0].normalize() );
        else
            _logZ += log(_Qa[_RTree[0].n1].normalize());

        // DistributeEvidence
        for( size_t i = 0; i < _RTree.size(); i++ ) {
    //      Make outer region _RTree[i].n2 consistent with outer region _RTree[i].n1
    //      IR(i) = seperator OR(_RTree[i].n1) && OR(_RTree[i].n2)
            Factor new_Qb = _Qa[_RTree[i].n1].marginal( grm().IR( i ) );
            _Qa[_RTree[i].n2] *= new_Qb.divided_by( _Qb[i] ); 
            _Qb[i] = new_Qb;
        }

        // Normalize
        for( size_t alpha = 0; alpha < grm().nr_ORs(); alpha++ )
            _Qa[alpha].normalize();
    }


    // Really needs no init! Initial messages can be anything.
    void JTree::runShaferShenoy() {
        // First pass
        _logZ = 0.0;
        for( size_t e = _RTree.size(); (e--) != 0; ) {
            // send a message from _RTree[e].n2 to _RTree[e].n1
            // or, actually, from the seperator IR(e) to _RTree[e].n1

            size_t i = _RTree[e].n2;
            size_t j = _RTree[e].n1;
            
            Factor piet = grm().OR(i);
            for( RegionGraph::R_nb_cit k = grm().nbOR(i).begin(); k != grm().nbOR(i).end(); k++ )
                if( *k != e )
                    piet *= message( i, *k );
            message( j, e ) = piet.partSum( grm().IR(e) );
            _logZ += log( message(j,e).normalize() );
        }

        // Second pass
        for( size_t e = 0; e < _RTree.size(); e++ ) {
            size_t i = _RTree[e].n1;
            size_t j = _RTree[e].n2;
            
            Factor piet = grm().OR(i);
            for( RegionGraph::R_nb_cit k = grm().nbOR(i).begin(); k != grm().nbOR(i).end(); k++ )
                if( *k != e )
                    piet *= message( i, *k );
            message( j, e ) = piet.marginal( grm().IR(e) );
        }

        // Calculate beliefs
        for( size_t alpha = 0; alpha < grm().nr_ORs(); alpha++ ) {
            Factor piet = grm().OR(alpha);
            for( RegionGraph::R_nb_cit k = grm().nbOR(alpha).begin(); k != grm().nbOR(alpha).end(); k++ )
                piet *= message( alpha, *k );
            if( _RTree.empty() ) {
                _logZ += log( piet.normalize() );
                _Qa[alpha] = piet;
            } else if( alpha == _RTree[0].n1 ) {
                _logZ += log( piet.normalize() );
                _Qa[alpha] = piet;
            } else
                _Qa[alpha] = piet.normalized();
        }

        // Only for logZ (and for belief)...
        for( size_t beta = 0; beta < grm().nr_IRs(); beta++ ) 
            _Qb[beta] = _Qa[grm().nbIR(beta)[0]].marginal( grm().IR(beta) );
    }


    double JTree::run() {
        if( Props.updates == UpdateType::HUGIN )
            runHUGIN();
        else if( Props.updates == UpdateType::SHSH )
            runShaferShenoy();
        return 0.0;
    }


    Complex JTree::logZ() const {
        Complex sum = 0.0;
        for( size_t beta = 0; beta < grm().nr_IRs(); beta++ )
            sum += Complex(grm().IR(beta).c()) * _Qb[beta].entropy();
        for( size_t alpha = 0; alpha < grm().nr_ORs(); alpha++ ) {
            sum += Complex(grm().OR(alpha).c()) * _Qa[alpha].entropy();
            sum += (grm().OR(alpha).log0() * _Qa[alpha]).totalSum();
        }
        return sum;
    }



    size_t JTree::findEfficientTree( const VarSet& ns, DEdgeVec &Tree, size_t PreviousRoot ) const {
        // find new root clique (the one with maximal statespace overlap with ns)
        size_t maxval = 0, maxalpha = 0;
        for( size_t alpha = 0; alpha < grm().nr_ORs(); alpha++ ) {
            size_t val = (ns & grm().OR(alpha).vars()).stateSpace();
            if( val > maxval ) {
                maxval = val;
                maxalpha = alpha;
            }
        }

    //  for( size_t e = 0; e < _RTree.size(); e++ )
    //      cout << grm().OR(_RTree[e].n1).vars() << "->" << grm().OR(_RTree[e].n2).vars() << ",  ";
    //  cout << endl;
        // grow new tree
        Graph oldTree;
        for( DEdgeVec::const_iterator e = _RTree.begin(); e != _RTree.end(); e++ )
            oldTree.insert( UEdge(e->n1, e->n2) );
        DEdgeVec newTree = GrowRootedTree( oldTree, maxalpha );
    //  cout << ns << ": ";
    //  for( size_t e = 0; e < newTree.size(); e++ )
    //      cout << grm().OR(newTree[e].n1).vars() << "->" << grm().OR(newTree[e].n2).vars() << ",  ";
    //  cout << endl;
        
        // identify subtree that contains variables of ns which are not in the new root
        VarSet nsrem = ns / grm().OR(maxalpha).vars();
    //  cout << "nsrem:" << nsrem << endl;
        set<DEdge> subTree;
        // for each variable in ns that is not in the root clique
        for( VarSet::const_iterator n = nsrem.begin(); n != nsrem.end(); n++ ) {
            // find first occurence of *n in the tree, which is closest to the root
            size_t e = 0;
            for( ; e != newTree.size(); e++ ) {
                if( grm().OR(newTree[e].n2).vars() && *n )
                    break;
            }
            assert( e != newTree.size() );

            // track-back path to root and add edges to subTree
            subTree.insert( newTree[e] );
            size_t pos = newTree[e].n1;
            for( ; e > 0; e-- )
                if( newTree[e-1].n2 == pos ) {
                    subTree.insert( newTree[e-1] );
                    pos = newTree[e-1].n1;
                }
        }
        if( PreviousRoot != (size_t)-1 && PreviousRoot != maxalpha) {
            // find first occurence of PreviousRoot in the tree, which is closest to the new root
            size_t e = 0;
            for( ; e != newTree.size(); e++ ) {
                if( newTree[e].n2 == PreviousRoot )
                    break;
            }
            assert( e != newTree.size() );

            // track-back path to root and add edges to subTree
            subTree.insert( newTree[e] );
            size_t pos = newTree[e].n1;
            for( ; e > 0; e-- )
                if( newTree[e-1].n2 == pos ) {
                    subTree.insert( newTree[e-1] );
                    pos = newTree[e-1].n1;
                }
        }
    //  cout << "subTree: " << endl;
    //  for( set<DEdge>::const_iterator sTi = subTree.begin(); sTi != subTree.end(); sTi++ )
    //      cout << grm().OR(sTi->n1).vars() << "->" << grm().OR(sTi->n2).vars() << ",  ";
    //  cout << endl;

        // Resulting Tree is a reordered copy of newTree
        // First add edges in subTree to Tree
        Tree.clear();
        for( DEdgeVec::const_iterator e = newTree.begin(); e != newTree.end(); e++ )
            if( subTree.count( *e ) ) {
                Tree.push_back( *e );
    //          cout << grm().OR(e->n1).vars() << "->" << grm().OR(e->n2).vars() << ",  ";
            }
    //  cout << endl;
        // Then add edges pointing away from nsrem
        // FIXME
    /*  for( DEdgeVec::const_iterator e = newTree.begin(); e != newTree.end(); e++ )
            for( set<DEdge>::const_iterator sTi = subTree.begin(); sTi != subTree.end(); sTi++ )
                if( *e != *sTi ) {
                    if( e->n1 == sTi->n1 || e->n1 == sTi->n2 ||
                        e->n2 == sTi->n1 || e->n2 == sTi->n2 ) {
                        Tree.push_back( *e );
    //                  cout << grm().OR(e->n1).vars() << "->" << grm().OR(e->n2).vars() << ",  ";
                    }
                }*/
        // FIXME
    /*  for( DEdgeVec::const_iterator e = newTree.begin(); e != newTree.end(); e++ )
            if( find( Tree.begin(), Tree.end(), *e) == Tree.end() ) {
                bool found = false;
                for( VarSet::const_iterator n = nsrem.begin(); n != nsrem.end(); n++ )
                    if( (grm().OR(e->n1).vars() && *n) ) {
                        found = true;
                        break;
                    }
                if( found ) {
                    Tree.push_back( *e );
                    cout << grm().OR(e->n1).vars() << "->" << grm().OR(e->n2).vars() << ",  ";
                }
            }
        cout << endl;*/
        size_t subTreeSize = Tree.size();
        // Then add remaining edges
        for( DEdgeVec::const_iterator e = newTree.begin(); e != newTree.end(); e++ )
            if( find( Tree.begin(), Tree.end(), *e ) == Tree.end() )
                Tree.push_back( *e );

        return subTreeSize;
    }


    // Cutset conditioning
    // assumes that run() has been called already
    Factor JTree::calcMarginal( const VarSet& ns ) {
        vector<Factor>::const_iterator beta;
        for( beta = _Qb.begin(); beta != _Qb.end(); beta++ )
            if( beta->vars() >> ns )
                break;
        if( beta != _Qb.end() )
            return( beta->marginal(ns) );
        else {
            vector<Factor>::const_iterator alpha;
            for( alpha = _Qa.begin(); alpha != _Qa.end(); alpha++ )
                if( alpha->vars() >> ns )
                    break;
            if( alpha != _Qa.end() )
                return( alpha->marginal(ns) );
            else {
                // Find subtree to do efficient inference
                DEdgeVec T;
                size_t Tsize = findEfficientTree( ns, T );

                // Find remaining variables (which are not in the new root)
                VarSet nsrem = ns / grm().OR(T.front().n1).vars();
                Factor Pns (ns, 0.0);
                
                multind mi( nsrem );

                // Save _Qa and _Qb on the subtree
                map<size_t,Factor> _Qa_old;
                map<size_t,Factor> _Qb_old;
                vector<size_t> b(Tsize, 0);
                for( size_t i = Tsize; (i--) != 0; ) {
                    size_t alpha1 = T[i].n1;
                    size_t alpha2 = T[i].n2;
                    size_t beta;
                    for( beta = 0; beta < grm().nr_IRs(); beta++ )
                        if( UEdge( _RTree[beta].n1, _RTree[beta].n2 ) == UEdge( alpha1, alpha2 ) )
                            break;
                    assert( beta != grm().nr_IRs() );
                    b[i] = beta;

                    if( !_Qa_old.count( alpha1 ) )
                        _Qa_old[alpha1] = _Qa[alpha1];
                    if( !_Qa_old.count( alpha2 ) )
                        _Qa_old[alpha2] = _Qa[alpha2];
                    if( !_Qb_old.count( beta ) )
                        _Qb_old[beta] = _Qb[beta];
                }
                    
                // For all states of nsrem
                for( size_t j = 0; j < mi.max(); j++ ) {
                    vector<size_t> vi = mi.vi( j );
                    
                    // CollectEvidence
                    double logZ = 0.0;
                    for( size_t i = Tsize; (i--) != 0; ) {
                //      Make outer region T[i].n1 consistent with outer region T[i].n2
                //      IR(i) = seperator OR(T[i].n1) && OR(T[i].n2)

                        size_t k = 0;
                        for( VarSet::const_iterator n = nsrem.begin(); n != nsrem.end(); n++, k++ )
                            if( _Qa[T[i].n2].vars() >> *n ) {
                                Factor piet( *n, 0.0 );
                                piet[vi[k]] = 1.0;
                                _Qa[T[i].n2] *= piet; 
                            }

                        Factor new_Qb = _Qa[T[i].n2].partSum( grm().IR( b[i] ) );
                        logZ += log(new_Qb.normalize());
                        _Qa[T[i].n1] *= new_Qb.divided_by( _Qb[b[i]] ); 
                        _Qb[b[i]] = new_Qb;
                    }
                    logZ += log(_Qa[T[0].n1].normalize());

                    Factor piet( nsrem, 0.0 );
                    piet[j] = exp(logZ);
                    Pns += piet * _Qa[T[0].n1].partSum( ns / nsrem );      // OPTIMIZE ME

                    // Restore clamped beliefs
                    for( map<size_t,Factor>::const_iterator alpha = _Qa_old.begin(); alpha != _Qa_old.end(); alpha++ )
                        _Qa[alpha->first] = alpha->second;
                    for( map<size_t,Factor>::const_iterator beta = _Qb_old.begin(); beta != _Qb_old.end(); beta++ )
                        _Qb[beta->first] = beta->second;
                }

                return( Pns.normalized() );
            }
        }
    }


}
