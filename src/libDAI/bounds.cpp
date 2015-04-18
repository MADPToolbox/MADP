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


#include "bounds.h"
#include "bp.h"


namespace libDAI {

    
    using namespace std;


    void BoundTree::buildTree( const FactorGraph &fg, const Var &v_root, size_t maxnodes, bool SAW ) {
        // init
        _new_node_id = 0;
        _maxnodes = maxnodes;
        _saw = SAW;
        tr.clear();

        // create root node (level 0)
        tree<BoundTreeNode>::iterator root = tr.insert( tr.begin(), BoundTreeNode( _new_node_id++, fg.findVar( v_root ), BoundTreeNode::VAR, false ) );

        // grow the tree breadth-first
        size_t nr_nodes_upto_current_level = 1;
        for( size_t level = 0; ; level++ ) {
            size_t children_added = 0; // child nodes added for this level
            tree<BoundTreeNode>::fixed_depth_iterator level_begin = tr.begin_fixed( tr.begin(), level );  // could go out of range in old version of tree.hh

            for( tree<BoundTreeNode>::fixed_depth_iterator node = level_begin; tr.is_valid(node); node++ ) { // for all nodes on this level
                if( (!node->_forcedleaf) && (level == 0 || (!tr.parent(node)->_forcedleaf)) ) { // if the node and its parent are not forced leaf nodes
                    if( node->_type == BoundTreeNode::VAR ) { // if the node is a variable
                        bool has_children = false;
                        for( FactorGraph::nb_cit I = fg.nbV(node->_index).begin(); I != fg.nbV(node->_index).end(); I++ )
                            if( (level == 0) || (tr.parent(node)->_type != BoundTreeNode::FACTOR) || (*I != tr.parent(node)->_index) ) {
                                has_children = true;
                                if( _saw ) // we only need to know has_children so we can stop if it is true
                                    break;
                                else {
                                    // if one of the candidate children of this node has already been used in the tree, this node should be a forced leaf node
                                    if( find( BoundTreeNode::FACTOR, *I ) != tr.end() ) {
                                        node->_forcedleaf = true;
                                        break;
                                    }
                                }
                            }
                        if( has_children ) {
                            if( nr_nodes_upto_current_level >= _maxnodes )
                                node->_forcedleaf = true; // if the maximum number of nodes is reached, stop
                            if( !node->_forcedleaf ) { // add all children if it is not a forced leaf node
                                for( FactorGraph::nb_cit I = fg.nbV(node->_index).begin(); I != fg.nbV(node->_index).end(); I++ ) 
                                    if( (level == 0) || (tr.parent(node)->_type != BoundTreeNode::FACTOR) || (*I != tr.parent(node)->_index) ) {
                                        tr.append_child( node, BoundTreeNode( _new_node_id++, *I, BoundTreeNode::FACTOR, _saw ? cycle( BoundTreeNode::FACTOR, *I, node ) : false ) );
                                        children_added++;
                                    }
                            }
                        }
                    } else if( node->_type == BoundTreeNode::FACTOR ) { // if the node is a factor
                        bool has_children = fg.nbF( node->_index ).size() > 1;
                        if( has_children ) {
                            if( nr_nodes_upto_current_level >= _maxnodes )
                                node->_forcedleaf = true; // if the maximum number of nodes is reached, stop
                            if( !node->_forcedleaf ) { // add all children if it is not a forced leaf node
                                for( FactorGraph::nb_cit i = fg.nbF(node->_index).begin(); i != fg.nbF(node->_index).end(); i++ )
                                    if( (level == 0) || (tr.parent(node)->_type != BoundTreeNode::VAR) || (*i != tr.parent(node)->_index) ) {
                                        tr.append_child( node, BoundTreeNode( _new_node_id++, *i, BoundTreeNode::VAR, _saw ? cycle( BoundTreeNode::VAR, *i, node ) : false ) );
                                        children_added++;
                                    }
                            }
                        }
                    } else
                        DAI_THROW(UNKNOWN_ENUM_VALUE);
                }
            }

            nr_nodes_upto_current_level += children_added;
            if( children_added == 0 )
                break;
        }
    }


    void BoundTree::display( std::ostream & os, const FactorGraph &fg ) {
//        if( true ) {
        os << "digraph G {" << endl;

        // variables
        os << "node[shape=circle,width=0.4,fixedsize=true];" << endl;
        for( tree<BoundTreeNode>::iterator node = tr.begin(); node != tr.end(); node++ )
            if( node->_type == BoundTreeNode::VAR ) {
                os << "\t" << (*node) << "[label=v" << fg.var(node->_index).label();
                if( node->_forcedleaf )
                    os << ",color=gray,style=filled];" << endl;
                else
                    os << "];" << endl;
            }

        //factors 
        os << "node[shape=box,width=0.3,height=0.3,fixedsize=true];" << endl;
        for( tree<BoundTreeNode>::iterator node = tr.begin(); node != tr.end(); node++ )
            if( node->_type == BoundTreeNode::FACTOR ) {
                os << "\t" << (*node) << "[label=f" << node->_index;
                if( node->_forcedleaf )
                    os << ",color=gray,style=filled];" << endl;
                else
                    os << "];" << endl;
            }

        // edges
        os << "edge[color=black];" << endl;
        for( tree<BoundTreeNode>::iterator node = tr.begin(); node != tr.end(); node++ )
            if( node != tr.begin() )
                os << "\t" << (*tr.parent(node)) << " -> " << (*node) << endl;

        os << "}" << endl;
/*        } else {
            os << "graph G {" << endl;

            // variables
            os << "node[shape=circle,width=0.4,fixedsize=true,color=lightgray];" << endl;
            for( size_t j = 0; j < fg.nrVars(); j++ ) {
                os << "\tv" << fg.var(j).label();
                tree<BoundTreeNode>::iterator node = find( BoundTreeNode::VAR, j );
                if( node == tr.begin() )
                    os << "[color=blue,style=filled];" << endl;
                else if( node != tr.end() ) {
                    if( node->_forcedleaf )
                        os << "[color=blue,style=filled];" << endl;
                    else
                        os << "[color=blue];" << endl;
                } else
                    os << ";" << endl;
            }

            //factors 
            os << "node[shape=box,width=0.3,height=0.3,fixedsize=true,color=lightgray];" << endl;
            for( size_t I = 0; I < fg.nrFactors(); I++ ) {
                os << "\tf" << I;
                tree<BoundTreeNode>::iterator node = find( BoundTreeNode::FACTOR, I );
                if( node != tr.end() ) {
                    if( node->_forcedleaf )
                        os << "[color=blue,style=filled];" << endl;
                    else
                        os << "[color=blue];" << endl;
                } else
                    os << ";" << endl;
            }

            // edges
            os << "edge[color=lightgray];" << endl;
            for( size_t e = 0; e < fg.nrEdges(); e++ ) {
                size_t i = fg.edge(e).first;
                size_t I = fg.edge(e).second;
                if( containsEdge( i, I ) )
                    os << "\tv" << fg.var(i).label() << " -- f" << I << "[color=blue];" << endl;
                else
                    os << "\tv" << fg.var(i).label() << " -- f" << I << ";" << endl;
            }
            os << "}" << endl;
        }*/
    }


    Box BoundTree::BoxProp( const FactorGraph &fg, bool useIhler, size_t verbose, const vector<Factor> & bp_beliefs ) {
        if( useIhler )
            deltas = vector<double>( tr.size() );
        else
            boxes = vector<Box>( tr.size() );
        for( tree<BoundTreeNode>::iterator node = tr.end(); ; ) {
            node--;

            Var v_i;
            if( node->_type == BoundTreeNode::VAR )
                v_i = fg.var( node->_index );
            else if( node->_type == BoundTreeNode::FACTOR )
                v_i = fg.var( tr.parent(node)->_index );

            if( node->_forcedleaf ) {
                if( verbose )
                    cerr << "initing forced leaf node " << (*node) << ": " << v_i << endl;

                if( useIhler )
                    deltas[node->_id] = INFINITY;
                else
                    boxes[node->_id] = Box( v_i );
            } else {
                if( node->_type == BoundTreeNode::VAR ) {
                    if( useIhler )
                        deltas[node->_id] = 1.0;
                    else
                        boxes[node->_id] = Box( v_i, 1.0, 1.0 );
                    // product of incoming bounds
                    for( tree<BoundTreeNode>::sibling_iterator child = tr.begin(node); child != tr.end(node); child++ )
                        if( useIhler )
                            deltas[node->_id] *= deltas[child->_id];
                        else
                            boxes[node->_id] *= boxes[child->_id];
//                  the following line seems to have no impact (which is good)                    
//                    boxes[node->_id].normalize();
                    if( verbose )
                        cerr << "propagating through variable node " << (*node) << ": " << boxes[node->_id] << endl;
                } else if( node->_type == BoundTreeNode::FACTOR ) {
                    if( useIhler ) {
                        size_t k = fg.factor( node->_index ).vars().size();
                        if( k == 1 )
                            deltas[node->_id] = 1.0;
                        else if( k == 2 ) {
                            tree<BoundTreeNode>::sibling_iterator child = tr.begin(node);
                            deltas[node->_id] = Ihler_recursion( fg.factor( node->_index ), deltas[child->_id] );
                        } else
                            DAI_THROW(NOT_IMPLEMENTED);
                    } else {
                        std::vector<Box> incoming;
                        for( tree<BoundTreeNode>::sibling_iterator child = tr.begin(node); child != tr.end(node); child++ )
                            incoming.push_back( boxes[child->_id] );
                        if( incoming.size() >= 1 ) {
                            boxes[node->_id] = boundSumProd( fg.factor( node->_index ), incoming, v_i, !_saw, true );
                        } else
                            boxes[node->_id] = Box( v_i, fg.factor( node->_index ).p(), fg.factor( node->_index ).p() );
                    }
                    if( verbose )
                        cerr << "propagating through factor node " << (*node) << ": " << boxes[node->_id] << endl;
                } else
                    DAI_THROW(UNKNOWN_ENUM_VALUE);
            }
            if( node == tr.begin() )
                break;
        }

        if( useIhler ) {
            return Ihler_box( fg.var( tr.begin()->_index ), bp_beliefs[tr.begin()->_index], deltas[tr.begin()->_id] );
        } else
            return boxes[tr.begin()->_id].normalized();
    }


}
