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


#ifndef __defined_libdai_bounds_h
#define __defined_libdai_bounds_h


#include "tree/tree.hh"
#include "box.h"
#include "daialg.h"
#include "factorgraph.h"
#include "enum.h"


namespace libDAI {


    class BoundTreeNode {
        public:
            enum nodeType { VAR, FACTOR };

            size_t   _id;             // unique id of this node (unique in this tree, at least)
            size_t   _index;          // factor/variable index
            nodeType _type;           // is this a variable or a factor node?
            bool     _forcedleaf;     // is this a forced leaf node (cycle-induced leaf node in case of SAW)?

        public:
            BoundTreeNode() {};
            BoundTreeNode( size_t id, size_t index, nodeType type, bool forcedleaf ) : _id(id), _index(index), _type(type), _forcedleaf(forcedleaf) {};

            friend std::ostream& operator << (std::ostream& os, const BoundTreeNode& n) { 
                if( n._type == BoundTreeNode::VAR )
                    return( os << "v" << n._id ); 
                else if( n._type == BoundTreeNode::FACTOR ) 
                    return( os << "f" << n._id ); 
                else
                    DAI_THROW(UNKNOWN_ENUM_VALUE);
            };
    };


    class BoundTree {
        protected:
            size_t _maxnodes;
            size_t _new_node_id;
            bool _saw;

            tree<BoundTreeNode> tr;
            std::vector<Box> boxes;        // vector of boxes that bound the belief at the corresponding node, for each id
            std::vector<double> deltas;    // vector of deltas that bound the belief at the corresponding node, for each id

        public:
            void buildTree( const FactorGraph &fg, const Var &v_root, size_t maxnodes, bool SAW );
            void display( std::ostream & os, const FactorGraph &fg );
            Box BoxProp( const FactorGraph &fg, bool useIhler, size_t verbose, const std::vector<Factor> & bp_beliefs );

            // returns true if the path between the root node and
            // the node pointed to with <node> contains a node with
            // nodeType <query_type> and index <query_index>
            bool cycle( BoundTreeNode::nodeType query_type, size_t query_index, const tree<BoundTreeNode>::iterator &node ) {
                if( node == tr.begin() ) // root node
                    return (node->_type == query_type) && (query_index == node->_index);
                else {
                    if( (node->_type == query_type) && (query_index == node->_index) )
                        return true;
                    else
                        return cycle( query_type, query_index, tr.parent( node ) );    
                }
            }

            // return true if the node described by nodeType <query_type> and
            // index <query_index> is present in the current tree
            tree<BoundTreeNode>::iterator find( BoundTreeNode::nodeType query_type, size_t query_index ) {
                for( tree<BoundTreeNode>::iterator node = tr.begin(); node != tr.end(); node++ ) {
                    if( (node->_type == query_type) && (query_index == node->_index) ) 
                        return node;
                }
                return tr.end();
            }

            // return true if the edge connecting variable i and factor I is in the tree
            bool containsEdge( size_t i, size_t I ) {
                bool found = false;
                for( tree<BoundTreeNode>::iterator node = tr.begin(); node != tr.end() && !found; node++ ) 
                    if( node != tr.begin() ) {
                        if( node->_type == BoundTreeNode::VAR ) {
                            if( node->_index == i && tr.parent(node)->_index == I )
                                found = true;
                        } else if( node->_type == BoundTreeNode::FACTOR ) {
                            if( node->_index == I && tr.parent(node)->_index == i )
                                found = true;
                        } else
                            DAI_THROW(UNKNOWN_ENUM_VALUE);
                    }
                return found;
            }
    };


}


#endif
