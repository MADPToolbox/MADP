/* This file is part of the Multiagent Decision Process (MADP) Toolbox v0.3. 
 *
 * The majority of MADP is free software released under GNUP GPL v.3. However,
 * some of the included libraries are released under a different license. For 
 * more information, see the included COPYING file. For other information, 
 * please refer to the included README file.
 *
 * This file has been written and/or modified by the following people:
 *
 * Frans Oliehoek 
 * Matthijs Spaan 
 *
 * For contact information please see the included AUTHORS file.
 */

/* Only include this header file once. */
#ifndef _TREENODE_H_
#define _TREENODE_H_ 1

/* the include directives */
#include <iostream>
#include <map>
#include "Globals.h"
#include "E.h"



#define DEBUG_TREENODE 0

/** \brief TreeNode represents a node in a tree of histories, for
 * instance observation histories.
 *
 * TreeNode is a class that represents a wrapper for the Tcontained
 * class.  Each node also specifies a (sub-)tree so there is no actual
 * difference between a tree and a node.  This implementation assumes
 * that ObservationHistories are always contained in exactly 1
 * TreeNode: i.e., deleting an object of TreeNode will free the memory
 * of the node and the subtree represented by it as well as the memory
 * of all the contained ObservationHistories. */
template <class Tcontained >
class TreeNode 
{
    private:
    protected:
        /**The map that stores the pointers to the successor
         * TreeNodes*/
        std::map< LIndex, TreeNode<Tcontained>* > _m_successor; 
        ///A Pointer to the predecessor.
        TreeNode<Tcontained>* _m_pred;
        /**The index of this TreeNode (and thus of the contained
         * Tcontained - typically an observation history).*/
        LIndex _m_index;
        ///Whether the index is valid.
        bool _m_indexValid;
        ///The contained element 
        Tcontained* _m_containedElem;
    

    
    public:
        // Constructor, destructor and copy assignment.
        /// (default) Constructor
        TreeNode()
        {    
            _m_index = 42; // weird number to help track it down in gdb etc
            _m_indexValid = false;
            _m_containedElem = 0;
            _m_pred = 0;
        }

        TreeNode(Tcontained *const oh)
        {
            _m_index = 0;
            _m_indexValid = false;
            _m_containedElem = oh;
            _m_pred = 0;
        }
        /// Copy constructor.
        TreeNode(const TreeNode& a)
        {
            _m_successor=a._m_successor;
            _m_pred=a._m_pred;
            _m_index=a._m_index;
            _m_indexValid=a._m_indexValid;
            _m_containedElem=new Tcontained(*a._m_containedElem);

#if DEBUG_TREENODE
            cerr << "Cloning TreeNode. This node ";
            PrintThisNode();
            cerr << endl;
#endif        
        }

        /// Destructor.
        virtual ~TreeNode()
        {
#if DEBUG_TREENODE
            cerr << "Deleting TreeNode. This node ";
            PrintThisNode();cerr << endl;
#endif
            delete(_m_containedElem);

            while(!_m_successor.empty())
            {
                delete _m_successor.begin()->second; // recursively
                                                     // delete the
                                                     // rest of the
                                                     // tree
                _m_successor.erase(_m_successor.begin());
            }
        }

        //operators:

        //data manipulation (set) functions:
        ///Sets the index to i
        void SetIndex(LIndex i){_m_index = i; _m_indexValid = true; };
        /** \brief Sets the sucI'th successor of this TreeNode to suc.
         *
         * For example, the successor for observation number
         * sucI. Also sets the predecessor of suc to this.*/
        void SetSuccessor(LIndex sucI, TreeNode<Tcontained>* suc);
        /** \brief Sets the predecessor of this node to be pred.
         *
         * This function is typically called by SetSuccessor (not
         * manually).*/
        void SetPredeccessor(TreeNode<Tcontained>* pred)
            {_m_pred = pred;}

        //get (data) functions:
        
        ///Get the succesor TreeNode* for the sucI'th successor
        TreeNode* GetSuccessor(LIndex sucI);
        ///Get the predecessor TreeNode*. 
        TreeNode* GetPredecessor() const
            {return(_m_pred);}
            
        /**\brief Returns the index of this TreeNode (and thus
         * corresponding to the contained element).*/
        LIndex GetIndex() const 
        {
            if(_m_indexValid) 
                return(_m_index);
            else
                throw E("This TreeNode's index is invalid (not yet set)");
        }
        ///Returns a pointer to the contained element (Tcontained) 
        Tcontained* GetContainedElement() const {return(_m_containedElem);};

        /// Check whether a particular successor sucI exists.
        bool ExistsSuccessor(LIndex sucI);

        /** \brief Prints the tree starting from this node of the
         * history tree (including the successors).*/
        void Print() const;
        /** \brief Prints only this node of the history tree (not the
         * successors).*/
        void PrintThisNode() const;
    
};

template <class Tcontained>
void TreeNode<Tcontained>::SetSuccessor(LIndex sucI, 
    TreeNode<Tcontained>* suc)
{
    if( ExistsSuccessor(sucI) )
    {
#if DEBUG_TREENODE
        cout << "_m_successor["<< sucI<< "] already set: overwriting!\n";
#endif
        _m_successor[sucI] = suc;
    }
    else
    {    
        std::pair< LIndex, TreeNode<Tcontained>* > arg_pair = 
            std::make_pair(sucI, suc);
        std::pair< typename std::map< LIndex, TreeNode<Tcontained>* >::iterator, bool> 
            result_pair;
        result_pair = _m_successor.insert(arg_pair);
        if(result_pair.second == false)
        {
            std::stringstream ss;
            ss << "TreeNode<Tcontained>::SetSuccessor insertion failed, "
               << "but _m_successor["<< sucI<< "] wasn't already set ?!?";
            throw(E(ss));
        }
    }

    suc->SetPredeccessor(this);
}

template <class Tcontained>
TreeNode<Tcontained>* TreeNode<Tcontained>::GetSuccessor(LIndex sucI)
{
    if(_m_successor.find(sucI)==_m_successor.end())
        throw EInvalidIndex("TreeNode::GetSuccessor successor not found");
    else
        return(_m_successor[sucI]);
}

template <class Tcontained>
bool TreeNode<Tcontained>::ExistsSuccessor(LIndex sucI)
{
    if(_m_successor.find(sucI)==_m_successor.end())
        return(false);
    else
        return(true);
}


template <class Tcontained>
void TreeNode<Tcontained>::PrintThisNode() const
{
    if(_m_containedElem != 0)
    {
        std::cout << "index: "<<_m_index<<" - ";
        _m_containedElem->Print();
    }
}

template <class Tcontained>
void TreeNode<Tcontained>::Print() const
{
    if(_m_containedElem != 0)
    {

        std::cout << "index: ";
        if(_m_indexValid)
            std::cout<< _m_index;
        else
            std::cout << "INVALID";
        std::cout << " - ";
        _m_containedElem->Print();
        std::cout << std::endl;
        //typname dependent on template should be called using typename
        typename std::map< LIndex, TreeNode<Tcontained>*>::const_iterator it = 
            _m_successor.begin();
        while(it != _m_successor.end())
        {
            if(it->second != 0) it->second->Print();
            it++;
        }
    }
}

#endif /* !_TREENODE_H_ */


// Local Variables: ***
// mode:c++ ***
// End: ***
