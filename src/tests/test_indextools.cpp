/**\file test_indextools.cpp
 *
 * Authors:
 * Frans Oliehoek <fao@csail.mit.edu>
 * Matthijs Spaan <mtjspaan@isr.ist.utl.pt>
 *
 * Copyright 2011 MIT, Instituto Superior Tecnico
 *
 * This file is part of MultiAgentDecisionProcess.
 *
 * MultiAgentDecisionProcess is free software: you can redistribute it
 * and/or modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation, either version 3 of
 * the License, or (at your option) any later version.
 *
 * MultiAgentDecisionProcess is distributed in the hope that it will
 * be useful, but WITHOUT ANY WARRANTY; without even the implied
 * warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with MultiAgentDecisionProcess.  If not, see
 * <http://www.gnu.org/licenses/>.
 *
 * $Id$
 */
#define BOOST_TEST_DYN_LINK
#define BOOST_TEST_MODULE test_indextools
#include <boost/test/unit_test.hpp>

#include "Globals.h"
#include "IndexTools.h"
#include <iostream>
using namespace std;

BOOST_AUTO_TEST_SUITE( test_indextools )

BOOST_AUTO_TEST_CASE(Increment)
{
    for (size_t nrElems=1;nrElems<30;++nrElems)
    {
        BOOST_TEST_MESSAGE("nrElems:"<<nrElems);
        Index index=0;
        bool finished;
        do
        {
            Index prevIndex=index;
            finished=IndexTools::Increment(index,nrElems);
            BOOST_TEST_MESSAGE("index:"<<index);
            if (!finished)
                BOOST_CHECK_EQUAL(index,prevIndex+1); // when not finished index should be 1 more then previous loop
            else
            {
                BOOST_CHECK_EQUAL(index,0); // otherwise should be 0
                BOOST_CHECK_EQUAL(prevIndex,nrElems-1); // and previous value equal to the maxium (nrElems-1)
            }
        } while (!finished);
    }
}

BOOST_AUTO_TEST_CASE(IncrementVector)
{
    size_t maxNrDimensions=5;
    vector<Index> prevIndexVec;
    vector<Index> indexVec;
    vector<size_t> nrElems;
    for (size_t nrDimensions=1;nrDimensions<=maxNrDimensions;++nrDimensions)
    {
        // prepare test data
        indexVec.clear();
        nrElems.clear();
        for (size_t dim=0;dim<nrDimensions;++dim)
        {
            nrElems.push_back(maxNrDimensions-dim+1); // some test data
            indexVec.push_back(0);
        }
        
        // run tests
        size_t count=0;
        bool finished;
        BOOST_TEST_MESSAGE("nrElems:"<<SoftPrintVector(nrElems));
        do
        {
            ++count;
            prevIndexVec=indexVec;
            finished=IndexTools::Increment(indexVec,nrElems);
            BOOST_TEST_MESSAGE("indexVec:"<<SoftPrintVector(indexVec));
        } while (!finished);
        
        // check values
        size_t multiplier=1;
        for (size_t dim=0;dim<nrDimensions;++dim)
        {
            multiplier*=nrElems[dim];
            BOOST_CHECK_EQUAL(prevIndexVec[dim],nrElems[dim]-1);
        }
        BOOST_CHECK_EQUAL(count,multiplier);
    }
}

BOOST_AUTO_TEST_CASE(IndividualToJointToIndividualIndices)
{
    for (size_t i=0;i<2000;++i)
    {
        // prepare test data
        size_t dimension=rand()%8; // allow for dimension==0
        BOOST_TEST_MESSAGE("dimension:"<<dimension);
        vector<size_t> nrElems;
        vector<Index> indices;
        for (size_t d=0;d<dimension;++d)
        {
            size_t nrElments=1+rand()%8; // should be at least 1
            nrElems.push_back(nrElments);
            if (nrElments>0)
                indices.push_back(rand()%nrElments);
            else
                indices.push_back(0);
        }
        BOOST_TEST_MESSAGE("nrElems:"<<SoftPrintVector(nrElems));
        BOOST_TEST_MESSAGE("indices :"<<SoftPrintVector(indices));
                
        Index jointI;
        vector<Index> indices2;

        // run test
        jointI=IndexTools::IndividualToJointIndices(indices,nrElems);
        BOOST_TEST_MESSAGE("jointI:"<<jointI);
        indices2=IndexTools::JointToIndividualIndices(jointI,nrElems);
        BOOST_TEST_MESSAGE("indices2:"<<SoftPrintVector(indices2));

        // check values
        for (size_t d=0;d<dimension;++d)
            BOOST_CHECK_EQUAL(indices[d],indices2[d]);

        // run test
        size_t* step_size=IndexTools::CalculateStepSize(nrElems,nrElems.size());
        jointI=IndexTools::IndividualToJointIndicesArrayStepSize(&(indices[0]),step_size,dimension);
        BOOST_TEST_MESSAGE("jointI:"<<jointI);
        IndexTools::JointToIndividualIndices(jointI,nrElems,indices2);
        const Index* indices3=IndexTools::JointToIndividualIndicesArrayStepSize(jointI,
                                                                              step_size,
                                                                              dimension);
        // check values
        for (size_t d=0;d<dimension;++d)
        {
            BOOST_CHECK_EQUAL(indices[d],indices2[d]);
            BOOST_CHECK_EQUAL(indices[d],indices3[d]);
        }
        delete indices3;
        delete step_size;
    }
}

BOOST_AUTO_TEST_SUITE_END()
