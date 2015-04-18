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


#include "boxbp.h"


namespace libDAI {


    using namespace std;


    void BoxBP::run( const FactorGraph &fg, size_t maxiter ) {
        _boxes.clear();
        _boxes.reserve( fg.nrEdges() );
        for( size_t iI = 0; iI < fg.nrEdges(); iI++ ) {
            size_t i = fg.edge(iI).first;
            _boxes.push_back( Box( fg.var(i) ) );
        }
        
        vector<size_t> edge_seq;
        edge_seq.reserve( fg.nrEdges() );
        for( size_t e = 0; e < fg.nrEdges(); e++ )
            edge_seq.push_back( e );

        for( size_t iter = 0; iter < maxiter; iter++ ) {
            for( size_t t = 0; t < fg.nrEdges(); t++ )
                calcNewBox(fg, t);
        }
    }


    void BoxBP::calcNewBox( const FactorGraph &fg, size_t iI ) {
        size_t i = fg.edge(iI).first;
        size_t I = fg.edge(iI).second;
        Var v_i = fg.var(i);
        VarSet v_I = fg.factor(I).vars();

        vector<Box> incoming;
        for( FactorGraph::nb_cit j = fg.nbF(I).begin(); j != fg.nbF(I).end(); j++ )
            if( *j != i ) {
                Var v_j = fg.var(*j);
                Box mjI( v_j, 1.0, 1.0 );
                for( FactorGraph::nb_cit J = fg.nbV(*j).begin(); J != fg.nbV(*j).end(); J++ ) 
                    if( *J != I )
                        mjI *= box( fg, *j, *J );
                incoming.push_back( mjI );
            }
        _boxes[iI] = boundSumProd( fg.factor(I), incoming, v_i, true, true );
    }


}
