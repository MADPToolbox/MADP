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
#include "alldai.h"


using namespace std;
using namespace libDAI;


int main( int argc, char *argv[] ) {
    if ( argc != 2 ) {
        cout << "Usage: " << argv[0] << " <filename.fg>" << endl << endl;
        cout << "Reads factor graph <filename.fg> and runs" << endl;
        cout << "Belief Propagation and JunctionTree on it." << endl << endl;
        return 1;
    } else {
        FactorGraph fg;
        fg.ReadFromFile( argv[1] );

        size_t  maxiter = 10000;
        double  tol = 1e-9;
        size_t  verb = 1;

        Properties opts;
        opts.Set("maxiter",maxiter);
        opts.Set("tol",tol);
        opts.Set("verbose",verb);

        JTree jt( fg, opts("updates",string("HUGIN")) );
        jt.init();
        jt.run();

        BP bp(fg, opts("updates",string("SEQFIX")));
        bp.init();
        bp.run();

        TRW trw;
        if( fg.isPairwise() ) {
            trw = TRW(fg, opts("updates",string("SEQFIX"))("outertol",1e-9)("maxouteriter",(size_t)0));
            trw.init();
            trw.runTRW();
        }

        cout << "Exact single node marginals:" << endl;
        for( size_t i = 0; i < fg.nrVars(); i++ )
            cout << jt.belief(fg.var(i)) << endl;

        cout << "Loopy Belief Propagation single node marginals:" << endl;
        for( size_t i = 0; i < fg.nrVars(); i++ )
            cout << bp.belief(fg.var(i)) << endl;

        if( fg.isPairwise() ) {
            cout << "Tree-Reweighted Belief Propagation single node marginals:" << endl;
            for( size_t i = 0; i < fg.nrVars(); i++ )
                cout << trw.belief(fg.var(i)) << endl;
        }

        cout << "Exact factor marginals:" << endl;
        for( size_t I = 0; I < fg.nrFactors(); I++ )
            cout << jt.belief(fg.factor(I).vars()) << endl;

        cout << "Loopy Belief Propagation factor marginals:" << endl;
        for( size_t I = 0; I < fg.nrFactors(); I++ )
            cout << bp.belief(fg.factor(I).vars()) << "=" << bp.beliefF(I) << endl;

        if( fg.isPairwise() ) {
            cout << "Tree-Reweighted Belief Propagation factor marginals:" << endl;
            for( size_t I = 0; I < fg.nrFactors(); I++ )
                cout << trw.belief(fg.factor(I).vars()) << "=" << trw.beliefF(I) << endl;
        }

        cout << "Exact log partition sum: " << real( jt.logZ() ) << endl;
        cout << "LBP log partition sum: " << real( bp.logZ() ) << endl;
        if( fg.isPairwise() )
            cout << "TRW log partition sum: " << real( trw.logZ() ) << endl;

        if( fg.isPairwise() ) {
            trw.run();
            cout << "Tree-Reweighted Belief Propagation single node marginals:" << endl;
            for( size_t i = 0; i < fg.nrVars(); i++ )
                cout << trw.belief(fg.var(i)) << endl;
            cout << "Tree-Reweighted Belief Propagation factor marginals:" << endl;
            for( size_t I = 0; I < fg.nrFactors(); I++ )
                cout << trw.belief(fg.factor(I).vars()) << "=" << trw.beliefF(I) << endl;
            cout << "TRW log partition sum: " << real( trw.logZ() ) << endl;
        }

        Factor prod;
        for( size_t I = 0; I < fg.nrFactors(); I++ )
            prod *= fg.factor(I);
        cout << prod << endl;
    }

    return 0;
}
