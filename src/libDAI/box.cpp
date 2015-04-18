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


#include "box.h"
#include "factorgraph.h"
#include <vector>
#include <set>
#include <algorithm>
#ifdef GLPK
    #include <glpk.h>
#endif


namespace libDAI {

    
    using namespace std;


#ifdef GLPK
    // Test if point is in the convex hull of points
    // by solving a linear problem
    // see http://www.cs.mcgill.ca/~fukuda/soft/polyfaq/node22.html#polytope:Vredundancy
    bool TestIfPointInConvexHull( vector<Prob> points, Prob point ) {
        using namespace std;
        using namespace libDAI;

        glp_prob *lp = glp_create_prob();
        glp_set_obj_dir(lp, GLP_MAX);

        int N = points.size();
        int d = point.size();

        int n = d + 1; // number of columns (structural variables)
        int m = N + 1; // number of rows (auxiliary variables)

        glp_add_cols(lp,n);
        glp_add_rows(lp,m);

        // set objective function
        glp_set_obj_coef(lp,n,-1.0);
        for( int j = 1; j <= d; j++ )
            glp_set_obj_coef(lp,j,point[j-1]);

        // define auxiliary variables
        // first i=1..N rows plus last row
        // last row is identical to objective function
        for( int i = 1; i <= m; i++ ) {
            vector<int> ind;
            vector<double> val;
            ind.push_back(0); val.push_back(0.0);
            for( int j = 1; j <= d; j++ ) {
                double x;
                if( i == m )
                    x = point[j-1];
                else
                    x = points[i-1][j-1];
                if( x != 0.0 ) {
                    ind.push_back(j); val.push_back(x);
                }
            }
            ind.push_back(n); val.push_back(-1.0);
            glp_set_mat_row(lp,i,ind.size()-1,&(ind[0]),&(val[0]));
        }

        // set bounds on auxiliary variables
        for( int i = 1; i <= N; i++ ) {
            glp_set_row_bnds(lp,i,GLP_UP,0.0,0.0);
        }
        glp_set_row_bnds(lp,m,GLP_UP,0.0,1.0);

        // structural variables are free
        for( int j = 1; j <= n; j++ )
            glp_set_col_bnds(lp,j,GLP_FR,0.0,0.0);

        // set parameters
    //	lpx_set_int_parm(lp, LPX_K_PRESOL, 1);      // use pre-solver
        lpx_set_int_parm(lp, LPX_K_MSGLEV, 0);      // quiet

        // solve
        if( lpx_simplex(lp) != LPX_E_OK ) {
            cerr << "WARNING: lpx_simplex did not terminate successfully!" << endl;
            assert( 0 == 1);
        }
        bool failed = (lpx_get_status(lp) != LPX_OPT);
        double result = glp_get_obj_val(lp);
        glp_delete_prob(lp);

        if( failed )
            cerr << "WARNING: solution is not optimal!" << endl;

        return (result <= 0.5);     // this may be better because of numerical roundoff errors?
//        return (result <= 0.0);
    }
#endif


    // Calculate a loose bound on the product of the boxes by
    // multiplying together their respective bounds
    Box boundProd( vector<Box> &boxes ) {
        VarSet dest;
        for( size_t l = 0; l < boxes.size(); l++ ) {
            dest |= boxes[l].vars();
        }

        Factor min(dest, 1.0), max(dest, 1.0);
        for( size_t l = 0; l < boxes.size(); l++ ) {
            min *= Factor( boxes[l].vars(), boxes[l]._min );
            max *= Factor( boxes[l].vars(), boxes[l]._max );
        }

        return Box( dest, min.p(), max.p() );
    }


    Box boundSumProd( Factor psi, vector<Box> &incomingBoxes, VarSet dest, bool independent, bool normed ) {
        // Calculates a bounding box for the marginal or partial sum on dest
        // of the product of psi with all incoming boxes.
        //
        // If independent is true, each pair of incoming boxes hould either have
        // identical or disjoint varsets; furthermore, incoming boxes with disjoint
        // varsets are assumed to be independent.
        //
        // Otherwise, there is no restriction on the varsets of incoming boxes and
        // we assume that incoming boxes may be dependent.
        //
        // If normed is true, take the norm after taking the sum-product with psi

        if( independent ) {
            // Multiply together incoming boxes defined on the same variables
            vector<Box> uniqueBoxes;
            uniqueBoxes.reserve( incomingBoxes.size() );
            for( size_t l = 0; l < incomingBoxes.size(); l++ ) {
                size_t k = 0;
                for( k = 0; k < uniqueBoxes.size(); k++ )
                    if( incomingBoxes[l].vars() == uniqueBoxes[k].vars() ) {
                        uniqueBoxes[k] *= incomingBoxes[l];
                        break;
                    }
                if( k == uniqueBoxes.size() )
                    uniqueBoxes.push_back( incomingBoxes[l] );
            }

            // Check that all uniqueBoxes have disjoint sets of variables
            for( size_t l = 0; l < incomingBoxes.size(); l++ )
                for( size_t k = l + 1; k < incomingBoxes.size(); k++ )
                    assert( !(uniqueBoxes[k].vars() && uniqueBoxes[l].vars()) );

            size_t nrBoxes = uniqueBoxes.size();

            // Prepare multi-index to step through the Cartesian products of 
            // the extreme points corresponding to all disjoint varsets in
            // uniqueBoxes
            vector<size_t> ep_dims( nrBoxes, 0 );
            vector<vector<Prob> > ep( nrBoxes );
            for( size_t k = 0; k < nrBoxes; k++ ) {
                ep[k] = uniqueBoxes[k].extremePoints();
                ep_dims[k] = ep[k].size();
            }
            multind ep_index( ep_dims );

            // For each product of extreme points, calculate its image and
            // update the bound
            Prob image_min( dest.stateSpace(), INFINITY );
            Prob image_max( dest.stateSpace(), -INFINITY );
            for( size_t ep_li = 0; ep_li < ep_index.max(); ep_li++ ) {
                // FIXME: this can be sped up massively by caching the indices
                vector<size_t> vi = ep_index.vi( ep_li );
                Factor prd;
                for( size_t k = 0; k < nrBoxes; k++ )
                    prd *= Factor( uniqueBoxes[k].vars(), ep[k][vi[k]] );
                prd *= psi;
                Prob image_ep = prd.marginal(dest,normed).p();
                image_min = min( image_ep, image_min );
                image_max = max( image_ep, image_max );
            }
            return Box( dest, image_min, image_max );
        } else {
            Box prod = boundProd( incomingBoxes );
            if( prod.vars().stateSpace() <= 16 )
                return boundProd( incomingBoxes ).boundSumProd( psi, dest, normed );
            else {
                cout << "# Warning: calculating looser bound in the interest of computation time" << endl;
                return Box( dest );
            }
        }
    }


    double Ihler_recursion( const Factor & psi, double delta ) {
        Var i = *(psi.vars().begin());
        Var j = *((psi.vars() / i).begin());
        double myE = psi.strength( i, j );
        double E = (1.0 + myE) / (1.0 - myE);
        if( std::isinf( delta ) )
            return E;
        else
            return (E * delta + 1.0) / (E + delta);
    }


    double Ihler_distance( const Factor & psi1, const Factor & psi2 ) {
        double dist = 0.0;
        assert( psi1.stateSpace() == psi2.stateSpace() );
        for( size_t a = 0; a < psi1.stateSpace(); a++ )
            for( size_t b = 0; b < psi2.stateSpace(); b++ ) {
                double x = (psi1[a] / psi2[a]) / (psi1[b] / psi2[b]);
                if( x > dist )
                    dist = x;
            }
        return sqrt(dist);
    }


    Box Ihler_box( Var v_i, const Factor & psi_i, double delta ) {
        Box box( v_i );
        assert( psi_i.vars() == VarSet(v_i) );
        delta *= delta;
        box.min() = psi_i.p() / (psi_i.p() * (1.0 - delta) + delta);
        box.max() = (psi_i.p() / (psi_i.p() * (delta - 1.0) + 1.0)) * delta;
        return box;
    }

    
    double sgn( double x ) {
        if( x < 0 )
            return -1.0;
        else if( x > 0 )
            return 1.0;
        else
            return 0.0;
    }
    
    double func( double x, double A1, double A2, double B1, double B2, double C1, double C2, double s ) {
        return ((A1 + B1 * x) / (B1 * x + C1) - (A2 + B2 * x) / (B2 * x + C2)) * s;
    }

    // calculate N(psi, i, j)
    // if h is in the box hbox
    // (not optimized yet)
    double FactorStrength( const Factor &psi, const Var &i, const Var &j, const Box &hbox ) {
        assert( psi.vars() && i );
        assert( psi.vars() && j );
        assert( i != j );
        assert( hbox.vars() == (psi.vars() / i) );
        VarSet ij = i | j;

        std::vector<Prob> hep = hbox.extremePoints();
//        Factor hextmax;

        double strength = 0.0;
        for( size_t a1 = 0; a1 < i.states(); a1++ )
            for( size_t a2 = 0; a2 < i.states(); a2++ ) 
                if( a1 != a2 ) {
                    Factor psi1 = psi.slice( i, a1 );
                    Factor psi2 = psi.slice( i, a2 );

/*                    for( size_t ep = 0; ep < hep.size(); ep++ ) {
                        Factor hext( hbox.vars(), hep[ep] );
    //                  f = @(h) sum(abs(sum(psi1.*h,2) / sum(sum(psi1.*h)) - sum(psi2.*h,2) / sum(sum(psi2.*h))));
                        cout << ((psi1 * hext).partSum(j) / (psi1 * hext).totalSum() - (psi2 * hext).partSum(j) / (psi2 * hext).totalSum()).abs().totalSum() << endl;
                    }*/

                    double max = 0.0;
            
                    Index b2( j, hbox.vars() );
                    for( size_t b2c2 = 0; b2c2 < hbox.vars().stateSpace(); b2c2++, ++b2 ) {
                        for( size_t ep = 0; ep < hep.size(); ep++ ) {
                            Factor hext( hbox.vars(), hep[ep] );
                            hext[b2c2] = 0.0;

                            double B1 = psi1[b2c2];
                            double B2 = psi2[b2c2];
                            double C1 = (psi1 * hext).totalSum();
                            double C2 = (psi2 * hext).totalSum();

//                            denom1 = @(x) (B1*x + C1);
//                            denom2 = @(x) (B2*x + C2);

                            Prob E1 = (psi1 * hext).partSum( j ).p();
                            Prob E2 = (psi2 * hext).partSum( j ).p();
//                          expr = @(x) (E1 / denom1(x) - E2 / denom2(x));
                            Prob crit_x = (E2 * C1 - E1 * C2) / (E1 * B2 - E2 * B1);
                            Prob signs = (E2 * B1 - E1 * B2).sgn();  // sign to the left of crit_x
//                          expr = @(x) ((E1 + B1*x) / denom1(x) - (E2 + B2*x) / denom2(x));   // for b2
                            crit_x[b2] = (E2[b2] * C1 - E1[b2] * C2) / ((E1[b2] - C1) * B2 - (E2[b2] - C2) * B1);
                            signs[b2] = sgn( (C1 - E1[b2]) * B2 - (C2 - E2[b2]) * B1 );

                            std::set<double> set_crit_x;
                            set_crit_x.insert( hbox.min()[b2c2] );
                            for( size_t b1 = 0; b1 < j.states(); b1++ )
                                if( hbox.min()[b2c2] < crit_x[b1] && crit_x[b1] < hbox.max()[b2c2] )
                                    set_crit_x.insert( crit_x[b1] );
                            set_crit_x.insert( hbox.max()[b2c2] );

                            std::set<double>::const_iterator left = set_crit_x.begin();
                            std::set<double>::const_iterator right = left; right++;
                            for( ; right != set_crit_x.end(); left++, right++ ) {
                                if( (*right - *left) > 1e-15 ) {
                                    double center = (*left + *right) / 2.0;
                                    Prob sigs( signs.size(), 0.0 );
                                    for( size_t b1 = 0; b1 < j.states(); b1++ ) {
                                        if( center < crit_x[b1] )
                                            sigs[b1] = signs[b1];
                                        else if( center > crit_x[b1] )
                                            sigs[b1] = -signs[b1];
                                        else
                                            DAI_THROW(INTERNAL_ERROR);
                                    }

                                    double A1 = (E1 * sigs).totalSum() * sigs[b2];
                                    double A2 = (E2 * sigs).totalSum() * sigs[b2];

    //                                numer1 = @(x) (A1 + B1 * x);
    //                                numer2 = @(x) (A2 + B2 * x);
    //                                func = @(x) (sigs(b2) * (numer1(x) / denom1(x) - numer2(x) / denom2(x)));

                                    double f_left = func( *left, A1, A2, B1, B2, C1, C2, sigs[b2] );
                                    double f_right = func( *right, A1, A2, B1, B2, C1, C2, sigs[b2] );
                                    if( f_left > max ) {
                                        max = f_left;
    //                                    hextmax = hext;
    //                                    hextmax[b2c2] = *left;
                                    }
                                    if( f_right > max ) {
                                        max = f_right;
    //                                    hextmax = hext;
    //                                    hextmax[b2c2] = *right;
                                    }

                                    double x1 = (C1*A2-A1*C2 - fabs(C2*B1-C1*B2) * sqrt((C1-A1)*(C2-A2)/(B1*B2))) / (B2*(A1-C1) + B1*(C2-A2));
                                    double f_x1 = func( x1, A1, A2, B1, B2, C1, C2, sigs[b2] );
                                    if( (*left < x1) && (x1 < *right) ) {
                                        if( f_x1 > max ) {
                                            max = f_x1;
    //                                        hextmax = hext;
    //                                        hextmax[b2c2] = x1;
                                        }
                                    }

                                    double x2 = (C1*A2-A1*C2 + fabs(C2*B1-C1*B2) * sqrt((C1-A1)*(C2-A2)/(B1*B2))) / (B2*(A1-C1) + B1*(C2-A2));
                                    double f_x2 = func( x2, A1, A2, B1, B2, C1, C2, sigs[b2] );
                                    if( (*left < x2) && (x2 < *right) ) {
                                        if( f_x2 > max ) {
                                            max = f_x2;
    //                                        hextmax = hext;
    //                                        hextmax[b2c2] = x2;
                                        }
                                    }
                                }
                            }
                        }
                    }

                    if( max > strength )
                        strength = max;
                }


        return 0.5 * strength;
    }


}
