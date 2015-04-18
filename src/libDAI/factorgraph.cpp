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
#include <iterator>
#include <map>
#include <set>
#include <fstream>
#include <string>
#include <algorithm>
#include <functional>
#include "factorgraph.h"
#include "exceptions.h"


namespace libDAI {


    using namespace std;


    FactorGraph::FactorGraph( const std::vector<Factor> &P ) : GraphicalModel(), _fg(), _backupFactors() {
        // add Factors
        set<Var> _vars;
        for(vector<Factor>::const_iterator p2 = P.begin(); p2 != P.end(); p2++ ) {
            _fg.V2s().push_back(*p2);
            copy( p2->vars().begin(), p2->vars().end(), inserter( _vars, _vars.begin() ) );
        }

        // add _vars
        for(set<Var>::const_iterator p1 = _vars.begin(); p1 != _vars.end(); p1++ )
            _fg.V1s().push_back(*p1);

        // create edges
        for(size_t i2 = 0; i2 < nrFactors(); i2++ ) {
            VarSet ns = factor(i2).vars();
            for(VarSet::const_iterator q = ns.begin(); q != ns.end(); q++ ) {
                for(size_t i1=0; i1 < nrVars(); i1++ ) {
                    if (var(i1) == *q) {
                        _fg.edges().push_back(edge_type(i1,i2));
                        break;
                    }
                }
            }
        }

        // calc neighbours and adjacency matrix
        Regenerate();
    }


    std::ostream& operator << (std::ostream& os, const FactorGraph& fg) {
        os << fg.nrFactors() << endl;

        for( size_t I = 0; I < fg.nrFactors(); I++ ) {
            os << endl;
            os << fg.factor(I).vars().size() << endl;
            for( VarSet::const_iterator i = fg.factor(I).vars().begin(); i != fg.factor(I).vars().end(); i++ )
                os << i->label() << " ";
            os << endl;
            for( VarSet::const_iterator i = fg.factor(I).vars().begin(); i != fg.factor(I).vars().end(); i++ )
                os << i->states() << " ";
            os << endl;
            size_t nr_nonzeros = 0;
            for( size_t k = 0; k < fg.factor(I).stateSpace(); k++ )
                if( fg.factor(I)[k] != 0.0 )
                    nr_nonzeros++;
            os << nr_nonzeros << endl;
            for( size_t k = 0; k < fg.factor(I).stateSpace(); k++ )
                if( fg.factor(I)[k] != 0.0 ) {
                    char buf[20];
                    sprintf(buf,"%18.14g", fg.factor(I)[k]);
                    os << k << " " << buf << endl;
                }
        }

        return(os);
    }


    std::istream& operator >> (std::istream& is, FactorGraph& fg) {
        long verbose = 0;

        try {
            vector<Factor> factors;
            size_t nr_f;
            string line;
            
            while( (is.peek()) == '#' )
                getline(is,line);
            is >> nr_f;
            if( is.fail() )
                DAI_THROW(INVALID_FACTORGRAPH_FILE);
            if( verbose >= 2 )
                cout << "Reading " << nr_f << " factors..." << endl;

            getline (is,line);
            if( is.fail() )
                DAI_THROW(INVALID_FACTORGRAPH_FILE);

            for( size_t I = 0; I < nr_f; I++ ) {
                if( verbose >= 3 )
                    cout << "Reading factor " << I << "..." << endl;
                size_t nr_members;
                while( (is.peek()) == '#' )
                    getline(is,line);
                is >> nr_members;
                if( verbose >= 3 )
                    cout << "  nr_members: " << nr_members << endl;

                vector<long> labels;
                for( size_t mi = 0; mi < nr_members; mi++ ) {
                    long mi_label;
                    while( (is.peek()) == '#' )
                        getline(is,line);
                    is >> mi_label;
                    labels.push_back(mi_label);
                }
                if( verbose >= 3 ) {
                    cout << "  labels: ";
                    copy (labels.begin(), labels.end(), ostream_iterator<int>(cout, " "));
                    cout << endl;
                }

                vector<size_t> dims;
                for( size_t mi = 0; mi < nr_members; mi++ ) {
                    size_t mi_dim;
                    while( (is.peek()) == '#' )
                        getline(is,line);
                    is >> mi_dim;
                    dims.push_back(mi_dim);
                }
                if( verbose >= 3 ) {
                    cout << "  dimensions: ";
                    copy (dims.begin(), dims.end(), ostream_iterator<int>(cout, " "));
                    cout << endl;
                }

                // add the Factor
                VarSet I_vars;
                for( size_t mi = 0; mi < nr_members; mi++ )
                    I_vars |= Var(labels[mi], dims[mi]);
                factors.push_back(Factor(I_vars,0.0));
                
                // calculate permutation sigma (internally, members are sorted)
                vector<long> sigma(nr_members,0);
                VarSet::iterator j = I_vars.begin();
                for( size_t mi = 0; mi < nr_members; mi++,j++ ) {
                    long search_for = j->label();
                    vector<long>::iterator j_loc = find(labels.begin(),labels.end(),search_for);
                    sigma[mi] = j_loc - labels.begin();
                }
                if( verbose >= 3 ) {
                    cout << "  sigma: ";
                    copy( sigma.begin(), sigma.end(), ostream_iterator<int>(cout," "));
                    cout << endl;
                }

                // calculate multindices
                vector<size_t> sdims(nr_members,0);
                for( size_t k = 0; k < nr_members; k++ ) {
                    sdims[k] = dims[sigma[k]];
                }
                multind mi(dims);
                multind smi(sdims);
                if( verbose >= 3 ) {
                    cout << "  mi.max(): " << mi.max() << endl;
                    cout << "       ";
                    for( size_t k=0; k < nr_members; k++ ) 
                        cout << labels[k] << " ";
                    cout << "   ";
                    for( size_t k=0; k < nr_members; k++ ) 
                        cout << labels[sigma[k]] << " ";
                    cout << endl;
                }
                
                // read values
                size_t nr_nonzeros;
                while( (is.peek()) == '#' )
                    getline(is,line);
                is >> nr_nonzeros;
                if( verbose >= 3 ) 
                    cout << "  nonzeroes: " << nr_nonzeros << endl;
                for( size_t k = 0; k < nr_nonzeros; k++ ) {
                    size_t li;
                    double val;
                    while( (is.peek()) == '#' )
                        getline(is,line);
                    is >> li;
                    while( (is.peek()) == '#' )
                        getline(is,line);
                    is >> val;

                    vector<size_t> vi = mi.vi(li);
                    vector<size_t> svi(vi.size(),0);
                    for( size_t k = 0; k < vi.size(); k++ )
                        svi[k] = vi[sigma[k]];
                    size_t sli = smi.li(svi);
                    if( verbose >= 3 ) {
                        cout << "    " << li << ": ";
                        copy( vi.begin(), vi.end(), ostream_iterator<size_t>(cout," "));
                        cout << "-> ";
                        copy( svi.begin(), svi.end(), ostream_iterator<size_t>(cout," "));
                        cout << ": " << sli << endl;
                    }
                    factors.back()[sli] = val;
                }
            }

            if( verbose >= 3 ) {
                cout << "factors:" << endl;
                copy(factors.begin(), factors.end(), ostream_iterator<Factor>(cout,"\n"));
            }

            fg = FactorGraph(factors);
        } catch (char *e) {
            cout << e << endl;
        }

        return is;
    }


    VarSet FactorGraph::delta( const Var & n ) const {
        return Delta(n) / n;
    }


    VarSet FactorGraph::Delta( const Var & n ) const {
        // calculate Markov Blanket including n
        size_t i = findVar( n );

        VarSet del;
        for( nb_cit I = nbV(i).begin(); I != nbV(i).end(); I++ )
            del |= factor(*I).vars();

        return del;
    }


    VarSet FactorGraph::Delta( const VarSet &ns ) const {
        VarSet result;
        for( VarSet::const_iterator n = ns.begin(); n != ns.end(); n++ ) 
            result |= Delta(*n);
        return result;
    }


    void FactorGraph::makeCavity(const Var & n) {
        // fills all Factors that include Var n with ones
        size_t i = findVar( n );
        map<size_t,Factor> newFacs;
        for( nb_cit I = nbV(i).begin(); I != nbV(i).end(); I++ )
            newFacs[*I] = Factor(factor(*I).vars(), 1.0);
        setFactors( newFacs );

    //    VarSet delta_n = Delta(n);
    //    for( size_t I = 0; I < nrFactors(); I++ )
    //       if( factor(I).vars() << delta_n )
    //            factor(I).fill(1.0);
    }


    void FactorGraph::ReadFromFile( const char *filename ) {
        ifstream infile;
        infile.open( filename );
        if( infile.is_open() ) {
            try { // FIXME: no exceptions are generated by iostreams by default
                infile >> *this;
                infile.close();
            } catch (...) {
                infile.close();
                throw;
            }
        } else
            DAI_THROW(CANNOT_READ_FILE);
    }


    void FactorGraph::WriteToFile( const char *filename ) const {
        ofstream outfile;
        outfile.open( filename );
        if( outfile.is_open() ) {
            try { // FIXME: no exceptions are generated by iostreams by default
                outfile << *this;
                outfile.close();
            } catch (...) {
                outfile.close();
                throw;
            }
        } else
            DAI_THROW(CANNOT_WRITE_FILE);
    }


    void FactorGraph::display( std::ostream& os ) const {
        os << "graph G {" << endl;
        os << "node[shape=circle,width=0.4,fixedsize=true];" << endl;
        for( size_t i = 0; i < nrVars(); i++ )
            os << "\tv" << var(i).label() << ";" << endl;
        os << "node[shape=box,width=0.3,height=0.3,fixedsize=true];" << endl;
        for( size_t I = 0; I < nrFactors(); I++ )
            os << "\tf" << I << ";" << endl;
        for( size_t iI = 0; iI < nrEdges(); iI++ )
            os << "\tv" << var(edge(iI).first).label() << " -- f" << edge(iI).second << ";" << endl;
        os << "}" << endl;
    }


    bool hasShortLoops( const std::vector<Factor> &P ) {
        bool found = false;
        vector<Factor>::const_iterator I, J;
        for( I = P.begin(); I != P.end(); I++ ) {
            J = I;
            J++;
            for( ; J != P.end(); J++ )
                if( (I->vars() & J->vars()).size() >= 2 ) {
                    found = true;
                    break;
                }
            if( found )
                break;
        }
        return found;
    }


    bool hasNegatives( const std::vector<Factor> &P ) {
        bool found = false;
        for( size_t I = 0; I < P.size(); I++ )
            if( P[I].hasNegatives() ) {
                found = true;
                break;
            }
        return found;
    }


    void RemoveShortLoops(std::vector<Factor> &P) {
        bool found = true;
        while( found ) {
            found = false;
            vector<Factor>::iterator I, J;
            for( I = P.begin(); I != P.end(); I++ ) {
                J = I;
                J++;
                for( ; J != P.end(); J++ )
                    if( (I->vars() & J->vars()).size() >= 2 ) {
                        found = true;
                        break;
                    }
                if( found )
                    break;
            }
            if( found ) {
                cout << "Merging factors " << I->vars() << " and " << J->vars() << endl;
                *I *= *J;
                P.erase(J);
            }
        }
    }


    std::vector<VarSet> FactorGraph::Cliques() const {
        vector<VarSet> result;
        
        for( size_t I = 0; I < nrFactors(); I++ ) {
            bool maximal = true;
            for( size_t J = 0; (J < nrFactors()) && maximal; J++ )
                if( (factor(J).vars() >> factor(I).vars()) && !(factor(J).vars() == factor(I).vars()) )
                    maximal = false;
            
            if( maximal )
                result.push_back( factor(I).vars() );
        }

        return result;
    }


    void FactorGraph::clamp( const Var & n, size_t i, bool backup ) {
        assert( i <= n.states() );

    /*  if( do_surgery ) {
            size_t ni = find( vars().begin(), vars().end(), n) - vars().begin();

            if( ni != nrVars() ) {
                for( nb_cit I = nb1(ni).begin(); I != nb1(ni).end(); I++ ) {
                    if( factor(*I).size() == 1 )
                        // Remove this single-variable factor
        //              I = (_V2.erase(I))--;
                        _E12.erase( _E12.begin() + VV2E(ni, *I) );
                    else {
                        // Replace it by the slice
                        Index ind_I_min_n( factor(*I), factor(*I) / n );
                        Index ind_n( factor(*I), n );
                        Factor slice_I( factor(*I) / n );
                        for( size_t ind_I = 0; ind_I < factor(*I).stateSpace(); ++ind_I, ++ind_I_min_n, ++ind_n )
                            if( ind_n == i )
                                slice_I[ind_I_min_n] = factor(*I)[ind_I];
                        factor(*I) = slice_I;

                        // Remove the edge between n and I
                        _E12.erase( _E12.begin() + VV2E(ni, *I) );
                    }
                }

                Regenerate();
                
                // remove all unconnected factors
                for( size_t I = 0; I < nrFactors(); I++ )
                    if( nb2(I).size() == 0 )
                        DeleteFactor(I--);

                DeleteVar( ni );

                // FIXME
            }
        } */

        // The cheap solution (at least in terms of coding time) is to multiply every factor
        // that contains the variable with a delta function

        Factor delta_n_i(n,0.0);
        delta_n_i[i] = 1.0;

        map<size_t, Factor> newFacs;
        // For all factors that contain n
        for( size_t I = 0; I < nrFactors(); I++ ) 
            if( factor(I).vars() && n )
                // Multiply it with a delta function
                newFacs[I] = factor(I) * delta_n_i;
        setFactors( newFacs, backup );

        return;
    }


    void FactorGraph::backupFactor( size_t I ) {
        map<size_t,Factor>::iterator it = _backupFactors.find( I );
        if( it != _backupFactors.end() )
            DAI_THROW( MULTIPLE_UNDO );
        _backupFactors[I] = factor(I);
    }


    void FactorGraph::restoreFactor( size_t I ) {
        map<size_t,Factor>::iterator it = _backupFactors.find( I );
        if( it != _backupFactors.end() ) {
            setFactor(I, it->second);
            _backupFactors.erase(it);
        }
    }


    void FactorGraph::backupFactors( const VarSet &ns ) {
        for( size_t I = 0; I < nrFactors(); I++ )
            if( factor(I).vars() && ns )
                backupFactor( I );
    }


    void FactorGraph::restoreFactors( const VarSet &ns ) {
        map<size_t,Factor> facs;
        for( map<size_t,Factor>::iterator uI = _backupFactors.begin(); uI != _backupFactors.end(); ) {
            if( factor(uI->first).vars() && ns ) {
                facs.insert( *uI );
                _backupFactors.erase(uI++);
            } else
                uI++;
        }
        setFactors( facs );
    }


    void FactorGraph::restoreFactors() {
        setFactors( _backupFactors );
        _backupFactors.clear();
    }

    void FactorGraph::backupFactors( const std::set<size_t> & facs ) {
        for( std::set<size_t>::const_iterator fac = facs.begin(); fac != facs.end(); fac++ )
            backupFactor( *fac );
    }


    bool FactorGraph::isPairwise() const {
        bool pairwise = true;
        for( size_t I = 0; I < nrFactors() && pairwise; I++ )
            if( factor(I).vars().size() > 2 )
                pairwise = false;
        return pairwise;
    }


    bool FactorGraph::isBinary() const {
        bool binary = true;
        for( size_t i = 0; i < nrVars() && binary; i++ )
            if( var(i).states() > 2 )
                binary = false;
        return binary;
    }


    FactorGraph FactorGraph::clamped( const Var & v_i, size_t state ) const {
        Real zeroth_order = 1.0;
        vector<Factor> clamped_facs;
        for( size_t I = 0; I < nrFactors(); I++ ) {
            VarSet v_I = factor(I).vars();
            Factor new_factor;
            if( v_I && v_i )
                new_factor = factor(I).slice( v_i, state );
            else
                new_factor = factor(I);

            if( new_factor.vars().size() != 0 ) {
                size_t J = 0;
                // if it can be merged with a previous one, do that
                for( J = 0; J < clamped_facs.size(); J++ )
                    if( clamped_facs[J].vars() == new_factor.vars() ) {
                        clamped_facs[J] *= new_factor;
                        break;
                    }
                // otherwise, push it back
                if( J == clamped_facs.size() || clamped_facs.size() == 0 )
                    clamped_facs.push_back( new_factor );
            } else
                zeroth_order *= new_factor[0];
        }
        *(clamped_facs.begin()) *= zeroth_order;
        return FactorGraph( clamped_facs );
    }


}
