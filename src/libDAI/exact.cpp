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
#include <sstream>
#include <map>
#include <set>
#include <algorithm>
#include "exact.h"
#include "diffs.h"
#include "util.h"
#include "properties.h"
#include "exceptions.h"


namespace libDAI {


    using namespace std;


    const char *Exact::Name = "EXACT";


    bool Exact::initProps() {
        if( HasProperty("verbose") )
            Props.verbose = FromStringTo<size_t>("verbose");
        else
            Props.verbose = 0UL;

        return true;
    }


    Exact::Exact(const FactorGraph & fg, const Properties &opts) : DAIAlgFG(fg, opts) {
        if( !initProps() )
            DAI_THROW(NOT_ALL_PROPERTIES_SPECIFIED);
        
        // clear variable beliefs and reserve space
        _beliefsV.clear();
        _beliefsV.reserve(grm().nrVars());
        for( size_t i = 0; i < grm().nrVars(); i++ )
            _beliefsV.push_back( Factor( grm().var(i) ) );

        // clear factor beliefs and reserve space
        _beliefsF.clear();
        _beliefsF.reserve(grm().nrFactors());
        for( size_t I = 0; I < grm().nrFactors(); I++ )
            _beliefsF.push_back( Factor( grm().factor(I).vars() ) );
    }


    void Exact::init() {
        if( !initProps() )
            DAI_THROW(NOT_ALL_PROPERTIES_SPECIFIED);
        for( size_t i = 0; i < grm().nrVars(); i++ )
            _beliefsV[i].fill(1.0);
        for( size_t I = 0; I < grm().nrFactors(); I++ )
            _beliefsF[I].fill(1.0);
    }


    double Exact::run() {
        if( Verbose() >= 1 )
            cout << "Starting " << identify() << "...";

        Factor P;
        for( size_t I = 0; I < grm().nrFactors(); I++ )
            P *= grm().factor(I);

        double Z = P.totalSum();
        _logZ = std::log(Z);
        for( size_t i = 0; i < grm().nrVars(); i++ )
            _beliefsV[i] = P.partSum(grm().var(i)).normalized();
        for( size_t I = 0; I < grm().nrFactors(); I++ )
            _beliefsF[I] = P.partSum(grm().factor(I).vars()).normalized();

        if( Verbose() >= 1 )
            cout << "finished" << endl;

        return 0.0;
    }


    vector<Factor> Exact::beliefs() const {
        vector<Factor> result = _beliefsV;
        result.insert( result.end(), _beliefsF.begin(), _beliefsF.end() );
        return result;
    }


    Factor Exact::belief( const VarSet &ns ) const {
        if( ns.size() == 0 )
            return Factor();
        else if( ns.size() == 1 ) {
            return belief( *(ns.begin()) );
        } else {
            size_t I;
            for( I = 0; I < grm().nrFactors(); I++ )
                if( grm().factor(I).vars() >> ns )
                    break;
            assert( I != grm().nrFactors() );
            return beliefF(I).marginal(ns);
        }
    }


    string Exact::identify() const { 
        stringstream result (stringstream::out);
        result << Name << GetProperties();
        return result.str();
    }


}
