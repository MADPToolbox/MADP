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


#include "alldai.h"
#include "exceptions.h"


namespace libDAI {


    InferenceAlgorithm *newInfAlg( const std::string &name, const FactorGraph &fg, const Properties &opts ) {
        if( name == Exact::Name )
            return new Exact (fg, opts);
#ifdef WITH_BP
        else if( name == BP::Name ) 
            return new BP (fg, opts);
#endif
#ifdef WITH_TRW
        else if( name == TRW::Name ) 
            return new TRW (fg, opts);
#endif
#ifdef WITH_LC
        else if( name == LC::Name )
            return new LC (fg, opts);
#endif
#ifdef WITH_LCLIN
        else if( name == LCLin::Name )
            return new LCLin (fg, opts);
#endif
#ifdef WITH_HAK
        else if( name == HAK::Name ) 
            return new HAK (fg, opts);
#endif
#ifdef WITH_MF
        else if( name == MF::Name ) 
            return new MF (fg, opts);
#endif
#ifdef WITH_JTREE
        else if( name == JTree::Name )
            return new JTree (fg, opts);
#endif
#ifdef WITH_IJGP
        else if( name == IJGP::Name )
            return new IJGP (fg, opts);
#endif
#ifdef WITH_TREEEP
        else if( name == TreeEP::Name )
            return new TreeEP (fg, opts);
#endif
#ifdef WITH_LCBP1
        else if( name == LCBP1::Name )
            return new LCBP1 (fg, opts);
#endif
#ifdef WITH_LCBP2
        else if( name == LCBP2::Name )
            return new LCBP2 (fg, opts);
#endif
#ifdef WITH_LCBP3
        else if( name == LCBP3::Name )
            return new LCBP3 (fg, opts);
#endif
#ifdef WITH_LCBP4
        else if( name == LCBP4::Name )
            return new LCBP4 (fg, opts);
#endif
#ifdef WITH_LCBP5
        else if( name == LCBP5::Name )
            return new LCBP5 (fg, opts);
#endif
#ifdef WITH_LCBPLIN
        else if( name == LCBPLin::Name )
            return new LCBPLin (fg, opts);
#endif
#ifdef WITH_MR
        else if( name == MR::Name )
            return new MR (fg, opts);
#endif
#ifdef WITH_GMR
        else if( name == GMR::Name )
            return new GMR (fg, opts);
#endif
#ifdef WITH_MAXPLUS
        else if( name == MaxPlus::Name )
            return new MaxPlus (fg, opts);
#endif        
        else
            DAI_THROW(UNKNOWN_DAI_ALGORITHM);
    }


}
