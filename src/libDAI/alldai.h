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


#ifndef __defined_libdai_alldai_h
#define __defined_libdai_alldai_h


#include "daialg.h"
#include "exact.h"
#ifdef WITH_BP
    #include "bp.h"
#endif
#ifdef WITH_TRW
    #include "trw.h"
#endif
#ifdef WITH_LC
    #include "lc.h"
#endif
#ifdef WITH_LCLIN
    #include "lclin.h"
#endif
#ifdef WITH_HAK
    #include "hak.h"
#endif
#ifdef WITH_MF
    #include "mf.h"
#endif
#ifdef WITH_JTREE
    #include "jtree.h"
#endif
#ifdef WITH_IJGP
    #include "ijgp.h"
#endif
#ifdef WITH_TREEEP
    #include "treeep.h"
#endif
#ifdef WITH_LCBP1
    #include "lcbp1.h"
#endif
#ifdef WITH_LCBP2
    #include "lcbp2.h"
#endif
#ifdef WITH_LCBP3
    #include "lcbp3.h"
#endif
#ifdef WITH_LCBP4
    #include "lcbp4.h"
#endif
#ifdef WITH_LCBP5
    #include "lcbp5.h"
#endif
#ifdef WITH_LCBPLIN
    #include "lcbplin.h"
#endif
#ifdef WITH_MR
    #include "mr.h"
#endif
#ifdef WITH_GMR
    #include "gmr.h"
#endif
#ifdef WITH_MAXPLUS
    #include "maxplus.h"
#endif



namespace libDAI {


    /// newInfAlg constructs a new approximate inference algorithm named name for the
    /// FactorGraph fg with optionts opts and returns a pointer to the new object.
    /// The caller needs to delete it (maybe some sort of smart_ptr might be useful here).
    InferenceAlgorithm *newInfAlg( const std::string &name, const FactorGraph &fg, const Properties &opts );


    /// DAINames contains the names of all approximate inference algorithms
    static const char* DAINames[] = {
        Exact::Name,
#ifdef WITH_BP
        BP::Name, 
#endif
#ifdef WITH_TRW
        TRW::Name, 
#endif
#ifdef WITH_MF
        MF::Name,
#endif
#ifdef WITH_HAK
        HAK::Name,
#endif
#ifdef WITH_LC
        LC::Name,
#endif
#ifdef WITH_LCLIN
        LCLin::Name, 
#endif
#ifdef WITH_TREEEP
        TreeEP::Name,
#endif
#ifdef WITH_JTREE
        JTree::Name,
#endif
#ifdef WITH_IJGP
        IJGP::Name,
#endif
#ifdef WITH_LCBPLIN
        LCBPLin::Name,
#endif
#ifdef WITH_LCBP1
        LCBP1::Name,
#endif
#ifdef WITH_LCBP2
        LCBP2::Name,
#endif
#ifdef WITH_LCBP3
        LCBP3::Name,
#endif
#ifdef WITH_LCBP4
        LCBP4::Name,
#endif
#ifdef WITH_LCBP5
        LCBP5::Name,
#endif
#ifdef WITH_MR
        MR::Name,
#endif
#ifdef WITH_GMR
        GMR::Name,
#endif
#ifdef WITH_MAXPLUS
        MaxPlus::Name,
#endif
        ""
    };


}


#endif
