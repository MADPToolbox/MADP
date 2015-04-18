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


#include "util.h"
#include <boost/random/linear_congruential.hpp>
#include <boost/random/uniform_real.hpp>
#include <boost/random/normal_distribution.hpp>
#include <boost/random/variate_generator.hpp>


namespace libDAI {


    clock_t toc() {
        tms tbuf;
        times(&tbuf);
        return( tbuf.tms_utime );
    }


    // This is a typedef for a random number generator.
    // Try boost::mt19937 or boost::ecuyer1988 instead of boost::minstd_rand
    typedef boost::minstd_rand _rnd_gen_type;

    _rnd_gen_type _rnd_gen(42u);

    // Define a uniform random number distribution which produces "double"
    // values between 0 and 1 (0 inclusive, 1 exclusive).
    boost::uniform_real<> _uni_dist(0,1);
    boost::variate_generator<_rnd_gen_type&, boost::uniform_real<> > _uni_rnd(_rnd_gen, _uni_dist);

    // Define a normal distribution with mean 0 and standard deviation 1.
    boost::normal_distribution<> _normal_dist;
    boost::variate_generator<_rnd_gen_type&, boost::normal_distribution<> > _normal_rnd(_rnd_gen, _normal_dist);


    void rnd_seed( size_t seed ) {
        _rnd_gen.seed(seed);
    }

    double rnd_uniform() {
        return _uni_rnd();
    }

    double rnd_stdnormal() {
        return _normal_rnd();
    }

    int rnd_int( int min, int max ) {
        return (int)floor(_uni_rnd() * (max - min) + min);
    }


}
