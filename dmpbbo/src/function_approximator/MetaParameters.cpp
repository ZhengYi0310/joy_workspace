/*************************************************************************
	> File Name: MetaParameters.cpp
	> Author: Yi Zheng 
	> Mail: hczhengcq@gmail.com
	> Created Time: Thu 17 Aug 2017 11:18:39 AM PDT
 ************************************************************************/
#include <iostream>
#include <assert.h>

#include <boost/serialization/export.hpp>
#include <boost/archive/text_iarchive.hpp>
#include <boost/archive/text_oarchive.hpp>
#include <boost/archive/xml_iarchive.hpp>
#include <boost/archive/xml_oarchive.hpp>
#include <function_approximator/MetaParameters.hpp>

/** For boost::serialization. See http://www.boost.org/doc/libs/1_55_0/libs/serialization/doc/special.html#export */
BOOST_CLASS_EXPORT_IMPLEMENT(function_approximator::MetaParameters);

using namespace std;

namespace DmpBbo {

MetaParameters::MetaParameters(int expected_input_dim)
: expected_input_dim_(expected_input_dim)
{
  assert(expected_input_dim_>0);
}
                                                                          
MetaParameters::~MetaParameters(void) 
{
}

ostream& operator<<(std::ostream& output, const MetaParameters& meta_parameters) {
  output << meta_parameters.toString();
  return output;
}

}

