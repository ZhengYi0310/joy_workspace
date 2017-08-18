#ifndef BOOSTSERIALIZATIONTOSTRING_HPP
#define BOOSTSERIALIZATIONTOSTRING_HPP

#include <boost/serialization/nvp.hpp>
#include <boost/archive/text_oarchive.hpp>
#include <boost/archive/text_iarchive.hpp>
#include <boost/archive/xml_oarchive.hpp>
#include <boost/archive/xml_iarchive.hpp>

/** Macro to convert the boost XML serialization of an object into a string.
 * For example, see ExponentialSystem::toString()
 */
#define RETURN_STRING_FROM_BOOST_SERIALIZATION_XML(name) \
  std::stringstream strstr; \
  unsigned int flags = boost::archive::no_header; \
  boost::archive::xml_oarchive oa(strstr,flags); \
  oa << boost::serialization::make_nvp(name, *this); \
  return strstr.str();
  
/** Macro to convert the boost text serialization of an object into a string.
 * For example, see ExponentialSystem::toString()
 */
#define RETURN_STRING_FROM_BOOST_SERIALIZATION_TXT(name) \
  std::stringstream strstr; \
  unsigned int flags = boost::archive::no_header; \
  boost::archive::txt_oarchive oa(strstr,flags); \
  oa << boost::serialization::make_nvp(name, *this); \
  return strstr.str();

#endif        //  #ifndef BOOSTSERIALIZATIONTOSTRING_HPP

