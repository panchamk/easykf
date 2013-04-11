#include <boost/python.hpp>
#include "ukf_math.h"

using namespace boost::python;

BOOST_PYTHON_MODULE(libpyeasykf)
{
	def("ukf_math_min", ukf::math::min);
}
