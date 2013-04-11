#include <boost/python.hpp>
#include "easykf.h"

using namespace boost::python;

// On utilise des templates pour le xxx_iterate mais 
// en pratrique on doit pouvoir avoir le type de notre fonction.

namespace easykf {

  namespace bindings {

  }
}


BOOST_PYTHON_MODULE(pyeasykf)
{
  ////////////////
  // ekf_types.h

  class_<ekf::EvolutionAnneal>("ekf_EvolutionAnneal", init<double, double, double>())
    .def("updateEvolutionNoise", ekf::EvolutionAnneal::updateEvolutionNoise, "")
    ;

  // Bindings for ekf.h
  // struct ekf::ekf_param, ekf::ekf_state
  // def("ekf_param", ekf::ekf_param);
  // def("ekf_state", ekf::ekf_state);

  def("ekf_init", ekf::ekf_init);
  def("ekf_free", ekf::ekf_free);
  //def("ekf_iterate", ekf::ekf_iterate);


  def("ukf_math_min", ukf::math::min);
  def("ukf_math_choleskyUpdate", ukf::math::choleskyUpdate);
	

}
