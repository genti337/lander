#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>

// Model Includes
#include "../math/Vec3.h"
#include "../dyn_body/DynBody.h"
#include "../eom/EoM.h"
#include "../math/vector_macros.h"
#include "../math/matrix_macros.h"

namespace py = pybind11;

void v_mag(double &vec_mag, py::array_t<double> numpy_vec) {
//   py::buffer_info numpy_buf = numpy_vec.request();

//   vec[0] = numpy_vec[0];
//   vec_mag = V_MAG(vec);
}

PYBIND11_MODULE(liblander, m) {
    m.doc() = "pybind11 example plugin"; // optional module docstring

    // Matrix and Vector Math Operations
    m.def("v_mag", &v_mag, "A function to return the magnitude of a vector.");

    // Generic Vector 3 Class
    py::class_<Vec3>(m, "Vec3")
        .def(py::init<>())                   // Constructor
        .def("Magnitude", &Vec3::Magnitude); // Expose member method

    // Dynamic Body Class
    py::class_<DynBody>(m, "DynBody")
        .def(py::init<>())                   	// Constructor
        .def("setMass", &DynBody::setMass)   	// Expose method to set the body mass
        .def("setCG", &DynBody::setCG)          // Expose method to set the center of gravity
        .def("setECIPos", &DynBody::setECIPos)  // Expose method to set the inertial position
        .def("getECIPos", &DynBody::getECIPos)  // Expose method to get the inertial position
        .def("setECIVel", &DynBody::setECIVel)  // Expose method to set the inertial velocity
        .def("getECIVel", &DynBody::getECIVel)  // Expose method to get the inertial velocity
        .def("getAlt", &DynBody::getAlt);       // Expose method to get the geocentric altitude

    // Equation of Motion Class
    py::class_<EoM>(m, "EoM")
        .def(py::init<>())                   // Constructor
        .def("Initialize", &EoM::Initialize) // Expose method to initialize the equations of motion
        .def("Update", &EoM::Update);    // Expose method to update the equations of motion
}
