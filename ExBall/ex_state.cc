// Compile with :
// g++ -o ex_state ex_state.cc -O3 `pkg-config --libs --cflags easykf`

// We seek to estimate the parameters (mass) and state (2D position, 2D velocity) of a ball thrown with an initial unknown speed at an unknown position, within the gravity field and resistance to air (modeled as a squared term to introduce some non linearity)

#include <fstream>
#include <cmath>
#include <ukf.h>

using namespace ukf::state;

#define G 9.81 // m.s^-2

// The object we throw is a Wood sphere of radius 10 cm
// We take its density to be 700 kg . m^-3
#define DENSITY 700 // kg . m^-3
#define RADIUS 0.1 // m
#define MASS ((4.0/3.0 * M_PI * RADIUS * RADIUS * RADIUS) * DENSITY) // kg
#define CROSS_SECTION (M_PI * RADIUS * RADIUS) // m^2 , required to compute the resistance to air

#define DRAG_COEFFICIENT 0.47 // dimensionless, for a sphere
#define DENSITY_AIR 1.2041 // kg . m^-3

// The parameters for our noisy observations
#define NOISE_AMPLITUDE 1.0 // m.

// The true initial positions (that are supposed unknown but required to simulate the observations)
#define X0 0
#define Y0 0
#define THETA0 M_PI_4 // Angle of the trajectory, in radians
#define SPEED0 10.0 // m.s^-1 Amplitude of the speed
#define DX0 (SPEED0*cos(THETA0))
#define DY0 (SPEED0*sin(THETA0))

#define DT 0.01 // s., Time step for the simulation of the system
#define DT_KALMAN 0.05 // s. , time step for the observations

/*************************************************************************************/
/*            Definition of the evolution and observation functions                  */
/*************************************************************************************/

// Evolution function
void evo_function(gsl_vector * params, gsl_vector * xk_1, gsl_vector * xk)
{
  // m dvx / dt = -1/2 rho Cd A v^2 v_x / sqrt(vx^2 + vy^2)
  //            = -1/2 rho Cd A sqrt(vx^2 + vy^2) v_x
  // m dvy / dt = -1/2 rho Cd A v^2 v_y / sqrt(vx^2 + vy^2) - m g
  //            = -1/2 rho Cd A sqrt(vx^2 + vy^2) v_y  - m g

  // Which gives the following Euler discretized evolution function
  // x(t+1) = x(t) + dt dx(t)
  // y(t+1) = y(t) + dt dy(t)
  // dx(t+1)= dx(t) + dt (-1/2 rho Cd A sqrt(dx^2 + dy^2) d_x)
  // dy(t+1)= dy(t) + dt (-1/2 rho Cd A sqrt(dx^2 + dy^2) d_y  - m g)
  // mass(t+1) = mass(t)

    double x = gsl_vector_get(xk_1,0);
    double y = gsl_vector_get(xk_1,1);
    double dx = gsl_vector_get(xk_1,2);
    double dy = gsl_vector_get(xk_1,3);
    double m = gsl_vector_get(xk_1,4);

    double dt = gsl_vector_get(params, 0);

    gsl_vector_set(xk, 0, x + dt * dx);
    gsl_vector_set(xk, 1, y + dt * dy);
    gsl_vector_set(xk, 2, dx + dt * (-0.5*DENSITY_AIR*DRAG_COEFFICIENT*CROSS_SECTION*sqrt(dx*dx + dy*dy)*dx));
    gsl_vector_set(xk, 3, dy + dt * (-0.5*DENSITY_AIR*DRAG_COEFFICIENT*CROSS_SECTION*sqrt(dx*dx + dy*dy)*dy - m * G));
    gsl_vector_set(xk, 4, m);
}

// Observation function
void obs_function(gsl_vector * xk , gsl_vector * yk)
{
  // We just observe the position of the object, which are the first components of the state
    for(int i = 0 ; i < yk->size ; ++i)
        gsl_vector_set(yk, i, gsl_vector_get(xk,i) + NOISE_AMPLITUDE*rand()/ double(RAND_MAX));  
}


int main(int argc, char * argv[])
{
    srand(time(NULL));

    // Definition of the parameters and state variables
    ukf_param p;
    ukf_state s, s_simulator;
    // The parameters for the evolution equation
    s.params = gsl_vector_alloc(1);
    s.params->data[0] = DT_KALMAN;

    // The simulator uses a smaller time step than the Kalman filter
    s_simulator.params = gsl_vector_alloc(1);
    s_simulator.params->data[0] = DT;

    // Initialization of the parameters
    p.n = 5;
    p.no = 2;
    p.kpa = 0.0;
    p.alpha = 0.9;
    p.beta = 2.0;

    EvolutionNoise * evolution_noise = new EvolutionRobbinsMonro(1e-4, 1e-3);
    p.evolution_noise = evolution_noise;

    p.measurement_noise = 1e-1;
    p.prior_x= 1.0;

    // Initialization of the state and parameters
    ukf_init(p,s);

    // Initialize the parameter vector to some random values in [-5, 5]
    for(int i = 0 ; i < p.n ; i++)
        gsl_vector_set(s.xi,i,5.0*(2.0*rand()/double(RAND_MAX-1)-1.0));

    // Allocate the input/output vectors
    gsl_vector * xi = gsl_vector_alloc(p.n);
    gsl_vector * yi = gsl_vector_alloc(p.no);
    gsl_vector_set_zero(yi);

    /***************************************************/
    /***** Iterate the learning on the simulation *****/
    /*************************************************/
    int epoch = 0;

    xi->data[0] = X0;
    xi->data[1] = Y0;
    xi->data[2] = DX0;
    xi->data[3] = DY0;
    xi->data[4] = MASS;

    std::ofstream outfile("trajectory.data");

    // Let's use the filter for 10 seconds
    double t_max = 10;
    double t = 0;
    double t_simu = 0;
    while(t < t_max)
      {
	// We run the simulator to get the observations at time t with a time step of dt
	while(t_simu < t) 
	  {
	    evo_function(s_simulator.params, xi, xi);
	    t_simu += DT;
	  }
	obs_function(xi, yi);

        // Provide the observation to the filter and iterate
        ukf_iterate(p,s,evo_function, obs_function,yi);

	// Record the true (xi->data[..]) and estimated states (s.xi->data[...])
	outfile << t << " ";
	for(int i = 0 ; i < p.n ; ++i)
	  outfile << xi->data[i] << " ";
	// we record also the observations
	for(int i = 0 ; i < p.no ; ++i)
	  outfile << yi->data[i] << " ";
	for(int i = 0 ; i < p.n ; ++i)
	  outfile << s.xi->data[i] << " ";
	outfile << std::endl;
	
	// Time step update
	t += DT_KALMAN;
      }

    outfile.close();

    std::cout << "Estimation completed, you can plot the trajectory with gnuplot for example" << std::endl;
    std::cout << " plot 'trajectory.data' using 2:3 with lines, 'trajectory.data' using 7:8, 'trajectory.data' using 9:10 " << std::endl;
    std::cout << "At the end, the mass was estimated as " << s.xi->data[p.n-1] << " kg while the true mass is " << MASS << " kg " << std::endl;

}
