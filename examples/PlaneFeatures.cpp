// Adapted from https://github.com/borglab/gtsam/issues/561
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/slam/OrientedPlane3Factor.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/nonlinear/DoglegOptimizer.h>

using namespace gtsam;
using namespace std; 

// Create unique keys for each state.
using symbol_shorthand::X; //< Pose3 (x,y,z,r,p,y)
using symbol_shorthand::P; //< Planes

int main(int argc, char** argv){
 // Typedefs
  using Plane       = OrientedPlane3;
  using PlaneFactor = gtsam::OrientedPlane3Factor;

  // Setup graph & object
  FastMap<char, Vector> thresholds;
  thresholds['x'] = (Vector(6) << 0.05, 0.05, 0.05, 0.03, 0.03, 0.03).finished();
  thresholds['p'] = Vector3{0.03, 0.03, 0.1};

  //ISAM2Params isam2params;
  //isam2params.setRelinearizeThreshold(thresholds);
  //isam2params.setRelinearizeSkip(3);
  //isam2params.findUnusedFactorSlots = true;
  //double lag = 1;
  //gtsam::IncrementalFixedLagSmoother smoother(); //lag, isam2params);

  NonlinearFactorGraph graph;
  Values initialEstimate;
  map<gtsam::Key, double> timestampMap;

  // Initial values
  Plane p849(0.211098835, 0.214292752, 0.95368543, 26.4269514);
  Plane p897(0.301901811, 0.151751467, 0.941183717, 33.4388229);

  Pose3 x1527(Rot3(0.799903913, -0.564527097,  0.203624376, 0.552537226,   0.82520195,  0.117236322, -0.234214312, 0.0187322547,  0.972004505), Vector3{-91.7500013,-0.47569366,-4.61067606});

  // Setup priors
  initialEstimate.insert(P(849) , p849 );
  initialEstimate.insert(P(897) , p897 );
  initialEstimate.insert(X(1527), x1527);

  // Setup prior factors
  Pose3 x1527_prior(Rot3(0.799903913, -0.564527097,  0.203624376, 0.552537226,   0.82520195,  0.117236322, -0.234214312, 0.0187322547,  0.972004505), Vector3{-91.7500013, -0.47569366, -4.61067606});
  auto x1527_noise = gtsam::noiseModel::Isotropic::Sigma(6, 0.01);

  graph.emplace_shared<PriorFactor<Pose3>>(X(1527), x1527_prior, x1527_noise);

  Plane p849_prior(0.211098835, 0.214292752, 0.95368543, 26.4269514);
  Plane p897_prior(0.301901811, 0.151751467, 0.941183717, 33.4388229);

  auto p849_noise  = gtsam::noiseModel::Diagonal::Sigmas(Vector3{0.785398163, 0.785398163, 5});
  auto p897_noise  = gtsam::noiseModel::Diagonal::Sigmas(Vector3{0.785398163, 0.785398163, 5});

  graph.emplace_shared<PriorFactor<Plane>>(P(849), p849_prior, p849_noise);
  graph.emplace_shared<PriorFactor<Plane>>(P(897), p897_prior, p897_noise);

  Plane p849_meas = Plane(0.0638967294, 0.0755284627, 0.995094297, 2.55956073);
  Plane p897_meas = Plane(0.104902077, -0.0275756528, 0.994100165, 1.32765088);

  // Setup other factors

  const auto x1527_p897_noise = noiseModel::Isotropic::Sigma(3, 0.0322889);

  graph.emplace_shared<PlaneFactor>(p897_meas.planeCoefficients(), x1527_p897_noise, X(1527), P(897));

  const auto x1527_p849_noise = noiseModel::Isotropic::Sigma(3, 0.0451801);

  // OFFENDING LINE BELOW:
  // graph.emplace_shared<PlaneFactor>(p849_meas.planeCoefficients(), x1527_p849_noise, X(1527), P(849));

  // Optimize
  Values result = DoglegOptimizer(graph, initialEstimate).optimize();

  result.print("Final results\n");
  cout << "initial error = " << graph.error(initialEstimate) << endl;
  cout << "final error = " << graph.error(result) << endl;

  return 0;
}
