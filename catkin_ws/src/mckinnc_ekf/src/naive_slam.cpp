#include "mckinnc_ekf/naive_slam.hpp"

namespace mckinnc_ekf {

NaiveSLAM::NaiveSLAM(const Covariance& modelCovariance,
    const Observation& observationVariance,
    double newLandmarkUncertaintyThreshold)
  : ekf_(modelCovariance, observationVariance),
    newLandmarkUncertaintyThreshold_(newLandmarkUncertaintyThreshold)
{ }

// ### STATIC FUNCTIONS AND STRUCTS FOR NAIVE SLAM RUN ########################

// ############################################################################

} // namespace
