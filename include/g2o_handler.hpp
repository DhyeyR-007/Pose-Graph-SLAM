/* ----------------------------------------------------------------------------
 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file dataset.cpp
 * @date Jan 22, 2010
 * @author Kai Ni, Luca Carlone, Frank Dellaert
 * @brief utility functions for loading datasets
 */


#include <gtsam/slam/dataset.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/geometry/Pose2.h>
#include <gtsam/nonlinear/GaussNewtonOptimizer.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <fstream>

using namespace std;
using namespace gtsam;

static SharedNoiseModel readNoiseModel(ifstream& is, bool smart,
    NoiseFormat noiseFormat, KernelFunctionType kernelFunctionType) {
  double v1, v2, v3, v4, v5, v6;
  is >> v1 >> v2 >> v3 >> v4 >> v5 >> v6;
  if (noiseFormat == NoiseFormatAUTO) {
    // Try to guess covariance matrix layout
    if (v1 != 0.0 && v2 == 0.0 && v3 != 0.0 && v4 != 0.0 && v5 == 0.0
        && v6 == 0.0) {
      // NoiseFormatGRAPH
      noiseFormat = NoiseFormatGRAPH;
    } else if (v1 != 0.0 && v2 == 0.0 && v3 == 0.0 && v4 != 0.0 && v5 == 0.0
        && v6 != 0.0) {
      // NoiseFormatCOV
      noiseFormat = NoiseFormatCOV;
    } else {
      throw std::invalid_argument(
          "load2D: unrecognized covariance matrix format in dataset file. Please specify the noise format.");
    }
  }

  // Read matrix and check that diagonal entries are non-zero
  Matrix M(3, 3);
  switch (noiseFormat) {
  case NoiseFormatG2O:
    M << v1, v2, v3, v2, v4, v5, v3, v5, v6;
    break;
  case NoiseFormatCOV:
    // i.e., [ v1 v2 v3; v2' v4 v5; v3' v5' v6 ]
    if (v1 == 0.0 || v4 == 0.0 || v6 == 0.0)
      throw runtime_error(
          "load2D::readNoiseModel looks like this is not G2O matrix order");
    M << v1, v2, v3, v2, v4, v5, v3, v5, v6;
    break;
  case NoiseFormatTORO:
  case NoiseFormatGRAPH:
    // http://www.openslam.org/toro.html
    // inf_ff inf_fs inf_ss inf_rr inf_fr inf_sr
    // i.e., [ v1 v2 v5; v2' v3 v6; v5' v6' v4 ]
    if (v1 == 0.0 || v3 == 0.0 || v4 == 0.0)
      throw invalid_argument(
          "load2D::readNoiseModel looks like this is not TORO matrix order");
    M << v1, v2, v5, v2, v3, v6, v5, v6, v4;
    break;
  default:
    throw runtime_error("load2D: invalid noise format");
  }
  // Now, create a Gaussian noise model
  // The smart flag will try to detect a simpler model, e.g., unit
  SharedNoiseModel model;
  switch (noiseFormat) {
  case NoiseFormatG2O:
    model = noiseModel::Gaussian::Information(M, smart);
    break;
  case NoiseFormatTORO:
    // In both cases, what is stored in file is the information matrix
    model = noiseModel::Gaussian::Information(M, smart);
    break;
  case NoiseFormatGRAPH:
  case NoiseFormatCOV:
    // These cases expect covariance matrix
    model = noiseModel::Gaussian::Covariance(M, smart);
    break;
  default:
    throw invalid_argument("load2D: invalid noise format");
  }

  switch (kernelFunctionType) {
  case KernelFunctionTypeNONE:
    return model;
    break;
  case KernelFunctionTypeHUBER:
    return noiseModel::Robust::Create(
        noiseModel::mEstimator::Huber::Create(1.345), model);
    break;
  case KernelFunctionTypeTUKEY:
    return noiseModel::Robust::Create(
        noiseModel::mEstimator::Tukey::Create(4.6851), model);
    break;
  default:
    throw invalid_argument("load2D: invalid kernel function type");
  }
}

void load_g2o_2D(const string& filename, std::vector<boost::optional<IndexedPose>>& pose_vec, \
                    std::vector<boost::optional<IndexedEdge>>& edge_vec, \
                    std::vector<SharedNoiseModel>& NoiseModels){
                
    string tag;

    ifstream is(filename.c_str());
    if (!is)
        throw invalid_argument("Can not find file " + filename);
    while (!is.eof()) {
        if (!(is >> tag))
            break;
        const auto indexed_pose = parseVertex(is, tag);
        if (indexed_pose) {
            pose_vec.push_back(indexed_pose);
        }

        auto between_pose = parseEdge(is, tag);
        if (between_pose) {
            edge_vec.push_back(between_pose);
            SharedNoiseModel noise_model = readNoiseModel(is, true, NoiseFormatG2O, KernelFunctionTypeNONE);
            NoiseModels.push_back(noise_model);
        }
    }
}

void load_g2o_3D(const string& filename, std::vector<boost::optional<IndexedPose>>& pose_vec, \
                    std::vector<boost::optional<IndexedEdge>>& edge_vec, \
                    std::vector<SharedNoiseModel>& NoiseModels){
                
    string tag;

    ifstream is(filename.c_str());
    if (!is)
        throw invalid_argument("Can not find file " + filename);
    while (!is.eof()) {
        if (!(is >> tag))
            break;
        const auto indexed_pose = parseVertex(is, tag);
        if (indexed_pose) {
            pose_vec.push_back(indexed_pose);
        }

        auto between_pose = parseEdge(is, tag);
        if (between_pose) {
            edge_vec.push_back(between_pose);
            SharedNoiseModel noise_model = readNoiseModel(is, true, NoiseFormatG2O, KernelFunctionTypeNONE);
            NoiseModels.push_back(noise_model);
        }
    }
}