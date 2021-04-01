#pragma once

#include <cmath>
#include <cstdio>
#include <iostream>
#include <fstream>
#include <string>

#include "ceres/ceres.h"
#include "ceres/rotation.h"

namespace simplebal {

// Read a Bundle Adjustment in the Large dataset.
class BALManager {
public:
  ~BALManager(); 

  const double* observations() const { return observations_; }
  int num_observations() const { return num_observations_; }

  double* mutable_cameras() { return parameters_; } // // return the pointer at the "start position" of the camera parameters 
  double* mutable_points() { return parameters_ + 9*num_cameras_; } // return the pointer at the "start position" of the landmakrs 
  double* mutable_camera_for_observation(int i);
  double* mutable_point_for_observation(int i);

  bool loadFile(const char* filename);
  void writeResultFile(const std::string& filename);
  void writeResultFile(void);
  void writeResultFile(int _iter_counter);

private:
  template <typename T>
  void FscanfOrDie(FILE* fptr, const char* format, T* value) {
    int num_scanned = fscanf(fptr, format, value);
    if (num_scanned != 1) {
      LOG(FATAL) << "Invalid UW data file.";
    }
  }

private:
  int num_cameras_;
  int num_points_;
  int num_observations_;
  int num_parameters_;

  int* point_index_;
  int* camera_index_;
  double* observations_;
  double* parameters_;

public:
  std::string fileName;
};

} // namespace simplebal


double* simplebal::BALManager::mutable_camera_for_observation(int i) {
  return mutable_cameras() + 9*camera_index_[i];
} // mutable_camera_for_observation

double* simplebal::BALManager::mutable_point_for_observation(int i) {
  return mutable_points() + 3*point_index_[i];
} // mutable_point_for_observation

bool simplebal::BALManager::loadFile(const char* filename) {
  FILE* fptr = fopen(filename, "r");
  if (fptr == NULL) {
    return false;
  };

  FscanfOrDie(fptr, "%d", &num_cameras_);
  FscanfOrDie(fptr, "%d", &num_points_);
  FscanfOrDie(fptr, "%d", &num_observations_);

  point_index_ = new int[num_observations_];
  camera_index_ = new int[num_observations_];
  observations_ = new double[2 * num_observations_];

  num_parameters_ = 9*num_cameras_ + 3*num_points_;
  parameters_ = new double[num_parameters_];

  for (int i = 0; i < num_observations_; ++i) {
    FscanfOrDie(fptr, "%d", camera_index_ + i);
    FscanfOrDie(fptr, "%d", point_index_ + i);
    for (int j = 0; j < 2; ++j) {
      FscanfOrDie(fptr, "%lf", observations_ + 2 * i + j);
    }
  }

  for (int i = 0; i < num_parameters_; ++i) {
    FscanfOrDie(fptr, "%lf", parameters_ + i);
  }

  return true;
} // loadFile

simplebal::BALManager::~BALManager() {
    delete[] point_index_;
    delete[] camera_index_;
    delete[] observations_;
    delete[] parameters_;
} // ~BALManager

void simplebal::BALManager::writeResultFile(const std::string& _filename) {
	// write File
  if(fileName.empty())
    fileName = _filename;

	std::ofstream writeFile(_filename.data());
	if( writeFile.is_open() ) {
		writeFile << "Hello World!\n";
		writeFile << "This is C++ File Contents.\n";
		writeFile.close();
	}
} // writeResultFile

void simplebal::BALManager::writeResultFile(void) {
  std::string fileNameTmp {"/tmp/result.txt"};
  if( ! fileName.empty())
    fileNameTmp = fileName;
    
	// write File
	std::ofstream writeFile(fileNameTmp.data());
	if( writeFile.is_open() ) {
    for(int i=0; i<num_points_; i++) {
      double x = *(mutable_points() + 3*i + 0);
      double y = *(mutable_points() + 3*i + 1);
      double z = *(mutable_points() + 3*i + 2);
      // std::cout << x << " " << y << " " << z << std::endl;
      writeFile << x << " " << y << " " << z << std::endl;
    }
		writeFile.close();
	}
} // writeResultFile

void simplebal::BALManager::writeResultFile(int _iter_counter) {
  std::string fileNameTmp {"/tmp/result.txt"};
  if( ! fileName.empty())
    fileNameTmp = fileName;
    
	// write File
	std::ofstream writeFile(fileNameTmp + "-" + std::to_string(_iter_counter) + ".csv");
	if( writeFile.is_open() ) {
    for(int i=0; i<num_points_; i++) {
      double x = *(mutable_points() + 3*i + 0);
      double y = *(mutable_points() + 3*i + 1);
      double z = *(mutable_points() + 3*i + 2);
      // std::cout << x << " " << y << " " << z << std::endl;
      writeFile << x << " " << y << " " << z << std::endl;
    }
		writeFile.close();
	}
} // writeResultFile
