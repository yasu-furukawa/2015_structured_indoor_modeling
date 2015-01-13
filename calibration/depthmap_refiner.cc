#include <iostream>
#include <Eigen/SparseCore>
#include <Eigen/Sparse>
#include <Eigen/SparseCholesky>
#include "depthmap_refiner.h"

using namespace std;

namespace structured_indoor_modeling {

void FillHolesAndSmooth(const int width, const int height, const double invalid,
                        std::vector<double>* depths) {
  int valid_count = 0, invalid_count = 0;
  for (const auto value : *depths) {
    if (value == invalid)
      ++invalid_count;
    else
      ++valid_count;
  }
  
  // Number of variables.
  const int num_vars = width * height;
  // Number of constraints
  const int num_constraints = width * height + valid_count;
  //Eigen::SparseMatrix<double,Eigen::RowMajor> A(num_constraints, num_vars);
  Eigen::SparseMatrix<double> A(num_constraints, num_vars);

  const double kDataCost = 1.0;
  
  Eigen::VectorXd b(num_constraints);
  // Data constraints.
  vector<Eigen::Triplet<double> > triplets;
  triplets.reserve(4 * width * height + valid_count);
  int index = 0;
  int constraint_index = 0;
  for (int y = 0; y < height; ++y) {
    for (int x = 0; x < width; ++x, ++index) {
      if (depths->at(index) == invalid)
        continue;
      // A.insert(index, index) = kDataCost;
      triplets.push_back(Eigen::Triplet<double>(constraint_index, index, kDataCost));
      b[constraint_index] = kDataCost * depths->at(index);
      ++constraint_index;
    }
  }
  // Regularization.
  index = 0;
  for (int y = 0; y < height; ++y) {
    for (int x = 0; x < width; ++x, ++index) {
      vector<int> neighbor_indexes;
      if (x != width - 1)
        neighbor_indexes.push_back(index + 1);
      if (x != 0)
        neighbor_indexes.push_back(index - 1);
      if (y != height - 1)
        neighbor_indexes.push_back(index + width);
      if (y != 0)
        neighbor_indexes.push_back(index - width);

      // A.insert(valid_count + index, index) = (int)neighbor_indexes.size();
      triplets.push_back(Eigen::Triplet<double>(valid_count + index, index, (int)neighbor_indexes.size()));
      for (const auto neighbor_index : neighbor_indexes) {
        // A.insert(valid_count + index, neighbor_index) = -1.0;
        triplets.push_back(Eigen::Triplet<double>(valid_count + index, neighbor_index, -1));
      }
      b[valid_count + index] = 0.0;
    }
  }/*
  Eigen::SparseQR<Eigen::SparseMatrix<double>, Eigen::AMDOrdering<double> > solver;
  solver.analyzePattern(A);
  solver.factorize(A);
  Eigen::VectorXd x = solver.solve(b);
   */
  A.setFromTriplets(triplets.begin(), triplets.end());

  Eigen::SparseMatrix<double> ATA = A.transpose() * A;
  Eigen::VectorXd ATb = A.transpose() * b;

  /*
  Eigen::SimplicialCholesky<Eigen::SparseMatrix<double> > chol(ATA);
  Eigen::VectorXd x = chol.solve(ATb);
  */
  Eigen::BiCGSTAB<Eigen::SparseMatrix<double>, Eigen::IncompleteLUT<double> > solver(ATA);
  solver.preconditioner().setDroptol(0.1);
  //solver.setMaxIterations(300);
  Eigen::VectorXd x = solver.solve(ATb);

  for (int i = 0; i < depths->size(); ++i)
    depths->at(i) = x[i];
}

}  // namespace structured_indoor_modeling
  
