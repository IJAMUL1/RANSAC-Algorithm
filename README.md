# RANSAC: Point Cloud Plane Fitting

This project demonstrates fitting a plane to a point cloud using a RANSAC-based adaptive approach.

## Introduction

This Python project utilizes the Open3D library to read point cloud data and fit a plane to it using an adaptive RANSAC algorithm. The fitted plane is visualized alongside the original point cloud with colored inliers.

## RANSAC Algorithm

The RANSAC (Random Sample Consensus) algorithm is a robust method for estimating parameters of a mathematical model from a set of observed data that may contain outliers. In the context of plane fitting to a point cloud, RANSAC iteratively samples subsets of points to estimate candidate planes and identifies the best-fitting plane based on the number of inliers.

# Core RANSAC Steps

## Sample Selection
- The `fit_plane_adaptive` function iteratively selects three random points from the point cloud to define a potential plane.

## Model Fitting
- Using the sampled points, the `plane_equation_from_points` function calculates the plane equation with coefficients (a, b, c, d).

## Inlier Determination
- The `is_inlier` function checks how many points in the point cloud lie within a `distance_threshold` of the calculated plane, considering them as inliers.

## Consensus Evaluation
- If the number of inliers for the current plane exceeds any previously found plane, it is stored as the `best_plane`.

## Iteration Update
- The algorithm dynamically adjusts the number of iterations needed (`N`) based on the proportion of inliers found in each iteration, improving efficiency.

## Termination
- The loop continues until the iteration count exceeds the calculated `N`.

# Additional Notes

- **Adaptive Nature**: This RANSAC implementation adapts the number of iterations dynamically based on inlier support, enhancing efficiency.
  
- **Distance Threshold**: The `distance_threshold` parameter determines the strictness of inlier determination.

- **Return Value**: The `fit_plane_adaptive` function returns the `best_plane` with the highest inlier support found during the RANSAC iterations.


## Installation

1. Ensure you have Python installed on your system.
2. Install the required dependencies using pip:
    ```
    pip install open3d matplotlib numpy
    ```

## Usage

1. Clone this repository to your local machine.
2. Run the `plane_fitting.py` script using Python:
    ```
    python ransac_algorithm.py
    ```

## Functionality

- **plane_equation_from_points**: Computes the plane equation from three given points.
- **distance**: Computes the distance from a point to a plane defined by its equation.
- **fit_plane_adaptive**: Fits a plane to a point cloud using an adaptive RANSAC approach.
- **is_inlier**: Determines if a point is an inlier based on its distance to the plane.

## Parameters

- **sample_size**: Number of points sampled to fit a plane in each RANSAC iteration.
- **p**: Confidence level for RANSAC termination criterion.

## Example
*Initial point cloud*
![initial point cloud](https://github.com/IJAMUL1/RANSAC-Point-Cloud-Plane-Fitting/assets/60096099/51413d48-f8c2-49a1-bb31-5afd42601488)

*point cloud after fitting*
![point cloud best plane](https://github.com/IJAMUL1/RANSAC-Point-Cloud-Plane-Fitting/assets/60096099/d0935b4a-a7fb-4a56-8682-3c81506222e3)

## Acknowledgments

- Open3D development team
- Inspired by [RANSAC algorithm](https://en.wikipedia.org/wiki/Random_sample_consensus)
