# RANSAC: Point Cloud Plane Fitting

This project demonstrates fitting a plane to a point cloud using a RANSAC-based adaptive approach.

## Introduction

This Python project utilizes the Open3D library to read point cloud data and fit a plane to it using an adaptive RANSAC algorithm. The fitted plane is visualized alongside the original point cloud with colored inliers.

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
    python plane_fitting.py
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

![Example](example.png)

## License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.

## Acknowledgments

- Open3D development team
- Inspired by [RANSAC algorithm](https://en.wikipedia.org/wiki/Random_sample_consensus)
