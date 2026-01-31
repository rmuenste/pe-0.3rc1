#ifndef _PE_INTERFACE_GEOMETRY_UTILS_H_
#define _PE_INTERFACE_GEOMETRY_UTILS_H_

#include <pe/core/Types.h>
#include <vector>
#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <cmath>
#include <limits>

namespace pe {

//*************************************************************************************************
/**
 * @brief Read 3D vectors from a text file
 *
 * Reads a text file containing 3D vectors (one per line with x, y, z components).
 * Each line should contain three floating-point values separated by whitespace.
 *
 * @param fileName Path to the input file
 * @return Vector of Vec3 objects read from the file (empty on error)
 */
inline std::vector<Vec3> readVectorsFromFile(const std::string& fileName) {
    std::vector<Vec3> vectors;
    std::ifstream file(fileName);

    // Check if the file was successfully opened
    if (!file.is_open()) {
        std::cerr << "Error: Unable to open file " << fileName << std::endl;
        return vectors; // Return an empty vector in case of error
    }

    std::string line;

    // Read the file line by line
    while (std::getline(file, line)) {
        std::stringstream ss(line);  // Create a string stream from the line
        float x, y, z;

        // Parse the line for three float values
        if (ss >> x >> y >> z) {
            // Create a Vec3 object and add it to the vector
            vectors.emplace_back(x, y, z);
        }
    }

    file.close();  // Close the file
    return vectors;
}
//*************************************************************************************************


//*************************************************************************************************
/**
 * @brief Generate sphere positions along a centerline defined by edge vertices
 *
 * This function generates positions for spheres arranged in concentric rings along a
 * piecewise linear curve (centerline). The centerline is defined by a sequence of edge
 * vertices. Spheres are placed at regular intervals along the curve.
 *
 * @param vecOfEdges Vector of vertices defining the piecewise linear centerline
 * @param sphereRadius Radius of each sphere to be placed
 * @param dt Distance from sphere surface to the circle center (default: 1.0 * sphereRadius)
 * @param num_rings Number of concentric rings around each centerline point (default: 4)
 * @param num_steps Number of divisions along the curve (default: 20)
 * @return Vector of Vec3 positions where spheres should be placed
 */
inline std::vector<Vec3> generatePointsAlongCenterline(
    std::vector<Vec3> &vecOfEdges,
    real sphereRadius,
    real dt = -1.0,  // Default: will be set to 1.0 * sphereRadius if < 0
    int num_rings = 4,
    int num_steps = 35)
{
    // Set default dt if not specified
    if (dt < 0.0) {
        dt = 1.0 * sphereRadius;
    }

    // Step 1: Measure the total length of the curve
    double curve_length = 0.0;
    std::vector<double> edge_lengths;
    std::vector<Vec3> wayPoints;
    std::vector<Vec3> sphere_positions;

    size_t num_edges = vecOfEdges.size() - 1;

    for (size_t i = 0; i < num_edges; ++i) {
        Vec3 v1 = vecOfEdges[i];
        Vec3 v2 = vecOfEdges[i + 1];
        double edge_length = (v2 - v1).length();
        edge_lengths.push_back(edge_length);
        curve_length += edge_length;
    }

    // Step 2: Set ds (step size)
    double ds = curve_length / real(num_steps);

    // Step 3: Compute cumulative lengths to help find the edge containing each step
    std::vector<double> cumulative_lengths;
    cumulative_lengths.push_back(0.0);  // Starting point

    for (size_t i = 0; i < edge_lengths.size(); ++i) {
        cumulative_lengths.push_back(cumulative_lengths.back() + edge_lengths[i]);
    }

    // Step 4: Traverse the curve in increments of ds
    for (double s = ds + 0.2 * ds; s <= curve_length - ds; s += ds) {
        // Find the edge that contains the current distance s
        size_t edge_index = 0;
        while (edge_index < num_edges && s > cumulative_lengths[edge_index + 1]) {
            ++edge_index;
        }

        if (edge_index >= num_edges) {
            break;  // Reached the end of the curve
        }

        // Compute the parameter t along the current edge
        double edge_start = cumulative_lengths[edge_index];
        double edge_end = cumulative_lengths[edge_index + 1];
        double t = (s - edge_start) / (edge_end - edge_start);

        // Calculate the point along the edge at distance s
        Vec3 v1 = vecOfEdges[edge_index];
        Vec3 v2 = vecOfEdges[edge_index + 1];
        Vec3 point_on_edge = v1 + (v2 - v1) * t;

        // Step 5: Take a user-defined action at the point
        wayPoints.push_back(point_on_edge);

        //-------------------- User Action Start --------------------
        // Generate a ring of spheres around the current edge at point_on_edge
        Vec3 edge_direction = v2 - v1;
        Vec3 someVector(1.0, 0., 0.);
        if ( std::abs( trans(edge_direction) * someVector ) > 0.999)
          someVector = Vec3(0.0, 1.0, 0.0);

        // Compute orthogonal vectors u and v in the plane perpendicular to edge_direction
        Vec3 u = (edge_direction % someVector).getNormalized();
        Vec3 v = (edge_direction % u).getNormalized();

        for (int j(0); j < num_rings; ++j) {

          // Compute circle radius
          real circle_radius = sphereRadius + dt + j * (2. * sphereRadius + dt);

          real circumference = 2. * M_PI * circle_radius;

          // Compute maximum number of spheres without overlap
          int max_spheres = int(circumference / (2. * sphereRadius)) - 1;

          if (max_spheres < 1)
             max_spheres = 1;

          // Compute exact angle step
          real theta_step = 2. * M_PI / max_spheres;

          // Place spheres around the circle
          for(int i(0); i < max_spheres; ++i) {
             real theta = i * theta_step;
             Vec3 sphere_offset = (std::cos(theta) * u + std::sin(theta) * v) * circle_radius;
             sphere_positions.push_back(Vec3(point_on_edge + sphere_offset));
          }

        }
    }

    real minDist = std::numeric_limits<real>::max();
    for (size_t i = 1; i < wayPoints.size(); ++i) {
        real dist = (wayPoints[i-1] - wayPoints[i]).length();
        if (minDist > dist) minDist = dist;
    }

    return sphere_positions;
}
//*************************************************************************************************

} // namespace pe

#endif  // _PE_INTERFACE_GEOMETRY_UTILS_H_
