//
// Created by Mark on 2/2/2020.
//

#ifndef BREADCRUMBS_TIFFOPS_H
#define BREADCRUMBS_TIFFOPS_H

#include <vector>
#include <string>

/*
 * Reads a TIFF into a 2D vector of floats.
 * The TIFF must be stored in tiles, and contain 32-bit floating point numbers.
 */
std::vector<std::vector<float>> readTIFF(const std::string & filename);

/*
 * Writes a 2D matrix of floats to a TIFF.
 * No spatial reference is written.
 * The units per pixel is also not recorded.
 */
void writeMatrixToTIFF(std::vector<std::vector<float>> matrix, const std::string & filename);

/*
 * Writes a 2D matrix of ints to a TIFF.
 * No spatial reference is written.
 * The units per pixel is also not recorded.
 */
void writePathToTIFF(std::vector<std::vector<int>> matrix, std::string filename);

#endif //BREADCRUMBS_TIFFOPS_H
