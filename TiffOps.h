//
// Created by Mark on 2/2/2020.
//

#ifndef BREADCRUMBS_TIFFOPS_H
#define BREADCRUMBS_TIFFOPS_H

#include <vector>
#include <string>

std::vector<std::vector<float>> readTIFF(const std::string & filename);

void writeMatrixToTIFF(std::vector<std::vector<float>> matrix, const std::string & filename);

void writePathToTIFF(std::vector<std::vector<int>> matrix, std::string filename);

#endif //BREADCRUMBS_TIFFOPS_H
