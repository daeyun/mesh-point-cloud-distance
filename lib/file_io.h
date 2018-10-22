//
// Created by daeyun on 12/18/17.
//

#pragma once

#include <array>
#include <functional>
#include <string>
#include <vector>

#include "lib/common.h"

namespace scene3d {
bool ReadTriangles(const std::string &filename,
                   const std::function<void(const std::array<std::array<float, 3>, 3> &)> &triangle_handler);

std::vector<std::array<std::array<float, 3>, 3>> ReadTriangles(const std::string &filename);

bool ReadFacesAndVertices(const std::string &filename,
                          std::vector<std::array<unsigned int, 3>> *faces,
                          std::vector<std::array<float, 3>> *vertices,
                          std::vector<int> *node_id_for_each_face,
                          std::vector<std::string> *node_name_for_each_face);

void WritePclTensor(const std::string &filename, const vector<Vec3> &pcl);

bool Exists(const std::string &filename);

void WritePly(const string &filename, const vector<array<unsigned int, 3>> &faces, const vector<array<float, 3>> &vertices, bool is_binary);

vector<string> DirectoriesInDirectory(const string &dir);

void ReadLines(const string &filename, vector<string> *lines);

bool PrepareDir(const string &dirname);

bool PrepareDirForFile(const string &filename);

string JoinPath(const string &a, const string &b);

void RemoveDirIfExists(const string &path);
void RemoveFileIfExists(const string &path);
}

