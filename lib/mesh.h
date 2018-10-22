#pragma once

#include "common.h"

namespace scene3d {

class TriMesh {
 public:
  vector<array<unsigned int, 3>> faces;
  vector<array<float, 3>> vertices;

  void AddTriangle(const array<float, 3> &v0, const array<float, 3> &v1, const array<float, 3> &v2) {
    auto offset = static_cast<unsigned int>(vertices.size());
    vertices.push_back(v0);
    vertices.push_back(v1);
    vertices.push_back(v2);
    faces.push_back(array<unsigned int, 3>{offset, offset + 1, offset + 2});
  }

  void AddTriangle(const Vec3 &v0, const Vec3 &v1, const Vec3 &v2) {
    auto offset = static_cast<unsigned int>(vertices.size());
    vertices.push_back(array<float, 3>{(float) v0[0], (float) v0[1], (float) v0[2]});
    vertices.push_back(array<float, 3>{(float) v1[0], (float) v1[1], (float) v1[2]});
    vertices.push_back(array<float, 3>{(float) v2[0], (float) v2[1], (float) v2[2]});
    faces.push_back(array<unsigned int, 3>{offset, offset + 1, offset + 2});
  }

  void RemoveFacesContainingVertices(const vector<bool> &is_vertex_invalid, TriMesh *new_mesh) const {
    Expects(is_vertex_invalid.size() == vertices.size());

    for (int i = 0; i < faces.size(); ++i) {
      bool remove_face = false;
      const auto &face = faces[i];
      for (int j = 0; j < 3; ++j) {
        if (is_vertex_invalid[face[j]]) {
          remove_face = true;
          break;
        }
      }

      if (remove_face) {
        continue;
      }

      new_mesh->AddTriangle(vertices[face[0]], vertices[face[1]], vertices[face[2]]);
    }
  }

  void AddMesh(const TriMesh &mesh) {
    const auto offset = vertices.size();
    vertices.insert(vertices.end(), mesh.vertices.begin(), mesh.vertices.end());
    for (int i = 0; i < mesh.faces.size(); ++i) {
      faces.push_back(array<unsigned int, 3>{
          static_cast<unsigned int>(mesh.faces[i][0] + offset),
          static_cast<unsigned int>(mesh.faces[i][1] + offset),
          static_cast<unsigned int>(mesh.faces[i][2] + offset),
      });
    }
  }

};

}

