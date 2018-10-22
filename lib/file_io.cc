//
// Created by daeyun on 12/18/17.
//

#include "file_io.h"

#include <fstream>
#include <chrono>
#include <type_traits>
#include <algorithm>
#include <numeric>
#include <set>
#include <iomanip>
#include <sys/stat.h>

#include "spdlog/spdlog.h"

#include "assimp/Importer.hpp"
#include "assimp/postprocess.h"
#include "assimp/scene.h"
#include "assimp/DefaultLogger.hpp"
#include "assimp/LogStream.hpp"
#include "boost/filesystem.hpp"

#define TINYPLY_IMPLEMENTATION
#include "tinyply.h"

#include "lib/common.h"
#include "lib/string_utils.h"

namespace scene3d {
namespace fs = boost::filesystem;

bool ReadTriangles(const std::string &filename,
                   const std::function<void(const std::array<std::array<float, 3>, 3> &)> &triangle_handler) {
  LOGGER->info("Importing {}", filename);

  if (!boost::filesystem::exists(filename)) {
    LOGGER->error("{} does not exist", filename);
    throw std::runtime_error("file not found");
  }

  Assimp::Importer importer;

  // List of post-processing flags can be found here:
  // http://sir-kimmi.de/assimp/lib_html/postprocess_8h.html#a64795260b95f5a4b3f3dc1be4f52e410
  const aiScene *scene = importer.ReadFile(filename, aiProcess_Triangulate | aiProcess_JoinIdenticalVertices | aiProcess_SortByPType);

  if (!scene) {
    LOGGER->error("ERROR in {}: {}", filename, importer.GetErrorString());
    return false;
  }

  // TODO: there seems to be a problem in reading binary ply files. this is a hack to detect parsing error.
  const float kMaxVertexValue = 1e7;

  int triangle_count = 0;
  for (int i = 0; i < scene->mNumMeshes; ++i) {
    const aiMesh *mesh = scene->mMeshes[i];
    for (int j = 0; j < mesh->mNumFaces; ++j) {
      auto face = mesh->mFaces[j];
      Expects(face.mNumIndices == 3);

      for (int k = 0; k < 3; ++k) {
        if (face.mIndices[k] >= mesh->mNumVertices) {
          LOGGER->warn("Invalid vertex index found. Skipping.");
          continue;
        }
      }
      auto a = mesh->mVertices[face.mIndices[0]];
      auto b = mesh->mVertices[face.mIndices[1]];
      auto c = mesh->mVertices[face.mIndices[2]];
      if (std::abs(a.x) > kMaxVertexValue || std::abs(a.y) > kMaxVertexValue || std::abs(a.z) > kMaxVertexValue) {
        LOGGER->error("vertex value above threshold: {}, {}, {}", a.x, a.y, a.z);
        throw std::runtime_error("");
      }
      triangle_handler({std::array<float, 3>{a.x, a.y, a.z},
                        std::array<float, 3>{b.x, b.y, b.z},
                        std::array<float, 3>{c.x, c.y, c.z}});
      ++triangle_count;
    }
  }

  if (triangle_count <= 0) {
    LOGGER->error("No triangles found in mesh file.");
  }

  return true;
}

bool ReadFacesAndVertices(const std::string &filename,
                          std::vector<std::array<unsigned int, 3>> *faces,
                          std::vector<std::array<float, 3>> *vertices,
                          std::vector<int> *node_id_for_each_face,
                          std::vector<std::string> *node_name_for_each_face) {
  LOGGER->info("Importing {}", filename);
  if (!boost::filesystem::exists(filename)) {
    LOGGER->error("{} does not exist", filename);
    throw std::runtime_error("file not found");
  }

  Assimp::Importer importer;

  // List of post-processing flags can be found here:
  // http://sir-kimmi.de/assimp/lib_html/postprocess_8h.html#a64795260b95f5a4b3f3dc1be4f52e410
  const aiScene *scene = importer.ReadFile(filename, aiProcess_Triangulate | aiProcess_JoinIdenticalVertices | aiProcess_SortByPType);

  if (!scene) {
    LOGGER->error("ERROR in {}: {}", filename, importer.GetErrorString());
    return false;
  }

  // TODO: there seems to be a problem reading binary ply files. this is a hack to detect parsing error.
  const float kMaxVertexValue = 1e7;

  int triangle_count = 0;
  int face_offset = 0;

  // Recursively collect nodes in DFS order.
  std::vector<aiNode *> nodes;
  std::function<void(aiNode *)> recursive_node_collector;
  recursive_node_collector = [&](aiNode *root) {
    nodes.push_back(root);
    for (int j = 0; j < root->mNumChildren; ++j) {
      recursive_node_collector(root->mChildren[j]);
    }
  };
  recursive_node_collector(scene->mRootNode);

  for (int node_index = 0; node_index < nodes.size(); ++node_index) {
    aiNode *node = nodes[node_index];
    for (int i = 0; i < node->mNumMeshes; ++i) {
      const int mesh_index = node->mMeshes[i];
      const aiMesh *mesh = scene->mMeshes[mesh_index];
      std::string meshname(mesh->mName.data);

      for (int j = 0; j < mesh->mNumVertices; ++j) {
        auto vertex = mesh->mVertices[j];
        if (std::abs(vertex.x) > kMaxVertexValue || std::abs(vertex.y) > kMaxVertexValue
            || std::abs(vertex.z) > kMaxVertexValue) {
          LOGGER->error("vertex value above threshold: {}, {}, {}", vertex.x, vertex.y, vertex.z);
          throw std::runtime_error("");
        }
        vertices->push_back({vertex.x, vertex.y, vertex.z});
      }
      for (int j = 0; j < mesh->mNumFaces; ++j) {
        auto face = mesh->mFaces[j];
        Expects(face.mNumIndices == 3);
        for (int k = 0; k < 3; ++k) {
          if (face.mIndices[k] >= mesh->mNumVertices) {
            LOGGER->warn("Invalid vertex index found. Skipping.");
            continue;
          }
        }

        faces->push_back({static_cast<unsigned int>(face_offset + face.mIndices[0]),
                          static_cast<unsigned int>(face_offset + face.mIndices[1]),
                          static_cast<unsigned int>(face_offset + face.mIndices[2])});

        node_id_for_each_face->push_back(node_index);
        node_name_for_each_face->push_back(std::string(node->mName.data));

        ++triangle_count;
      }
      face_offset += mesh->mNumVertices;
      Ensures(face_offset == vertices->size());
    }
  }

  for (int i = 0; i < scene->mNumMeshes; ++i) {
  }

  if (triangle_count <= 0) {
    LOGGER->error("No triangles found in mesh file.");
  }

  return true;
}

vector<array<array<float, 3>, 3>>
ReadTriangles(const std::string &filename) {
  vector<array<array<float, 3>, 3 >> triangles;
  ReadTriangles(filename,
                [&](const array<array<float, 3>, 3> triangle) {
                  triangles.push_back(triangle);
                });
  return triangles;
}

bool Exists(const std::string &filename) {
  if (filename.empty()) {
    return false;
  }

  struct stat buffer;
  return (stat(filename.c_str(), &buffer) == 0);
}

string SystemTempDir() {
  return fs::temp_directory_path().string();
}

bool PrepareDir(const string &dirname) {
  auto path = fs::absolute(dirname);
  if (!fs::is_directory(path)) {
    Expects(!fs::is_regular_file(path));
    bool ok = fs::create_directories(path);
    if (ok) {
      LOGGER->debug("mkdir -p {}", path.string());
      return true;
    }
  }
  return false;
}

bool PrepareDirForFile(const string &filename) {
  auto parent_path = fs::absolute(filename).parent_path();
  return PrepareDir(parent_path.string());
}

vector<string> DirectoriesInDirectory(const string &dir) {
  fs::path path(dir);
  vector<string> paths;
  if (fs::exists(path) && fs::is_directory(path)) {
    fs::directory_iterator end_iter;
    for (fs::directory_iterator dir_iter(path); dir_iter != end_iter; ++dir_iter) {
      if (fs::is_directory(dir_iter->status())) {
        paths.push_back(dir_iter->path().string());
      }
    }
  }
  std::sort(std::begin(paths), std::end(paths));

  return paths;
}

void ReadLines(const string &filename, vector<string> *lines) {
  Ensures(Exists(filename));
  std::ifstream f(filename);

  std::string line;
  while (std::getline(f, line)) {  // does not include linebreak.
    lines->push_back(line);
  }
}

string JoinPath(const string &a, const string &b) {
  return (fs::path(a) / fs::path(b)).string();
}

void RemoveDirIfExists(const string &path) {
  if (fs::is_directory(path)) {
    LOGGER->info("rm -rf {}", fs::absolute(path).string());
    fs::remove_all(path);
  }
}

void RemoveFileIfExists(const string &path) {
  if (fs::is_regular_file(path)) {
    LOGGER->info("rm {}", fs::absolute(path).string());
    fs::remove(path);
  }
}

void WritePly(const string &filename, const vector<array<unsigned int, 3>> &faces, const vector<array<float, 3>> &vertices, bool is_binary) {
  Ensures(EndsWith(filename, ".ply"));
  Ensures(sizeof(unsigned int) == 4);

  std::filebuf buf;
  if (is_binary) {
    buf.open(filename, std::ios::out | std::ios::binary);
  } else {
    buf.open(filename, std::ios::out);
  }
  std::ostream outstream(&buf);
  if (outstream.fail()) {
    throw std::runtime_error("failed to open " + filename);
  }

  using namespace tinyply;

  PlyFile ply_file;

  ply_file.add_properties_to_element("vertex", {"x", "y", "z"}, Type::FLOAT32, vertices.size(), const_cast<uint8_t *>(reinterpret_cast<const uint8_t *>(&vertices[0][0])), Type::INVALID, 0);
  ply_file.add_properties_to_element("face", {"vertex_indices"}, Type::UINT32, faces.size(), const_cast<uint8_t *>(reinterpret_cast<const uint8_t *>(&faces[0][0])), Type::UINT8, 3);

  LOGGER->info("Writing {}", filename);

  // Write a binary file
  ply_file.write(outstream, is_binary);
}
}
