#include <iostream>
#include <fstream>
#include <algorithm>
#include <set>

#include "cxxopts.hpp"
#include "spdlog/spdlog.h"
#include "spdlog/sinks/stdout_color_sinks.h"

#include "lib/common.h"
#include "lib/file_io.h"
#include "lib/benchmark.h"
#include "lib/meshdist.h"
#include "lib/string_utils.h"

using namespace scene3d;
using meshdist_cgal::Triangle;

int main(int argc, char **argv) {
  cxxopts::Options options("mesh_pcl", "IID point sampling on mesh surface.");

  options.add_options()
      ("mesh", "Source mesh filename.", cxxopts::value<string>())
      ("out", "Output filename.", cxxopts::value<string>())
      ("density", "Number of points to sample per unit area.", cxxopts::value<float>()->default_value("1000"))
      ("binary", "Save non-plaintext ply file.", cxxopts::value<bool>()->default_value("false")->implicit_value("true"))
      ("help", "Display help.", cxxopts::value<bool>()->default_value("false")->implicit_value("true"));
  auto flags = options.parse(argc, argv);

  if (flags["help"].as<bool>()) {
    std::cout << options.help() << std::endl;
    exit(0);
  }

  // Initialize logging.
  spdlog::stdout_color_mt("console");

  vector<string> required_flags = {"mesh", "out"};
  for (const string &name: required_flags) {
    if (!flags.count(name)) {
      LOGGER->error("No argument specified for required option --{}. See --help.", name);
      throw std::runtime_error("");
    }
  }

  const string mesh_filename = flags["mesh"].as<string>();
  const string out_pcl_filename = flags["out"].as<string>();
  const float density = flags["density"].as<float>();
  const bool is_binary = flags["binary"].as<bool>();

  if (!Exists(mesh_filename)) {
    LOGGER->error("File does not exist: {}", mesh_filename);
    throw std::runtime_error("");
  }

  if (!EndsWith(out_pcl_filename, ".ply")) {
    LOGGER->error("--out filename must end with .ply", mesh_filename);
    throw std::runtime_error("");
  }

  auto io_start_time_ms = MicroSecondsSinceEpoch();

  vector<Triangle> triangles;
  ReadTriangles(mesh_filename,
                [&](const array<array<float, 3>, 3> triangle) {
                  triangles.emplace_back(
                      Vec3{triangle[0][0], triangle[0][1], triangle[0][2]},
                      Vec3{triangle[1][0], triangle[1][1], triangle[1][2]},
                      Vec3{triangle[2][0], triangle[2][1], triangle[2][2]}
                  );
                });

  LOGGER->info("Mesh reading took {} ms", static_cast<int>((MicroSecondsSinceEpoch() - io_start_time_ms) / 1e3));

  auto sampling_start_time_ms = MicroSecondsSinceEpoch();

  Points3d points;
  SamplePointsOnTriangles(triangles, density, &points);

  LOGGER->info("Sampled {} points in {} ms", points.cols(), static_cast<int>((MicroSecondsSinceEpoch() - sampling_start_time_ms) / 1e3));

  vector<array<unsigned int, 3>> faces;
  vector<array<float, 3>> vertices;

  for (int j = 0; j < points.cols(); ++j) {
    vertices.push_back(array<float, 3>{
        static_cast<float>(points.col(j)[0]),
        static_cast<float>(points.col(j)[1]),
        static_cast<float>(points.col(j)[2])
    });
  }

  PrepareDirForFile(out_pcl_filename);

  WritePly(out_pcl_filename, faces, vertices, is_binary);

}
