//
// Created by daeyun on 5/10/17.
//

#include "meshdist.h"

#include <Eigen/Dense>
#include <type_traits>
#include <chrono>

#include "lib/common.h"
#include "lib/random_utils.h"
#include "lib/benchmark.h"

namespace scene3d {
namespace meshdist_cgal {

typedef CGAL::Simple_cartesian<float> K;
typedef std::list<K::Triangle_3>::iterator Iterator;
typedef CGAL::AABB_triangle_primitive<K, Iterator> Primitive;
typedef CGAL::AABB_traits<K, Primitive> AABB_triangle_traits;
typedef CGAL::AABB_tree<AABB_triangle_traits> Tree;

void Triangle::ApplyRt(const Mat34 &M) {
  auto R = M.leftCols<3>();
  auto t = M.col(3);
  a = (R * a + t).eval();
  b = (R * b + t).eval();
  c = (R * c + t).eval();
  OnTransformation();
}

void Triangle::ApplyR(const Mat33 &M) {
  a = (M * a).eval();
  b = (M * b).eval();
  c = (M * c).eval();
  OnTransformation();
}

void Triangle::Translate(const Vec3 &dxyz) {
  a += dxyz;
  b += dxyz;
  c += dxyz;
  OnTransformation();
}

void Triangle::Print() const {
  std::cout << a.transpose() << ",      "
            << b.transpose() << ",      "
            << c.transpose() << std::endl;
}

Vec3 Triangle::SamplePoint() const {
  Vec2 v12{scene3d::Random::Rand(), scene3d::Random::Rand()};
  if (v12.sum() > 1) {
    v12 = 1 - v12.array();
  }
  return a + (v12(0) * ab_) + (v12(1) * ac_);
}

double Triangle::Area() const {
  return ab_.cross(ac_).norm() * 0.5f;
}

void Triangle::OnTransformation() {
  assert(a.allFinite());
  assert(b.allFinite());
  assert(c.allFinite());

  ab_ = b - a;
  ac_ = c - a;
}

void SamplePointsOnTriangles(const std::vector<Triangle> &triangles, float density, Points3d *points, std::vector<size_t> *triangle_indices) {
  std::vector<double> areas;
  areas.reserve(triangles.size());
  for (auto &&triangle : triangles) {
    areas.push_back(triangle.Area());
  }

  double surface_area = std::accumulate(areas.begin(), areas.end(), static_cast<double>(0));
  Expects(surface_area > 0.0);

  int num_samples = static_cast<int>(surface_area * density);
  if (num_samples <= 0) {
    // Surface area is non-zero
    // the density parameter is not big enough. This should be a rare case. For now, print warning and just make sure to sample at least one point.
    num_samples = 1;
    LOGGER->warn("density is too small. Surface area is {}", surface_area);
  }
  Expects(num_samples > 0);

  std::discrete_distribution<> distribution(std::begin(areas), std::end(areas));

  Eigen::Matrix<int, Dynamic, 1> counts;

  Eigen::Matrix<int, Dynamic, Dynamic> local_counts;

#pragma omp parallel if (USE_OMP && num_samples > 1e6)
  {
    const int num_threads = omp_get_num_threads();
    const int i_thread = omp_get_thread_num();

#pragma omp single
    {
      local_counts.resize(triangles.size(), num_threads);
      local_counts.fill(0);
    }

#pragma omp for
    for (int i = 0; i < num_samples; ++i) {
      ++local_counts(distribution(scene3d::Random::Engine()), i_thread);
    }
  }

  counts = local_counts.rowwise().sum();

  points->resize(3, num_samples);
  if (triangle_indices) {
    triangle_indices->resize(static_cast<size_t>(num_samples));
  }
  int k = 0;
  for (int i = 0; i < triangles.size(); ++i) {
    for (int j = 0; j < counts(i); ++j) {
      points->col(k) = triangles[i].SamplePoint();
      if (triangle_indices) {
        triangle_indices->push_back((size_t) i);
      }
      ++k;
    }
  }
  Expects(k == num_samples);

  Eigen::PermutationMatrix<Dynamic, Dynamic> perm(points->cols());
  perm.setIdentity();
  std::shuffle(perm.indices().data(), perm.indices().data() + perm.indices().size(), scene3d::Random::Engine());
  *points *= perm; // permute columns
}

// TODO(daeyun): divide by mean distance

float MeshToMeshDistanceOneDirection(const std::vector<Triangle> &from,
                                     const std::vector<Triangle> &to,
                                     float sampling_density,
                                     std::vector<float> *distances) {

  Points3d points;

  SamplePointsOnTriangles(from, sampling_density, &points);

  std::list<K::Triangle_3> triangle_list;
  for (const auto &triangle : to) {
    triangle_list.emplace_back(K::Point_3{triangle.a[0], triangle.a[1], triangle.a[2]},
                               K::Point_3{triangle.b[0], triangle.b[1], triangle.b[2]},
                               K::Point_3{triangle.c[0], triangle.c[1], triangle.c[2]});
  }
  std::vector<K::Point_3> point_list;
  for (int i = 0; i < points.cols(); ++i) {
    point_list.emplace_back(points(0, i), points(1, i), points(2, i));
  }

  int num_triangles = static_cast<int>(to.size());
  int num_points = static_cast<int>(points.cols());

  LOGGER->debug("Computing minimum distances from {} points to {} triangles.", num_points, num_triangles);

  auto start = scene3d::TimeSinceEpoch<std::milli>();
  Tree tree(triangle_list.begin(), triangle_list.end());
  tree.build();
  tree.accelerate_distance_queries();
  LOGGER->debug("Time elapsed for building tree (CGAL): {}", scene3d::TimeSinceEpoch<std::milli>() - start);

  float distance_sum = 0;

#pragma omp parallel for if(USE_OMP) reduction(+:distance_sum) schedule(static)
  for (int i = 0; i < point_list.size(); ++i) {
    float dist = tree.squared_distance(point_list[i]);
    distance_sum += dist;

// TODO: needs refactoring
    if (distances) {
#pragma omp critical
      distances->push_back(dist);
    }
  }

  LOGGER->debug("distance: {}", distance_sum);
  float rms = static_cast<float>(std::sqrt(distance_sum / static_cast<double>(point_list.size())));
  LOGGER->debug("RMS: {}", rms);
  auto elapsed = scene3d::TimeSinceEpoch<std::milli>() - start;
  LOGGER->debug("Time elapsed (CGAL): {} ms", elapsed);

  return rms;
}

float PointsToMeshDistanceOneDirection(const std::vector<std::array<float, 3>> &from,
                                       const std::vector<Triangle> &to) {
  std::list<K::Triangle_3> triangle_list;
  for (const auto &triangle : to) {
    triangle_list.emplace_back(K::Point_3{triangle.a[0], triangle.a[1], triangle.a[2]},
                               K::Point_3{triangle.b[0], triangle.b[1], triangle.b[2]},
                               K::Point_3{triangle.c[0], triangle.c[1], triangle.c[2]});
  }
  std::vector<K::Point_3> point_list;
  for (int i = 0; i < from.size(); ++i) {
    point_list.emplace_back(from[i][0], from[i][1], from[i][2]);
  }

  int num_triangles = static_cast<int>(to.size());
  int num_points = static_cast<int>(from.size());

  LOGGER->debug("Computing minimum distances from {} points to {} triangles.", num_points, num_triangles);

  auto start = scene3d::TimeSinceEpoch<std::milli>();
  Tree tree(triangle_list.begin(), triangle_list.end());
  tree.build();
  tree.accelerate_distance_queries();
  LOGGER->debug("Time elapsed for building tree (CGAL): {}", scene3d::TimeSinceEpoch<std::milli>() - start);

  float distance_sum = 0;

#pragma omp parallel for if(USE_OMP) reduction(+:distance_sum) schedule(static)
  for (int i = 0; i < point_list.size(); ++i) {
    float dist = tree.squared_distance(point_list[i]);
    distance_sum += dist;
  }

  LOGGER->debug("distance: {}", distance_sum);
  float rms = static_cast<float>(std::sqrt(distance_sum / static_cast<double>(point_list.size())));

  LOGGER->debug("RMS: {}", rms);
  auto elapsed = scene3d::TimeSinceEpoch<std::milli>() - start;
  LOGGER->debug("Time elapsed (CGAL): {} ms", elapsed);

  return rms;
}

float MeshToPointsDistanceOneDirection(const std::vector<Triangle> &from,
                                       const std::vector<std::array<float, 3>> &target_points,
                                       float sampling_density) {
  std::list<K::Triangle_3> triangle_list;
  for (const auto &triangle : from) {
    triangle_list.emplace_back(K::Point_3{triangle.a[0], triangle.a[1], triangle.a[2]},
                               K::Point_3{triangle.b[0], triangle.b[1], triangle.b[2]},
                               K::Point_3{triangle.c[0], triangle.c[1], triangle.c[2]});
  }

  Points3d points_on_mesh;
  SamplePointsOnTriangles(from, sampling_density, &points_on_mesh);

  int num_source_points = static_cast<int>(points_on_mesh.cols());
  int num_target_points = static_cast<int>(target_points.size());

  LOGGER->debug("Computing minimum distances from {} points on triangles to {} points.", num_source_points, num_target_points);

  auto start = scene3d::TimeSinceEpoch<std::milli>();

  float distance_sum = 0;

#pragma omp parallel for if(USE_OMP) reduction(+:distance_sum) schedule(static)
  for (int i = 0; i < num_source_points; ++i) {
    const auto source_point = points_on_mesh.col(i);
    double min_dist = kInfinity;
    // TODO(daeyun): this is brute force
    for (int j = 0; j < num_target_points; ++j) {
      double dx = (source_point(0) - target_points[j][0]);
      double dy = (source_point(1) - target_points[j][1]);
      double dz = (source_point(2) - target_points[j][2]);
      double dist = std::sqrt(dx * dx + dy * dy + dz * dz);
      if (dist < min_dist) {
        min_dist = dist;
      }
    }
    distance_sum += min_dist;
  }

  LOGGER->debug("distance: {}", distance_sum);
  float rms = static_cast<float>(std::sqrt(distance_sum / static_cast<double>(num_source_points)));

  LOGGER->debug("RMS: {}", rms);
  auto elapsed = scene3d::TimeSinceEpoch<std::milli>() - start;
  LOGGER->debug("Time elapsed (CGAL): {} ms", elapsed);

  return rms;
}

float MeshToMeshDistance(const std::vector<Triangle> &a, const std::vector<Triangle> &b) {
  auto start = scene3d::TimeSinceEpoch<std::milli>();

  constexpr int kSamplingDensity = 300;

  float d1 = meshdist_cgal::MeshToMeshDistanceOneDirection(a, b, kSamplingDensity);
  float d2 = meshdist_cgal::MeshToMeshDistanceOneDirection(b, a, kSamplingDensity);

  auto elapsed = scene3d::TimeSinceEpoch<std::milli>() - start;
  LOGGER->debug("Time elapsed (MeshToMeshDistance): {} ms", elapsed);
  LOGGER->debug("{}, {}", d1, d2);
  return static_cast<float>((d1 + d2) * 0.5);
}

void TrianglesFromTriMesh(const TriMesh &mesh, std::vector<Triangle> *out) {
  for (const auto &face: mesh.faces) {
    const auto &v1 = mesh.vertices[face[0]];
    const auto &v2 = mesh.vertices[face[1]];
    const auto &v3 = mesh.vertices[face[2]];
    out->push_back(Triangle(Vec3{v1[0], v1[1], v1[2]}, Vec3{v2[0], v2[1], v2[2]}, Vec3{v3[0], v3[1], v3[2]}));
  }
}

}
}
