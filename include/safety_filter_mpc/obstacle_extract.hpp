#ifndef OBSTACLE_EXTRACT_HPP
#define OBSTACLE_EXTRACT_HPP

#include <Eigen/Dense>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <vector>
#include <queue>
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <utility>

namespace safety_filter_mpc
{

struct SurfaceBoundaryParams {
    int8_t  occupancy_threshold     = 1;    // >= threshold => belegt
    double  max_search_radius_m     = 6.0;  // Flood-Fill Radius
    bool    eight_connectivity      = false;
    int     num_best_obstacles      = 10;   // Anzahl zurückzugeben
    int     num_candidates_for_fps  = 80;   // Poollimit für nächstgelegene Boundary-Zellen
    bool    include_unknown_as_blocking = true;
    double  min_separation_m        = 0.2;  // Mindestabstand (m)
    double  relax_factor            = 0.5;  // Reduktionsfaktor bei zu wenigen Treffern

    // --- Precomputed values ---
    double relax_factor_sq          = 0.25; // relax_factor * relax_factor
    double min_separation_cells_sq  = 0.0;  // To be set once resolution is known
    double max_search_radius_cells_sq = 0.0; // Same
    int dir_count                   = 4;    // Adjusted if eight_connectivity=true

    void finalize(double resolution) {
        relax_factor_sq = relax_factor * relax_factor;
        double max_r_cells = max_search_radius_m / resolution;
        max_search_radius_cells_sq = max_r_cells * max_r_cells;
        double sep_cells = min_separation_m / resolution;
        min_separation_cells_sq = sep_cells * sep_cells;
        dir_count = eight_connectivity ? 8 : 4;
    }
};

struct BoundaryCandidate {
    int r, c;
    int dist2;
    bool operator<(const BoundaryCandidate& o) const {
        return dist2 < o.dist2; // Max-Heap (größtes dist2 oben)
    }
};

inline std::vector<Eigen::Vector2d> get_surface_boundary_obstacles(
    const nav_msgs::msg::OccupancyGrid::ConstSharedPtr& map,
    const Eigen::Vector2d& robot_position,
    const SurfaceBoundaryParams& p)
{
    std::vector<Eigen::Vector2d> out;
    if (!map || p.num_best_obstacles <= 0) return out;
    const auto& info = map->info;
    if (info.width == 0 || info.height == 0) return out;

    // --- Pose / Auflösung ---
    const double res = info.resolution;
    const auto& origin = info.origin;
    const double qx = origin.orientation.x, qy = origin.orientation.y, qz = origin.orientation.z, qw = origin.orientation.w;
    const double yaw = std::atan2(2.0*(qw*qz + qx*qy), 1.0 - 2.0*(qy*qy + qz*qz));
    const double cy = std::cos(yaw), sy = std::sin(yaw);

    // Welt -> lokales Grid (gedrehte Map)
    const double rx = robot_position.x() - origin.position.x;
    const double ry = robot_position.y() - origin.position.y;
    const double gx_m =  cy * rx + sy * ry;
    const double gy_m = -sy * rx + cy * ry;
    const int rc = std::clamp((int)std::floor(gx_m / res), 0, (int)info.width  - 1);
    const int rr = std::clamp((int)std::floor(gy_m / res), 0, (int)info.height - 1);

    const int W = (int)info.width;
    const int H = (int)info.height;
    const auto *data_ptr = map->data.data(); // single pointer to data (avoid repeated map->data)
    const int start_idx = rr * W + rc;
    const int8_t occ_th = p.occupancy_threshold;
    const bool include_unknown = p.include_unknown_as_blocking;
    auto is_occ_fast = [&](int index)->bool {
        const int8_t v = data_ptr[index];
        return (v < 0) ? include_unknown : (v >= occ_th);
    };
    if (is_occ_fast(start_idx)) return out; // start cell occupied -> empty result

    // --- BFS (freier Raum) + Sammeln nächster Boundary-Zellen (beschränkt) ---
    std::priority_queue<BoundaryCandidate> heap; // hält jeweils die aktuell n nächsten
    std::vector<uint8_t> visited(H * W, 0);
    std::vector<uint8_t> boundary(H * W, 0);
    std::vector<std::pair<int,int>> q; q.reserve(4096); q.emplace_back(rr, rc);
    visited[start_idx] = 1;

    static const int dirs4[4][2] = {{1,0},{-1,0},{0,1},{0,-1}};
    static const int dirs8[8][2] = {{1,0},{-1,0},{0,1},{0,-1},{1,1},{1,-1},{-1,1},{-1,-1}};
    const int (*dirs)[2] = p.eight_connectivity ? dirs8 : dirs4;

    size_t head = 0;
    const int pool_limit = std::max(1, p.num_candidates_for_fps);
    while (head < q.size()) {
        const int r = q[head].first;
        const int c = q[head].second;
        ++head;

        const int dr0 = r - rr;
        const int dc0 = c - rc;
        const int dist2 = dr0 * dr0 + dc0 * dc0;
        if (dist2 > p.max_search_radius_cells_sq) continue;

        // Early exit: when pool full and current BFS shell is farther than worst candidate
        if ((int)heap.size() >= pool_limit && dist2 > heap.top().dist2) break;

        for (int k = 0; k < p.dir_count; ++k) {
            int nr = r + dirs[k][0];
            int nc = c + dirs[k][1];
            if ((unsigned)nr >= (unsigned)H || (unsigned)nc >= (unsigned)W) continue;

            const int nindex = nr * W + nc;
            if (is_occ_fast(nindex)) {
                if (!boundary[nindex]) {
                    boundary[nindex] = 1;
                    const int d2 = (nr - rr)*(nr - rr) + (nc - rc)*(nc - rc);
                    if ((int)heap.size() < pool_limit) {
                        heap.push({nr, nc, d2});
                    } else if (d2 < heap.top().dist2) {
                        heap.pop();
                        heap.push({nr, nc, d2});
                    }
                }
            } else if (!visited[nindex]) {
                visited[nindex] = 1;
                q.emplace_back(nr, nc);
            }
        }
    }

    if (heap.empty()) return out;

    // Heap -> sortierter Vektor (aufsteigend nach dist2)
    std::vector<BoundaryCandidate> cand;
    cand.reserve(std::min((size_t)pool_limit, heap.size()));
    while (!heap.empty()) { cand.push_back(heap.top()); heap.pop(); }
    std::sort(cand.begin(), cand.end(), [](auto& a, auto& b){ return a.dist2 < b.dist2; });

    // --- Auswahl mit Mindestabstand + Relaxation ---
    std::vector<BoundaryCandidate> selected;
    selected.reserve(std::min((int)cand.size(), p.num_best_obstacles));

    const int target = std::min(p.num_best_obstacles, (int)cand.size());
    double current_sep2 = p.min_separation_cells_sq;
    const int max_attempts = 4;
    int attempt = 0;

    while ((int)selected.size() < target && attempt < max_attempts) {
        for (const auto& cc : cand) {
            if ((int)selected.size() >= target) break;
            bool ok = true;
            for (const auto& s : selected) {
                int drr = cc.r - s.r;
                int dcc = cc.c - s.c;
                if (drr*drr + dcc*dcc < current_sep2) { ok = false; break; }
            }
            if (ok) selected.push_back(cc);
        }
        if ((int)selected.size() < target) {
            current_sep2 *= p.relax_factor_sq;
            ++attempt;
        }
    }
    if ((int)selected.size() < target) {
        for (const auto& cc : cand) {
            if ((int)selected.size() >= target) break;
            bool dup = false;
            for (auto& s : selected) if (s.r == cc.r && s.c == cc.c) { dup = true; break; }
            if (!dup) selected.push_back(cc);
        }
    }

    // --- Grid -> Welt ---
    out.reserve(selected.size());
    for (auto& s : selected) {
        double lx = (s.c + 0.5) * res;
        double ly = (s.r + 0.5) * res;
        double wx = origin.position.x + cy * lx - sy * ly;
        double wy = origin.position.y + sy * lx + cy * ly;
        out.emplace_back(wx, wy);
    }
    return out;
}

} // end namespace

#endif //OBSTACLE_EXTRACT_HPP