// Copyright by BeeX [2026]
#include <plan.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <map>
#include <numeric>
#include <random>

namespace final_v1 {

const char *outcomeName(Outcome o) {
    switch (o) {
    case Outcome::UNREACHABLE: return "unreachable";
    case Outcome::JOINT_LIMIT: return "joint_limit";
    case Outcome::NO_JAW_ROLL: return "no_jaw_roll";
    case Outcome::HULL: return "hull";
    case Outcome::OBSTACLE: return "obstacle";
    case Outcome::NOT_PLANNED: return "not_planned";
    case Outcome::NO_PATH: return "no_path";
    case Outcome::OK: return "ok";
    }
    return "unknown";
}

Outcome outcomeOf(Verdict v) {
    switch (v) {
    case Verdict::CLEAR: return Outcome::OK;
    case Verdict::JOINT_LIMIT: return Outcome::JOINT_LIMIT;
    case Verdict::HULL: return Outcome::HULL;
    case Verdict::OBSTACLE: return Outcome::OBSTACLE;
    }
    return Outcome::OBSTACLE;
}

Outcome graspGoals(const GraspPose &candidate, size_t index, const Joints &start, double along, Collision &collision,
                   std::vector<GraspGoal> &goals) {
    const Arm &arm      = collision.arm();
    Outcome    furthest = Outcome::UNREACHABLE;
    const auto reached  = [&](Outcome o) { furthest = std::max(furthest, o); };
    bool       found    = false;
    for (const bool elbow_up : {false, true}) {
        Joints   q;
        const Ik ik = arm.solve(candidate.point, along, elbow_up, start, q);
        if (ik == Ik::JOINT_LIMIT) {
            reached(Outcome::JOINT_LIMIT);
        }
        double rolls[2];
        if (ik != Ik::SOLVED) {
            continue;
        }
        if (!arm.rollsAcrossBar(q, candidate.bar, rolls)) {
            reached(Outcome::NO_JAW_ROLL);
            continue;
        }
        for (const double roll : rolls) {
            if (!arm.fitIntoLimits(WRIST, roll, start[WRIST], q[WRIST])) {
                reached(Outcome::NO_JAW_ROLL);
                continue;
            }
            const Verdict v = collision.check(q);
            if (v == Verdict::CLEAR) {
                goals.push_back({index, q, largestMove(start, q)});
                found = true;
            } else {
                reached(outcomeOf(v));
            }
        }
    }
    return found ? Outcome::NOT_PLANNED : furthest;
}

namespace {

std::mt19937 &rng() {
    static std::mt19937 r(1);
    return r;
}

double distance(const Joints &a, const Joints &b) {
    double d = 0.0;
    for (int j = 0; j < JOINT_COUNT; ++j) {
        d += (a[j] - b[j]) * (a[j] - b[j]);
    }
    return std::sqrt(d);
}

double length(const std::vector<Joints> &path) {
    double l = 0.0;
    for (size_t k = 1; k < path.size(); ++k) {
        l += distance(path[k - 1], path[k]);
    }
    return l;
}

struct Tree {
    std::vector<Joints> q;
    std::vector<int>    parent;
};

enum class Grow { TRAPPED, ADVANCED, REACHED };

// One RRTConnect query; the start is taken as valid so the arm can always leave where it is.
class Connect {
public:
    Connect(const Joints &start, Collision &collision, const PlanSettings &s) : start_(start), collision_(collision), s_(s) {}

    bool free(const Joints &a, const Joints &b) {
        return collision_.sweep(a, b, s_.edge_step) == Verdict::CLEAR && (b == start_ || collision_.check(b) == Verdict::CLEAR);
    }

    Grow extend(Tree &tree, const Joints &target) {
        size_t nearest = 0;
        for (size_t n = 1; n < tree.q.size(); ++n) {
            if (distance(tree.q[n], target) < distance(tree.q[nearest], target)) {
                nearest = n;
            }
        }
        const Joints &from = tree.q[nearest];
        const double  d    = distance(from, target);
        const Joints  q    = d <= s_.range ? target : lerp(from, target, s_.range / d);
        if (!free(from, q)) {
            return Grow::TRAPPED;
        }
        tree.q.push_back(q);
        tree.parent.push_back(static_cast<int>(nearest));
        return q == target ? Grow::REACHED : Grow::ADVANCED;
    }

    Grow reach(Tree &tree, const Joints &target) {
        Grow g = Grow::ADVANCED;
        while (g == Grow::ADVANCED) {
            g = extend(tree, target);
        }
        return g;
    }

    Joints sample() {
        Joints q;
        for (int j = 0; j < JOINT_COUNT; ++j) {
            q[j] = std::uniform_real_distribution<double>(collision_.arm().lower(j), collision_.arm().upper(j))(rng());
        }
        return q;
    }

    void shorten(std::vector<Joints> &path, const std::function<bool()> &out_of_time) {
        skipCorners(path);
        for (int misses = 0; misses < 40 && path.size() > 2 && !out_of_time(); ++misses) {
            std::uniform_int_distribution<size_t> segment(0, path.size() - 2);
            std::uniform_real_distribution<double> along(0.0, 1.0);
            size_t i = segment(rng()), j = segment(rng());
            if (i > j) {
                std::swap(i, j);
            }
            if (i == j) {
                continue;
            }
            const Joints a = lerp(path[i], path[i + 1], along(rng())), b = lerp(path[j], path[j + 1], along(rng()));
            if (distance(path[i], a) + distance(a, b) + distance(b, path[j + 1]) >= length({path.begin() + i, path.begin() + j + 2}) - 1e-9) {
                continue;
            }
            if (!free(path[i], a) || !free(a, b) || !free(b, path[j + 1])) {
                continue;
            }
            std::vector<Joints> shorter(path.begin(), path.begin() + i + 1);
            shorter.push_back(a);
            shorter.push_back(b);
            shorter.insert(shorter.end(), path.begin() + j + 1, path.end());
            path   = shorter;
            misses = -1;
        }
        skipCorners(path);
    }

private:
    void skipCorners(std::vector<Joints> &path) {
        std::vector<Joints> kept{path.front()};
        for (size_t i = 0; i + 1 < path.size();) {
            size_t j = path.size() - 1;
            while (j > i + 1 && !free(path[i], path[j])) {
                --j;
            }
            kept.push_back(path[j]);
            i = j;
        }
        path = kept;
    }

    const Joints       start_;
    Collision         &collision_;
    const PlanSettings &s_;
};

bool connect(const Joints &start, const Joints &goal, Collision &collision, const PlanSettings &s, double budget_s,
             const std::function<bool()> &cancelled, std::vector<Joints> &path) {
    if (collision.sweep(start, goal, s.edge_step) == Verdict::CLEAR) {
        path = {start, goal};
        return true;
    }
    const auto started     = std::chrono::steady_clock::now();
    const auto out_of_time = [&] {
        return cancelled() || std::chrono::duration<double>(std::chrono::steady_clock::now() - started).count() >= budget_s;
    };
    Connect c(start, collision, s);
    Tree    from_start{{start}, {-1}}, from_goal{{goal}, {-1}};
    Tree   *a = &from_start, *b = &from_goal;
    while (!out_of_time()) {
        if (c.extend(*a, c.sample()) != Grow::TRAPPED && c.reach(*b, a->q.back()) == Grow::REACHED) {
            std::vector<Joints> head, tail;
            for (int n = static_cast<int>(from_start.q.size()) - 1; n >= 0; n = from_start.parent[n]) {
                head.push_back(from_start.q[n]);
            }
            for (int n = from_goal.parent[from_goal.q.size() - 1]; n >= 0; n = from_goal.parent[n]) {
                tail.push_back(from_goal.q[n]);
            }
            path.assign(head.rbegin(), head.rend());
            path.insert(path.end(), tail.begin(), tail.end());
            c.shorten(path, out_of_time);
            return true;
        }
        std::swap(a, b);
    }
    return false;
}

}  // namespace

Plan planGrasp(const std::vector<GraspPose> &candidates, const Joints &start, Collision &collision, const PlanSettings &s,
               const std::function<bool()> &cancelled) {
    const auto started   = std::chrono::steady_clock::now();
    const auto remaining = [&] { return s.budget_s - std::chrono::duration<double>(std::chrono::steady_clock::now() - started).count(); };
    const double along = collision.arm().mountDistance() + s.grasp_point_from_mount;

    Plan plan;
    plan.outcomes.assign(candidates.size(), Outcome::UNREACHABLE);
    std::vector<GraspGoal> goals;
    for (size_t i = 0; i < candidates.size(); ++i) {
        plan.outcomes[i] = graspGoals(candidates[i], i, start, along, collision, goals);
    }
    std::sort(goals.begin(), goals.end(), [](const GraspGoal &a, const GraspGoal &b) { return a.swing < b.swing; });

    for (const GraspGoal &goal : goals) {
        if (plan.ok || cancelled() || remaining() < s.goal_budget_s) {
            break;
        }
        if (connect(start, goal.joints, collision, s, std::min(s.goal_budget_s, remaining()), cancelled, plan.path)) {
            plan.ok        = true;
            plan.candidate = goal.candidate;
            plan.outcomes[goal.candidate] = Outcome::OK;
            for (size_t k = 1; k < plan.path.size(); ++k) {
                plan.time_s += largestMove(plan.path[k - 1], plan.path[k]) / s.joint_speed;
            }
        } else {
            plan.outcomes[goal.candidate] = Outcome::NO_PATH;
        }
    }

    std::map<std::string, int> tally;
    for (const Outcome o : plan.outcomes) {
        ++tally[outcomeName(o)];
    }
    char line[160];
    const double spent = s.budget_s - remaining();
    if (plan.ok) {
        std::snprintf(line, sizeof(line), "chose candidate %zu of %zu, %.2f s of arm motion over %zu corners (%.2f s):",
                      plan.candidate, candidates.size(), plan.time_s, plan.path.size(), spent);
    } else {
        std::snprintf(line, sizeof(line), "no path to any of %zu candidates (%.2f s):", candidates.size(), spent);
    }
    plan.summary = line;
    for (const auto &t : tally) {
        plan.summary += " " + std::to_string(t.second) + " " + t.first;
    }
    return plan;
}

Plan planTo(const std::vector<Joints> &goals, const Joints &start, Collision &collision, const PlanSettings &s,
            const std::function<bool()> &cancelled) {
    std::vector<size_t> order(goals.size());
    std::iota(order.begin(), order.end(), 0);
    std::sort(order.begin(), order.end(), [&](size_t a, size_t b) { return largestMove(start, goals[a]) < largestMove(start, goals[b]); });
    Plan plan;
    for (const size_t g : order) {
        if (connect(start, goals[g], collision, s, s.goal_budget_s, cancelled, plan.path)) {
            plan.ok        = true;
            plan.candidate = g;
            for (size_t k = 1; k < plan.path.size(); ++k) {
                plan.time_s += largestMove(plan.path[k - 1], plan.path[k]) / s.joint_speed;
            }
            break;
        }
    }
    char line[120];
    std::snprintf(line, sizeof(line), plan.ok ? "%.2f s of arm motion over %zu corners" : "no path", plan.time_s, plan.path.size());
    plan.summary = line;
    return plan;
}

void seedPlanner(unsigned seed) { rng().seed(seed); }

}  // namespace final_v1
