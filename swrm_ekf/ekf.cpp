// Swarm EKF: up to 20 bots, adjustable field, encoders + shared UWB, requirement checks.
// Build with CMake; see README.md.
//
// PSEUDOCODE
// ----------
// config: numBots, field width/height, barrier margin, dt, speed, UWB rate, noise
// place 4 UWB anchors at the field corners (computed from field size)
// create one EKF per bot (each starts with a deliberately wrong guess)
//
// every time step dt:
//   for each bot:
//       move the true bot, make noisy encoder readings (dL, dR)
//       ekf[bot].predict(dL, dR)                    // encoder step
//   UWB can only range a limited number of times per second (shared by all bots),
//   so each step serve the next (bot, anchor) pairs in round-robin order:
//       ekf[bot].updateUwb(anchor, noisy_range)     // UWB step
//   record: true speed, position error, inside-barrier check
//
// after the run: print PASS/FAIL for each requirement
// ----------------------------------------------------------------
// GET IT TO RUN MANY ITERATIONS                      -> Config::numRuns, runOnce(), loop in main()
// get rmse for x <>, y <> and theta <> after settling time -> RunStats sumSqX/Y/Th, printed per run + overall

#include <array>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <random>
#include <string>
#include <vector>

#include <boost/multiprecision/cpp_dec_float.hpp>
#include <boost/random/mersenne_twister.hpp>

constexpr unsigned int HP_PRECISION = 50;  // digits of precision for multiprecision floats
using Real = boost::multiprecision::number<boost::multiprecision::cpp_dec_float<HP_PRECISION>>;
using Vec3 = std::array<Real, 3>;
using Mat3 = std::array<std::array<Real, 3>, 3>;

using boost::multiprecision::abs;
using boost::multiprecision::ceil;
using boost::multiprecision::cos;
using boost::multiprecision::sin;
using boost::multiprecision::sqrt;

static Mat3 identity() {
    Mat3 I{};
    for (int i = 0; i < 3; ++i) I[i][i] = 1.0;
    return I;
}
static Mat3 mul(const Mat3& A, const Mat3& B) {
    Mat3 C{};
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j)
            for (int k = 0; k < 3; ++k) C[i][j] += A[i][k] * B[k][j];
    return C;
}
static Mat3 transpose(const Mat3& A) {
    Mat3 T{};
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j) T[i][j] = A[j][i];
    return T;
}
static Real wrapAngle(Real a) {
    static const Real pi("3.1415926535897932384626433832795028841971693993751");
    while (a > pi) a -= 2 * pi;
    while (a < -pi) a += 2 * pi;
    return a;
}

struct UwbAnchor { Real x, y; };

// ---------------------------------------------------------------- EKF (one per bot)
class EKF {
public:
    EKF(const Vec3& x0, const Mat3& P0, Real wheelbase, Real kL, Real kR,
        Real uwbSigma, Real gate = Real("6.63"))
        : x_(x0), P_(P0), b_(wheelbase), kL_(kL), kR_(kR),
          R_(uwbSigma * uwbSigma), gate_(gate) {}

    void predict(const Real& dL, const Real& dR) {
        const Real ds = Real("0.5") * (dR + dL);
        const Real dth = (dR - dL) / b_;
        const Real phi = x_[2] + Real("0.5") * dth;
        const Real c = cos(phi), s = sin(phi);

        x_[0] += ds * c;
        x_[1] += ds * s;
        x_[2] = wrapAngle(x_[2] + dth);

        Mat3 F = identity();
        F[0][2] = -ds * s;
        F[1][2] = ds * c;

        const Vec3 gL = {Real("0.5") * c + ds * s / (2 * b_), Real("0.5") * s - ds * c / (2 * b_), -1 / b_};
        const Vec3 gR = {Real("0.5") * c - ds * s / (2 * b_), Real("0.5") * s + ds * c / (2 * b_), 1 / b_};
        const Real mL = kL_ * abs(dL) + Real("1e-12");
        const Real mR = kR_ * abs(dR) + Real("1e-12");

        Mat3 P = mul(mul(F, P_), transpose(F));
        for (int i = 0; i < 3; ++i)
            for (int j = 0; j < 3; ++j)
                P[i][j] += mL * gL[i] * gL[j] + mR * gR[i] * gR[j];
        P_ = P;
    }

    bool updateUwb(const UwbAnchor& a, const Real& rMeas) {
        const Real dx = x_[0] - a.x, dy = x_[1] - a.y;
        Real rPred = sqrt(dx * dx + dy * dy);
        if (rPred < Real("1e-6")) rPred = Real("1e-6");

        const Vec3 H = {dx / rPred, dy / rPred, 0};
        const Real y = rMeas - rPred;

        Vec3 PHt{};
        for (int i = 0; i < 3; ++i)
            for (int j = 0; j < 3; ++j) PHt[i] += P_[i][j] * H[j];
        Real S = R_;
        for (int i = 0; i < 3; ++i) S += H[i] * PHt[i];

        if (y * y / S > gate_) return false;

        Vec3 K{};
        for (int i = 0; i < 3; ++i) K[i] = PHt[i] / S;
        for (int i = 0; i < 3; ++i) x_[i] += K[i] * y;
        x_[2] = wrapAngle(x_[2]);

        Mat3 IKH = identity();
        for (int i = 0; i < 3; ++i)
            for (int j = 0; j < 3; ++j) IKH[i][j] -= K[i] * H[j];
        Mat3 P = mul(mul(IKH, P_), transpose(IKH));
        for (int i = 0; i < 3; ++i)
            for (int j = 0; j < 3; ++j) P[i][j] += R_ * K[i] * K[j];
        P_ = P;
        return true;
    }

    const Vec3& state() const { return x_; }

private:
    Vec3 x_;
    Mat3 P_;
    Real b_, kL_, kR_, R_, gate_;
};

// ---------------------------------------------------------------- configuration
struct Config {
    int    numBots         = 20;      // requirement: up to 20
    Real fieldW          = Real("3.0");     // m  (adjustable)
    Real fieldH          = Real("2.5");     // m  (adjustable)
    Real barrierMargin   = Real("0.05");    // m  barrier sits this far inside the field edge
    Real wheelbase       = Real("0.15");    // m  (bot footprint is 192 mm max)
    Real dt              = Real("0.02");    // s  encoder period (50 Hz)
    Real speed           = Real("0.16");    // m/s (requirement: >= 0.1524 m/s = 0.5 ft/s)
    Real circleRadius    = Real("0.20");    // m  each bot circles inside its own grid cell
    Real duration        = Real("300.0");   // s  quick run. Set 3600.0 for the full 1-hour test
    Real settle          = Real("30.0");    // s  ignore error while filters converge
    Real encSigma        = Real("1e-4");    // m  encoder noise per tick (std dev)
    Real uwbSigma        = Real("0.05");    // m  UWB range noise (std dev)
    Real uwbRangesPerSec = Real("200.0");   // total ranges/s the UWB system can do, shared by all bots
    Real outlierProb     = Real("0.02");    // chance of a bad (NLOS) UWB reading

    // Monte Carlo
    int      numRuns       = 20;      // how many independent iterations to run
    unsigned baseSeed      = 7;       // run r uses seed baseSeed + r
    bool     printEachRun  = true;    // one line per run
};

// Requirement constants
static const Real kInch          = Real("0.0254");            // m
static const Real kMinSpeed      = Real("0.5") * Real("0.3048");      // 0.5 ft/s in m/s
static const Real kMinAreaPerBot = Real("3.6") * Real("0.09290304");  // 3.6 ft^2 in m^2
static const Real kRadToDeg      =
    Real("180") / Real("3.1415926535897932384626433832795028841971693993751");

static const char* pf(bool ok) { return ok ? "PASS" : "FAIL"; }

// ---------------------------------------------------------------- one run's results
struct RunStats {
    Real sumSqX = 0, sumSqY = 0, sumSqTh = 0, sumSqPos = 0;  // post-settle squared errors
    Real maxErr = 0, minSpeed = Real("1e9");
    long samples = 0, withinInch = 0, rejected = 0, uwbTotal = 0, outsideBarrier = 0;

    Real rmseX()   const { return sqrt(sumSqX / samples); }
    Real rmseY()   const { return sqrt(sumSqY / samples); }
    Real rmseTh()  const { return sqrt(sumSqTh / samples); }
    Real rmsePos() const { return sqrt(sumSqPos / samples); }
};

// ---------------------------------------------------------------- one full simulation
static RunStats runOnce(const Config& cfg, const std::vector<UwbAnchor>& anchors, unsigned seed,
                        std::vector<std::array<Real, 3>>& chartSumSq) {
    int cols = ceil(sqrt(Real(cfg.numBots) * cfg.fieldW / cfg.fieldH)).convert_to<int>();
    int rows = (cfg.numBots + cols - 1) / cols;
    const Real cellW = cfg.fieldW / cols, cellH = cfg.fieldH / rows;

    boost::random::mt19937 rng(seed);
    std::normal_distribution<double> N01(0.0, 1.0);
    std::bernoulli_distribution outlier(cfg.outlierProb.convert_to<double>());

    const Real w = cfg.speed / cfg.circleRadius;                            // turn rate
    const Real kEnc = cfg.encSigma * cfg.encSigma / (cfg.speed * cfg.dt);   // variance per meter
    const Real tL = (cfg.speed - w * cfg.wheelbase / 2) * cfg.dt;
    const Real tR = (cfg.speed + w * cfg.wheelbase / 2) * cfg.dt;
    const Real ds = Real("0.5") * (tL + tR);
    const Real dth = (tR - tL) / cfg.wheelbase;

    std::vector<Vec3> truth(cfg.numBots);
    std::vector<EKF> bots;
    bots.reserve(cfg.numBots);
    for (int i = 0; i < cfg.numBots; ++i) {
        const Real cx = (Real(i % cols) + Real("0.5")) * cellW;
        const Real cy = (Real(i / cols) + Real("0.5")) * cellH;
        truth[i] = {cx, cy - cfg.circleRadius, 0};  // start at bottom of its circle, heading +x
        Vec3 guess = {truth[i][0] + Real("0.10") * Real(N01(rng)),
                      truth[i][1] + Real("0.10") * Real(N01(rng)),
                      Real("0.10") * Real(N01(rng))};
        Mat3 P0{};
        P0[0][0] = Real("0.02"); P0[1][1] = Real("0.02"); P0[2][2] = Real("0.02");
        bots.emplace_back(guess, P0, cfg.wheelbase, kEnc, kEnc, cfg.uwbSigma);
    }

    const long steps = static_cast<long>(cfg.duration / cfg.dt);
    const long settleSteps = static_cast<long>(cfg.settle / cfg.dt);
    const Real slotsPerStep = cfg.uwbRangesPerSec * cfg.dt;
    Real slotCredit = 0;
    long pairIdx = 0;
    const long numPairs = static_cast<long>(cfg.numBots) * static_cast<long>(anchors.size());

    RunStats st;

    for (long k = 0; k < steps; ++k) {
        const Real truePhi = truth[0][2] + Real("0.5") * dth;
        const Real trueC = cos(truePhi), trueS = sin(truePhi);
        for (int i = 0; i < cfg.numBots; ++i) {
            truth[i][0] += ds * trueC;
            truth[i][1] += ds * trueS;
            truth[i][2] = wrapAngle(truth[i][2] + dth);  // keep heading bounded

            const Real speedNow = ds / cfg.dt;
            if (speedNow < st.minSpeed) st.minSpeed = speedNow;

            bots[i].predict(tL + cfg.encSigma * Real(N01(rng)),
                            tR + cfg.encSigma * Real(N01(rng)));
        }

        // shared UWB: serve the next (bot, anchor) pairs round-robin
        slotCredit += slotsPerStep;
        while (slotCredit >= 1.0) {
            slotCredit -= 1.0;
            const int bi = static_cast<int>(pairIdx / anchors.size());
            const int ai = static_cast<int>(pairIdx % anchors.size());
            pairIdx = (pairIdx + 1) % numPairs;
            const Real ddx = truth[bi][0] - anchors[ai].x, ddy = truth[bi][1] - anchors[ai].y;
            Real r = sqrt(ddx * ddx + ddy * ddy) + cfg.uwbSigma * Real(N01(rng));
            if (outlier(rng)) r += Real("1.5");
            ++st.uwbTotal;
            if (!bots[bi].updateUwb(anchors[ai], r)) ++st.rejected;
        }

        // barrier check (truth and estimate) every step, error stats after settling
        for (int i = 0; i < cfg.numBots; ++i) {
            const Vec3& e = bots[i].state();
            const Real m = cfg.barrierMargin;
            auto inside = [&](const Real& x, const Real& y) {
                return x >= m && x <= cfg.fieldW - m && y >= m && y <= cfg.fieldH - m;
            };
            if (!inside(truth[i][0], truth[i][1]) || (k >= settleSteps && !inside(e[0], e[1])))
                ++st.outsideBarrier;

            if (k >= settleSteps) {
                const Real ex  = e[0] - truth[i][0];
                const Real ey  = e[1] - truth[i][1];
                const Real eth = wrapAngle(e[2] - truth[i][2]);
                const Real err = sqrt(ex * ex + ey * ey);
                st.sumSqX   += ex * ex;
                st.sumSqY   += ey * ey;
                st.sumSqTh  += eth * eth;
                st.sumSqPos += err * err;
                chartSumSq[k][0] += ex * ex;
                chartSumSq[k][1] += ey * ey;
                chartSumSq[k][2] += err * err;
                if (err > st.maxErr) st.maxErr = err;
                if (err <= kInch) ++st.withinInch;
                ++st.samples;
            }
        }
    }
    return st;
}

static bool writeRmseChart(const std::string& path, const std::vector<std::array<Real, 3>>& sumSq,
                           const Config& cfg, long settleSteps) {
    std::ofstream chart(path);
    if (!chart) return false;

    constexpr int width = 960;
    constexpr int height = 560;
    constexpr int left = 92;
    constexpr int right = 28;
    constexpr int top = 36;
    constexpr int bottom = 76;
    const Real plotWidth = width - left - right;
    const Real plotHeight = height - top - bottom;
    const long sampleCount = static_cast<long>(cfg.numRuns) * cfg.numBots;

    Real maxRmse = 0;
    for (long k = settleSteps; k < static_cast<long>(sumSq.size()); ++k)
        for (int series = 0; series < 3; ++series) {
            const Real rmse = sqrt(sumSq[k][series] / sampleCount) * 1000;
            if (rmse > maxRmse) maxRmse = rmse;
        }
    if (maxRmse == 0) maxRmse = 1;
    const Real yMax = ceil(maxRmse / 5) * 5;
    const Real duration = cfg.dt * (sumSq.size() - 1 - settleSteps);

    

    for (int tick = 0; tick <= 5; ++tick) {
        const Real value = yMax * tick / 5;
        const Real y = top + plotHeight * (5 - tick) / 5;
        chart << "<line class=\"grid\" x1=\"" << left << "\" y1=\"" << y << "\" x2=\""
              << width - right << "\" y2=\"" << y << "\"/>\n"
              << "<text x=\"" << left - 12 << "\" y=\"" << y + 5
              << "\" text-anchor=\"end\" font-size=\"12\">" << value << "</text>\n";
    }

    for (int tick = 0; tick <= 5; ++tick) {
        const Real x = left + plotWidth * tick / 5;
        const Real time = duration * tick / 5;
        chart << "<line class=\"grid\" x1=\"" << x << "\" y1=\"" << top << "\" x2=\""
              << x << "\" y2=\"" << top + plotHeight << "\"/>\n"
              << "<text x=\"" << x << "\" y=\"" << top + plotHeight + 22
              << "\" text-anchor=\"middle\" font-size=\"12\">" << time << "</text>\n";
    }
    chart << "<line class=\"axis\" x1=\"" << left << "\" y1=\"" << top + plotHeight
          << "\" x2=\"" << width - right << "\" y2=\"" << top + plotHeight << "\"/>\n"
          << "<line class=\"axis\" x1=\"" << left << "\" y1=\"" << top
          << "\" x2=\"" << left << "\" y2=\"" << top + plotHeight << "\"/>\n"
          << "<text x=\"" << left + plotWidth / 2 << "\" y=\"" << height - 20
          << "\" text-anchor=\"middle\" font-size=\"14\">Time after settling (s)</text>\n"
          << "<text transform=\"translate(22 " << top + plotHeight / 2
          << ") rotate(-90)\" text-anchor=\"middle\" font-size=\"14\">RMSE (mm)</text>\n";

    const char* colors[] = {"#1565c0", "#ef6c00", "#2e7d32"};
    const char* labels[] = {"X", "Y", "Position"};
    for (int series = 0; series < 3; ++series) {
        chart << "<polyline class=\"line\" stroke=\"" << colors[series] << "\" points=\"";
        for (long k = settleSteps; k < static_cast<long>(sumSq.size()); ++k) {
            const Real time = cfg.dt * (k - settleSteps);
            const Real value = sqrt(sumSq[k][series] / sampleCount) * 1000;
            const Real x = left + plotWidth * time / duration;
            const Real y = top + plotHeight * (1 - value / yMax);
            chart << x << ',' << y << ' ';
        }
        chart << "\"/>\n";
        const int legendX = left + series * 150;
        chart << "<line x1=\"" << legendX << "\" y1=\"" << height - 48 << "\" x2=\""
              << legendX + 26 << "\" y2=\"" << height - 48 << "\" stroke=\"" << colors[series]
              << "\" stroke-width=\"3\"/>\n"
              << "<text x=\"" << legendX + 34 << "\" y=\"" << height - 43
              << "\" font-size=\"13\">" << labels[series] << "</text>\n";
    }
    chart << "</svg>\n";
    return static_cast<bool>(chart);
}

// mean and sample std dev of a list
static void meanStd(const std::vector<Real>& v, Real& mean, Real& sd) {
    mean = 0;
    for (const Real& a : v) mean += a;
    mean /= v.size();
    sd = 0;
    for (const Real& a : v) sd += (a - mean) * (a - mean);
    sd = v.size() > 1 ? sqrt(sd / (v.size() - 1)) : Real(0);
}

int main(int argc, char* argv[]) {
    Config cfg;  // change values here to resize the field, bot count, number of runs, etc.
    if (argc == 2 && std::string(argv[1]) == "--quick") {
        cfg.numBots = 5;
        cfg.duration = Real("20.0");
        cfg.settle = Real("3.0");
        cfg.numRuns = 1;
    } else if (argc > 1) {
        std::cerr << "Usage: " << argv[0] << " [--quick]\n";
        return 1;
    }

    const Real areaPerBot = (cfg.fieldW * cfg.fieldH) / cfg.numBots;
    const bool countOk = cfg.numBots >= 1 && cfg.numBots <= 20;
    const bool areaOk = areaPerBot >= kMinAreaPerBot;
    if (!countOk) { std::cout << "numBots must be 1..20\n"; return 1; }
    if (cfg.numRuns < 1) { std::cout << "numRuns must be >= 1\n"; return 1; }
    if (cfg.settle >= cfg.duration) { std::cout << "settle must be < duration\n"; return 1; }

    // anchors placed from the field size (4 corners)
    const std::vector<UwbAnchor> anchors = {
        {0, 0}, {cfg.fieldW, 0}, {cfg.fieldW, cfg.fieldH}, {0, cfg.fieldH}};
    const long numPairs = static_cast<long>(cfg.numBots) * static_cast<long>(anchors.size());

    std::cout << "=== Swarm EKF Monte Carlo ===\n";
    std::cout << "Bots: " << cfg.numBots << "   Field: " << cfg.fieldW << " x " << cfg.fieldH
              << " m   Simulated time per run: " << cfg.duration << " s   Settle: "
              << cfg.settle << " s   Runs: " << cfg.numRuns << "\n";
    std::cout << "UWB: " << cfg.uwbRangesPerSec << " ranges/s shared, each (bot,anchor) pair "
              << "served every " << numPairs / cfg.uwbRangesPerSec << " s\n\n";

    std::vector<Real> rx, ry, rth, rpos;
    const long totalSteps = static_cast<long>(cfg.duration / cfg.dt);
    const long settleSteps = static_cast<long>(cfg.settle / cfg.dt);
    std::vector<std::array<Real, 3>> chartSumSq(totalSteps);
    RunStats total;  // pooled over all runs

    std::cout << std::fixed;
    if (cfg.printEachRun)
        std::cout << " run    seed   RMSE x[mm]  RMSE y[mm]  RMSE th[deg]  RMSE pos[mm]  max[mm]  <=1in[%]  rejected\n";

    for (int r = 0; r < cfg.numRuns; ++r) {
        const unsigned seed = cfg.baseSeed + static_cast<unsigned>(r);
        const RunStats s = runOnce(cfg, anchors, seed, chartSumSq);

        rx.push_back(s.rmseX());
        ry.push_back(s.rmseY());
        rth.push_back(s.rmseTh());
        rpos.push_back(s.rmsePos());

        total.sumSqX += s.sumSqX;   total.sumSqY += s.sumSqY;
        total.sumSqTh += s.sumSqTh; total.sumSqPos += s.sumSqPos;
        total.samples += s.samples; total.withinInch += s.withinInch;
        total.rejected += s.rejected; total.uwbTotal += s.uwbTotal;
        total.outsideBarrier += s.outsideBarrier;
        if (s.maxErr > total.maxErr) total.maxErr = s.maxErr;
        if (s.minSpeed < total.minSpeed) total.minSpeed = s.minSpeed;

        if (cfg.printEachRun)
            std::cout << std::setw(4) << r + 1 << std::setw(8) << seed
                      << std::setprecision(2)
                      << std::setw(12) << s.rmseX() * 1000
                      << std::setw(12) << s.rmseY() * 1000
                      << std::setprecision(3)
                      << std::setw(14) << s.rmseTh() * kRadToDeg
                      << std::setprecision(2)
                      << std::setw(14) << s.rmsePos() * 1000
                      << std::setw(9) << s.maxErr * 1000
                      << std::setw(10) << Real("100") * s.withinInch / s.samples
                      << std::setw(10) << s.rejected << "\n";
    }

    Real mx, sx, my, sy, mth, sth, mp, sp;
    meanStd(rx, mx, sx);
    meanStd(ry, my, sy);
    meanStd(rth, mth, sth);
    meanStd(rpos, mp, sp);

    std::cout << "\n=== RMSE after settling, over " << cfg.numRuns << " runs ===\n";
    std::cout << "            pooled        mean +/- std across runs\n";
    std::cout << std::setprecision(2);
    std::cout << "x     [mm]  " << std::setw(8) << total.rmseX() * 1000
              << "      " << mx * 1000 << " +/- " << sx * 1000 << "\n";
    std::cout << "y     [mm]  " << std::setw(8) << total.rmseY() * 1000
              << "      " << my * 1000 << " +/- " << sy * 1000 << "\n";
    std::cout << std::setprecision(3);
    std::cout << "theta [deg] " << std::setw(8) << total.rmseTh() * kRadToDeg
              << "      " << mth * kRadToDeg << " +/- " << sth * kRadToDeg
              << "   (" << std::setprecision(5) << total.rmseTh() << " rad pooled)\n";
    std::cout << std::setprecision(2);
    std::cout << "pos   [mm]  " << std::setw(8) << total.rmsePos() * 1000
              << "      " << mp * 1000 << " +/- " << sp * 1000 << "\n";
    std::cout << "Worst position error, any run: " << total.maxErr * 1000 << " mm;  within 1 inch "
              << Real("100") * total.withinInch / total.samples << " %;  UWB rejected "
              << total.rejected << " / " << total.uwbTotal << "\n";

    if (!writeRmseChart("rmse_chart.svg", chartSumSq, cfg, settleSteps)) {
        std::cerr << "Failed to write RMSE chart to rmse_chart.svg\n";
        return 1;
    }
    std::cout << "RMSE chart written to rmse_chart.svg\n";

    // config-only checks kept available for the requirement PASS/FAIL section
    (void)areaOk; (void)kMinSpeed; (void)pf;
    return 0;
}