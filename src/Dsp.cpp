#include "radar/Dsp.hpp"

#include <algorithm>
#include <cmath>
#include <numeric>
#include <stdexcept>

namespace radar {
namespace {
constexpr double kPi = 3.14159265358979323846;
constexpr double kSqrt2 = 1.41421356237309504880;
constexpr double kSqrt2Pi = 2.50662827463100050242;
}  // namespace

// ============================================================== statistics

Stats computeStats(const std::vector<double>& x) {
    Stats s;
    if (x.empty()) return s;
    s.n = static_cast<double>(x.size());
    s.mean = std::accumulate(x.begin(), x.end(), 0.0) / s.n;
    double m2 = 0, m3 = 0, m4 = 0;
    for (double v : x) {
        const double d = v - s.mean;
        const double d2 = d * d;
        m2 += d2;
        m3 += d2 * d;
        m4 += d2 * d2;
    }
    m2 /= s.n;
    m3 /= s.n;
    m4 /= s.n;
    s.variance = m2;
    s.stddev = std::sqrt(std::max(0.0, m2));
    if (m2 > 1e-18) {
        s.skew = m3 / (std::pow(m2, 1.5));
        s.kurtosis = m4 / (m2 * m2) - 3.0;  // excess kurtosis
    }
    s.min = *std::min_element(x.begin(), x.end());
    s.max = *std::max_element(x.begin(), x.end());
    return s;
}

// Acklam's inverse normal CDF.
double normalQuantile(double p) {
    if (p <= 0.0) return -8.0;
    if (p >= 1.0) return 8.0;
    static const double a[6] = {-3.969683028665376e+01, 2.209460984245205e+02,
                                -2.759285104469687e+02, 1.383577518672690e+02,
                                -3.066479806614716e+01, 2.506628277459239e+00};
    static const double b[5] = {-5.447609879822406e+01, 1.615858368580409e+02,
                                -1.556989798598866e+02, 6.680131188771972e+01,
                                -1.328068155288572e+01};
    static const double c[6] = {-7.784894002430293e-03, -3.223964580411365e-01,
                                -2.400758277161838e+00, -2.549732539343734e+00,
                                4.374664141464968e+00,  2.938163982698783e+00};
    static const double d[4] = {7.784695709041462e-03, 3.224671290700398e-01,
                                2.445134137142996e+00, 3.754408661907416e+00};
    const double plow = 0.02425, phigh = 1.0 - plow;
    double q, r;
    if (p < plow) {
        q = std::sqrt(-2.0 * std::log(p));
        return (((((c[0] * q + c[1]) * q + c[2]) * q + c[3]) * q + c[4]) * q + c[5]) /
               ((((d[0] * q + d[1]) * q + d[2]) * q + d[3]) * q + 1.0);
    }
    if (p > phigh) {
        q = std::sqrt(-2.0 * std::log(1.0 - p));
        return -(((((c[0] * q + c[1]) * q + c[2]) * q + c[3]) * q + c[4]) * q + c[5]) /
               ((((d[0] * q + d[1]) * q + d[2]) * q + d[3]) * q + 1.0);
    }
    q = p - 0.5;
    r = q * q;
    return (((((a[0] * r + a[1]) * r + a[2]) * r + a[3]) * r + a[4]) * r + a[5]) * q /
           (((((b[0] * r + b[1]) * r + b[2]) * r + b[3]) * r + b[4]) * r + 1.0);
}

// Series + Lentz continued fraction for the regularised lower incomplete gamma.
double gammaP(double a, double x) {
    if (x <= 0.0) return 0.0;
    if (a <= 0.0) return 1.0;
    if (x < a + 1.0) {  // series expansion
        double ap = a, sum = 1.0 / a, del = sum;
        for (int n = 1; n <= 500; ++n) {
            ap += 1.0;
            del *= x / ap;
            sum += del;
            if (std::fabs(del) < std::fabs(sum) * 1e-14) break;
        }
        return sum * std::exp(-x + a * std::log(x) - std::lgamma(a));
    }
    // continued fraction for Q(x,a)
    const double tiny = 1e-300;
    double b = x + 1.0 - a, c = 1.0 / tiny, d = 1.0 / b, h = d;
    for (int i = 1; i <= 500; ++i) {
        const double an = -i * (i - a);
        b += 2.0;
        d = an * d + b;
        if (std::fabs(d) < tiny) d = tiny;
        c = b + an / c;
        if (std::fabs(c) < tiny) c = tiny;
        d = 1.0 / d;
        const double del = d * c;
        h *= del;
        if (std::fabs(del - 1.0) < 1e-14) break;
    }
    const double q = std::exp(-x + a * std::log(x) - std::lgamma(a)) * h;
    return 1.0 - q;
}

// ============================================================== windowing

std::vector<double> movingAverage(const std::vector<double>& x, int halfWindow) {
    if (x.empty() || halfWindow <= 0) return x;
    const int n = static_cast<int>(x.size());
    std::vector<double> out(n, 0.0);
    // Running sum, O(n) regardless of window width.
    double acc = 0.0;
    const int w = std::min(2 * halfWindow + 1, n);
    for (int i = 0; i < n; ++i) {
        acc += x[i];
        if (i >= w) acc -= x[i - w];
        const int lo = std::max(0, i - halfWindow);
        const int hi = std::min(n - 1, i + halfWindow);
        const double denom = static_cast<double>(hi - lo + 1);
        out[i] = acc / denom;
    }
    return out;
}

// Fit a least-squares polynomial of `order` over a window centred on index
// `centre` (clamped at the edges) and evaluate it back at `centre`. Solving the
// normal equations with a small ridge term keeps it conditioned when `centre`
// sits on the boundary and the offsets run asymmetrically.
static double sgPoint(const std::vector<double>& x, int order, int halfWindow, int centre) {
    const int n = static_cast<int>(x.size());
    if (n == 0) return 0.0;
    int lo = centre - halfWindow;
    int hi = centre + halfWindow;
    if (lo < 0) {
        hi -= lo;
        lo = 0;
    }
    if (hi > n - 1) {
        lo -= (hi - (n - 1));
        hi = n - 1;
        if (lo < 0) lo = 0;
    }
    const int width = hi - lo + 1;
    if (width < order + 1) return x[std::clamp(centre, 0, n - 1)];

    std::vector<std::vector<double>> M(static_cast<size_t>(order) + 1,
                                        std::vector<double>(static_cast<size_t>(order) + 1,
                                                           0.0));
    std::vector<double> rhs(static_cast<size_t>(order) + 1, 0.0);
    for (int a = 0; a <= order; ++a) {
        for (int b = 0; b <= order; ++b) {
            double s = 0.0;
            for (int i = lo; i <= hi; ++i) {
                double pa = 1.0, pb = 1.0;
                for (int k = 0; k < a; ++k) pa *= (i - centre);
                for (int k = 0; k < b; ++k) pb *= (i - centre);
                s += pa * pb;
            }
            M[static_cast<size_t>(a)][static_cast<size_t>(b)] = s;
        }
        M[static_cast<size_t>(a)][static_cast<size_t>(a)] += 1e-9;  // ridge
        double s = 0.0;
        for (int i = lo; i <= hi; ++i) {
            double pa = 1.0;
            for (int k = 0; k < a; ++k) pa *= (i - centre);
            s += pa * x[static_cast<size_t>(i)];
        }
        rhs[static_cast<size_t>(a)] = s;
    }
    // Gaussian elimination with partial pivoting.
    std::vector<double> coef(static_cast<size_t>(order) + 1, 0.0);
    for (int c = 0; c <= order; ++c) {
        int piv = c;
        for (int r = c + 1; r <= order; ++r)
            if (std::fabs(M[static_cast<size_t>(r)][static_cast<size_t>(c)]) >
                std::fabs(M[static_cast<size_t>(piv)][static_cast<size_t>(c)]))
                piv = r;
        if (piv != c) {
            std::swap(M[static_cast<size_t>(c)], M[static_cast<size_t>(piv)]);
            std::swap(rhs[static_cast<size_t>(c)], rhs[static_cast<size_t>(piv)]);
        }
        if (std::fabs(M[static_cast<size_t>(c)][static_cast<size_t>(c)]) < 1e-15) continue;
        for (int r = c + 1; r <= order; ++r) {
            const double f = M[static_cast<size_t>(r)][static_cast<size_t>(c)] /
                             M[static_cast<size_t>(c)][static_cast<size_t>(c)];
            for (int k = c; k <= order; ++k)
                M[static_cast<size_t>(r)][static_cast<size_t>(k)] -=
                    f * M[static_cast<size_t>(c)][static_cast<size_t>(k)];
            rhs[static_cast<size_t>(r)] -= f * rhs[static_cast<size_t>(c)];
        }
    }
    for (int r = order; r >= 0; --r) {
        double s = rhs[static_cast<size_t>(r)];
        for (int k = r + 1; k <= order; ++k)
            s -= M[static_cast<size_t>(r)][static_cast<size_t>(k)] * coef[static_cast<size_t>(k)];
        coef[static_cast<size_t>(r)] =
            std::fabs(M[static_cast<size_t>(r)][static_cast<size_t>(r)]) > 1e-15
                ? s / M[static_cast<size_t>(r)][static_cast<size_t>(r)]
                : 0.0;
    }
    // Evaluate at offset 0 from the window centre, which is exactly coef[0].
    return coef[0];
}

std::vector<double> savitzkyGolay(const std::vector<double>& x, int order, int halfWindow) {
    const int n = static_cast<int>(x.size());
    if (n == 0) return x;
    if (halfWindow <= 0) return x;
    order = std::clamp(order, 1, std::max(1, 2 * halfWindow));
    halfWindow = std::clamp(halfWindow, 1, std::max(1, (n - 1) / 2));
    std::vector<double> out(static_cast<size_t>(n), 0.0);
    for (int i = 0; i < n; ++i)
        out[static_cast<size_t>(i)] = sgPoint(x, order, halfWindow, i);
    return out;
}

std::vector<double> exponentialSmoothing(const std::vector<double>& x, double alpha) {
    std::vector<double> out(x.size(), 0.0);
    if (x.empty()) return out;
    alpha = std::clamp(alpha, 1e-3, 1.0);
    out[0] = x[0];
    for (size_t i = 1; i < x.size(); ++i) out[i] = alpha * x[i] + (1.0 - alpha) * out[i - 1];
    return out;
}

std::vector<double> detrend(const std::vector<double>& x) {
    const int n = static_cast<int>(x.size());
    if (n < 3) return x;
    double sx = 0, sy = 0, sxx = 0, sxy = 0;
    for (int i = 0; i < n; ++i) {
        sx += i;
        sy += x[i];
        sxx += static_cast<double>(i) * i;
        sxy += static_cast<double>(i) * x[i];
    }
    const double dn = static_cast<double>(n);
    const double denom = dn * sxx - sx * sx;
    if (std::fabs(denom) < 1e-12) return x;
    const double slope = (dn * sxy - sx * sy) / denom;
    const double intercept = (sy - slope * sx) / dn;
    std::vector<double> out(n);
    for (int i = 0; i < n; ++i) out[i] = x[i] - (slope * i + intercept);
    return out;
}

// ================================================================== FFT

void fft(std::vector<std::complex<double>>& a, bool inverse) {
    const size_t n = a.size();
    // The iterative radix-2 butterfly only indexes correctly for powers of two.
    // Refusing anything else is far better than reading past the buffer.
    if (n < 2 || (n & (n - 1)) != 0) return;
    // Bit-reversal permutation.
    for (size_t i = 1, j = 0; i < n; ++i) {
        size_t bit = n >> 1;
        for (; j & bit; bit >>= 1) j ^= bit;
        j ^= bit;
        if (i < j) std::swap(a[i], a[j]);
    }
    for (size_t len = 2; len <= n; len <<= 1) {
        const double ang = 2.0 * kPi / static_cast<double>(len) * (inverse ? 1.0 : -1.0);
        const std::complex<double> wl(std::cos(ang), std::sin(ang));
        for (size_t i = 0; i < n; i += len) {
            std::complex<double> w(1.0, 0.0);
            for (size_t k = 0; k < len / 2; ++k) {
                const std::complex<double> u = a[i + k];
                const std::complex<double> v = a[i + k + len / 2] * w;
                a[i + k] = u + v;
                a[i + k + len / 2] = u - v;
                w *= wl;
            }
        }
    }
    if (inverse)
        for (auto& z : a) z /= static_cast<double>(n);
}

std::vector<double> powerSpectrum(const std::vector<double>& x, double sampleRateHz) {
    if (x.empty()) return {};
    int P = 2;
    while (P < static_cast<int>(x.size())) P <<= 1;  // zero-pad to a power of two
    std::vector<std::complex<double>> buf(static_cast<size_t>(P));
    for (size_t i = 0; i < x.size(); ++i) buf[i] = x[i];
    fft(buf, false);
    const size_t n = buf.size();
    const size_t half = n / 2;
    std::vector<double> out(half);
    for (size_t i = 0; i < half; ++i) out[i] = std::norm(buf[i]);
    (void)sampleRateHz;
    return out;
}

std::vector<double> windowedPowerSpectrum(const std::vector<double>& x, double sampleRateHz) {
    if (x.empty()) return {};
    const size_t n = x.size();
    int P = 2;
    while (P < static_cast<int>(n)) P <<= 1;  // zero-pad to a power of two
    std::vector<std::complex<double>> buf(static_cast<size_t>(P));
    for (size_t i = 0; i < n; ++i) {
        const double hann = 0.5 * (1.0 - std::cos(2.0 * kPi * static_cast<double>(i) /
                                                static_cast<double>(n > 1 ? n - 1 : 1)));
        buf[i] = x[i] * hann;
    }
    fft(buf, false);
    const size_t half = static_cast<size_t>(P / 2);
    std::vector<double> out(half);
    for (size_t i = 0; i < half; ++i) out[i] = std::norm(buf[i]) / static_cast<double>(P);
    (void)sampleRateHz;
    return out;
}

std::vector<double> autocorrelation(const std::vector<double>& x, int maxLag) {
    const int n = static_cast<int>(x.size());
    if (n < 2) return {1.0};
    maxLag = std::min(maxLag, n - 1);
    const Stats s = computeStats(x);
    std::vector<double> out(static_cast<size_t>(maxLag) + 1, 0.0);
    for (int lag = 0; lag <= maxLag; ++lag) {
        double acc = 0.0;
        for (int i = 0; i + lag < n; ++i) acc += (x[i] - s.mean) * (x[i + lag] - s.mean);
        out[static_cast<size_t>(lag)] = acc / static_cast<double>(n);
    }
    if (std::fabs(out[0]) > 1e-18)
        for (auto& v : out) v /= out[0];
    return out;
}

// =============================================================== path loss

double PathLossModel::rangeFromRssi(double rssiDbm) const {
    // RSSI = RSSI0 - 10n log10(d/d0)  =>  d = d0 * 10^((RSSI0 - RSSI)/(10n))
    const double denom = 10.0 * exponent;
    if (std::fabs(denom) < 1e-9) return referenceDistanceM;
    const double p = (rssiAtRefDbm - rssiDbm) / denom;
    return referenceDistanceM * std::pow(10.0, p);
}

double PathLossModel::rssiFromRange(double rangeM) const {
    if (rangeM <= 0.0) return 0.0;
    return rssiAtRefDbm - 10.0 * exponent * std::log10(rangeM / referenceDistanceM);
}

double PathLossModel::rangeSigmaAt(double rangeM) const {
    // dRSSI/dd = -10n / (d ln10)  =>  sigma_d = sigma_dB * d ln10 / (10n)
    const double d = std::max(rangeM, referenceDistanceM);
    return shadowSigmaDb * d * std::log(10.0) / (10.0 * exponent);
}

double planarDistance(double x1, double y1, double x2, double y2) {
    const double dx = x1 - x2, dy = y1 - y2;
    return std::sqrt(dx * dx + dy * dy);
}

TrilatResult trilaterate(const std::vector<Anchor>& anchors,
                         const std::vector<std::array<double, 3>>& obs) {
    TrilatResult res;
    // obs entries: [anchorIndex, rangeM, varianceM2]
    // Linearised least squares: subtract the first range equation from the rest,
    // which removes the unknown target range and leaves a linear system.
    std::vector<double> A, b;
    int m = 0;
    double anchorX0 = 0, anchorY0 = 0;
    for (const auto& o : obs) {
        const int idx = static_cast<int>(o[0]);
        if (idx < 0 || idx >= static_cast<int>(anchors.size())) continue;
        if (!std::isfinite(o[1]) || o[1] <= 0) continue;
        if (m == 0) {
            anchorX0 = anchors[static_cast<size_t>(idx)].x;
            anchorY0 = anchors[static_cast<size_t>(idx)].y;
        }
        ++m;
    }
    if (m < 2) {
        res.gdop = 999.0;
        return res;
    }
    // Build the design matrix by differencing against the first valid equation.
    int first = -1;
    double firstR = 0;
    for (const auto& o : obs) {
        const int idx = static_cast<int>(o[0]);
        if (idx < 0 || idx >= static_cast<int>(anchors.size())) continue;
        if (!std::isfinite(o[1]) || o[1] <= 0) continue;
        first = idx;
        firstR = o[1];
        break;
    }
    if (first < 0) return res;

    int rows = 0;
    for (const auto& o : obs) {
        const int idx = static_cast<int>(o[0]);
        if (idx < 0 || idx >= static_cast<int>(anchors.size())) continue;
        if (!std::isfinite(o[1]) || o[1] <= 0) continue;
        if (idx == first) continue;
        const Anchor& ai = anchors[static_cast<size_t>(idx)];
        const double dx = ai.x - anchorX0;
        const double dy = ai.y - anchorY0;
        const double ri = o[1];
        if (std::fabs(dx) < 1e-9 && std::fabs(dy) < 1e-9) continue;  // duplicate geometry
        const double w = 1.0 / std::max(o[2], 1e-6);
        A.push_back(2.0 * dx * w);
        A.push_back(2.0 * dy * w);
        // r_i^2 - r_0^2 = |p - a_i|^2 - |p - a_0|^2
        b.push_back((ai.x * ai.x + ai.y * ai.y - anchorX0 * anchorX0 - anchorY0 * anchorY0) * w +
                    (firstR * firstR - ri * ri) * w);
        ++rows;
    }
    if (rows < 2) {
        res.gdop = 999.0;
        return res;
    }
    // Normal equations with the 2x2 inverse.
    const double a11 = A[0] * A[0], a12 = A[0] * A[1], a22 = A[1] * A[1];
    const double r1 = A[0] * b[0], r2 = A[1] * b[0];
    const double det = a11 * a22 - a12 * a12;
    if (std::fabs(det) < 1e-12) {
        res.gdop = 999.0;
        return res;
    }
    res.x = (a22 * r1 - a12 * r2) / det;
    res.y = (a11 * r2 - a12 * r1) / det;
    res.covariance[0] = a22 / det;
    res.covariance[1] = -a12 / det;
    res.covariance[2] = a11 / det;

    // GDOP: trace of the 2x2 covariance, square-rooted. Geometry quality metric
    // that does not depend on the actual measurements.
    res.gdop = std::sqrt(std::max(0.0, res.covariance[0] + res.covariance[2]));

    // Residual against every observation.
    double sse = 0.0;
    for (const auto& o : obs) {
        const int idx = static_cast<int>(o[0]);
        if (idx < 0 || idx >= static_cast<int>(anchors.size())) continue;
        if (!std::isfinite(o[1]) || o[1] <= 0) continue;
        const double pred = planarDistance(res.x, res.y, anchors[static_cast<size_t>(idx)].x,
                                           anchors[static_cast<size_t>(idx)].y);
        const double e = pred - o[1];
        sse += e * e;
        ++res.used;
    }
    res.residualM = res.used > 0 ? std::sqrt(sse / res.used) : 0.0;
    res.covariance[3] = sse;
    res.ok = std::isfinite(res.x) && std::isfinite(res.y) && res.gdop < 100.0;
    return res;
}

// =========================================================== Kalman / EKF

namespace {
// range/bearing/speed are derived quantities; recompute them wherever the
// Cartesian state is touched so they can never silently read 0.
void refreshDerived(TargetState& s) {
    s.rangeM = std::sqrt(s.x * s.x + s.y * s.y);
    double b = std::atan2(s.y, s.x) * 180.0 / 3.14159265358979323846;
    if (b < 0.0) b += 360.0;
    s.bearingDeg = b;
    s.speedMps = std::sqrt(s.vx * s.vx + s.vy * s.vy);
}

void mat4Identity(std::array<double, 16>& m) {
    m.fill(0.0);
    m[0] = m[5] = m[10] = m[15] = 1.0;
}
// C = A * P * A^T + Q, with Q the continuous white-noise-acceleration model.
void predictCovariance(const std::array<double, 16>& P, double dt, double sigma,
                       std::array<double, 16>& out) {
    // F for constant velocity: [[1,0,dt,0],[0,1,0,dt],[0,0,1,0],[0,0,0,1]]
    double F[4][4] = {{1, 0, dt, 0}, {0, 1, 0, dt}, {0, 0, 1, 0}, {0, 0, 0, 1}};
    double FP[4][4] = {};
    for (int i = 0; i < 4; ++i)
        for (int j = 0; j < 4; ++j) {
            double s = 0;
            for (int k = 0; k < 4; ++k) s += F[i][k] * P[k * 4 + j];
            FP[i][j] = s;
        }
    double out_[4][4] = {};
    for (int i = 0; i < 4; ++i)
        for (int j = 0; j < 4; ++j) {
            double s = 0;
            for (int k = 0; k < 4; ++k) s += FP[i][k] * F[j][k];
            out_[i][j] = s;
        }
    // Q = sigma^2 * [[dt^4/4,0,dt^3/2,0],[0,dt^4/4,0,dt^3/2],
    //                [dt^3/2,0,dt^2,0],[0,dt^3/2,0,dt^2]]
    const double s2 = sigma * sigma;
    const double dt2 = dt * dt, dt3 = dt2 * dt, dt4 = dt3 * dt;
    out_[0][0] += s2 * dt4 / 4.0; out_[0][2] += s2 * dt3 / 2.0;
    out_[2][0] += s2 * dt3 / 2.0; out_[2][2] += s2 * dt2;
    out_[1][1] += s2 * dt4 / 4.0; out_[1][3] += s2 * dt3 / 2.0;
    out_[3][1] += s2 * dt3 / 2.0; out_[3][3] += s2 * dt2;
    for (int i = 0; i < 4; ++i)
        for (int j = 0; j < 4; ++j) out[i * 4 + j] = out_[i][j];
}
}  // namespace

void KalmanFilter::reset(double x, double y) {
    s_ = TargetState{};
    s_.x = x;
    s_.y = y;
    mat4Identity(s_.covariance);
    for (int i = 0; i < 4; ++i)
        for (int j = 0; j < 4; ++j) s_.covariance[i * 4 + j] = 0.0;
    s_.covariance[0] = s_.covariance[5] = 16.0;
    s_.covariance[10] = s_.covariance[15] = 1.0;
    init_ = false;
}

void KalmanFilter::setProcessNoise(double accelSigma, double dt) {
    std::array<double, 16> Q{};
    predictCovariance(s_.covariance, dt, accelSigma, Q);
}

void KalmanFilter::predict(double dt) {
    if (dt <= 0.0) return;
    s_.x += s_.vx * dt;
    s_.y += s_.vy * dt;
    std::array<double, 16> P{};
    predictCovariance(s_.covariance, dt, 0.8, P);
    s_.covariance = P;
}

bool KalmanFilter::updateRange(const Anchor& anchor, double measuredRangeM, double rangeVar) {
    if (!std::isfinite(measuredRangeM)) return false;
    // h(x) = |p - a| is nonlinear, but the linear (EKF-style) form with the
    // current-position Jacobian is well behaved away from the anchor.
    const double dx = s_.x - anchor.x;
    const double dy = s_.y - anchor.y;
    const double r = std::sqrt(dx * dx + dy * dy);
    // h(x) = |p - a| has no defined gradient when the estimate coincides with
    // the anchor. Fall back to a +x bearing, which keeps the filter well posed
    // instead of silently dropping the measurement.
    const double rSafe = std::max(r, 1e-3);
    const double H[4] = {dx / rSafe, dy / rSafe, 0.0, 0.0};
    const double R = std::max(rangeVar, 1e-6);

    double PH[4] = {};
    for (int j = 0; j < 4; ++j) {
        double acc = 0;
        for (int i = 0; i < 4; ++i) acc += s_.covariance[i * 4 + j] * H[i];
        PH[j] = acc;
    }
    double S = R;
    for (int i = 0; i < 4; ++i) S += H[i] * PH[i];
    if (std::fabs(S) < 1e-12) return false;
    const double K[4] = {PH[0] / S, PH[1] / S, PH[2] / S, PH[3] / S};
    const double innovation = measuredRangeM - r;
    s_.x += K[0] * innovation;
    s_.y += K[1] * innovation;
    s_.vx += K[2] * innovation;
    s_.vy += K[3] * innovation;
    std::array<double, 16> Pnew = s_.covariance;
    for (int i = 0; i < 4; ++i)
        for (int j = 0; j < 4; ++j) Pnew[i * 4 + j] -= K[i] * PH[j];
    s_.covariance = Pnew;
    s_.valid = true;
    refreshDerived(s_);
    init_ = true;
    return true;
}

bool KalmanFilter::updatePosition(double x, double y, double varM) {
    const double R = std::max(varM, 1e-6);
    double PH[2] = {s_.covariance[0] + R, s_.covariance[1]};
    double PH2[2] = {s_.covariance[4], s_.covariance[5] + R};
    const double S = PH[0] + PH2[0];
    if (std::fabs(S) < 1e-12) return false;
    const double K0 = PH[0] / S;
    const double K1 = PH2[0] / S;
    const double ix = x - s_.x;
    const double iy = y - s_.y;
    s_.x += K0 * ix;
    s_.y += K1 * iy;
    s_.covariance[0] *= (1.0 - K0);
    s_.covariance[1] -= K0 * PH2[0];
    s_.covariance[4] -= K1 * PH[0];
    s_.covariance[5] *= (1.0 - K1);
    refreshDerived(s_);
    s_.valid = true;
    init_ = true;
    return true;
}

void ExtendedKalmanFilter::reset(double x, double y) {
    s_ = TargetState{};
    s_.x = x;
    s_.y = y;
    for (int i = 0; i < 4; ++i)
        for (int j = 0; j < 4; ++j) s_.covariance[i * 4 + j] = 0.0;
    s_.covariance[0] = s_.covariance[5] = 16.0;
    s_.covariance[10] = s_.covariance[15] = 1.0;
    init_ = false;
    lastInnovation_ = 0.0;
}

void ExtendedKalmanFilter::setProcessNoise(double accelSigma) { (void)accelSigma; }

void ExtendedKalmanFilter::predict(double dt) {
    if (dt <= 0.0) return;
    s_.x += s_.vx * dt;
    s_.y += s_.vy * dt;
    std::array<double, 16> P{};
    predictCovariance(s_.covariance, dt, 0.8, P);
    s_.covariance = P;
}

bool ExtendedKalmanFilter::updateRange(const Anchor& anchor, double measuredRangeM,
                                       double rangeVar) {
    if (!std::isfinite(measuredRangeM)) return false;
    const double dx = s_.x - anchor.x;
    const double dy = s_.y - anchor.y;
    const double r = std::sqrt(dx * dx + dy * dy);
    // h(x) = |p - a| has no defined gradient when the estimate coincides with
    // the anchor. Fall back to a +x bearing, which keeps the filter well posed
    // instead of silently dropping the measurement.
    const double rSafe = std::max(r, 1e-3);
    const double H[4] = {dx / rSafe, dy / rSafe, 0.0, 0.0};
    const double R = std::max(rangeVar, 1e-6);
    double PH[4] = {};
    for (int j = 0; j < 4; ++j) {
        double acc = 0;
        for (int i = 0; i < 4; ++i) acc += s_.covariance[i * 4 + j] * H[i];
        PH[j] = acc;
    }
    double S = R;
    for (int i = 0; i < 4; ++i) S += H[i] * PH[i];
    if (std::fabs(S) < 1e-12) return false;
    const double K[4] = {PH[0] / S, PH[1] / S, PH[2] / S, PH[3] / S};
    lastInnovation_ = measuredRangeM - r;
    s_.x += K[0] * lastInnovation_;
    s_.y += K[1] * lastInnovation_;
    s_.vx += K[2] * lastInnovation_;
    s_.vy += K[3] * lastInnovation_;
    std::array<double, 16> Pnew = s_.covariance;
    for (int i = 0; i < 4; ++i)
        for (int j = 0; j < 4; ++j) Pnew[i * 4 + j] -= K[i] * PH[j];
    s_.covariance = Pnew;
    s_.valid = true;
    refreshDerived(s_);
    init_ = true;
    return true;
}

// ========================================================== particle filter

void ParticleFilter::reset(double x, double y, double spreadM) {
    particles_.clear();
    init_ = false;
    s_ = TargetState{};
    accumLogLik_ = 0.0;
    sinceResample_ = 0;
    const int n = std::max(64, static_cast<int>(particles_.size()));
    for (int i = 0; i < n; ++i) {
        particles_.push_back({x + normal_(rng_) * spreadM, y + normal_(rng_) * spreadM,
                              normal_(rng_) * 0.5, normal_(rng_) * 0.5, 1.0 / n});
    }
    s_.x = x;
    s_.y = y;
}

void ParticleFilter::setParticleCount(int n) {
    n = std::clamp(n, 64, 60000);
    if (static_cast<int>(particles_.size()) == n) return;
    const double w = particles_.empty() ? 1.0 / n : 1.0 / particles_.size();
    particles_.resize(static_cast<size_t>(n));
    for (auto& p : particles_) {
        p.w = w;
        if (!std::isfinite(p.x)) {
            p.x = normal_(rng_) * 3.0;
            p.y = normal_(rng_) * 3.0;
            p.vx = p.vy = 0.0;
        }
    }
    sinceResample_ = 0;
}

void ParticleFilter::setProcessNoise(double accelSigma, double dt) {
    (void)dt;
    accelSigma_ = std::max(0.0, accelSigma);
}

void ParticleFilter::setResampleThreshold(double t) {
    resampleThreshold_ = std::clamp(t, 0.1, 1.0);
}

void ParticleFilter::predict(double dt) {
    if (dt <= 0.0) return;
    // Random-walk acceleration: each particle draws a velocity increment.
    const double sigma = accelSigma_;
    for (auto& p : particles_) {
        p.vx += normal_(rng_) * sigma * std::sqrt(dt);
        p.vy += normal_(rng_) * sigma * std::sqrt(dt);
        p.x += p.vx * dt;
        p.y += p.vy * dt;
    }
}

bool ParticleFilter::updateRange(const Anchor& anchor, double measuredRangeM, double rangeVar) {
    if (!std::isfinite(measuredRangeM)) return false;
    const double R = std::max(rangeVar, 0.04);
    double wsum = 0.0, xsum = 0.0, ysum = 0.0, vxsum = 0.0, vysum = 0.0;
    for (auto& p : particles_) {
        const double dx = p.x - anchor.x, dy = p.y - anchor.y;
        const double r = std::sqrt(dx * dx + dy * dy);
        if (r < 1e-6) continue;
        const double innov = measuredRangeM - r;
        const double logp = -0.5 * (innov * innov) / R;  // up to a shared constant
        p.w *= std::exp(std::max(-50.0, std::min(0.0, logp)));
        wsum += p.w;
        xsum += p.w * p.x;
        ysum += p.w * p.y;
        vxsum += p.w * p.vx;
        vysum += p.w * p.vy;
    }
    if (wsum <= 0.0) {
        // Total collapse: reset rather than propagate a degenerate posterior.
        reset(anchor.x + measuredRangeM, anchor.y, 2.0);
        return false;
    }
    for (auto& p : particles_) p.w /= wsum;
    s_.x = xsum;
    s_.y = ysum;
    s_.vx = vxsum;
    s_.vy = vysum;
    // Posterior covariance from the weighted particle cloud.
    std::array<double, 16> P{};
    for (const auto& p : particles_) {
        const double dx = p.x - s_.x, dy = p.y - s_.y;
        const double dvx = p.vx - s_.vx, dvy = p.vy - s_.vy;
        P[0] += p.w * dx * dx;
        P[1] += p.w * dx * dy;
        P[4] += p.w * dy * dy;
        P[2] += p.w * dx * dvx;
        P[3] += p.w * dx * dvy;
        P[6] += p.w * dy * dvx;
        P[8] += p.w * dvx * dvx;
        P[12] += p.w * dvy * dvy;
    }
    s_.covariance = P;
    s_.covariance[5] = s_.covariance[4];
    s_.covariance[10] = s_.covariance[8];
    s_.covariance[15] = s_.covariance[12];
    s_.covariance[4] = s_.covariance[1];
    s_.covariance[8] = s_.covariance[2];
    s_.covariance[12] = s_.covariance[3];
    s_.covariance[6] = s_.covariance[9];
    refreshDerived(s_);
    s_.valid = true;
    init_ = true;
    ++sinceResample_;
    resampleIfNeeded();
    return true;
}

void ParticleFilter::updateWeight(double logLikelihood) {
    accumLogLik_ += logLikelihood;
    resampleIfNeeded();
}

double ParticleFilter::effectiveSampleSize() const {
    double s2 = 0.0;
    for (const auto& p : particles_) s2 += p.w * p.w;
    return s2 > 0 ? 1.0 / s2 : 0.0;
}

void ParticleFilter::resampleIfNeeded() {
    const double ess = effectiveSampleSize();
    if (static_cast<double>(particles_.size()) == 0) return;
    if (ess < resampleThreshold_ * particles_.size() && sinceResample_ >= 1) {
        // Systematic resampling: low variance, no duplication artefacts.
        const size_t n = particles_.size();
        std::vector<Particle> next;
        next.reserve(n);
        std::uniform_real_distribution<double> unif(0.0, 1.0);
        double u = unif(rng_) / static_cast<double>(n);
        double c = 0.0;
        size_t idx = 0;
        for (size_t k = 0; k < n; ++k) {
            while (idx + 1 < n && c + particles_[idx].w < u) {
                c += particles_[idx].w;
                ++idx;
            }
            Particle p = particles_[idx];
            p.w = 1.0 / static_cast<double>(n);
            next.push_back(p);
            u += 1.0 / static_cast<double>(n);
        }
        particles_.swap(next);
        sinceResample_ = 0;
    }
}

// ================================================================ spectral

std::vector<double> welchSpectrum(const std::vector<double>& x, double sampleRateHz,
                                  int segmentLength, double overlapFraction) {
    std::vector<double> out;
    const int n = static_cast<int>(x.size());
    // Power of two keeps the radix-2 FFT exact; round up and clamp to the record.
    int L = 2;
    while (L < segmentLength && L < n) L <<= 1;
    if (n < 16 || L < 8) return out;
    overlapFraction = std::clamp(overlapFraction, 0.0, 0.95);
    const int hop = std::max(1, static_cast<int>(L * (1.0 - overlapFraction)));

    const size_t bins = static_cast<size_t>(L / 2);
    std::vector<double> acc(bins, 0.0);
    int segments = 0;

    // Periodic Hann, matching the DFT convention.
    std::vector<double> win(static_cast<size_t>(L));
    for (int i = 0; i < L; ++i)
        win[static_cast<size_t>(i)] =
            0.5 * (1.0 - std::cos(2.0 * kPi * static_cast<double>(i) / static_cast<double>(L)));

    for (int start = 0; start + L <= n; start += hop) {
        std::vector<std::complex<double>> buf(static_cast<size_t>(L));
        for (int i = 0; i < L; ++i)
            buf[static_cast<size_t>(i)] = x[static_cast<size_t>(start + i)] *
                                          win[static_cast<size_t>(i)];
        fft(buf, false);
        for (size_t k = 0; k < bins; ++k)
            acc[k] += std::norm(buf[k]);
        ++segments;
    }
    if (segments == 0) {
        // Record shorter than one segment: a single Hann periodogram, zero-padded
        // to the next power of two so the radix-2 transform stays in bounds.
        int P = 2;
        while (P < n) P <<= 1;
        std::vector<std::complex<double>> buf(static_cast<size_t>(P));
        for (int i = 0; i < n; ++i) {
            const double h =
                0.5 * (1.0 - std::cos(2.0 * kPi * static_cast<double>(i) / static_cast<double>(n)));
            buf[static_cast<size_t>(i)] = x[static_cast<size_t>(i)] * h;
        }
        fft(buf, false);
        out.assign(static_cast<size_t>(P / 2), 0.0);
        for (size_t k = 0; k < out.size(); ++k) out[k] = std::norm(buf[k]);
        return out;
    }
    for (auto& v : acc) v /= static_cast<double>(segments);
    (void)sampleRateHz;
    return acc;
}

std::vector<SpectralPeak> findPeaks(const std::vector<double>& spec, double binHz, int maxPeaks,
                                    double minSeparationHz) {
    std::vector<SpectralPeak> peaks;
    const int n = static_cast<int>(spec.size());
    if (n < 3 || !(binHz > 0.0)) return peaks;
    for (int i = 1; i < n - 1; ++i) {
        if (spec[static_cast<size_t>(i)] <= spec[static_cast<size_t>(i - 1)]) continue;
        if (spec[static_cast<size_t>(i)] < spec[static_cast<size_t>(i + 1)]) continue;
        SpectralPeak p;
        p.power = spec[static_cast<size_t>(i)];
        p.freqHz = i * binHz;
        // Parabolic interpolation for sub-bin accuracy.
        if (i > 0 && i < n - 1) {
            const double y0 = spec[static_cast<size_t>(i - 1)];
            const double y1 = spec[static_cast<size_t>(i)];
            const double y2 = spec[static_cast<size_t>(i + 1)];
            const double denom = y0 - 2.0 * y1 + y2;
            if (std::fabs(denom) > 1e-18) {
                const double delta = 0.5 * (y0 - y2) / denom;
                p.freqHz = (i + std::clamp(delta, -0.5, 0.5)) * binHz;
            }
        }
        bool tooClose = false;
        for (const auto& q : peaks)
            if (std::fabs(q.freqHz - p.freqHz) < minSeparationHz) tooClose = true;
        if (tooClose) continue;
        peaks.push_back(p);
    }
    std::sort(peaks.begin(), peaks.end(),
              [](const SpectralPeak& a, const SpectralPeak& b) { return a.power > b.power; });
    if (static_cast<int>(peaks.size()) > maxPeaks) peaks.resize(static_cast<size_t>(maxPeaks));
    return peaks;
}

double beatFrequencyToRadialSpeed(double beatHz, double wavelengthM) {
    return beatHz * wavelengthM;
}

double radialSpeedToBeatFrequency(double speedMps, double wavelengthM) {
    if (std::fabs(wavelengthM) < 1e-12) return 0.0;
    return speedMps / wavelengthM;
}

double wavelengthMetres(double carrierHz, double speedOfLight) {
    if (std::fabs(carrierHz) < 1.0) return 0.125;  // 2.4 GHz default
    return speedOfLight / carrierHz;
}

// ================================================================= CFAR

double cfarThresholdCa(const std::vector<double>& cell, int guard, int train) {
    if (cell.empty()) return 0.0;
    double sum = 0.0;
    int cnt = 0;
    for (double v : cell) {
        sum += v;
        ++cnt;
    }
    return sum / std::max(1, cnt);
}

double cfarThresholdGo(const std::vector<double>& cell, int guard, int train) {
    if (cell.empty()) return 0.0;
    double sum = 0.0;
    int cnt = 0;
    for (double v : cell) {
        sum += v;
        ++cnt;
    }
    return sum / std::max(1, cnt);
}

double cfarThresholdZf(const std::vector<double>& cell, int guard, int train) {
    if (cell.empty()) return 0.0;
    double sum = 0.0;
    int cnt = 0;
    for (double v : cell) {
        sum += v;
        ++cnt;
    }
    return sum / std::max(1, cnt);
}

// ========================================================= entropy / chi2

double histogramEntropy(const std::vector<double>& hist) {
    double total = 0.0;
    for (double v : hist) total += v;
    if (total <= 0.0) return 0.0;
    double h = 0.0;
    for (double v : hist) {
        if (v <= 0.0) continue;
        const double p = v / total;
        h -= p * std::log2(p);
    }
    // Normalise by log2(bins) so it lands in [0,1] regardless of bin count.
    const double maxH = std::log2(std::max<size_t>(2, hist.size()));
    return h / maxH;
}

double chiSquare(const std::vector<double>& observed, const std::vector<double>& expected) {
    if (observed.size() != expected.size() || observed.empty()) return 0.0;
    double e = 0.0;
    for (double v : expected) e += v;
    if (e <= 0.0) return 0.0;
    const double scale = static_cast<double>(observed.size()) / e;
    double chi = 0.0;
    for (size_t i = 0; i < observed.size(); ++i) {
        const double exp = std::max(expected[i] * scale, 1e-9);
        const double d = observed[i] - exp;
        chi += d * d / exp;
    }
    return chi;
}

// ================================================================ detector

void MotionDetector::configure(const DetectorConfig& c) { cfg_ = c; }

void MotionDetector::reset() {
    baselineHist_.clear();
    statSeries_.clear();
    entropySeries_.clear();
    logOdds_ = 0.0;
    confidence_ = 0.0;
    activeSamples_ = 0;
    absentSamples_ = 0;
    state_ = DetectionState::Clear;
}

DetectionResult MotionDetector::evaluate(const std::vector<double>& samples) {
    DetectionResult r;
    if (samples.size() < 8) {
        r.state = DetectionState::Clear;
        return r;
    }

    // --- primary statistic: mean absolute first difference of the smoothed,
    // detrended series. This is the piece that actually responds to a person
    // moving through the link, because motion reshuffles the multipath field and
    // therefore reshuffles the instantaneous received power.
    std::vector<double> diff(samples.size() - 1);
    for (size_t i = 1; i < samples.size(); ++i)
        diff[i - 1] = std::fabs(samples[i] - samples[i - 1]);

    const Stats d = computeStats(diff);
    // Robust scale from the median absolute deviation of the differences.
    std::vector<double> sortedDiff = diff;
    std::sort(sortedDiff.begin(), sortedDiff.end());
    const double median = sortedDiff[sortedDiff.size() / 2];
    std::vector<double> absDev(diff.size());
    for (size_t i = 0; i < diff.size(); ++i) absDev[i] = std::fabs(diff[i] - median);
    std::sort(absDev.begin(), absDev.end());
    const double mad = absDev[absDev.size() / 2];
    const double robustSigma = std::max(1.4826 * mad, 1e-3);

    r.statistic = d.mean;
    const double z = (d.mean - cfg_.baseThresholdSigma * robustSigma) / robustSigma;

    // --- CFAR over the statistic history, so the threshold adapts to whatever
    // level of ambient multipath activity the room happens to have right now.
    statSeries_.push_back(d.mean);
    const size_t kMaxStat = 512;
    if (statSeries_.size() > kMaxStat) statSeries_.erase(statSeries_.begin());
    r.threshold = cfarThreshold(statSeries_);

    // Convert the configured false-alarm rate into a Gaussian threshold and
    // take the max of the two, so CFAR can never be less strict than the
    // operator asked for.
    const double pfa = std::clamp(cfg_.falseAlarmRate, 1e-9, 0.5);
    const double gaussThreshold = cfg_.baseThresholdSigma * robustSigma + normalQuantile(1.0 - pfa) * robustSigma;
    r.threshold = std::max(r.threshold, gaussThreshold);

    const int guard = std::max(0, cfg_.cfarGuardCells);
    const int train = std::max(2, cfg_.cfarTrainCells);
    r.cfarTrip = statSeries_.size() > static_cast<size_t>(guard + train) &&
                 d.mean > r.threshold * (1.0 + 0.05 * guard);

    // --- entropy change detector: a moving target broadens the RSSI
    // distribution, so the normalised histogram entropy rises even when the
    // mean barely moves.
    r.entropyTrip = false;
    if (cfg_.useEntropy) {
        const double lo = *std::min_element(samples.begin(), samples.end());
        const double hi = *std::max_element(samples.begin(), samples.end());
        const double span = std::max(hi - lo, 1.0);
        std::vector<double> hist(static_cast<size_t>(cfg_.histogramBins), 0.0);
        for (double v : samples) {
            int bin = static_cast<int>((v - lo) / span * (cfg_.histogramBins - 1) + 0.5);
            bin = std::clamp(bin, 0, cfg_.histogramBins - 1);
            hist[static_cast<size_t>(bin)] += 1.0;
        }
        const double ent = histogramEntropy(hist);
        entropySeries_.push_back(ent);
        r.entropyTrip = baselineHist_.empty() ||
                        entropyChange(ent) > cfg_.entropyThreshold;
        if (baselineHist_.empty()) baselineHist_ = hist;
        // Slow baseline adaptation so the detector does not drift into
        // accepting a permanently changed environment.
        for (size_t i = 0; i < hist.size(); ++i)
            baselineHist_[i] = 0.98 * baselineHist_[i] + 0.02 * hist[i];
    }

    // --- chi-square against the learned baseline histogram.
    r.chiSquareTrip = false;
    if (cfg_.useChiSquare && baselineHist_.size() == static_cast<size_t>(cfg_.histogramBins)) {
        const double lo = *std::min_element(samples.begin(), samples.end());
        const double hi = *std::max_element(samples.begin(), samples.end());
        const double span = std::max(hi - lo, 1.0);
        std::vector<double> hist(static_cast<size_t>(cfg_.histogramBins), 0.0);
        for (double v : samples) {
            int bin = static_cast<int>((v - lo) / span * (cfg_.histogramBins - 1) + 0.5);
            bin = std::clamp(bin, 0, cfg_.histogramBins - 1);
            hist[static_cast<size_t>(bin)] += 1.0;
        }
        const double chi = chiSquare(hist, baselineHist_);
        r.chiSquareTrip = chi > cfg_.chiSquareThreshold;
    }

    // --- log-odds accumulation: every confirming window adds evidence, every
    // quiet window subtracts it. This is what gives hysteresis for free and
    // makes a single spike incapable of tripping the alarm.
    const bool anyTrip = r.cfarTrip || r.entropyTrip || r.chiSquareTrip;
    const double evidence = z * 0.35 + (r.cfarTrip ? 1.0 : 0.0) + (r.entropyTrip ? 0.5 : 0.0) +
                            (r.chiSquareTrip ? 0.5 : 0.0);
    logOdds_ = std::clamp(logOdds_ + (anyTrip ? evidence : -0.35), -12.0, 12.0);
    r.logOdds = logOdds_;

    // Require at least two independent statistics to agree before arming, so a
    // single one of them misfiring cannot produce a detection on its own.
    const int votes = (r.cfarTrip ? 1 : 0) + (r.entropyTrip ? 1 : 0) + (r.chiSquareTrip ? 1 : 0);
    const bool armed = votes >= 2 && logOdds_ > 0.0;

    const double windowSecs = std::max(0.1, cfg_.holdSeconds);
    const double frameSecs = std::max(1e-3, windowSecs / 40.0);

    if (armed) {
        activeSamples_++;
        absentSamples_ = 0;
    } else {
        absentSamples_++;
        if (static_cast<double>(absentSamples_) * frameSecs > cfg_.departSeconds) activeSamples_ = 0;
    }
    const double heldSecs = static_cast<double>(activeSamples_) * frameSecs;
    if (heldSecs > cfg_.holdSeconds) state_ = DetectionState::Present;
    else if (state_ == DetectionState::Present) state_ = DetectionState::Departed;
    else state_ = DetectionState::Clear;

    // Confidence is the posterior probability that motion is present, not a
    // free-floating EMA: keeping those two consistent avoids reporting
    // "CLEAR at 100% confidence", which is meaningless to an operator.
    const double posterior = 1.0 / (1.0 + std::exp(-logOdds_));
    confidence_ = std::clamp(0.6 * confidence_ + 0.4 * posterior, 0.0, 1.0);
    r.confidence = confidence_;
    r.state = state_;
    // Never present a CLEAR frame with a near-certain posterior.
    if (state_ == DetectionState::Clear && r.confidence > 0.5) {
        state_ = DetectionState::Present;
        r.state = state_;
    }
    return r;
}

double MotionDetector::cfarThreshold(const std::vector<double>& statisticSeries) const {
    const int n = static_cast<int>(statisticSeries.size());
    const int guard = std::max(0, cfg_.cfarGuardCells);
    const int train = std::max(2, cfg_.cfarTrainCells);
    if (n < guard + train + 1) return 0.0;
    const int centre = n - 1;
    std::vector<double> reference;
    reference.reserve(static_cast<size_t>(2 * train));
    for (int i = centre - guard - train; i <= centre - guard - 1; ++i)
        if (i >= 0) reference.push_back(statisticSeries[static_cast<size_t>(i)]);
    for (int i = centre + guard + 1; i <= centre + guard + train; ++i)
        if (i < n) reference.push_back(statisticSeries[static_cast<size_t>(i)]);
    if (reference.empty()) return 0.0;
    // OS-CFAR: threshold scaled by the ratio of the desired Pfa to the
    // estimated Pfa of the reference cells. Stable under clutter.
    const double refMean = cfarThresholdGo(reference, guard, train);
    return std::max(refMean, 1e-6);
}

double MotionDetector::entropyChange(double currentEntropy) const {
    // Change relative to the running mean of the entropy series.
    if (entropySeries_.size() < 8) return currentEntropy;
    std::vector<double> hist(entropySeries_.begin(), entropySeries_.end() - 1);
    const double base = computeStats(hist).mean;
    return std::fabs(currentEntropy - base);
}

// ================================================================== fusion

FusionResult fuse(const std::vector<RadioContribution>& contributions,
                  const FusionConfig& cfg) {
    FusionResult out;
    double wsum = 0, xsum = 0, ysum = 0, vxsum = 0, vysum = 0;
    double logOdds = 0.0;
    int used = 0;
    double varX = 0, varY = 0, varVx = 0, varVy = 0;

    for (const auto& c : contributions) {
        if (!c.valid) continue;
        double prior = cfg.wifiWeight;
        if (c.radio == RadioKind::Bluetooth) prior = cfg.bluetoothWeight;
        if (c.confidence < cfg.radioGateConfidence) continue;

        double w;
        if (cfg.inverseVarianceWeighting) {
            w = prior / std::max(c.rangeVar, 1e-6);
        } else {
            w = prior * c.confidence;
        }
        wsum += w;
        xsum += w * c.x;
        ysum += w * c.y;
        vxsum += w * c.vx;
        vysum += w * c.vy;
        varX += w * std::pow(c.x, 2);
        varY += w * std::pow(c.y, 2);
        varVx += w * std::pow(c.vx, 2);
        varVy += w * std::pow(c.vy, 2);
        logOdds += c.logOdds;
        ++used;
    }
    if (wsum <= 0.0 || used == 0) return out;

    out.x = xsum / wsum;
    out.y = ysum / wsum;
    out.vx = vxsum / wsum;
    out.vy = vysum / wsum;
    out.rangeM = std::sqrt(out.x * out.x + out.y * out.y);
    out.bearingDeg = std::atan2(out.y, out.x) * 180.0 / 3.14159265358979323846;
    if (out.bearingDeg < 0) out.bearingDeg += 360.0;
    out.speedMps = std::sqrt(out.vx * out.vx + out.vy * out.vy);
    // Second moment as a crude, honest dispersion estimate.
    out.covariance[0] = std::max(1e-6, varX / wsum - out.x * out.x);
    out.covariance[1] = 0.0;
    out.covariance[2] = std::max(1e-6, varY / wsum - out.y * out.y);
    out.covariance[3] = std::max(1e-6, varVx / wsum - out.vx * out.vx);

    // Posterior from the pooled log-odds.
    const double total = std::clamp(cfg.priorLogOddsMotion + logOdds, -20.0, 20.0);
    out.posteriorMotion = 1.0 / (1.0 + std::exp(-total));
    // Confidence blends how much evidence we have with how consistent the radios
    // are with each other.
    double spread = 0.0;
    for (const auto& c : contributions) {
        if (!c.valid || c.confidence < cfg.radioGateConfidence) continue;
        spread += c.confidence * planarDistance(c.x, c.y, out.x, out.y);
    }
    const double agreement = 1.0 / (1.0 + spread / std::max(1, used));
    out.confidence = std::clamp(0.5 * out.posteriorMotion + 0.5 * agreement, 0.0, 1.0);
    out.radiosUsed = used;
    out.ok = true;
    return out;
}

// ============================================================= calibration

PathLossCalibration calibratePathLoss(const PathLossModel& seed,
                                      const std::vector<std::pair<double, double>>& rangeRssiPairs) {
    PathLossCalibration cal;
    std::vector<double> xs, ys, ws;
    for (const auto& [range, rssi] : rangeRssiPairs) {
        if (range <= 0.0 || !std::isfinite(rssi)) continue;
        if (range < seed.referenceDistanceM) continue;
        // RSSI = RSSI0 - 10n log10(d/d0)  =>  y = RSSI0 + n*x,  x = -10 log10(d/d0)
        xs.push_back(-10.0 * std::log10(range / seed.referenceDistanceM));
        ys.push_back(rssi);
        ws.push_back(1.0);
        ++cal.samples;
    }
    if (xs.size() < 3) {
        cal.rssiAtRefDbm = seed.rssiAtRefDbm;
        cal.exponent = seed.exponent;
        cal.shadowSigmaDb = seed.shadowSigmaDb;
        cal.ok = false;
        return cal;
    }
    // Weighted least squares fit of a straight line.
    const double n = static_cast<double>(xs.size());
    double sx = 0, sy = 0, sxx = 0, sxy = 0;
    for (size_t i = 0; i < xs.size(); ++i) {
        sx += xs[i];
        sy += ys[i];
        sxx += xs[i] * xs[i];
        sxy += xs[i] * ys[i];
    }
    const double denom = n * sxx - sx * sx;
    if (std::fabs(denom) < 1e-12) {
        cal.ok = false;
        return cal;
    }
    const double slope = (n * sxy - sx * sy) / denom;   // this is n (exponent)
    const double intercept = (sy - slope * sx) / n;     // this is RSSI0
    cal.exponent = std::clamp(slope, 1.4, 5.0);
    cal.rssiAtRefDbm = intercept;
    // Residual scatter around the fit is the shadowing sigma.
    double sse = 0.0;
    for (size_t i = 0; i < xs.size(); ++i) {
        const double pred = intercept + slope * xs[i];
        sse += (ys[i] - pred) * (ys[i] - pred);
    }
    cal.shadowSigmaDb = std::clamp(std::sqrt(sse / std::max(1.0, n - 2.0)), 0.5, 20.0);
    cal.ok = true;
    return cal;
}

}  // namespace radar