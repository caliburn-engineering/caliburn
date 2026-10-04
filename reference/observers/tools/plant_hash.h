#pragma once
#include <cstdint>
#include <cstdio>

// DJB2-64 string hash
static inline uint64_t djb2_64(const char* s) {
    uint64_t hash = 5381;
    int c;
    while ((c = (unsigned char)*s++))
        hash = ((hash << 5) + hash) ^ static_cast<uint64_t>(c);
    return hash;
}

// Plant hash for the Kalman DARE fixture: double-mass-spring-damper with
// noise parameters.  Both the exporter and the staleness guard call this
// function so that one definition covers both.
static inline uint64_t kalman_dare_dmsd_plant_hash(
        double m1, double m2,
        double k1, double k2,
        double c1, double c2,
        double dt,
        double q_scale, double r_scale) {
    char buf[512];
    std::snprintf(buf, sizeof(buf),
        "kalman-dare-dmsd:"
        "m1=%.17g,m2=%.17g,"
        "k1=%.17g,k2=%.17g,"
        "c1=%.17g,c2=%.17g,"
        "dt=%.17g,"
        "q=%.17g*I4,r=%.17g*I2",
        m1, m2, k1, k2, c1, c2, dt, q_scale, r_scale);
    return djb2_64(buf);
}
