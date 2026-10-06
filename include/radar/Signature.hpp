// Signal signature: a comparable fingerprint of what the channel is doing.
//
// The numbers an anomaly detector computes are usually thrown away the moment
// they have served their purpose. This keeps them, and makes two things possible
// that are otherwise missing:
//
//   * The operator can *see* why something was flagged, instead of being told
//     "anomaly" and having to take it on trust.
//   * Two events can be compared. Recording a signature and matching a later one
//     against it answers a question the log cannot: "was this the same thing
//     happening again", which is the question that matters when deciding whether
//     a room is occupied by the same person or a different one.
//
// The signature is not a fingerprint of a person. It is a fingerprint of a
// *channel state*, and the same signature can be produced by a person, a pet, a
// curtain, a fan or an access point changing transmit power. Any match reported
// here is a similarity in the radio environment, and is labelled as such.
#pragma once

#include <cstdint>
#include <deque>
#include <string>
#include <vector>

namespace radar {

// Fixed-length feature vector. The order is part of the on-disk format and must
// not be reordered without bumping the version below.
inline constexpr size_t kSignatureDim = 8;
inline constexpr int kSignatureVersion = 1;

struct Signature {
    double v[kSignatureDim] = {0, 0, 0, 0, 0, 0, 0, 0};
    double wallTime = 0.0;
    std::string label;

    // Normalised Euclidean distance to another signature, 0..1, where 0 is
    // identical and 1 is as far apart as the feature ranges allow.
    double distanceTo(const Signature& o) const;
    bool valid() const;
};

const char* signatureFeatureName(size_t i);

class SignatureStore {
  public:
    void configure(size_t capacity) { capacity_ = capacity; }
    size_t size() const { return entries_.size(); }
    const Signature& at(size_t i) const { return entries_[static_cast<size_t>(i)]; }
    void clear() { entries_.clear(); }

    // Store one, dropping the oldest once full.
    void add(const Signature& s);
    // Closest stored signature and its distance. Returns false when empty.
    bool nearest(const Signature& s, Signature& match, double& distance) const;

  private:
    size_t capacity_ = 32;
    std::deque<Signature> entries_;
};

}  // namespace radar