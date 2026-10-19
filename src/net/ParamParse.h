#pragma once
#include <Arduino.h>

// Entier decimal STRICT dans [lo, hi] : "" , "abc", "12x", "99999999" et hors bornes -> false.
// Logique pure (testee sur PC, voir test/host).
inline bool parseIntStrict(const String& v, int lo, int hi, int& out) {
  const size_t n = v.length();
  size_t i = (n > 0 && v[0] == '-') ? 1 : 0;
  if (i >= n || n > 7) return false;
  for (size_t k = i; k < n; ++k) if (v[k] < '0' || v[k] > '9') return false;
  const long x = v.toInt();
  if (x < lo || x > hi) return false;
  out = static_cast<int>(x);
  return true;
}
