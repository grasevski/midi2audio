#pragma once
enum { SAMPLE_SIZE = 128 };

/// Calculates the fundamental frequency for the given audio slice.
float fastf0nls(const float x[SAMPLE_SIZE]);
