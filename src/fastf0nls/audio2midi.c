#include "midiguitar.h"
#include <arpa/inet.h>
#include <string.h>
#include <unistd.h>

/// Converts the signed pcm value to a uint32_t.
static uint32_t parse(const uint8_t x[]) {
  uint32_t ret =
      (uint32_t)x[0] | ((uint32_t)x[1] << 8) | ((uint32_t)x[2] << 16);
  if (ret & 0x800000)
    ret |= 0xff000000;
  return (int32_t)ret + OFFSET;
}

/// Writes midi to the given buffer along with a timestamp.
static int8_t audio2midi(uint8_t buf[], int32_t len, int32_t dt,
                         const uint8_t output[MIDI_CAP], int8_t n) {
  if (!n)
    return 0;
  uint8_t t[4];
  t[0] = 0;
  int8_t k;
  for (k = 0; k < sizeof(t) && dt; ++k) {
    t[k] = dt & 0x7f;
    if (dt >= 0x80)
      t[k] |= 0x80;
    dt >>= 7;
  }
  if (dt || k + n > len)
    return -1;
  else if (!k)
    k = 1;
  for (int8_t i = k - 1; i >= 0; --i)
    *buf++ = t[i];
  memcpy(buf, output, 3);
  buf += 3;
  output += 3;
  n /= 3;
  for (int8_t i = 1; i < n; ++i) {
    *buf++ = 0;
    memcpy(buf, output, 3);
    buf += 3;
    output += 3;
  }
  return (n << 2) + k - 1;
}

int main() {
  int16_t n;
  int32_t k = 0, len = 0, dt = 0;
  struct midiguitar mg = {};
  uint32_t input[AUDIO_CAP];
  uint8_t inbuf[3 * AUDIO_CAP], output[MIDI_CAP];
  static uint8_t buf[0x400000];
  for (;;) {
    n = read(0, inbuf + k, sizeof(inbuf) - k);
    if (n <= 0) {
      if (k < sizeof(inbuf))
        break;
      for (k = 0; k < AUDIO_CAP; ++k)
        input[k] = parse(inbuf + 3 * k);
      n = midiguitar(&mg, input, 0, output);
      n = audio2midi(buf + len, sizeof(buf) - len, dt, output, n);
      if (n < 0)
        return n;
      else if (n) {
        len += n;
        dt = 0;
      }
      ++dt;
      break;
    }
    k += n;
    if (k < sizeof(inbuf))
      continue;
    for (k = 0; k < AUDIO_CAP; ++k)
      input[k] = parse(inbuf + 3 * k);
    n = midiguitar(&mg, input, 0, output);
    n = audio2midi(buf + len, sizeof(buf) - len, dt, output, n);
    if (n < 0)
      return n;
    else if (n) {
      len += n;
      dt = 0;
    }
    ++dt;
    k = 0;
  }
  output[0] = 0xff;
  output[1] = 0x2f;
  output[2] = 0x00;
  n = audio2midi(buf + len, sizeof(buf) - len, dt, output, 3);
  if (n < 0)
    return -1;
  len += n;
  const uint8_t HEADER[18] = {
      'M', 'T', 'h', 'd', 0, 0, 0, 6, 0, 0, 0, 1, 0, 160, 'M', 'T', 'r', 'k',
  };
  int32_t r;
  for (k = 0; k < sizeof(HEADER); k += r) {
    r = write(1, HEADER + k, sizeof(HEADER) - k);
    if (r < 0)
      return -1;
  }
  const uint32_t s = htonl(len);
  for (k = 0; k < sizeof(s); k += r) {
    r = write(1, (const uint8_t *)&s + k, sizeof(s) - k);
    if (r < 0)
      return -1;
  }
  for (k = 0; k < len; k += r) {
    r = write(1, buf + k, len - k);
    if (r < 0)
      return -1;
  }
  return 0;
}
