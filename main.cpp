// Car wing light - ESP32 + FastLED
// 288x WS2812 on GPIO12
// Inputs (INPUT_PULLUP, bridge the pin to GND to activate):
//   GPIO27 brake | GPIO26 right blinker | GPIO25 left blinker | GPIO33 reverse

#include <Arduino.h>
#include <FastLED.h>

// ===================== Hardware =====================
#define LED_PIN        12
#define NUM_LEDS       288
#define COLOR_ORDER    GRB      // WS2812/WS2812B. If red shows up green, try RGB.
#define MAX_MILLIAMPS  8000     // Set this to what your 5V supply can really deliver

#define PIN_BRAKE      27
#define PIN_RIGHT      26
#define PIN_LEFT       25
#define PIN_REVERSE    33

#define SWAP_SIDES      false   // true if left/right blinkers come out mirrored
#define BLINK_OFF_DARK  false   // true = blinking half goes dark between flashes

// ===================== Look & timing =====================
const float    IDLE_LEVEL      = 0.75f;   // steady red
const float    BRAKE_LEVEL     = 1.00f;   // brake red
const CRGB     BLINK_COLOR     = CRGB(255, 100, 0);

const uint32_t FRAME_MS        = 10;      // ~100 fps target
const uint32_t DEBOUNCE_MS     = 30;

// Startup animation
const uint32_t ROLL_MS         = 1400;    // heads run from both ends to the middle
const uint32_t BOOM_MS         = 1300;    // red explosion fills the strip
const int      HEAD_LEN        = 4;       // LEDs in each head
const uint8_t  TAIL_FADE       = 70;      // higher = shorter tail
const float    BOOM_FRONT_W    = 18.0f;   // width of the bright wave front (LEDs)

// Blinker (sequential, middle -> outwards)
const uint32_t BLINK_PERIOD_MS = 800;
const uint32_t BLINK_SWEEP_MS  = 380;
const uint32_t BLINK_HOLD_MS   = 220;     // the rest of the period is the "off" part

// Level / colour fades (seconds for a full 0..1 change)
const float    LEVEL_UP_S      = 0.25f;
const float    LEVEL_DOWN_S    = 0.80f;
const float    WHITE_FADE_S    = 1.20f;   // red <-> white when reversing

// ===================== State =====================
static const int HALF = NUM_LEDS / 2;
static CRGB leds[NUM_LEDS];

enum { B_BRAKE, B_RIGHT, B_LEFT, B_REVERSE, B_COUNT };

struct Button {
  uint8_t  pin;
  bool     stable;     // debounced state, true = pressed (pin pulled to GND)
  bool     lastRaw;
  uint32_t changedAt;
};

static Button btn[B_COUNT] = {
  { PIN_BRAKE,   false, false, 0 },
  { PIN_RIGHT,   false, false, 0 },
  { PIN_LEFT,    false, false, 0 },
  { PIN_REVERSE, false, false, 0 },
};

static uint32_t lastFrame     = 0;
static bool     startupBegun  = false;
static bool     startupDone   = false;
static uint32_t startT0       = 0;
static int      rollPrev      = -1;

static float    level         = IDLE_LEVEL;
static float    whiteMix      = 0.0f;
static bool     blinkWasOn    = false;
static uint32_t blinkStart    = 0;

// ===================== Helpers =====================
static inline uint8_t toByte(float v) {
  if (v < 0.0f) v = 0.0f;
  if (v > 1.0f) v = 1.0f;
  return (uint8_t)(v * 255.0f + 0.5f);
}

static inline float stepToward(float cur, float target, float step) {
  if (cur < target) { cur += step; return cur > target ? target : cur; }
  cur -= step;
  return cur < target ? target : cur;
}

static inline float outCubic(float u) {
  float v = 1.0f - u;
  return 1.0f - v * v * v;
}

static inline float smoothCurve(float x) { return x * x * (3.0f - 2.0f * x); }

static void readInputs(uint32_t now) {
  for (int i = 0; i < B_COUNT; i++) {
    Button &b = btn[i];
    bool raw = (digitalRead(b.pin) == LOW);
    if (raw != b.lastRaw) { b.lastRaw = raw; b.changedAt = now; }
    if (now - b.changedAt >= DEBOUNCE_MS) b.stable = raw;
  }
}

// ===================== Startup animation =====================
static void startupFrame(uint32_t now) {
  if (!startupBegun) {
    startupBegun = true;
    startT0 = now;
    rollPrev = -1;
    fill_solid(leds, NUM_LEDS, CRGB::Black);
  }
  const uint32_t t = now - startT0;

  if (t < ROLL_MS) {
    // Phase 1: a few LEDs start at each end and accelerate towards the middle
    float u = (float)t / (float)ROLL_MS;
    int pos = (int)(u * u * (HALF - 1));
    fadeToBlackBy(leds, NUM_LEDS, TAIL_FADE);

    int lo = pos - HEAD_LEN + 1;
    if (rollPrev + 1 < lo) lo = rollPrev + 1;   // fill gaps if a frame was late
    if (lo < 0) lo = 0;
    for (int i = lo; i <= pos; i++) {
      CRGB c = (i == pos) ? CRGB(255, 50, 0) : CRGB(255, 0, 0);
      leds[i] = c;
      leds[NUM_LEDS - 1 - i] = c;
    }
    rollPrev = pos;
    return;
  }

  if (t < ROLL_MS + BOOM_MS) {
    // Phase 2: red explosion spreads from the middle until the whole strip is covered
    float u = (float)(t - ROLL_MS) / (float)BOOM_MS;
    const float centre = (NUM_LEDS - 1) * 0.5f;
    const float rMax   = centre + BOOM_FRONT_W + 1.0f;
    const float r      = outCubic(u) * rMax;

    for (int i = 0; i < NUM_LEDS; i++) {
      float d = fabsf((float)i - centre);
      float v;
      if (d <= r - BOOM_FRONT_W) {
        v = IDLE_LEVEL;
      } else if (d <= r) {
        float k = (r - d) / BOOM_FRONT_W;            // 0 at the front, 1 behind it
        v = 1.0f - (1.0f - IDLE_LEVEL) * k;          // front is 100%, settles to idle
      } else if (d <= r + 3.0f) {
        v = 1.0f - (d - r) / 3.0f;                   // soft leading edge
      } else {
        v = 0.0f;
      }
      leds[i] = CRGB(toByte(v), 0, 0);
    }
    return;
  }

  // Done: hand over to normal operation (already showing idle red)
  fill_solid(leds, NUM_LEDS, CRGB(toByte(IDLE_LEVEL), 0, 0));
  level = IDLE_LEVEL;
  whiteMix = 0.0f;
  startupDone = true;
}

// ===================== Normal operation =====================
// Paints one half of the strip. Sweeps outwards from the middle.
static void paintBlinkHalf(bool highSide, float lit) {
  const int   full = (int)lit;
  const float frac = lit - (float)full;
  for (int k = 0; k < HALF; k++) {
    int idx = highSide ? (HALF + k) : (HALF - 1 - k);
    CRGB &p = leds[idx];
    if (k < full) {
      p = BLINK_COLOR;
    } else {
      if (BLINK_OFF_DARK) p = CRGB::Black;
      if (k == full && frac > 0.0f) nblend(p, BLINK_COLOR, (uint8_t)(frac * 255.0f));
    }
  }
}

static void normalFrame(uint32_t now, float dt) {
  const bool brake   = btn[B_BRAKE].stable;
  const bool reverse = btn[B_REVERSE].stable;
  const bool leftOn  = btn[B_LEFT].stable;
  const bool rightOn = btn[B_RIGHT].stable;

  // Brightness: 75% idle, 100% on brake (quick up, softer down)
  level = stepToward(level, brake ? BRAKE_LEVEL : IDLE_LEVEL,
                   dt / (brake ? LEVEL_UP_S : LEVEL_DOWN_S));

  // Red <-> white fade for reverse
  whiteMix = stepToward(whiteMix, reverse ? 1.0f : 0.0f, dt / WHITE_FADE_S);
  const float w = smoothCurve(whiteMix);

  CRGB base(toByte(level), toByte(level * w), toByte(level * w));
  fill_solid(leds, NUM_LEDS, base);

  // Sequential blinkers (both together = hazard lights)
  const bool anyBlink = leftOn || rightOn;
  if (anyBlink) {
    if (!blinkWasOn) blinkStart = now;
    uint32_t ph = (now - blinkStart) % BLINK_PERIOD_MS;
    float lit;
    if (ph < BLINK_SWEEP_MS)                    lit = HALF * (float)ph / (float)BLINK_SWEEP_MS;
    else if (ph < BLINK_SWEEP_MS + BLINK_HOLD_MS) lit = (float)HALF;
    else                                        lit = 0.0f;

    const bool lowHalf  = SWAP_SIDES ? rightOn : leftOn;   // indices 0..HALF-1
    const bool highHalf = SWAP_SIDES ? leftOn  : rightOn;   // indices HALF..N-1
    if (lowHalf)  paintBlinkHalf(false, lit);
    if (highHalf) paintBlinkHalf(true,  lit);
  }
  blinkWasOn = anyBlink;
}

// ===================== Arduino entry points =====================
void setup() {
  Serial.begin(115200);
  pinMode(PIN_BRAKE,   INPUT_PULLUP);
  pinMode(PIN_RIGHT,   INPUT_PULLUP);
  pinMode(PIN_LEFT,    INPUT_PULLUP);
  pinMode(PIN_REVERSE, INPUT_PULLUP);

  FastLED.addLeds<WS2812B, LED_PIN, COLOR_ORDER>(leds, NUM_LEDS);
  FastLED.setCorrection(UncorrectedColor);
  FastLED.setMaxPowerInVoltsAndMilliamps(5, MAX_MILLIAMPS);
  FastLED.setBrightness(255);
  FastLED.clear(true);

  lastFrame = millis();
}

void loop() {
  const uint32_t now = millis();
  if (now - lastFrame < FRAME_MS) return;
  const float dt = (float)(now - lastFrame) / 1000.0f;
  lastFrame = now;

  readInputs(now);
  if (!startupDone) startupFrame(now);
  else              normalFrame(now, dt);

  FastLED.show();
}
