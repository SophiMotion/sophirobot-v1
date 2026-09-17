// Sophicar ESP32-S3-N16R8 carrier staged bring-up for:
//   1) External A4988 first folding joint plus HTD-ID11 second folding joint.
//   2) Six HX-30HM arm servos on the shared BusLinker UART.
//   3) Four MG513P30 12V wheel motors through the carrier's two onboard DRV8870
//      channels: both left motors in parallel on M1, both right motors in
//      parallel on M2.
//
// Safety policy:
//   - Nothing moves automatically after boot.
//   - Use Serial Monitor commands to run one small test at a time.
//   - all_stop stops local STEP pulses first, sends HTD stop, holds the HX arm,
//     and stops both onboard chassis-driver channels.
//
// Arduino IDE:
//   Board: ESP32S3 Dev Module
//   USB CDC On Boot: Enabled
//   PSRAM: Disabled (GPIO35..38 are required by the onboard DRV8870 channels)
//   Serial Monitor: 115200
//
// Upper-computer protocol v0.1:
//   Link: USB Serial, 115200 baud, 8N1, one JSON object per line ending with \n.
//   ESP32 may also print debug lines; the upper computer should parse lines that
//   start with "{" and ignore other text during bring-up.
//   Examples:
//     {"seq":1,"cmd":"ping"}
//     {"seq":2,"cmd":"status"}
//     {"seq":3,"cmd":"heartbeat"}
//     {"seq":4,"cmd":"stop"}
//     {"seq":5,"cmd":"drive","left_pwm":60,"right_pwm":60}
//     {"seq":6,"cmd":"htd","htd_delta":10,"time_ms":800}
//     {"seq":7,"cmd":"motion","htd_delta":10,"pwm":50}

#include <Arduino.h>
#include <cJSON.h>
#include <DNSServer.h>
#include <Preferences.h>
#include <WiFi.h>
#include <unistd.h>
#include "esp_http_server.h"
#include <esp_wifi.h>
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "web_ui.h"
#include <driver/gptimer.h>
#include <driver/gpio.h>
#include <esp_rom_sys.h>
#include <esp_timer.h>
#include <math.h>
#include <limits.h>
#include <soc/gpio_sig_map.h>

#if defined(BOARD_HAS_PSRAM)
#error "This carrier uses GPIO35..38 for DRV8870. Select PSRAM Disabled."
#endif
constexpr const char *FIRMWARE_BUILD_ID = "foldbot-20260916-ros-esp32-v2";
bool motionStopPending();

struct MotionProfile {
  uint32_t maxRateStepsPerSecond;
  uint32_t accelerationStepsPerSecond2;
};

class StepperMotion {
 public:
  StepperMotion(uint8_t dirPin, uint8_t stepPin, int8_t enablePin);

  bool begin();
  void service();

  bool confirmFoldedZero();
  bool confirmUnfoldedEndpoint();
  bool restoreConfiguration(int32_t travelSteps,
                            bool directionInverted,
                            uint32_t formalDurationMs);
  bool setTravelSteps(int32_t steps, uint32_t formalDurationMs);
  void clearTravelSteps();
  bool setDirectionInverted(bool inverted);

  bool startEndpointMove(int32_t targetSteps,
                         uint32_t durationMs,
                         uint64_t completionLimitUs);
  bool startJog(int32_t deltaSteps);
  void emergencyStopAndDisable();
  void disableHoldingTorque();

  bool isHomed() const;
  bool hasTravel() const;
  bool isActive() const;
  bool driverEnabled() const;
  bool hasEnableControl() const { return enablePin_ >= 0; }
  bool directionInverted() const;
  bool pulseEngineReady() const;
  bool completedNormally() const;
  bool completedWithinLimit() const;
  uint64_t completedElapsedUs() const;
  bool timerFaulted() const;
  void printDiagnostics() const;
  bool startLevelTest();
  bool levelTestMode() const;
  bool levelTestActive() const;
  void printLevelTestStatus() const;
  int32_t currentSteps() const;
  int32_t targetSteps() const;
  int32_t travelSteps() const;
  const char *lastError() const { return lastError_; }

 private:
  // 200 full steps/rev * 4 microsteps * 80:1 * 90/360 = 16000.
  // This software ceiling does not sense a physical mechanical endpoint.
  static constexpr int32_t kCalibrationMaxSteps = 16000;
  static constexpr uint32_t kMaxRampTableSteps = 2048;

  static bool IRAM_ATTR pulseTimerAlarmEntry(
      gptimer_handle_t timer,
      const gptimer_alarm_event_data_t *eventData,
      void *userContext);
  bool IRAM_ATTR pulseTimerAlarm(gptimer_handle_t timer);
  bool IRAM_ATTR levelTestTimerAlarm(gptimer_handle_t timer);
  uint32_t IRAM_ATTR stepPeriodFromSnapshot(uint32_t stepIndex) const;
  void IRAM_ATTR setTimerFaultStateFromIsrLocked();
  void IRAM_ATTR latchTimerFaultFromIsr(gptimer_handle_t timer);
  void latchTimerFault(const char *message);
  bool prepareRampTable(uint32_t totalSteps, const MotionProfile &profile);
  bool startPulseTimer(uint32_t firstPeriodUs);
  bool startMoveInternal(int32_t targetSteps,
                         uint32_t durationMs,
                         uint64_t completionLimitUs,
                         bool endpointLimited,
                         const MotionProfile &profile);
  bool profileCanComplete(uint32_t steps,
                          uint32_t durationMs,
                          const MotionProfile &profile) const;
  bool chooseProfile(uint32_t steps,
                     uint32_t durationMs,
                     uint32_t &peakPeriodUs,
                     uint64_t &estimatedDurationUs,
                     const MotionProfile &profile) const;
  uint64_t estimateProfileDurationUs(uint32_t steps,
                                     uint32_t peakPeriodUs,
                                     const MotionProfile &profile) const;
  uint32_t stepPeriodUs(uint32_t stepIndex,
                        uint32_t totalSteps,
                        uint32_t peakPeriodUs,
                        const MotionProfile &profile) const;
  void setDriverEnabled(bool enabled);
  void setError(const char *message);
  void clearError();
  bool validateOutputPins();

  uint8_t dirPin_;
  uint8_t stepPin_;
  int8_t enablePin_;
  bool pinsReady_ = false;
  volatile uint8_t gpioFaultMask_ = 0;
  volatile uint32_t verifiedPulses_ = 0;
  // Session latch: only reboot permits normal motion again. No NVS writes.
  volatile bool levelTestMode_ = false;
  volatile bool levelTestActive_ = false;
  volatile uint8_t levelTestPhase_ = 0;
  uint8_t levelTestReportedPhase_ = UINT8_MAX;

  mutable portMUX_TYPE stateMux_ = portMUX_INITIALIZER_UNLOCKED;
  gptimer_handle_t pulseTimer_ = nullptr;

  volatile bool homed_ = false;
  volatile bool active_ = false;
  volatile bool driverEnabled_ = false;
  volatile bool directionInverted_ = false;
  volatile bool positiveMove_ = true;
  volatile bool stopRequested_ = true;
  volatile bool completionPending_ = false;
  volatile bool completedNormally_ = false;
  volatile bool timerEnabled_ = false;
  volatile bool timerRunning_ = false;
  volatile bool timerFaultLatched_ = false;
  volatile bool timerFaultReportPending_ = false;
  volatile bool completedWithinLimit_ = false;

  volatile int32_t currentSteps_ = 0;
  volatile int32_t targetSteps_ = 0;
  volatile int32_t travelSteps_ = 0;
  volatile uint32_t totalSteps_ = 0;
  volatile uint32_t emittedSteps_ = 0;
  volatile uint32_t moveGeneration_ = 0;
  volatile uint32_t timerAlarmGeneration_ = 0;
  volatile uint64_t completionLimitUs_ = UINT64_MAX;
  volatile uint64_t completedElapsedUs_ = 0;
  uint32_t moveDurationMs_ = 0;
  uint32_t profilePeakPeriodUs_ = 0;
  MotionProfile moveProfile_ = {0U, 0U};
  uint32_t rampTableSteps_ = 0;
  uint32_t rampPeriodsUs_[kMaxRampTableSteps + 1U] = {};
  const char *lastError_ = "";
};

namespace {
constexpr uint8_t kEnableActive = LOW;
constexpr uint8_t kEnableInactive = HIGH;
constexpr uint32_t kStepPulseHighUs = 4;
constexpr uint32_t kStepPulseLowMinUs = 4;
constexpr uint32_t kGptimerMinimumAlarmUs = 5;
// Quarter the pulse rate/acceleration to retain the former 1/16 jog angular speed.
constexpr MotionProfile kJogProfile = {125U, 125U};
constexpr MotionProfile kFormalProfile = {6000U, 10000U};
constexpr uint32_t kMinMoveDurationMs = 100;
constexpr uint32_t kPulseTimerResolutionHz = 1000000;
constexpr uint32_t kLevelTestPhaseUs = 5000000;
constexpr uint32_t kLevelTestTickUs = 10000;
constexpr uint8_t kLevelTestPhases = 6;
constexpr uint64_t kLevelTestDurationUs = 30000000ULL;
constexpr uint64_t kMicrosecondsSquaredPerSecondSquared =
    1000000000000ULL;

static_assert(kStepPulseHighUs >= 2, "STEP high pulse must stay conservative");
static_assert(kStepPulseLowMinUs >= 2, "STEP low pulse must stay conservative");
static_assert((1000000UL + kFormalProfile.maxRateStepsPerSecond - 1U) /
                      kFormalProfile.maxRateStepsPerSecond >=
                  kStepPulseHighUs + kGptimerMinimumAlarmUs,
              "maximum rate must preserve HIGH and safe GPTimer LOW alarm");

uint32_t ceilSquareRoot(uint64_t value) {
  if (value == 0) return 0;

  // The floating-point result is only an initial estimate. Integer correction
  // below makes the returned ceiling exact, including perfect-square edges.
  uint64_t root = static_cast<uint64_t>(sqrt(static_cast<double>(value)));
  if (root > UINT32_MAX) root = UINT32_MAX;
  while (root < UINT32_MAX && root + 1U <= value / (root + 1U)) ++root;
  while (root > 0 && root > value / root) --root;
  if (root * root < value && root < UINT32_MAX) ++root;
  return static_cast<uint32_t>(root);
}

uint32_t accelerationArrivalUs(uint32_t completedSteps,
                               const MotionProfile &profile) {
  if (completedSteps == 0) return 0;
  if (profile.accelerationStepsPerSecond2 == 0) return UINT32_MAX;

  // x <= 0.5*a*t^2  =>  t >= sqrt(2*x/a). Work in integer
  // microseconds and round upward so a pulse can never be scheduled early.
  if (completedSteps >
      UINT64_MAX / (2ULL * kMicrosecondsSquaredPerSecondSquared)) {
    return UINT32_MAX;
  }
  const uint64_t numerator =
      2ULL * completedSteps * kMicrosecondsSquaredPerSecondSquared;
  const uint64_t radicand =
      numerator / profile.accelerationStepsPerSecond2 +
      (numerator % profile.accelerationStepsPerSecond2 == 0 ? 0ULL : 1ULL);
  return ceilSquareRoot(radicand);
}

uint32_t accelerationIntervalUs(uint32_t oneBasedStep,
                                const MotionProfile &profile) {
  return accelerationArrivalUs(oneBasedStep, profile) -
         accelerationArrivalUs(oneBasedStep - 1U, profile);
}

uint32_t fastestPeriodUs(const MotionProfile &profile) {
  if (profile.maxRateStepsPerSecond == 0) return UINT32_MAX;
  uint32_t period =
      (1000000UL + profile.maxRateStepsPerSecond - 1U) /
      profile.maxRateStepsPerSecond;
  const uint32_t pulseMinimum = kStepPulseHighUs + kStepPulseLowMinUs;
  if (period < pulseMinimum) period = pulseMinimum;
  return period;
}
}  // namespace

StepperMotion::StepperMotion(uint8_t dirPin,
                             uint8_t stepPin,
                             int8_t enablePin)
    : dirPin_(dirPin),
      stepPin_(stepPin),
      enablePin_(enablePin) {}

bool StepperMotion::begin() {
  // The external A4988 uses a dedicated active-low ENABLE signal. Establish
  // the disabled level before exposing GPIO10 as an output; the required
  // external 10 kOhm pull-up also covers reset/boot high-impedance windows.
  if (!hasEnableControl() ||
      !GPIO_IS_VALID_OUTPUT_GPIO(static_cast<uint8_t>(enablePin_))) {
    latchTimerFault("step_enable_gpio_invalid");
    return false;
  }
  digitalWrite(enablePin_, kEnableInactive);
  pinMode(enablePin_, OUTPUT);
  const uint8_t pins[] = {stepPin_, dirPin_};
  gpio_config_t io = {};
  for (uint8_t pin : pins) {
    if (!GPIO_IS_VALID_OUTPUT_GPIO(pin) ||
        gpio_set_level(static_cast<gpio_num_t>(pin), LOW) != ESP_OK) {
      latchTimerFault("step_gpio_invalid");
      return false;
    }
    io.pin_bit_mask |= 1ULL << pin;
  }
  // Input remains enabled to verify the pad level, not just the output latch.
  // All STEP/DIR access uses the IDF GPIO API consistently.
  io.mode = GPIO_MODE_INPUT_OUTPUT;
  io.pull_up_en = GPIO_PULLUP_DISABLE;
  io.pull_down_en = GPIO_PULLDOWN_DISABLE;
  io.intr_type = GPIO_INTR_DISABLE;
  if (gpio_config(&io) != ESP_OK || !validateOutputPins()) {
    latchTimerFault("step_gpio_configuration_failed");
    return false;
  }
  esp_rom_delay_us(2);
  for (uint8_t pin : pins) {
    if (gpio_get_level(static_cast<gpio_num_t>(pin)) != LOW) {
      latchTimerFault("step_gpio_idle_readback_failed");
      return false;
    }
  }
  setDriverEnabled(false);

  gptimer_config_t timerConfig = {};
  timerConfig.clk_src = GPTIMER_CLK_SRC_DEFAULT;
  timerConfig.direction = GPTIMER_COUNT_UP;
  timerConfig.resolution_hz = kPulseTimerResolutionHz;
  timerConfig.intr_priority = 0;
  timerConfig.flags.intr_shared = true;
  if (gptimer_new_timer(&timerConfig, &pulseTimer_) != ESP_OK) {
    pulseTimer_ = nullptr;
    latchTimerFault("pulse_timer_create_failed");
    return false;
  }

  gptimer_event_callbacks_t callbacks = {};
  callbacks.on_alarm = pulseTimerAlarmEntry;
  if (gptimer_register_event_callbacks(pulseTimer_, &callbacks, this) !=
      ESP_OK) {
    gptimer_del_timer(pulseTimer_);
    pulseTimer_ = nullptr;
    latchTimerFault("pulse_timer_callback_register_failed");
    return false;
  }
  if (gptimer_enable(pulseTimer_) != ESP_OK) {
    gptimer_del_timer(pulseTimer_);
    pulseTimer_ = nullptr;
    latchTimerFault("pulse_timer_enable_failed");
    return false;
  }

  portENTER_CRITICAL(&stateMux_);
  timerEnabled_ = true;
  timerFaultLatched_ = false;
  timerFaultReportPending_ = false;
  portEXIT_CRITICAL(&stateMux_);
  clearError();
  return true;
}

void StepperMotion::service() {
  portENTER_CRITICAL(&stateMux_);
  const bool reportLevelTest = levelTestMode_ &&
      levelTestReportedPhase_ != levelTestPhase_;
  if (reportLevelTest) levelTestReportedPhase_ = levelTestPhase_;
  portEXIT_CRITICAL(&stateMux_);
  if (reportLevelTest) printLevelTestStatus();
  bool reportCompletion = false;
  bool reportTimerFault = false;
  bool completedWithinLimit = false;
  int32_t position = 0;
  uint64_t elapsedUs = 0;
  portENTER_CRITICAL(&stateMux_);
  if (completionPending_) {
    completionPending_ = false;
    reportCompletion = true;
    position = currentSteps_;
    completedWithinLimit = completedWithinLimit_;
    elapsedUs = completedElapsedUs_;
  }
  if (timerFaultReportPending_) {
    timerFaultReportPending_ = false;
    reportTimerFault = true;
  }
  portEXIT_CRITICAL(&stateMux_);

  if (reportTimerFault) setError(gpioFaultMask_ ? "step_gpio_readback_failed" : "pulse_timer_runtime_failed");
  if (reportCompletion) {
    Serial.printf(
        "STEPPER pad_verified pulse emission complete position=%ld "
        "elapsed_us=%llu within_limit=%s; mechanical endpoint "
        "not verified\n",
        static_cast<long>(position), elapsedUs,
        completedWithinLimit ? "yes" : "no");
    printDiagnostics();
  }
}

bool StepperMotion::confirmFoldedZero() {
  portENTER_CRITICAL(&stateMux_);
  if (levelTestMode_) {
    portEXIT_CRITICAL(&stateMux_);
    setError("step_level_test_mode_reboot_required");
    return false;
  }
  if (active_) {
    portEXIT_CRITICAL(&stateMux_);
    setError("motion_active");
    return false;
  }
  currentSteps_ = 0;
  targetSteps_ = 0;
  homed_ = true;
  portEXIT_CRITICAL(&stateMux_);
  clearError();
  Serial.println("STEPPER folded zero confirmed position=0");
  return true;
}

bool StepperMotion::confirmUnfoldedEndpoint() {
  // Human-supplied reference, not homing: absolutely no GPIO/timer/servo IO.
  // Keep stopRequested_ as-is (it is true at boot); only a later explicit
  // motion command may start the pulse engine after its normal safety checks.
  const char *error = nullptr;
  int32_t confirmedPosition = 0;
  portENTER_CRITICAL(&stateMux_);
  if (levelTestMode_) error = "step_level_test_mode_reboot_required";
  else if (active_ || timerRunning_) error = "motion_active";
  else if (motionStopPending()) error = "stop_requested_reconfirm_zero";
  else if (!pinsReady_ || pulseTimer_ == nullptr || !timerEnabled_ ||
           timerFaultLatched_ || gpioFaultMask_ != 0) error = "pulse_engine_unavailable";
  else if (travelSteps_ <= 0 || travelSteps_ > kCalibrationMaxSteps) error = "travel_not_calibrated";
  else {
    confirmedPosition = travelSteps_;
    currentSteps_ = confirmedPosition;
    targetSteps_ = confirmedPosition;
    homed_ = true;
  }
  portEXIT_CRITICAL(&stateMux_);
  if (error != nullptr) {
    setError(error);
    return false;
  }
  clearError();
  Serial.printf("STEPPER human unfolded endpoint confirmed position=%ld; no_motion=yes; mechanical endpoint not sensed\n",
                static_cast<long>(confirmedPosition));
  return true;
}

bool StepperMotion::restoreConfiguration(int32_t travelSteps,
                                         bool directionInverted,
                                         uint32_t formalDurationMs) {
  const bool travelInRange =
      travelSteps == 0 ||
      (travelSteps > 0 && travelSteps <= kCalibrationMaxSteps);
  const bool profileSafe =
      travelSteps == 0 ||
      profileCanComplete(static_cast<uint32_t>(travelSteps), formalDurationMs,
                         kFormalProfile);

  portENTER_CRITICAL(&stateMux_);
  travelSteps_ = (travelInRange && profileSafe) ? travelSteps : 0;
  directionInverted_ = directionInverted;
  portEXIT_CRITICAL(&stateMux_);

  if (!travelInRange) {
    setError("stored_travel_out_of_range");
    return false;
  }
  if (!profileSafe) {
    setError("stored_travel_exceeds_safe_profile");
    return false;
  }
  clearError();
  Serial.printf("STEPPER config restored travel=%ld direction_inverted=%s\n",
                static_cast<long>(travelSteps),
                directionInverted ? "yes" : "no");
  return true;
}

bool StepperMotion::setTravelSteps(int32_t steps,
                                   uint32_t formalDurationMs) {
  if (isActive()) {
    setError("motion_active");
    return false;
  }
  if (!isHomed()) {
    setError("zero_not_confirmed");
    return false;
  }
  if (steps <= 0 || steps > kCalibrationMaxSteps) {
    setError("travel_out_of_range");
    return false;
  }
  if (!profileCanComplete(static_cast<uint32_t>(steps), formalDurationMs,
                          kFormalProfile)) {
    setError("travel_exceeds_6s_safe_profile");
    return false;
  }

  portENTER_CRITICAL(&stateMux_);
  travelSteps_ = steps;
  portEXIT_CRITICAL(&stateMux_);
  clearError();
  Serial.printf("STEPPER travel endpoint=%ld profile_duration_ms=%lu safe=yes\n",
                static_cast<long>(steps),
                static_cast<unsigned long>(formalDurationMs));
  return true;
}

void StepperMotion::clearTravelSteps() {
  portENTER_CRITICAL(&stateMux_);
  if (active_) {
    portEXIT_CRITICAL(&stateMux_);
    return;
  }
  travelSteps_ = 0;
  portEXIT_CRITICAL(&stateMux_);
  clearError();
  Serial.println("STEPPER saved travel cleared");
}

bool StepperMotion::setDirectionInverted(bool inverted) {
  portENTER_CRITICAL(&stateMux_);
  if (active_) {
    portEXIT_CRITICAL(&stateMux_);
    setError("motion_active");
    return false;
  }
  if (!homed_ || currentSteps_ != 0) {
    portEXIT_CRITICAL(&stateMux_);
    setError("return_to_folded_zero_first");
    return false;
  }
  if (directionInverted_ != inverted) {
    // Direction and a previously calibrated physical endpoint are inseparable.
    // Invalidate the runtime endpoint even if a caller forgets to do so.
    travelSteps_ = 0;
    directionInverted_ = inverted;
  }
  portEXIT_CRITICAL(&stateMux_);
  clearError();
  Serial.printf("STEPPER direction_inverted=%s travel_invalidated=yes\n",
                inverted ? "yes" : "no");
  return true;
}

bool StepperMotion::startEndpointMove(int32_t targetSteps,
                                      uint32_t durationMs,
                                      uint64_t completionLimitUs) {
  const int32_t position = currentSteps();
  const int32_t travel = travelSteps();
  if (targetSteps == travel && position != 0) {
    setError("unfold_requires_folded_endpoint");
    return false;
  }
  if (targetSteps == 0 && position != travel) {
    setError("fold_requires_unfolded_endpoint");
    return false;
  }
  return startMoveInternal(targetSteps, durationMs, completionLimitUs, true,
                           kFormalProfile);
}

bool StepperMotion::startJog(int32_t deltaSteps) {
  // Direct comparisons avoid absolute-value overflow at INT32_MIN.
  if (deltaSteps < -2000 || deltaSteps > 2000 || deltaSteps == 0) {
    setError("invalid_jog_delta");
    return false;
  }
  if (!isHomed()) {
    setError("zero_not_confirmed");
    return false;
  }

  const int32_t position = currentSteps();
  const int32_t travel = travelSteps();
  const int32_t limit = travel > 0 ? travel : kCalibrationMaxSteps;
  int64_t requested = static_cast<int64_t>(position) + deltaSteps;
  if (requested < 0) requested = 0;
  if (requested > limit) requested = limit;
  const int32_t target = static_cast<int32_t>(requested);
  const uint32_t distance = static_cast<uint32_t>(
      target >= position ? target - position : position - target);
  if (distance == 0) {
    setError("jog_at_software_limit");
    return false;
  }

  const uint64_t fastestUs =
      estimateProfileDurationUs(distance, fastestPeriodUs(kJogProfile),
                                kJogProfile);
  uint32_t durationMs =
      static_cast<uint32_t>((fastestUs + 999ULL) / 1000ULL) + 100U;
  if (durationMs < 300U) durationMs = 300U;
  return startMoveInternal(target, durationMs, UINT64_MAX, false, kJogProfile);
}

void StepperMotion::emergencyStopAndDisable() {
  int32_t stoppedPosition = 0;
  uint8_t diagnosticLowFault = 0;
  portENTER_CRITICAL(&stateMux_);
  ++moveGeneration_;
  if (moveGeneration_ == 0) ++moveGeneration_;
  stopRequested_ = true;
  levelTestActive_ = false;
  levelTestPhase_ = kLevelTestPhases;
  active_ = false;
  completionPending_ = false;
  completedNormally_ = false;
  completedWithinLimit_ = false;
  completedElapsedUs_ = 0;
  timerRunning_ = false;
  targetSteps_ = currentSteps_;
  totalSteps_ = 0;
  emittedSteps_ = 0;
  const esp_err_t firstLow = gpio_set_level(static_cast<gpio_num_t>(stepPin_), LOW);
  if (levelTestMode_) {
    esp_rom_delay_us(1);
    if (firstLow != ESP_OK || gpio_get_level(static_cast<gpio_num_t>(stepPin_)) != LOW) diagnosticLowFault |= 4;
    gpioFaultMask_ |= diagnosticLowFault;
  }
  if (hasEnableControl()) {
    gpio_set_level(static_cast<gpio_num_t>(enablePin_), kEnableInactive);
  }
  driverEnabled_ = false;
  homed_ = false;
  stoppedPosition = currentSteps_;
  portEXIT_CRITICAL(&stateMux_);

  esp_err_t stopResult = ESP_ERR_INVALID_STATE;
  if (pulseTimer_ != nullptr && timerEnabled_) {
    stopResult = gptimer_stop(pulseTimer_);
  }
  if (diagnosticLowFault) {
    latchTimerFault("step_level_test_stop_low_failed");
  } else if (stopResult != ESP_OK && stopResult != ESP_ERR_INVALID_STATE) {
    latchTimerFault("pulse_timer_stop_failed");
  } else {
    setError("stopped_by_user");
  }
  Serial.printf("STEPPER software stop position=%ld pulses=stopped enable_control=%s\n",
                static_cast<long>(stoppedPosition),
                hasEnableControl() ? "yes" : "no");
}

void StepperMotion::disableHoldingTorque() {
  portENTER_CRITICAL(&stateMux_);
  if (active_) {
    portEXIT_CRITICAL(&stateMux_);
    return;
  }
  if (hasEnableControl()) {
    gpio_set_level(static_cast<gpio_num_t>(enablePin_), kEnableInactive);
  }
  driverEnabled_ = false;
  homed_ = false;
  portEXIT_CRITICAL(&stateMux_);
  Serial.println(hasEnableControl()
                     ? "STEPPER holding torque released"
                     : "STEPPER pulses stopped; holding torque is not software-controlled");
}

bool StepperMotion::isHomed() const {
  portENTER_CRITICAL(&stateMux_);
  const bool value = homed_;
  portEXIT_CRITICAL(&stateMux_);
  return value;
}

bool StepperMotion::hasTravel() const {
  portENTER_CRITICAL(&stateMux_);
  const bool value = travelSteps_ > 0;
  portEXIT_CRITICAL(&stateMux_);
  return value;
}

bool StepperMotion::isActive() const {
  portENTER_CRITICAL(&stateMux_);
  const bool value = active_;
  portEXIT_CRITICAL(&stateMux_);
  return value;
}

bool StepperMotion::driverEnabled() const {
  portENTER_CRITICAL(&stateMux_);
  const bool value = driverEnabled_;
  portEXIT_CRITICAL(&stateMux_);
  return value;
}

bool StepperMotion::directionInverted() const {
  portENTER_CRITICAL(&stateMux_);
  const bool value = directionInverted_;
  portEXIT_CRITICAL(&stateMux_);
  return value;
}

bool StepperMotion::pulseEngineReady() const {
  portENTER_CRITICAL(&stateMux_);
  const bool ready =
      pinsReady_ && pulseTimer_ != nullptr && timerEnabled_ && !timerFaultLatched_;
  portEXIT_CRITICAL(&stateMux_);
  return ready;
}

bool StepperMotion::completedNormally() const {
  portENTER_CRITICAL(&stateMux_);
  const bool completed = completedNormally_;
  portEXIT_CRITICAL(&stateMux_);
  return completed;
}

bool StepperMotion::completedWithinLimit() const {
  portENTER_CRITICAL(&stateMux_);
  const bool withinLimit = completedWithinLimit_;
  portEXIT_CRITICAL(&stateMux_);
  return withinLimit;
}

uint64_t StepperMotion::completedElapsedUs() const {
  portENTER_CRITICAL(&stateMux_);
  const uint64_t elapsedUs = completedElapsedUs_;
  portEXIT_CRITICAL(&stateMux_);
  return elapsedUs;
}

bool StepperMotion::timerFaulted() const {
  portENTER_CRITICAL(&stateMux_);
  const bool faulted = timerFaultLatched_;
  portEXIT_CRITICAL(&stateMux_);
  return faulted;
}

int32_t StepperMotion::currentSteps() const {
  portENTER_CRITICAL(&stateMux_);
  const int32_t value = currentSteps_;
  portEXIT_CRITICAL(&stateMux_);
  return value;
}

int32_t StepperMotion::targetSteps() const {
  portENTER_CRITICAL(&stateMux_);
  const int32_t value = targetSteps_;
  portEXIT_CRITICAL(&stateMux_);
  return value;
}

int32_t StepperMotion::travelSteps() const {
  portENTER_CRITICAL(&stateMux_);
  const int32_t value = travelSteps_;
  portEXIT_CRITICAL(&stateMux_);
  return value;
}

bool IRAM_ATTR StepperMotion::pulseTimerAlarmEntry(
    gptimer_handle_t timer,
    const gptimer_alarm_event_data_t *eventData,
    void *userContext) {
  (void)eventData;
  return static_cast<StepperMotion *>(userContext)->pulseTimerAlarm(timer);
}

bool IRAM_ATTR StepperMotion::pulseTimerAlarm(gptimer_handle_t timer) {
  portENTER_CRITICAL_ISR(&stateMux_);
  const bool diagnostic = levelTestMode_;
  portEXIT_CRITICAL_ISR(&stateMux_);
  if (diagnostic) return levelTestTimerAlarm(timer);
  uint32_t generation = 0;
  uint32_t lowIntervalUs = 0;
  uint64_t currentCount = 0;
  bool lastPulse = false;
  bool complete = false;
  bool stale = false;

  portENTER_CRITICAL_ISR(&stateMux_);
  if (stopRequested_ || !active_ ||
      timerAlarmGeneration_ != moveGeneration_) {
    gpio_set_level(static_cast<gpio_num_t>(stepPin_), LOW);
    timerRunning_ = false;
    stale = true;
  } else if (gptimer_get_raw_count(timer, &currentCount) != ESP_OK ||
             currentCount >= completionLimitUs_ ||
             completionLimitUs_ - currentCount <= kStepPulseHighUs + 1U) {
    // Enforce the finite formal-motion deadline in the ISR itself. A delayed
    // loop must not allow another pulse after the safety time budget expires.
    setTimerFaultStateFromIsrLocked();
    stale = true;
  } else {
    generation = moveGeneration_;

    // STEP HIGH, its complete 4 us width, STEP LOW, and the corresponding
    // software count are one short critical transaction. STOP can run before
    // or after this block, never between a hardware-recognized pulse and its
    // software count.
    gpio_set_level(static_cast<gpio_num_t>(stepPin_), HIGH);
    esp_rom_delay_us(kStepPulseHighUs);
    uint8_t fault = 0;
    if (gpio_get_level(static_cast<gpio_num_t>(stepPin_)) != HIGH) fault |= 1;
    gpio_set_level(static_cast<gpio_num_t>(stepPin_), LOW);
    esp_rom_delay_us(1);
    if (gpio_get_level(static_cast<gpio_num_t>(stepPin_)) != LOW) fault |= 4;
    if (fault) {
      gpioFaultMask_ = fault;
      setTimerFaultStateFromIsrLocked();
      stale = true;
    } else {
      currentSteps_ += positiveMove_ ? 1 : -1;
      ++emittedSteps_;
      ++verifiedPulses_;
      lastPulse = emittedSteps_ >= totalSteps_;
    }
  }
  portEXIT_CRITICAL_ISR(&stateMux_);

  if (stale) {
    gptimer_stop(timer);
    return false;
  }

  // Capture immediately after the atomic LOW/count transaction. The value is
  // conservative (never earlier than the falling edge) and includes ISR
  // latency plus the ROM HIGH delay.
  if (gptimer_get_raw_count(timer, &currentCount) != ESP_OK) {
    latchTimerFaultFromIsr(timer);
    return false;
  }

  portENTER_CRITICAL_ISR(&stateMux_);
  const bool stillCurrent =
      active_ && !stopRequested_ && moveGeneration_ == generation &&
      timerAlarmGeneration_ == generation;
  if (stillCurrent) {
    if (lastPulse) {
      targetSteps_ = currentSteps_;
      completedElapsedUs_ = currentCount;
      completedWithinLimit_ = currentCount <= completionLimitUs_;
      active_ = false;
      stopRequested_ = true;
      completionPending_ = true;
      completedNormally_ = true;
      timerRunning_ = false;
      complete = true;
    } else {
      const uint32_t nextPeriodUs = stepPeriodFromSnapshot(emittedSteps_);
      if (nextPeriodUs > kStepPulseHighUs) {
        lowIntervalUs = nextPeriodUs - kStepPulseHighUs;
      } else {
        lowIntervalUs = 0;
      }
      if (lowIntervalUs < kGptimerMinimumAlarmUs) {
        setTimerFaultStateFromIsrLocked();
      }
    }
  }
  portEXIT_CRITICAL_ISR(&stateMux_);

  if (!stillCurrent) return false;

  if (complete || lowIntervalUs < kGptimerMinimumAlarmUs) {
    const esp_err_t stopped = gptimer_stop(timer);
    if (complete && stopped != ESP_OK && stopped != ESP_ERR_INVALID_STATE) {
      latchTimerFaultFromIsr(timer);
    }
    return false;
  }

  gptimer_alarm_config_t alarmConfig = {};
  alarmConfig.alarm_count = currentCount + lowIntervalUs;
  alarmConfig.reload_count = 0;
  alarmConfig.flags.auto_reload_on_alarm = false;
  if (gptimer_set_alarm_action(timer, &alarmConfig) != ESP_OK) {
    latchTimerFaultFromIsr(timer);
  }
  return false;
}

uint32_t IRAM_ATTR StepperMotion::stepPeriodFromSnapshot(
    uint32_t stepIndex) const {
  uint32_t periodUs = profilePeakPeriodUs_;
  const uint32_t fromStart = stepIndex + 1U;
  const uint32_t fromEnd = totalSteps_ - stepIndex;
  if (fromStart <= rampTableSteps_ && rampPeriodsUs_[fromStart] > periodUs) {
    periodUs = rampPeriodsUs_[fromStart];
  }
  if (fromEnd <= rampTableSteps_ && rampPeriodsUs_[fromEnd] > periodUs) {
    periodUs = rampPeriodsUs_[fromEnd];
  }
  return periodUs;
}

void IRAM_ATTR StepperMotion::setTimerFaultStateFromIsrLocked() {
  ++moveGeneration_;
  if (moveGeneration_ == 0) ++moveGeneration_;
  stopRequested_ = true;
  levelTestActive_ = false;
  levelTestPhase_ = kLevelTestPhases;
  active_ = false;
  completionPending_ = false;
  completedNormally_ = false;
  completedWithinLimit_ = false;
  completedElapsedUs_ = 0;
  timerRunning_ = false;
  timerFaultLatched_ = true;
  timerFaultReportPending_ = true;
  targetSteps_ = currentSteps_;
  gpio_set_level(static_cast<gpio_num_t>(stepPin_), LOW);
  if (hasEnableControl()) {
    gpio_set_level(static_cast<gpio_num_t>(enablePin_), kEnableInactive);
  }
  driverEnabled_ = false;
  homed_ = false;
}

void IRAM_ATTR StepperMotion::latchTimerFaultFromIsr(gptimer_handle_t timer) {
  portENTER_CRITICAL_ISR(&stateMux_);
  setTimerFaultStateFromIsrLocked();
  portEXIT_CRITICAL_ISR(&stateMux_);
  gptimer_stop(timer);
}

void StepperMotion::latchTimerFault(const char *message) {
  portENTER_CRITICAL(&stateMux_);
  ++moveGeneration_;
  if (moveGeneration_ == 0) ++moveGeneration_;
  stopRequested_ = true;
  levelTestActive_ = false;
  levelTestPhase_ = kLevelTestPhases;
  active_ = false;
  completionPending_ = false;
  completedNormally_ = false;
  completedWithinLimit_ = false;
  completedElapsedUs_ = 0;
  timerRunning_ = false;
  timerFaultLatched_ = true;
  timerFaultReportPending_ = false;
  targetSteps_ = currentSteps_;
  gpio_set_level(static_cast<gpio_num_t>(stepPin_), LOW);
  if (hasEnableControl()) {
    gpio_set_level(static_cast<gpio_num_t>(enablePin_), kEnableInactive);
  }
  driverEnabled_ = false;
  homed_ = false;
  portEXIT_CRITICAL(&stateMux_);
  setError(message);
}

bool StepperMotion::prepareRampTable(uint32_t totalSteps,
                                     const MotionProfile &profile) {
  const uint32_t tableSteps =
      totalSteps < kMaxRampTableSteps ? totalSteps : kMaxRampTableSteps;
  for (uint32_t oneBasedStep = 1; oneBasedStep <= tableSteps;
       ++oneBasedStep) {
    rampPeriodsUs_[oneBasedStep] =
        accelerationIntervalUs(oneBasedStep, profile);
  }

  // Beyond the table the acceleration interval must already be no slower
  // than the profile rate floor, so the peak period completely dominates it.
  if (totalSteps > kMaxRampTableSteps &&
      accelerationIntervalUs(kMaxRampTableSteps + 1U, profile) >
          fastestPeriodUs(profile)) {
    setError("motion_ramp_table_too_short");
    return false;
  }
  rampTableSteps_ = tableSteps;
  return true;
}

bool StepperMotion::startPulseTimer(uint32_t firstPeriodUs) {
  if (pulseTimer_ == nullptr || !timerEnabled_) {
    latchTimerFault("pulse_timer_unavailable");
    return false;
  }
  if (firstPeriodUs < kGptimerMinimumAlarmUs) {
    latchTimerFault("pulse_timer_alarm_too_short");
    return false;
  }
  if (gptimer_set_raw_count(pulseTimer_, 0) != ESP_OK) {
    latchTimerFault("pulse_timer_reset_failed");
    return false;
  }

  gptimer_alarm_config_t alarmConfig = {};
  alarmConfig.alarm_count = firstPeriodUs;
  alarmConfig.reload_count = 0;
  alarmConfig.flags.auto_reload_on_alarm = false;
  if (gptimer_set_alarm_action(pulseTimer_, &alarmConfig) != ESP_OK) {
    latchTimerFault("pulse_timer_alarm_config_failed");
    return false;
  }

  portENTER_CRITICAL(&stateMux_);
  timerRunning_ = true;
  portEXIT_CRITICAL(&stateMux_);
  if (gptimer_start(pulseTimer_) != ESP_OK) {
    latchTimerFault("pulse_timer_start_failed");
    return false;
  }
  return true;
}

bool StepperMotion::startMoveInternal(int32_t targetSteps,
                                      uint32_t durationMs,
                                      uint64_t completionLimitUs,
                                      bool endpointLimited,
                                      const MotionProfile &profile) {
  if (levelTestMode()) {
    setError("step_level_test_mode_reboot_required");
    return false;
  }
  if (!pulseEngineReady()) {
    setError("pulse_engine_unavailable");
    return false;
  }
  if (isActive()) {
    setError("motion_active");
    return false;
  }
  if (!isHomed()) {
    setError("zero_not_confirmed");
    return false;
  }

  const int32_t travel = travelSteps();
  const int32_t upperLimit =
      endpointLimited ? travel : (travel > 0 ? travel : kCalibrationMaxSteps);
  if (endpointLimited && travel <= 0) {
    setError("travel_not_calibrated");
    return false;
  }
  if (targetSteps < 0 || targetSteps > upperLimit) {
    setError("target_out_of_software_limits");
    return false;
  }
  if (durationMs < kMinMoveDurationMs) {
    setError("duration_too_short");
    return false;
  }
  if (profile.maxRateStepsPerSecond == 0 ||
      profile.accelerationStepsPerSecond2 == 0) {
    setError("invalid_motion_profile");
    return false;
  }

  const int32_t start = currentSteps();
  if (targetSteps == start) {
    setError("target_already_reached");
    return false;
  }
  const bool positive = targetSteps > start;
  const uint32_t total = static_cast<uint32_t>(
      positive ? targetSteps - start : start - targetSteps);
  uint32_t peakPeriodUs = 0;
  uint64_t estimatedDurationUs = 0;
  const int64_t planningStartedUs = esp_timer_get_time();
  if (!chooseProfile(total, durationMs, peakPeriodUs, estimatedDurationUs,
                     profile)) {
    setError("profile_cannot_meet_requested_duration");
    return false;
  }
  if (!prepareRampTable(total, profile)) return false;
  const uint32_t planningElapsedUs = static_cast<uint32_t>(esp_timer_get_time() - planningStartedUs);

  const bool physicalPositive = positive != directionInverted();
  if (!validateOutputPins()) {
    latchTimerFault("step_gpio_configuration_changed");
    return false;
  }
  gpio_set_level(static_cast<gpio_num_t>(stepPin_), LOW);
  gpio_set_level(static_cast<gpio_num_t>(dirPin_), physicalPositive ? HIGH : LOW);
  setDriverEnabled(true);
  // One yielding millisecond is more conservative than the required 10 us DIR
  // setup time and cannot starve Web/serial STOP handling.
  delay(1);

  if (motionStopPending()) {
    emergencyStopAndDisable();
    return false;
  }

  if (gpio_get_level(static_cast<gpio_num_t>(dirPin_)) != (physicalPositive ? HIGH : LOW)) {
    gpioFaultMask_ = 16;
    latchTimerFault("step_direction_readback_failed");
    return false;
  }

  uint32_t generation = 0;
  portENTER_CRITICAL(&stateMux_);
  targetSteps_ = targetSteps;
  positiveMove_ = positive;
  totalSteps_ = total;
  emittedSteps_ = 0;
  verifiedPulses_ = 0;
  moveDurationMs_ = durationMs;
  profilePeakPeriodUs_ = peakPeriodUs;
  moveProfile_ = profile;
  completionLimitUs_ = completionLimitUs;
  completedElapsedUs_ = 0;
  stopRequested_ = false;
  completionPending_ = false;
  completedNormally_ = false;
  completedWithinLimit_ = false;
  timerRunning_ = false;
  ++moveGeneration_;
  if (moveGeneration_ == 0) ++moveGeneration_;
  generation = moveGeneration_;
  timerAlarmGeneration_ = generation;
  active_ = true;
  portEXIT_CRITICAL(&stateMux_);
  clearError();

  const uint32_t firstPeriodUs =
      stepPeriodUs(0, total, peakPeriodUs, profile);
  if (!startPulseTimer(firstPeriodUs)) return false;
  Serial.printf(
      "STEPPER GPTimer plan from=%ld target=%ld steps=%lu duration_ms=%lu "
      "estimated_us=%llu peak_period_us=%lu peak_rate=%.2f "
      "max_rate=%lu accel=%lu "
      "generation=%lu planning_us=%lu\n",
      static_cast<long>(start), static_cast<long>(targetSteps),
      static_cast<unsigned long>(total),
      static_cast<unsigned long>(durationMs), estimatedDurationUs,
      static_cast<unsigned long>(peakPeriodUs),
      1000000.0 / static_cast<double>(peakPeriodUs),
      static_cast<unsigned long>(profile.maxRateStepsPerSecond),
      static_cast<unsigned long>(profile.accelerationStepsPerSecond2),
      static_cast<unsigned long>(generation), static_cast<unsigned long>(planningElapsedUs));
  return true;
}

bool StepperMotion::profileCanComplete(uint32_t steps,
                                       uint32_t durationMs,
                                       const MotionProfile &profile) const {
  if (steps == 0 || durationMs < kMinMoveDurationMs) return false;
  const uint64_t allowedUs = static_cast<uint64_t>(durationMs) * 1000ULL;
  const uint32_t fastestUs = fastestPeriodUs(profile);
  if (fastestUs == UINT32_MAX || steps > allowedUs / fastestUs) return false;
  return estimateProfileDurationUs(steps, fastestUs, profile) <= allowedUs;
}

bool StepperMotion::chooseProfile(uint32_t steps,
                                  uint32_t durationMs,
                                  uint32_t &peakPeriodUs,
                                  uint64_t &estimatedDurationUs,
                                  const MotionProfile &profile) const {
  if (!profileCanComplete(steps, durationMs, profile)) return false;
  const uint64_t targetUs = static_cast<uint64_t>(durationMs) * 1000ULL;
  const uint32_t fastestUs = fastestPeriodUs(profile);

  // Find the longest integer peak period that still finishes inside the
  // requested time. estimateProfileDurationUs() and GPTimer edge scheduling
  // use this
  // exact same integer period, so binary-search rounding cannot make runtime
  // pulses faster than the validated profile.
  // Every pulse is independently timed from the preceding physical edge;
  // scheduler delays stretch the move and are never repaid as catch-up pulses.
  uint32_t accepted = fastestUs;
  uint32_t low = fastestUs;
  uint32_t high = static_cast<uint32_t>(targetUs / steps);
  while (low <= high) {
    const uint32_t candidate = low + (high - low) / 2U;
    if (estimateProfileDurationUs(steps, candidate, profile) <= targetUs) {
      accepted = candidate;
      low = candidate + 1U;
    } else {
      if (candidate == 0) break;
      high = candidate - 1U;
    }
  }
  peakPeriodUs = accepted;
  estimatedDurationUs =
      estimateProfileDurationUs(steps, peakPeriodUs, profile);
  return estimatedDurationUs <= targetUs;
}

uint64_t StepperMotion::estimateProfileDurationUs(uint32_t steps,
                                                  uint32_t peakPeriodUs,
                                                  const MotionProfile &profile) const {
  if (steps == 0) return kStepPulseHighUs;
  if (profile.accelerationStepsPerSecond2 == 0) return UINT64_MAX;
  const uint32_t floorUs = fastestPeriodUs(profile);
  const uint32_t periodUs = peakPeriodUs > floorUs ? peakPeriodUs : floorUs;
  if (periodUs == 0) return UINT64_MAX;

  // Exact, not an approximate acceleration curve: arrival(n)=ceil(sqrt(2*n/a)*1e6).
  // Its integer interval <= ceil(real interval). For n > ceil(5e11/(a*P*P)),
  // the real interval <= sqrt(5e11/(a*(n-1))) <= P. Thus every interior
  // period is exactly P. Inspect both symmetric edges with the ORIGINAL
  // stepPeriodUs(), including its integer rounding; count the interior at once.
  // Successive ceiling divisions avoid overflow from multiplying a*P*P.
  uint64_t edgeBound = kMicrosecondsSquaredPerSecondSquared / 2ULL;
  const uint32_t divisors[] = {profile.accelerationStepsPerSecond2, periodUs, periodUs};
  for (uint32_t divisor : divisors) {
    edgeBound = edgeBound / divisor + (edgeBound % divisor != 0 ? 1ULL : 0ULL);
  }
  const uint32_t halfSteps = steps / 2U;
  const uint32_t edgeSteps = edgeBound < halfSteps ? static_cast<uint32_t>(edgeBound) : halfSteps;
  const uint32_t centerSteps = steps % 2U;
  uint64_t totalUs = static_cast<uint64_t>(steps - 2U * edgeSteps - centerSteps) * periodUs;
  for (uint32_t index = 0; index < edgeSteps; ++index) {
    totalUs += 2ULL * stepPeriodUs(index, steps, periodUs, profile);
  }
  if (centerSteps != 0) {
    totalUs += stepPeriodUs(halfSteps, steps, periodUs, profile);
  }
  // A move is complete only after the final rising edge has remained HIGH for
  // the full pulse width and the final falling edge has been emitted/counted.
  return totalUs + kStepPulseHighUs;
}

uint32_t StepperMotion::stepPeriodUs(uint32_t stepIndex,
                                     uint32_t totalSteps,
                                     uint32_t peakPeriodUs,
                                     const MotionProfile &profile) const {
  const uint32_t fromStart = stepIndex + 1U;
  const uint32_t fromEnd = totalSteps - stepIndex;
  const uint32_t accelerationPeriodUs =
      accelerationIntervalUs(fromStart, profile);
  const uint32_t decelerationPeriodUs =
      accelerationIntervalUs(fromEnd, profile);
  const uint32_t minimumPeriodUs = fastestPeriodUs(profile);
  uint32_t periodUs = peakPeriodUs;
  if (periodUs < minimumPeriodUs) periodUs = minimumPeriodUs;
  if (periodUs < accelerationPeriodUs) periodUs = accelerationPeriodUs;
  if (periodUs < decelerationPeriodUs) periodUs = decelerationPeriodUs;
  return periodUs;
}

void StepperMotion::setDriverEnabled(bool enabled) {
  if (hasEnableControl()) {
    gpio_set_level(static_cast<gpio_num_t>(enablePin_),
                   enabled ? kEnableActive : kEnableInactive);
  }
  portENTER_CRITICAL(&stateMux_);
  driverEnabled_ = enabled;
  portEXIT_CRITICAL(&stateMux_);
}

void StepperMotion::setError(const char *message) {
  lastError_ = message;
  Serial.printf("STEPPER rejected/error=%s\n", message);
}

void StepperMotion::clearError() {
  lastError_ = "";
}

bool StepperMotion::validateOutputPins() {
  const uint8_t pins[] = {stepPin_, dirPin_};
  pinsReady_ = true;
  for (uint8_t pin : pins) {
    gpio_io_config_t io = {};
    if (gpio_get_io_config(static_cast<gpio_num_t>(pin), &io) != ESP_OK ||
        !io.oe || !io.ie || io.od || io.sig_out != SIG_GPIO_OUT_IDX) {
      pinsReady_ = false;
    }
  }
  return pinsReady_;
}

void StepperMotion::printDiagnostics() const {
  portENTER_CRITICAL(&stateMux_);
  const uint32_t verified = verifiedPulses_;
  const uint8_t fault = gpioFaultMask_;
  portEXIT_CRITICAL(&stateMux_);
  Serial.printf("STEP_DIAG GPIO%u verified_pulses=%lu fault_mask=%u enable_gpio=%d; pad feedback only, not motor feedback\n",
                stepPin_, static_cast<unsigned long>(verified), fault, enablePin_);
  const uint8_t pins[] = {stepPin_, dirPin_};
  for (uint8_t pin : pins) {
    gpio_io_config_t io = {};
    const esp_err_t err = gpio_get_io_config(static_cast<gpio_num_t>(pin), &io);
    Serial.printf("GPIO%u config=%s output=%u input=%u signal=%lu level=%d\n",
                  pin, esp_err_to_name(err), io.oe, io.ie,
                  static_cast<unsigned long>(io.sig_out),
                  gpio_get_level(static_cast<gpio_num_t>(pin)));
  }
  if (levelTestMode()) printLevelTestStatus();
}

bool StepperMotion::levelTestMode() const {
  portENTER_CRITICAL(&stateMux_);
  const bool value = levelTestMode_;
  portEXIT_CRITICAL(&stateMux_);
  return value;
}

bool StepperMotion::levelTestActive() const {
  portENTER_CRITICAL(&stateMux_);
  const bool value = levelTestActive_;
  portEXIT_CRITICAL(&stateMux_);
  return value;
}

bool StepperMotion::startLevelTest() {
  if (isActive() || levelTestActive() || motionStopPending()) {
    setError("step_level_test_busy_or_stop_pending");
    return false;
  }
  if (!pulseEngineReady()) {
    setError("pulse_engine_unavailable");
    return false;
  }
  emergencyStopAndDisable();  // Stop GPTimer, invalidate homing; preserve travel.
  portENTER_CRITICAL(&stateMux_);
  levelTestMode_ = true;
  portEXIT_CRITICAL(&stateMux_);
  if (!pulseEngineReady() || !validateOutputPins()) {
    latchTimerFault("step_level_test_pin_configuration_failed");
    return false;
  }
  if (gpio_get_level(static_cast<gpio_num_t>(stepPin_)) != LOW) {
    latchTimerFault("step_level_test_idle_not_low");
    return false;
  }
  portENTER_CRITICAL(&stateMux_);
  levelTestActive_ = true;
  levelTestPhase_ = 0;
  levelTestReportedPhase_ = UINT8_MAX;
  stopRequested_ = false;
  timerAlarmGeneration_ = moveGeneration_;
  portEXIT_CRITICAL(&stateMux_);
  clearError();
  if (!startPulseTimer(kLevelTestTickUs)) return false;
  return true;
}

bool IRAM_ATTR StepperMotion::levelTestTimerAlarm(gptimer_handle_t timer) {
  uint64_t nowUs = 0;
  bool finish = false;
  portENTER_CRITICAL_ISR(&stateMux_);
  if (!levelTestActive_ || stopRequested_ || motionStopPending() ||
      timerAlarmGeneration_ != moveGeneration_) {
    finish = true;
  } else if (gptimer_get_raw_count(timer, &nowUs) != ESP_OK) {
    setTimerFaultStateFromIsrLocked();
    finish = true;
  } else if (nowUs >= kLevelTestDurationUs) {
    finish = true;
  } else {
    levelTestPhase_ = static_cast<uint8_t>(nowUs / kLevelTestPhaseUs);
    const uint32_t level = (levelTestPhase_ & 1U) ? HIGH : LOW;
    const esp_err_t first = gpio_set_level(static_cast<gpio_num_t>(stepPin_), level);
    esp_rom_delay_us(1);
    uint8_t fault = 0;
    if (first != ESP_OK || gpio_get_level(static_cast<gpio_num_t>(stepPin_)) != level) fault |= 1;
    if (fault) {
      gpioFaultMask_ = fault;
      setTimerFaultStateFromIsrLocked();
      finish = true;
    }
  }
  if (finish) {
    const esp_err_t firstLow = gpio_set_level(static_cast<gpio_num_t>(stepPin_), LOW);
    esp_rom_delay_us(1);
    uint8_t lowFault = 0;
    if (firstLow != ESP_OK || gpio_get_level(static_cast<gpio_num_t>(stepPin_)) != LOW) lowFault |= 4;
    if (lowFault) {
      gpioFaultMask_ |= lowFault;
      setTimerFaultStateFromIsrLocked();
    }
    levelTestActive_ = false;
    levelTestPhase_ = kLevelTestPhases;
    stopRequested_ = true;
    timerRunning_ = false;
  }
  portEXIT_CRITICAL_ISR(&stateMux_);
  if (finish) {
    const esp_err_t result = gptimer_stop(timer);
    if (result != ESP_OK && result != ESP_ERR_INVALID_STATE) latchTimerFaultFromIsr(timer);
    return false;
  }
  // Fixed absolute end; a delayed main loop never lengthens this diagnostic.
  gptimer_alarm_config_t alarm = {};
  alarm.alarm_count = nowUs + kLevelTestTickUs;
  if (alarm.alarm_count > kLevelTestDurationUs) alarm.alarm_count = kLevelTestDurationUs;
  if (gptimer_set_alarm_action(timer, &alarm) != ESP_OK) latchTimerFaultFromIsr(timer);
  return false;
}

void StepperMotion::printLevelTestStatus() const {
  portENTER_CRITICAL(&stateMux_);
  const bool mode = levelTestMode_;
  const bool active = levelTestActive_;
  const uint8_t phase = levelTestPhase_;
  portEXIT_CRITICAL(&stateMux_);
  Serial.printf("STEP_LEVEL_TEST mode=%s active=%s phase=%u/6 GPIO%u=%d; "
                "5s LOW/HIGH x3; 30s end LOW; reboot required for motion; not motor feedback\n",
                mode ? "latched" : "off", active ? "yes" : "no", phase,
                stepPin_, gpio_get_level(static_cast<gpio_num_t>(stepPin_)));
}


// ---------- Pin plan ----------
// Sophicar carrier pin plan. GPIO35..38 select the two onboard DRV8870 channels,
// so the ESP32-S3-N16R8 must be compiled and flashed with PSRAM disabled.
constexpr int PIN_HTD_TX = 6;   // ESP32 TX -> BusLinker RX
constexpr int PIN_HTD_RX = 7;   // ESP32 RX <- BusLinker TX

// First folding joint: external A4988 + MINIFZ14-80. All logic wires are taken
// directly from the ESP32 GPIO3/GPIO8/GPIO10/3V3/GND pins; STP slots stay empty.
// Quarter-step hardware: MS1=GND, MS2=3V3, MS3=GND. ENABLE is active LOW with an
// external approximately 10 kOhm pull-up to 3.3 V.
constexpr uint8_t PIN_FOLD_DIR = 3;
constexpr uint8_t PIN_FOLD_STEP = 8;
constexpr int8_t PIN_FOLD_ENABLE = 10;
constexpr uint16_t FOLD_ACTION_DURATION_MS = 6000;
// Reserve 200 ms for cumulative ISR/software overhead; never accelerate catch-up.
constexpr uint16_t FOLD_STEPPER_PLAN_MS = 5800;
constexpr uint16_t FOLD_ACTION_TIMEOUT_MARGIN_MS = 2000;
constexpr uint64_t FOLD_COMPLETION_LIMIT_US = 6000000ULL;
// User-saved endpoints after reassembly on 2026-09-11: folded=38, unfolded=719.
// Powered servo travel, mechanical clearance and synchronized motion await validation.
constexpr int16_t FOLD_ID11_FOLDED_POSITION = 38;
constexpr int16_t FOLD_ID11_UNFOLDED_POSITION = 719;
// Quarter-step conversion invalidates every saved 1/16 travel, including small
// counts that would otherwise pass the new numeric range. Servo NVS is untouched.
constexpr uint32_t FOLD_MOTION_SCHEMA_VERSION = 4;
constexpr char FOLD_NVS_NAMESPACE[] = "fold-demo";
constexpr char FOLD_NVS_KEY_SCHEMA[] = "motion_schema";
constexpr char FOLD_NVS_KEY_TRAVEL_VALID[] = "travel_valid";
constexpr char FOLD_NVS_KEY_TRAVEL_STEPS[] = "travel_steps";
constexpr char FOLD_NVS_KEY_DIRECTION[] = "dir_inv";
static_assert(FOLD_COMPLETION_LIMIT_US ==
                  static_cast<uint64_t>(FOLD_ACTION_DURATION_MS) * 1000ULL,
              "fold GPTimer hard pulse limit must match 6 seconds");
static_assert(FOLD_STEPPER_PLAN_MS < FOLD_ACTION_DURATION_MS,
              "stepper planning must leave runtime timing headroom");

// HTD and HX share the same physical BusLinker/UART at 115200 baud. They still
// use different packet headers (HTD: 55 55; HX: FF FF), so every request is
// completed before the next protocol transaction begins.

constexpr uint32_t ACTION_TIMEOUT_GRACE_MS = 1000;
constexpr uint32_t MAX_BASE_DURATION_SEC = 2147482;

// Onboard brushed-DC drive.  M1 carries both left motors in parallel and M2
// carries both right motors in parallel.  There is no separate PWM pin: PWM is
// applied directly to one IN pin while the other IN pin stays LOW.
constexpr int PIN_ONBOARD_M1_IN1 = 38;
constexpr int PIN_ONBOARD_M1_IN2 = 37;
constexpr int PIN_ONBOARD_M2_IN1 = 36;
constexpr int PIN_ONBOARD_M2_IN2 = 35;
// Preserve the previously verified logical left/right direction convention.
// If a complete side runs backwards after rewiring, change only its flag.  If
// one motor on a parallel pair runs backwards, swap only that motor's two leads.
constexpr bool ONBOARD_M1_INVERTED = true;
constexpr bool ONBOARD_M2_INVERTED = false;
constexpr int DRIVE_TEST_PWM = 45;   // Low first-test PWM, 0..255
constexpr uint16_t DRIVE_TEST_MS = 500;
constexpr int DRIVE_PROTOCOL_MAX_PWM = 120;
constexpr int BASE_DEMO_PWM = 105;
constexpr uint32_t BASE_FORWARD_MS = 2500;
constexpr uint32_t BASE_TURN_MS = 1000;
constexpr bool ENABLE_BLOCKING_USB_DEBUG = false;
constexpr bool ENABLE_DRIVE_PWM_TRACE = false;
constexpr uint32_t CALIBRATION_SCHEMA_VERSION = 2;

// ---------- Upper-computer protocol ----------
// The host computer sends one JSON object per line over USB Serial, 115200 baud.
// Non-JSON debug commands such as r_ping / h_pos / d_pwm are still supported.
constexpr const char *PROTOCOL_VERSION = "esp32s3-actuator-v0.1";
constexpr uint32_t HOST_WATCHDOG_MS = 1000;

// ---------- Direct phone/tablet web control ----------
constexpr const char *WEB_AP_SSID = "Suzhou-FoldBot";
constexpr const char *WEB_AP_PASSWORD = "suzhou2026";
const IPAddress WEB_AP_IP(192, 168, 4, 1);
const IPAddress WEB_AP_GATEWAY(192, 168, 4, 1);
const IPAddress WEB_AP_SUBNET(255, 255, 255, 0);
constexpr uint16_t WEB_HTTP_PORT = 80;
constexpr uint32_t WEB_CONTROL_LEASE_MS = 750;
constexpr uint32_t WEB_OWNER_TIMEOUT_MS = 2000;
constexpr uint32_t WEB_DRIVE_COMMAND_TIMEOUT_MS = 250;
constexpr uint32_t WEB_STATUS_INTERVAL_MS = 100;
constexpr uint32_t WEB_JOINT_POLL_INTERVAL_MS = 100;
constexpr uint32_t WEB_JOINT_ONLINE_WINDOW_MS = 1500;
constexpr uint32_t ARM_CONTROL_PERIOD_MS = 20;
constexpr uint32_t ARM_MOTION_WATCHDOG_MS = 120;
constexpr uint32_t ARM_POSITION_STALE_MS = 500;
constexpr int32_t HX30_TEMP_MIN_POSITION = -10000;
constexpr int32_t HX30_TEMP_MAX_POSITION = 10000;
constexpr int16_t HX30_JOG_STEP = 8;
// ID15 was removed: the first folding joint is now the onboard S2 stepper.  Only
// ID11 remains in the BusLinker folding chain.
constexpr uint8_t WEB_JOINT_POLL_IDS[] = {1, 2, 3, 4, 5, 6, 11};
constexpr bool WEB_JOINT_POLL_ENABLED = true;
constexpr size_t WEB_MAX_FRAME_BYTES = 512;
constexpr size_t WEB_COMMAND_QUEUE_DEPTH = 24;
constexpr int WEB_DRIVE_FULL_PWM = 255;
constexpr uint8_t WEB_DRIVE_POWER_LOW_PERCENT = 30;
constexpr uint8_t WEB_DRIVE_POWER_MEDIUM_PERCENT = 60;
constexpr uint8_t WEB_DRIVE_POWER_HIGH_PERCENT = 100;
constexpr uint32_t WEB_HTTP_KEEPALIVE_IDLE_S = 5;
constexpr uint32_t WEB_HTTP_KEEPALIVE_INTERVAL_S = 3;
constexpr uint32_t WEB_HTTP_KEEPALIVE_COUNT = 2;
constexpr size_t WEB_HTTP_MAX_OPEN_SOCKETS = 4;
constexpr uint32_t WEB_HTTP_SESSION_AUDIT_MS = 1000;

// ---------- HTD / Hiwonder bus servo constants ----------
HardwareSerial HtdSerial(1);
constexpr uint8_t HTD_ID = 11;
constexpr uint32_t HTD_DEFAULT_BAUD = 115200;
constexpr uint32_t ARM_HX_BAUD = HTD_DEFAULT_BAUD;
uint32_t htdBaud = HTD_DEFAULT_BAUD;

constexpr uint8_t HTD_CMD_MOVE_TIME_WRITE = 1;
constexpr uint8_t HTD_CMD_MOVE_STOP = 12;
constexpr uint8_t HTD_CMD_ID_READ = 14;
constexpr uint8_t HTD_CMD_ANGLE_LIMIT_WRITE = 20;
constexpr uint8_t HTD_CMD_ANGLE_LIMIT_READ = 21;
constexpr uint8_t HTD_CMD_POS_READ = 28;
constexpr uint8_t HTD_CMD_LOAD_OR_UNLOAD_WRITE = 31;
constexpr uint8_t HTD_CMD_LOAD_OR_UNLOAD_READ = 32;

constexpr int HTD_SMALL_DELTA_UNITS = 20;
constexpr uint16_t HTD_MOVE_TIME_MS = 1000;
constexpr int HTD_POSITION_MIN = 0;
constexpr int HTD_POSITION_MAX = 1000;
constexpr uint8_t HTD_POSITION_READ_RETRIES = 3;
constexpr uint16_t HX_DEMO_SPEED = 200;
constexpr uint8_t HX_DEMO_ACCEL = 20;
enum class ServoProtocol : uint8_t { Htd, Hx };

struct ServoDescriptor {
  uint8_t id;
  ServoProtocol protocol;
  const char *name;
  const char *model;
};

constexpr ServoDescriptor SERVOS[] = {
    {1, ServoProtocol::Hx, "arm_joint_1_base", "HX-30HM"},
    {2, ServoProtocol::Hx, "arm_joint_2_shoulder", "HX-30HM"},
    {3, ServoProtocol::Hx, "arm_joint_3_elbow", "HX-30HM"},
    {4, ServoProtocol::Hx, "arm_joint_4_wrist_pitch", "HX-30HM"},
    {5, ServoProtocol::Hx, "arm_joint_5_wrist_roll", "HX-30HM"},
    {6, ServoProtocol::Hx, "arm_gripper", "HX-30HM"},
    {11, ServoProtocol::Htd, "torso_second_board", "HTD-85H"},
};
constexpr size_t SERVO_COUNT = sizeof(SERVOS) / sizeof(SERVOS[0]);
constexpr size_t ARM_SERVO_COUNT = 6;
static_assert(ARM_SERVO_COUNT == 6, "new arm requires six HX-30HM servos");

struct JointCalibration {
  int32_t minimum = 0;
  int32_t center = 0;
  int32_t maximum = 0;
  bool inverted = false;
  bool calibrated = false;
};

struct BaseMotionState {
  bool active = false;
  uint32_t deadlineMs = 0;
  int leftPwm = 0;
  int rightPwm = 0;
};

// SHOWCASE_CORE_BEGIN
namespace Showcase {
struct Profile {
  bool pathVerified = false;
  int32_t pose[4][6]; // folded, raised, left, right; raw HX position units
  int32_t gripperOpen;
  uint16_t speed[6];
  uint8_t acceleration[6];
  uint16_t tolerance[6];
};
struct Runtime {
  bool active = false;
  uint8_t stage = 0;
  uint8_t sendAxis = 0;
  uint8_t readAxis = 0;
  bool roundWithin = true;
  uint8_t settledRounds = 0;
  uint32_t stageAt = 0;
  bool verifyTorso = false;
};
enum class Result { Running, Finished, Failed };
bool allowed();
bool startTorso();
bool torsoDone();
bool torsoFailed();
bool readTorso(int32_t &position);
bool commandAxis(uint8_t id, int32_t target, uint16_t speed, uint8_t acceleration);
bool readAxis(uint8_t id, int32_t &position);

bool configured(const Profile &p, const char *&error) {
  error = "showcase_profile_unconfigured";
  if (!p.pathVerified) return false;
  for (uint8_t axis = 0; axis < 6; ++axis) {
    if (!p.speed[axis] || p.speed[axis] > 200 || !p.acceleration[axis] ||
        p.acceleration[axis] > 20 || !p.tolerance[axis] || p.tolerance[axis] > 50) return false;
    for (uint8_t pose = 0; pose < 4; ++pose)
      if (p.pose[pose][axis] < INT16_MIN || p.pose[pose][axis] > INT16_MAX) return false;
  }
  const int64_t gripDelta = static_cast<int64_t>(p.gripperOpen) - p.pose[1][5];
  if (p.gripperOpen < INT16_MIN || p.gripperOpen > INT16_MAX ||
      (gripDelta >= -2 * static_cast<int64_t>(p.tolerance[5]) &&
       gripDelta <= 2 * static_cast<int64_t>(p.tolerance[5]))) return false;
  for (uint8_t pose = 0; pose < 4; ++pose)
    if (p.pose[pose][5] != p.pose[1][5]) return false;
  const uint8_t pairs[6][2] = {{0,1}, {0,2}, {0,3}, {1,2}, {1,3}, {2,3}};
  for (const auto &pair : pairs) {
    bool differs = false;
    for (uint8_t axis = 0; axis < 5; ++axis)
      differs = differs || p.pose[pair[0]][axis] != p.pose[pair[1]][axis];
    if (!differs) return false;
  }
  error = "";
  return true;
}
bool begin(Runtime &r, const Profile &p, uint32_t now, const char *&error) {
  r = Runtime();
  if (!configured(p, error)) return false;
  error = "showcase_stop_or_lease";
  if (!allowed()) return false;
  error = "showcase_torso_start_failed";
  if (!startTorso()) return false;
  error = "showcase_stop_or_lease";
  if (!allowed()) return false;
  r.active = true;
  r.stageAt = now;
  error = "";
  return true;
}
int32_t targetFor(const Profile &p, uint8_t stage, uint8_t axis) {
  if (stage == 4 && axis == 5) return p.gripperOpen;
  return p.pose[stage == 1 ? 2 : stage == 2 ? 3 : 1][axis];
}
Result fail(Runtime &r, const char *&error, const char *reason) {
  r.active = false;
  error = reason;
  return Result::Failed;
}
Result advance(Runtime &r, uint32_t now) {
  ++r.stage;
  r.stageAt = now;
  r.sendAxis = r.stage >= 4 ? 5 : 0;
  r.readAxis = 0;
  r.roundWithin = true;
  r.settledRounds = 0;
  r.verifyTorso = false;
  if (r.stage == 6) { r.active = false; return Result::Finished; }
  return Result::Running;
}
Result service(Runtime &r, const Profile &p, uint32_t now, const char *&error) {
  if (!r.active) return r.stage == 6 ? Result::Finished : Result::Failed;
  if (!allowed()) return fail(r, error, "showcase_stop_or_lease");
  if (torsoFailed()) return fail(r, error, "showcase_torso_failed");
  const uint32_t elapsed = now - r.stageAt;
  if (elapsed >= 12000) return fail(r, error, "showcase_feedback_timeout");
  if (r.sendAxis < 6) {
    const uint8_t a = r.sendAxis;
    const bool ok = commandAxis(a + 1, targetFor(p, r.stage, a), p.speed[a], p.acceleration[a]);
    if (!allowed()) return fail(r, error, "showcase_stop_or_lease");
    if (!ok) return fail(r, error, "showcase_axis_command_failed");
    ++r.sendAxis;
    return Result::Running;
  }
  if (r.verifyTorso) {
    int32_t position = 0;
    const bool ok = readTorso(position);
    if (!allowed()) return fail(r, error, "showcase_stop_or_lease");
    if (!ok) return fail(r, error, "showcase_torso_read_failed");
    if (position >= 714 && position <= 724 && torsoDone()) return advance(r, now);
    r.verifyTorso = false;
    r.settledRounds = 0;
    return Result::Running;
  }
  int32_t position = 0;
  const uint8_t a = r.readAxis;
  const bool ok = readAxis(a + 1, position);
  if (!allowed()) return fail(r, error, "showcase_stop_or_lease");
  if (!ok) return fail(r, error, "showcase_axis_read_failed");
  const int64_t delta = static_cast<int64_t>(position) - targetFor(p, r.stage, a);
  r.roundWithin = r.roundWithin && delta >= -static_cast<int64_t>(p.tolerance[a]) &&
                  delta <= p.tolerance[a];
  if (++r.readAxis == 6) {
    r.readAxis = 0;
    r.settledRounds = r.roundWithin ? (r.settledRounds < 2 ? r.settledRounds + 1 : 2) : 0;
    r.roundWithin = true;
    const uint32_t dwell = r.stage == 0 ? 6000 : r.stage < 4 ? 2000 : 1200;
    if (r.settledRounds >= 2 && elapsed >= dwell) {
      if (r.stage != 0) return advance(r, now);
      if (torsoDone()) r.verifyTorso = true;
    }
  }
  return Result::Running;
}
} // namespace Showcase
// SHOWCASE_CORE_END

enum class RobotActionKind : uint8_t {
  None,
  BaseForward,
  BaseBackward,
  TurnLeft,
  TurnRight,
  TorsoUnfold,
  TorsoFold,
  // SHOWCASE_ENUM_BEGIN
  Showcase
  // SHOWCASE_ENUM_END
};

enum class ControlOwner : uint8_t {
  None,
  Web,
  Usb
};

struct RobotActionState {
  RobotActionKind kind = RobotActionKind::None;
  int seq = 0;
  uint8_t phase = 0;
  uint32_t deadlineMs = 0;
  uint32_t timeoutDeadlineMs = 0;
  bool active = false;
};

struct HxHoldState {
  bool active = false;
  uint8_t servoCursor = 0;
};

struct PoseDefinition {
  const char *name;
  const char *prefix;
  const uint8_t *ids;
  uint8_t count;
};

// First carbon board is driven by external A4988; ID11 drives the second carbon board.
constexpr uint8_t TORSO_SECOND_BOARD_ID = 11;
// Kept as the legacy/default diagnostic ID for the existing serial commands.
constexpr uint8_t TORSO_LOWER_ID = TORSO_SECOND_BOARD_ID;
// Keep folding-board moves deliberately slow for the current prototype.
constexpr uint16_t TORSO_LOWER_ACTION_TIME_MS = 6000;
constexpr uint8_t TORSO_IDS[] = {11};
constexpr PoseDefinition POSES[] = {
    {"torso_fold", "tf", TORSO_IDS, 1},
    {"torso_unfold", "tu", TORSO_IDS, 1},
};
enum class WebCommandType : uint8_t {
  Hello,
  Heartbeat,
  StatusRequest,
  MotionEnable,
  ArmExternal,
  DriveSpeed,
  Drive,
  DriveStop,
  Action,
  JogState,
  JogStep,
  FoldZero,
  FoldConfirmUnfolded,
  FoldJog,
  FoldSave,
  FoldReset,
  FoldToggleDirection,
  FoldRescan,
  Stop
};

struct WebControlCommand {
  WebCommandType type = WebCommandType::StatusRequest;
  int clientFd = -1;
  uint32_t driveGeneration = 0;
  uint32_t controlGeneration = 0;
  uint32_t armSequence = 0;
  uint32_t receivedAtMs = 0;
  int valueA = 0;
  int valueB = 0;
  int valueC = 0;
  bool enabled = false;
  int8_t armDirections[ARM_SERVO_COUNT] = {0};
  char action[32] = {0};
};

String inputLine;
bool serialLineDiscarding = false;
constexpr uint16_t SERIAL_RX_BYTES_PER_LOOP = 64;
String robotState = "stopped";
bool emergencyStopLatched = false;
volatile long driveTicks = 0;
int driveLeftPwm = 0;
int driveRightPwm = 0;
bool drivePwmReady = false;
bool drivePwmFault = false;
BaseMotionState baseMotion;
RobotActionState robotAction;
ControlOwner robotActionOwner = ControlOwner::None;
Preferences jointPrefs;
Preferences foldPrefs;
StepperMotion foldStepper(PIN_FOLD_DIR, PIN_FOLD_STEP, PIN_FOLD_ENABLE);

enum class FoldControlState : uint8_t {
  ZeroRequired,
  Idle,
  Calibrating,
  Unfolding,
  Folding,
  Stopped,
  Fault,
};

FoldControlState foldState = FoldControlState::ZeroRequired;
bool foldPreferencesReady = false;
bool foldPulseEngineStarted = false;
bool foldId11Online = false;
bool foldFormalCompletionObserved = false;
// Informational execution state, not a persistent mode or offline fallback.
bool foldActionStepperOnly = false;
uint32_t foldActionFinishMs = 0;
uint32_t foldActionDeadlineMs = 0;
String foldError;
JointCalibration jointCal[SERVO_COUNT];
int32_t lastServoPosition[SERVO_COUNT] = {0};
uint32_t lastServoPositionMs[SERVO_COUNT] = {0};
bool lastServoPositionValid[SERVO_COUNT] = {false};
HxHoldState hxHold;
int selectedServoIndex = -1;
bool jsonProtocolActive = false;
unsigned long lastHostCommandMs = 0;

DNSServer webDnsServer;
httpd_handle_t webHttpServer = nullptr;
QueueHandle_t webCommandQueue = nullptr;
portMUX_TYPE webTxMux = portMUX_INITIALIZER_UNLOCKED;
uint8_t webPendingTx = 0;
volatile int webActiveClientFd = -1;
volatile bool webDisconnectStopRequested = false;
volatile bool webImmediateStopRequested = false;
volatile bool webImmediateDriveStopRequested = false;
volatile uint32_t webDriveGeneration = 0;
volatile uint32_t webControlGeneration = 0;
volatile bool webStationDisconnectEvent = false;
volatile bool webFreshAssociationPending = false;
volatile bool webFreshAssociationForced = false;
volatile uint16_t webFreshAssociationAid = 0;
uint8_t webLastHttpClientCount = UINT8_MAX;
uint32_t lastWebHttpSessionAuditMs = 0;
bool webClientConnected = false;
bool webMotionEnabled = false;
// RAM-only HX inhibition. Boot/first connection may still access HX: physical
// isolation is required before attaching another controller to the arm bus.
bool externalArm = false;
uint32_t webOwnerReceiptMs = 0; // guarded by webLeaseMux
uint32_t webControlSession = 0; // increments for each accepted connection
uint32_t webStatusSequence = 0;
bool webDriveActive = false;
bool webOwnsMotion = false;
uint8_t webLastStationCount = 0;
int webLinearCommand = 0;
int webTurnCommand = 0;
uint8_t webDrivePowerPercent = WEB_DRIVE_POWER_MEDIUM_PERCENT;
uint32_t lastWebHeartbeatMs = 0;
// V5 lease state: accessed only inside webLeaseMux, including the timestamp above.
portMUX_TYPE webLeaseMux = portMUX_INITIALIZER_UNLOCKED;
bool webLeaseValid = false;
int webLeaseClientFd = -1;
uint32_t webLeaseGeneration = 0;
// End V5 lease state.
uint32_t lastWebDriveCommandMs = 0;
uint32_t lastWebStatusMs = 0;
uint32_t lastWebJointPollMs = 0;
uint8_t webJointPollCursor = 0;
bool webServoOnline[SERVO_COUNT] = {false};
uint32_t webServoLastConfirmedMs[SERVO_COUNT] = {0};
int8_t webArmDirections[ARM_SERVO_COUNT] = {0};
int32_t webArmShadowTargets[ARM_SERVO_COUNT] = {0};
bool webArmShadowValid[ARM_SERVO_COUNT] = {false};
bool webArmTorqueReady[ARM_SERVO_COUNT] = {false};
uint32_t webArmPositionSequence[ARM_SERVO_COUNT] = {0};
uint32_t webArmLastJogSequence = 0;
uint32_t webArmAppliedJogSequence = 0;
uint32_t webArmJogStepSequence = 0;
const char *webArmJogStepStatus = "idle";
uint32_t webArmLastNonZeroCommandMs = 0;
uint32_t webArmLastMotionTickMs = 0;
bool webArmWatchdogArmed = false;
uint8_t webArmHoldPendingMask = 0;
uint8_t webArmMotionCursor = 0;

void allStop();
void serviceBaseMotion();
void serviceRobotAction();
void serviceHxHold();
// SHOWCASE_GLUE_BEGIN
// Explicit declarations: Arduino does not reliably generate namespace dependencies.
int servoIndex(uint8_t id);
bool htdReadPositionById(uint8_t servoId, int &position, uint32_t timeoutMs, bool verbose);
bool hxReadPosition(uint8_t id, int32_t &position, uint32_t timeoutMs);
void webMarkMotionActive();
void rejectActiveAction(const char *reason);
// Fill with MEASURED safe poses after assembly; never copy host-test fixture values.
// Rows: folded / raised / left / right. Columns: HX IDs 1..6, raw position units.
// pathVerified certifies intermediate swept paths, not just endpoint clearance.
const Showcase::Profile SHOWCASE_PROFILE = {
  false,
  {{INT32_MIN,INT32_MIN,INT32_MIN,INT32_MIN,INT32_MIN,INT32_MIN},
   {INT32_MIN,INT32_MIN,INT32_MIN,INT32_MIN,INT32_MIN,INT32_MIN},
   {INT32_MIN,INT32_MIN,INT32_MIN,INT32_MIN,INT32_MIN,INT32_MIN},
   {INT32_MIN,INT32_MIN,INT32_MIN,INT32_MIN,INT32_MIN,INT32_MIN}},
  INT32_MIN, {0,0,0,0,0,0}, {0,0,0,0,0,0}, {0,0,0,0,0,0}
};
Showcase::Runtime showcaseRuntime;
uint32_t showcaseGeneration = 0;
ControlOwner showcaseOwner = ControlOwner::None;

bool showcaseSetupReady(const char *&reason) {
  if (externalArm) { reason = "external_arm"; return false; }
  if (!Showcase::configured(SHOWCASE_PROFILE, reason)) return false;
  for (uint8_t id = 1; id <= 6; ++id) {
    const int index = servoIndex(id);
    reason = "showcase_arm_calibration_required";
    if (index < 0 || SERVOS[index].protocol != ServoProtocol::Hx || !jointCal[index].calibrated) return false;
    const auto &cal = jointCal[index];
    reason = "showcase_target_outside_range";
    if (cal.minimum < servoHardMinimum(SERVOS[index]) || cal.maximum > servoHardMaximum(SERVOS[index]) ||
        !(cal.minimum < cal.center && cal.center < cal.maximum)) return false;
    for (uint8_t pose = 0; pose < 4; ++pose) {
      const int32_t target = SHOWCASE_PROFILE.pose[pose][id-1];
      if (target < cal.minimum || target > cal.maximum) return false;
    }
    if (id == 6 && (SHOWCASE_PROFILE.gripperOpen < cal.minimum || SHOWCASE_PROFILE.gripperOpen > cal.maximum)) return false;
  }
  reason = "";
  return true;
}
namespace Showcase {
bool allowed() {
  return !motionSafetyLocked() && !motionStopPending() &&
      (showcaseOwner == ControlOwner::Usb ||
       (showcaseOwner == ControlOwner::Web && webMotionStartAllowed(showcaseGeneration) && webLeaseFresh(showcaseGeneration)));
}
bool startTorso() { return allowed() && startTorsoLowerAction(true) && allowed(); }
bool torsoDone() {
  return foldState == FoldControlState::Idle && foldStepper.isHomed() &&
         foldStepper.currentSteps() == foldStepper.travelSteps();
}
bool torsoFailed() { return foldState == FoldControlState::Fault || foldState == FoldControlState::Stopped; }
bool readTorso(int32_t &position) {
  int actual = 0;
  if (!allowed() || !htdReadPositionById(11, actual, 40, false)) return false;
  position = actual;
  return allowed();
}
bool readAxis(uint8_t id, int32_t &position) {
  const int index = servoIndex(id);
  return index >= 0 && jointCal[index].calibrated && allowed() &&
         hxReadPosition(id, position, 15) && allowed() &&
         position >= jointCal[index].minimum && position <= jointCal[index].maximum;
}
bool commandAxis(uint8_t id, int32_t target, uint16_t speed, uint8_t acceleration) {
  if (!allowed()) return false;
  const uint8_t torque = 1;
  if (!hxWrite(id, 0x28, &torque, 1) || !allowed()) return false;
  delay(5);
  if (!allowed()) return false;
  const uint16_t raw = hxEncodeSignedPosition(target);
  const uint8_t profile[] = {acceleration, static_cast<uint8_t>(raw), static_cast<uint8_t>(raw >> 8),
      0, 0, static_cast<uint8_t>(speed), static_cast<uint8_t>(speed >> 8)};
  return hxWrite(id, 0x29, profile, sizeof(profile)) && allowed();
}
}
bool startShowcaseAction(ControlOwner owner, int seq, String &reason) {
  if (robotAction.active || baseMotion.active || webDriveActive || webOwnsMotion || isWebJogActive() || foldAnyMotionActive()) {
    reason = "busy"; return false;
  }
  showcaseGeneration = webGetControlGeneration();
  showcaseOwner = owner;
  const char *error = "";
  if (!showcaseSetupReady(error)) { reason = error; return false; }
  if (!Showcase::allowed()) { reason = "showcase_stop_or_lease"; return false; }
  if (!foldStepper.isHomed() || !foldStepper.hasTravel() || foldStepper.currentSteps() != 0) {
    reason = "showcase_confirm_folded_start"; return false;
  }
  if (!validateTorsoLowerAction(reason, true)) return false;
  if (!Showcase::allowed()) { reason = "showcase_stop_or_lease"; return false; }
  int32_t torso = 0;
  if (!Showcase::readTorso(torso) || torso < 33 || torso > 43) {
    reason = "showcase_id11_not_folded"; return false;
  }
  for (uint8_t id = 1; id <= 6; ++id) {
    int32_t actual = 0;
    if (!Showcase::readAxis(id, actual)) { reason = "showcase_arm_read_failed"; return false; }
    const int64_t delta = static_cast<int64_t>(actual) - SHOWCASE_PROFILE.pose[0][id-1];
    if (delta < -static_cast<int64_t>(SHOWCASE_PROFILE.tolerance[id-1]) || delta > SHOWCASE_PROFILE.tolerance[id-1]) {
      reason = "showcase_arm_not_at_folded_pose"; return false;
    }
  }
  if (!Showcase::allowed()) { reason = "showcase_stop_or_lease"; return false; }
  robotAction = RobotActionState();
  robotAction.kind = RobotActionKind::Showcase;
  robotAction.seq = seq;
  robotAction.active = true;
  robotActionOwner = owner;
  if (!Showcase::begin(showcaseRuntime, SHOWCASE_PROFILE, millis(), error)) {
    allStop(); reason = error; return false;
  }
  robotState = "running";
  if (owner == ControlOwner::Web) webMarkMotionActive();
  Serial.println("SHOWCASE started: unfold+raise, left, right, raised, grip_open, grip_close; base excluded");
  return true;
}
void serviceShowcaseAction() {
  const char *error = "";
  const auto result = Showcase::service(showcaseRuntime, SHOWCASE_PROFILE, millis(), error);
  if (robotAction.phase != showcaseRuntime.stage) {
    robotAction.phase = showcaseRuntime.stage;
    Serial.printf("SHOWCASE stage=%u\n", showcaseRuntime.stage);
  }
  if (result == Showcase::Result::Failed) rejectActiveAction(error);
  else if (result == Showcase::Result::Finished) {
    Serial.println("SHOWCASE complete: fresh final arm feedback, holding raised pose; no automatic fold");
    finishRobotAction();
  }
}
// SHOWCASE_GLUE_END

bool startNamedAction(const String &name, int seq, ControlOwner owner, String &rejectReason,
                      uint32_t baseDurationMs = 0);
void webMarkMotionActive();
void webMarkMotionIdle();
void rejectActiveAction(const char *reason);
void stopForWebControl();
void htdSetTorqueEnabled(uint8_t servoId, bool enabled);
int servoIndex(uint8_t id);
void beginWebControl();
void serviceWebControl();
void webResetClientSessions(const char *reason);
void webAuditHttpSessions();
void serviceWebArmMotion();
void serviceFoldMotion();
bool foldConfirmZero(String &reason);
bool foldConfirmUnfolded(String &reason);
bool foldJog(int32_t steps, String &reason);
bool foldSaveTravel(String &reason);
bool foldResetTravel(String &reason);
bool foldToggleDirection(String &reason);
bool foldRescan(String &reason);
bool validateFoldStepperAction(String &reason);
bool startStepperOnlyTest(bool unfolding, uint32_t durationMs, String &reason);
bool handleStepperOnlyTestCommand(const String &command);

uint32_t webGetControlGeneration() {
  return __atomic_load_n(&webControlGeneration, __ATOMIC_RELAXED);
}

void webInvalidateControlCommands() {
  __atomic_add_fetch(&webControlGeneration, 1U, __ATOMIC_RELAXED);
}

bool motionStopPending() {
  return webImmediateStopRequested || webDisconnectStopRequested ||
         emergencyStopLatched || drivePwmFault;
}

bool webMotionStartAllowed(uint32_t generation) {
  return webMotionEnabled && !foldStepper.levelTestMode() && !motionStopPending() &&
         generation == webGetControlGeneration();
}

// V5 lease receipt: HTTP ingress does not wait behind blocking UART commands.
void webNoteLeaseReceipt(int fd, uint32_t generation, uint32_t receivedAtMs) {
  portENTER_CRITICAL(&webLeaseMux);
  if (fd >= 0 && fd == webActiveClientFd && generation == webGetControlGeneration() &&
      !motionStopPending()) {
    // A command processed after UART preflight must not overwrite a newer
    // heartbeat. Signed difference permits normal uint32_t millis() wrap.
    if (!webLeaseValid || webLeaseClientFd != fd || webLeaseGeneration != generation ||
        static_cast<int32_t>(receivedAtMs - lastWebHeartbeatMs) >= 0) {
      webLeaseValid = true;
      webLeaseClientFd = fd;
      webLeaseGeneration = generation;
      lastWebHeartbeatMs = receivedAtMs;
    }
  }
  portEXIT_CRITICAL(&webLeaseMux);
}

uint32_t webLeaseAgeMs(uint32_t generation) {
  portENTER_CRITICAL(&webLeaseMux);
  // Read the clock AFTER the receipt snapshot under the same lock: an HTTP
  // update cannot introduce a timestamp newer than this watchdog sample.
  const uint32_t receivedAtMs = lastWebHeartbeatMs;
  const bool valid = webLeaseValid && webLeaseClientFd == webActiveClientFd &&
      webActiveClientFd >= 0 && webLeaseGeneration == generation &&
      generation == webGetControlGeneration();
  const uint32_t now = millis();
  const uint32_t age = valid ? now - receivedAtMs : UINT32_MAX;
  portEXIT_CRITICAL(&webLeaseMux);
  return age;
}

bool webLeaseFresh(uint32_t generation) {
  return !motionStopPending() && webLeaseAgeMs(generation) <= WEB_CONTROL_LEASE_MS;
}
// End V5 lease receipt.

void webConfirmJointOnline(int index) {
  if (index < 0 || index >= static_cast<int>(SERVO_COUNT)) return;
  webServoOnline[index] = true;
  webServoLastConfirmedMs[index] = millis();
}

void webResetArmControlSession() {
  for (size_t i = 0; i < ARM_SERVO_COUNT; ++i) {
    webArmDirections[i] = 0;
    webArmShadowValid[i] = false;
    webArmTorqueReady[i] = false;
  }
  webArmLastJogSequence = 0;
  webArmAppliedJogSequence = 0;
  webArmJogStepSequence = 0;
  webArmJogStepStatus = "idle";
  webArmWatchdogArmed = false;
  webArmHoldPendingMask = 0;
}

uint32_t servoRegistrySignature() {
  uint32_t hash = 2166136261UL;
  for (const ServoDescriptor &servo : SERVOS) {
    hash = (hash ^ servo.id) * 16777619UL;
    hash = (hash ^ static_cast<uint8_t>(servo.protocol)) * 16777619UL;
    for (const char *p = servo.name; *p; ++p) {
      hash = (hash ^ static_cast<uint8_t>(*p)) * 16777619UL;
    }
  }
  return hash;
}

void printHexByte(uint8_t value) {
  if (value < 0x10) {
    Serial.print('0');
  }
  Serial.print(value, HEX);
}

uint8_t htdChecksum(uint8_t id, uint8_t length, uint8_t cmd, const uint8_t *params, uint8_t paramLen) {
  uint16_t sum = id + length + cmd;
  for (uint8_t i = 0; i < paramLen; i++) {
    sum += params[i];
  }
  return static_cast<uint8_t>(~sum);
}

void htdSend(uint8_t id, uint8_t cmd, const uint8_t *params, uint8_t paramLen) {
  uint8_t length = paramLen + 3;
  HtdSerial.write(0x55);
  HtdSerial.write(0x55);
  HtdSerial.write(id);
  HtdSerial.write(length);
  HtdSerial.write(cmd);
  for (uint8_t i = 0; i < paramLen; i++) {
    HtdSerial.write(params[i]);
  }
  HtdSerial.write(htdChecksum(id, length, cmd, params, paramLen));
  HtdSerial.flush();
}

bool htdReadResponse(uint8_t &id, uint8_t &cmd, uint8_t *params, uint8_t &paramLen,
                     uint32_t timeoutMs = 80, bool verbose = true) {
  unsigned long start = millis();
  int state = 0;

  while (millis() - start < timeoutMs) {
    if (!HtdSerial.available()) {
      delay(1);
      continue;
    }
    uint8_t byteValue = HtdSerial.read();
    if (state == 0 && byteValue == 0x55) {
      state = 1;
    } else if (state == 1 && byteValue == 0x55) {
      state = 2;
      break;
    } else {
      state = 0;
    }
  }

  if (state != 2) {
    if (verbose) Serial.println("HTD RX timeout/header not found");
    return false;
  }

  while (HtdSerial.available() < 3 && millis() - start < timeoutMs) {
    delay(1);
  }
  if (HtdSerial.available() < 3) {
    if (verbose) Serial.println("HTD RX short header");
    return false;
  }

  id = HtdSerial.read();
  uint8_t length = HtdSerial.read();
  cmd = HtdSerial.read();
  if (length < 3 || length > 10) {
    if (verbose) Serial.println("HTD RX invalid length");
    return false;
  }

  paramLen = length - 3;
  while (HtdSerial.available() < paramLen + 1 && millis() - start < timeoutMs) {
    delay(1);
  }
  if (HtdSerial.available() < paramLen + 1) {
    if (verbose) Serial.println("HTD RX short body");
    return false;
  }

  for (uint8_t i = 0; i < paramLen; i++) {
    params[i] = HtdSerial.read();
  }
  uint8_t checksum = HtdSerial.read();
  uint8_t expected = htdChecksum(id, length, cmd, params, paramLen);
  if (checksum != expected) {
    if (verbose) {
      Serial.print("HTD checksum mismatch got=0x");
      Serial.print(checksum, HEX);
      Serial.print(" expected=0x");
      Serial.println(expected, HEX);
    }
    return false;
  }
  return true;
}

void printHtdFrame(const char *prefix, uint8_t id, uint8_t cmd, const uint8_t *params, uint8_t paramLen) {
  Serial.print(prefix);
  Serial.print(" id=");
  Serial.print(id);
  Serial.print(" cmd=");
  Serial.print(cmd);
  Serial.print(" params=");
  for (uint8_t i = 0; i < paramLen; i++) {
    printHexByte(params[i]);
    if (i + 1 < paramLen) {
      Serial.print(' ');
    }
  }
  Serial.println();
}

bool htdReadExpectedResponse(uint8_t expectedId, uint8_t expectedCmd, uint8_t minParamLen, uint8_t &id, uint8_t &cmd,
                             uint8_t *params, uint8_t &paramLen, uint32_t totalTimeoutMs = 250,
                             bool verbose = true) {
  unsigned long start = millis();
  while (millis() - start < totalTimeoutMs) {
    if (!htdReadResponse(id, cmd, params, paramLen, 80, verbose)) {
      continue;
    }

    if (verbose) printHtdFrame("HTD frame", id, cmd, params, paramLen);
    if (id == expectedId && cmd == expectedCmd && paramLen >= minParamLen) {
      return true;
    }

    if (cmd == expectedCmd && paramLen == 0 && minParamLen > 0) {
      if (verbose) Serial.println("HTD skip empty echo frame");
      continue;
    }

    if (verbose) Serial.println("HTD skip non-matching frame");
  }

  if (verbose) Serial.println("HTD expected response timeout");
  return false;
}

void htdDrain() {
  while (HtdSerial.available()) {
    HtdSerial.read();
  }
}

void beginHtd(uint32_t baud) {
  HtdSerial.end();
  delay(20);
  htdBaud = baud;
  HtdSerial.begin(htdBaud, SERIAL_8N1, PIN_HTD_RX, PIN_HTD_TX);
  HtdSerial.setTimeout(40);
  htdDrain();
  Serial.print("HTD UART baud=");
  Serial.println(htdBaud);
}

void armDrain() {
  if (externalArm) return;
  htdDrain();
}

void beginArmBus() {
  if (externalArm) return;
  // HX is now intentionally on the same BusLinker and UART as HTD. Do not
  // re-open a second serial port here: re-opening would drop the shared bus.
  if (htdBaud != ARM_HX_BAUD) beginHtd(ARM_HX_BAUD);
  armDrain();
  Serial.print("Shared HTD/HX BusLinker pins TX/RX = ");
  Serial.print(PIN_HTD_TX);
  Serial.print("/");
  Serial.print(PIN_HTD_RX);
  Serial.print(" baud=");
  Serial.println(ARM_HX_BAUD);
}

bool htdReadPositionById(uint8_t servoId, int &position, uint32_t timeoutMs = 300,
                         bool verbose = true) {
  for (uint8_t attempt = 0; attempt < HTD_POSITION_READ_RETRIES; ++attempt) {
    htdDrain();
    htdSend(servoId, HTD_CMD_POS_READ, nullptr, 0);

    uint8_t id = 0, cmd = 0, paramLen = 0;
    uint8_t params[8] = {0};
    if (!htdReadExpectedResponse(servoId, HTD_CMD_POS_READ, 2, id, cmd, params, paramLen, timeoutMs,
                                 verbose && attempt == 0)) {
      continue;
    }

    const int candidate = params[0] | (params[1] << 8);
    if (candidate >= HTD_POSITION_MIN && candidate <= HTD_POSITION_MAX) {
      position = candidate;
      if (verbose) {
        Serial.print("HTD position=");
        Serial.println(position);
      }
      return true;
    }

    if (verbose) {
      Serial.printf("HTD invalid position=%d; retry %u/%u\n", candidate,
                    static_cast<unsigned>(attempt + 1),
                    static_cast<unsigned>(HTD_POSITION_READ_RETRIES));
    }
  }

  if (verbose) Serial.println("HTD valid position response timeout");
  return false;
}

bool htdReadAngleLimitsById(uint8_t servoId, int &minimum, int &maximum,
                             uint32_t timeoutMs = 300) {
  htdDrain();
  htdSend(servoId, HTD_CMD_ANGLE_LIMIT_READ, nullptr, 0);

  uint8_t id = 0, cmd = 0, paramLen = 0;
  uint8_t params[8] = {0};
  if (!htdReadExpectedResponse(servoId, HTD_CMD_ANGLE_LIMIT_READ, 4, id, cmd, params,
                               paramLen, timeoutMs)) {
    return false;
  }
  minimum = params[0] | (params[1] << 8);
  maximum = params[2] | (params[3] << 8);
  return true;
}

bool htdWriteAngleLimitsById(uint8_t servoId, int minimum, int maximum) {
  if (minimum < 0 || maximum > 1000 || minimum >= maximum) return false;
  const uint8_t params[4] = {
      static_cast<uint8_t>(minimum & 0xFF),
      static_cast<uint8_t>((minimum >> 8) & 0xFF),
      static_cast<uint8_t>(maximum & 0xFF),
      static_cast<uint8_t>((maximum >> 8) & 0xFF)};
  htdSend(servoId, HTD_CMD_ANGLE_LIMIT_WRITE, params, sizeof(params));
  return true;
}

bool htdReadTorqueLoadedById(uint8_t servoId, bool &loaded, uint32_t timeoutMs = 300) {
  htdDrain();
  htdSend(servoId, HTD_CMD_LOAD_OR_UNLOAD_READ, nullptr, 0);

  uint8_t id = 0, cmd = 0, paramLen = 0;
  uint8_t params[8] = {0};
  if (!htdReadExpectedResponse(servoId, HTD_CMD_LOAD_OR_UNLOAD_READ, 1, id, cmd, params,
                               paramLen, timeoutMs)) {
    return false;
  }
  loaded = params[0] != 0;
  return true;
}

void htdDiagnoseById(uint8_t servoId) {
  int position = 0;
  int minimum = 0;
  int maximum = 0;
  bool loaded = false;
  const bool positionOk = htdReadPositionById(servoId, position, 300, false);
  const bool limitsOk = htdReadAngleLimitsById(servoId, minimum, maximum);
  const bool loadOk = htdReadTorqueLoadedById(servoId, loaded);
  Serial.printf("HTD diag id=%u position=%s%d limits=%s%d..%d torque=%s%s\n",
                static_cast<unsigned>(servoId), positionOk ? "" : "?", position,
                limitsOk ? "" : "?", minimum, maximum,
                loadOk ? (loaded ? "loaded" : "unloaded") : "unknown",
                "");
}

bool isRegisteredHtdServoId(uint8_t servoId) {
  const int index = servoIndex(servoId);
  return index >= 0 && SERVOS[index].protocol == ServoProtocol::Htd;
}

// ID11 commissioning only. These helpers never send torque, mode or motion
// writes. Readback is protocol evidence, NOT an independent mechanical interlock.
bool id11SetupIdle(String &reason) {
  if (motionSafetyLocked() || motionStopPending()) {
    reason = "id11_setup_motion_locked";
    return false;
  }
  if (localMotionActive() || webMotionEnabled) {
    reason = "id11_setup_requires_idle_and_web_motion_disabled";
    return false;
  }
  return true;
}

bool id11SavedSetupLimits(int &minimum, int &maximum, String &reason) {
  const int index = servoIndex(TORSO_SECOND_BOARD_ID);
  if (index < 0 || !isRegisteredHtdServoId(TORSO_SECOND_BOARD_ID)) {
    reason = "id11_registry_mismatch";
    return false;
  }
  if (!jointCal[index].calibrated || jointCal[index].inverted ||
      jointCal[index].minimum != FOLD_ID11_FOLDED_POSITION ||
      jointCal[index].maximum != FOLD_ID11_UNFOLDED_POSITION ||
      jointCal[index].center < jointCal[index].minimum ||
      jointCal[index].center > jointCal[index].maximum) {
    reason = "id11_saved_calibration_mismatch_expected_38_719";
    return false;
  }
  minimum = jointCal[index].minimum;
  maximum = jointCal[index].maximum;
  return true;
}

bool id11ReadSetupFrame(uint8_t command, uint8_t expectedLength,
                       uint8_t *params, String &reason) {
  htdDrain();
  htdSend(TORSO_SECOND_BOARD_ID, command, nullptr, 0);
  uint8_t id = 0, cmd = 0, paramLen = 0;
  if (!htdReadExpectedResponse(TORSO_SECOND_BOARD_ID, command, expectedLength,
                               id, cmd, params, paramLen, 300, false) ||
      id != TORSO_SECOND_BOARD_ID || cmd != command || paramLen != expectedLength) {
    reason = "id11_setup_read_failed_or_malformed";
    return false;
  }
  return true;
}

bool id11ReadSetup(int &position, int &minimum, int &maximum,
                   bool &positionMode, bool &loaded, String &reason) {
  if (!isRegisteredHtdServoId(TORSO_SECOND_BOARD_ID)) {
    reason = "id11_registry_mismatch";
    return false;
  }
  uint8_t params[8] = {0};
  // HTD direct-servo command 30: mode, reserved, speed low, speed high.
  // Do not confuse these commands with a separate multi-servo controller protocol.
  if (!id11ReadSetupFrame(30, 4, params, reason)) return false;
  if (params[0] > 1) {
    reason = "id11_setup_invalid_mode";
    return false;
  }
  positionMode = params[0] == 0;
  if (!id11ReadSetupFrame(HTD_CMD_ANGLE_LIMIT_READ, 4, params, reason)) return false;
  minimum = params[0] | (params[1] << 8);
  maximum = params[2] | (params[3] << 8);
  if (minimum < 0 || maximum > 1000 || minimum >= maximum) {
    reason = "id11_setup_invalid_hardware_limits";
    return false;
  }
  if (!id11ReadSetupFrame(HTD_CMD_POS_READ, 2, params, reason)) return false;
  position = params[0] | (params[1] << 8);
  if (position < 0 || position > 1000) {
    reason = "id11_setup_invalid_position";
    return false;
  }
  if (!id11ReadSetupFrame(HTD_CMD_LOAD_OR_UNLOAD_READ, 1, params, reason)) return false;
  if (params[0] > 1) {
    reason = "id11_setup_invalid_torque_state";
    return false;
  }
  loaded = params[0] == 1;
  return true;
}

bool id11WriteLimitsOnly(String &reason) {
  if (!id11SetupIdle(reason)) return false;
  int savedMinimum = 0, savedMaximum = 0;
  if (!id11SavedSetupLimits(savedMinimum, savedMaximum, reason)) return false;
  int position = 0, minimum = 0, maximum = 0, firstPosition = 0;
  bool positionMode = false, loaded = true;
  for (uint8_t sample = 0; sample < 2; ++sample) {
    if (!id11ReadSetup(position, minimum, maximum, positionMode, loaded, reason)) return false;
    if (!positionMode) {
      reason = "id11_setup_requires_position_mode";
      return false;
    }
    if (loaded) {
      reason = "id11_setup_requires_torque_unloaded_no_auto_unload";
      return false;
    }
    if (position < savedMinimum || position > savedMaximum) {
      reason = "id11_setup_position_outside_saved_range";
      return false;
    }
    if (sample == 0) {
      firstPosition = position;
      delay(100);
    } else if (abs(position - firstPosition) > 2) {
      reason = "id11_setup_position_changed_support_board";
      return false;
    }
  }
  // A STOP/disconnect may arrive while read requests are waiting for replies.
  if (!id11SetupIdle(reason)) return false;
  if (minimum == savedMinimum && maximum == savedMaximum) {
    reason = "id11_limits_already_match_torque_unloaded";
    return true;
  }
  htdDrain();
  if (!htdWriteAngleLimitsById(TORSO_SECOND_BOARD_ID, savedMinimum, savedMaximum)) {
    reason = "id11_limits_write_rejected";
    return false;
  }
  delay(30);
  if (!id11ReadSetup(position, minimum, maximum, positionMode, loaded, reason)) {
    reason = "id11_limits_write_result_uncertain_readback_failed";
    return false;
  }
  if (minimum != savedMinimum || maximum != savedMaximum) {
    reason = "id11_limits_write_result_uncertain_limits_mismatch";
    return false;
  }
  if (!positionMode || loaded || position < savedMinimum || position > savedMaximum ||
      abs(position - firstPosition) > 2) {
    reason = "id11_limits_written_but_state_changed_no_auto_recovery";
    return false;
  }
  if (!id11SetupIdle(reason)) {
    reason = "id11_limits_written_but_stop_or_busy_observed";
    return false;
  }
  reason = "id11_limits_written_verified_torque_unloaded";
  return true;
}

bool id11ValidateHardwareForMotion(String &reason, bool unfolding) {
  int savedMinimum = 0, savedMaximum = 0;
  if (!id11SavedSetupLimits(savedMinimum, savedMaximum, reason)) return false;
  int position = 0, minimum = 0, maximum = 0;
  bool positionMode = false, loaded = false;
  if (!id11ReadSetup(position, minimum, maximum, positionMode, loaded, reason)) return false;
  if (!positionMode) {
    reason = "id11_setup_requires_position_mode";
    return false;
  }
  if (minimum != savedMinimum || maximum != savedMaximum) {
    reason = "id11_hardware_limits_mismatch_run_setup";
    return false;
  }
  // User-approved START margin only: 33..37 may unfold; 720..724 may fold.
  // Targets and saved/hardware limits remain 38..719. Never move farther out.
  // This is not proof of mechanical clearance at every permitted sample.
  constexpr int kStartPositionTolerance = 5;
  const bool inwardFromFoldedMargin = unfolding && position < savedMinimum &&
      position >= savedMinimum - kStartPositionTolerance;
  const bool inwardFromUnfoldedMargin = !unfolding && position > savedMaximum &&
      position <= savedMaximum + kStartPositionTolerance;
  if ((position < savedMinimum || position > savedMaximum) &&
      !inwardFromFoldedMargin && !inwardFromUnfoldedMargin) {
    reason = "id11_position_outside_saved_range";
    return false;
  }
  if (motionSafetyLocked() || motionStopPending()) {
    reason = "id11_setup_motion_locked";
    return false;
  }
  // The existing movement path enables torque. Passing this gate is NOT
  // permission for first enable, proof of physical alignment, or collision QA.
  return true;
}

bool handleId11SetupCommand(const String &command) {
  if (command != "h_id11_setup_check" && command != "h_id11_limits_write") return false;
  String reason;
  if (command == "h_id11_limits_write") {
    const bool ok = id11WriteLimitsOnly(reason);
    Serial.println((ok ? "OK " : "ERR ") + reason);
    return true;
  }
  if (!id11SetupIdle(reason)) {
    Serial.println("ERR " + reason);
    return true;
  }
  int position = 0, minimum = 0, maximum = 0;
  bool positionMode = false, loaded = false;
  if (!id11ReadSetup(position, minimum, maximum, positionMode, loaded, reason)) {
    Serial.println("ERR " + reason);
    return true;
  }
  Serial.printf("ID11_SETUP position=%d limits=%d..%d mode=%s torque=%s; read_only=yes\n",
                position, minimum, maximum, positionMode ? "position" : "motor",
                loaded ? "loaded" : "unloaded");
  return true;
}

bool applyHtdHardwareSafetyLimitsFromCalibration(uint8_t servoId) {
  const int index = servoIndex(servoId);
  if (!isRegisteredHtdServoId(servoId)) {
    Serial.println("HTD limit apply rejected: unknown_or_non_htd_servo");
    return false;
  }
  if (index < 0 || !jointCal[index].calibrated) {
    Serial.printf("ID%u limit apply rejected: joint_not_calibrated\n",
                  static_cast<unsigned>(servoId));
    return false;
  }
  const int minimum = jointCal[index].minimum;
  const int maximum = jointCal[index].maximum;
  if (!htdWriteAngleLimitsById(servoId, minimum, maximum)) {
    Serial.printf("ID%u limit apply rejected: target_out_of_range\n",
                  static_cast<unsigned>(servoId));
    return false;
  }
  delay(30);
  htdSetTorqueEnabled(servoId, true);
  delay(30);

  int readMinimum = 0;
  int readMaximum = 0;
  bool loaded = false;
  const bool limitsVerified = htdReadAngleLimitsById(servoId, readMinimum, readMaximum) &&
                              readMinimum == minimum && readMaximum == maximum;
  const bool torqueVerified = htdReadTorqueLoadedById(servoId, loaded) && loaded;
  if (!limitsVerified || !torqueVerified) {
    Serial.printf("ID%u limit apply verification_failed\n", static_cast<unsigned>(servoId));
    htdDiagnoseById(servoId);
    return false;
  }
  Serial.printf("ID%u hardware limits saved=%d..%d torque=loaded\n",
                static_cast<unsigned>(servoId), minimum, maximum);
  return true;
}

bool prepareHtdCalibrationById(uint8_t servoId) {
  if (!isRegisteredHtdServoId(servoId)) {
    Serial.println("HTD calibration prepare rejected: unknown_or_non_htd_servo");
    return false;
  }
  // Calibration must start from the complete HTD travel range. This is a
  // persistent servo setting, so it is only changed by an explicit USB command.
  if (!htdWriteAngleLimitsById(servoId, 0, 1000)) {
    Serial.println("HTD calibration prepare rejected: full_range_write_failed");
    return false;
  }
  delay(30);
  htdSetTorqueEnabled(servoId, true);
  delay(30);

  int readMinimum = 0;
  int readMaximum = 0;
  bool loaded = false;
  const bool limitsVerified = htdReadAngleLimitsById(servoId, readMinimum, readMaximum) &&
                              readMinimum == 0 && readMaximum == 1000;
  const bool torqueVerified = htdReadTorqueLoadedById(servoId, loaded) && loaded;
  if (!limitsVerified || !torqueVerified) {
    Serial.printf("ID%u calibration prepare verification_failed\n",
                  static_cast<unsigned>(servoId));
    htdDiagnoseById(servoId);
    return false;
  }
  Serial.printf("ID%u calibration ready limits=0..1000 torque=loaded\n",
                static_cast<unsigned>(servoId));
  return true;
}

bool applyTorsoLowerHardwareSafetyLimits() {
  return applyHtdHardwareSafetyLimitsFromCalibration(TORSO_LOWER_ID);
}

bool htdPingId(uint8_t servoId) {
  htdDrain();
  htdSend(servoId, HTD_CMD_ID_READ, nullptr, 0);

  uint8_t id = 0, cmd = 0, paramLen = 0;
  uint8_t params[8] = {0};
  if (!htdReadExpectedResponse(servoId, HTD_CMD_ID_READ, 1, id, cmd, params, paramLen, 300)) {
    return false;
  }
  printHtdFrame("HTD id response", id, cmd, params, paramLen);
  return true;
}

void htdSetTorqueEnabled(uint8_t servoId, bool enabled) {
  // Bus-servo write commands have no acknowledgement.  Match the proven
  // commissioning transport sequence: clear a pending read reply before the
  // write, then leave the bus idle briefly before the following command.
  // Without that gap an immediately preceding read can make a torque-load
  // request unreliable even though position reads still work.
  htdDrain();
  const uint8_t parameter = enabled ? 1 : 0;
  htdSend(servoId, HTD_CMD_LOAD_OR_UNLOAD_WRITE, &parameter, 1);
  delay(3);
}

void htdMoveToId(uint8_t servoId, uint16_t position, uint16_t timeMs) {
  // ServoStudio can leave a bus servo in unloaded mode. An unloaded HTD
  // continues to reply to position reads but deliberately produces no torque.
  // Restore torque before every position command so web, actions and USB
  // calibration all use the same safe, deterministic path.
  htdSetTorqueEnabled(servoId, true);
  delay(5);
  uint8_t params[4] = {
      static_cast<uint8_t>(position & 0xFF),
      static_cast<uint8_t>((position >> 8) & 0xFF),
      static_cast<uint8_t>(timeMs & 0xFF),
      static_cast<uint8_t>((timeMs >> 8) & 0xFF)};
  htdSend(servoId, HTD_CMD_MOVE_TIME_WRITE, params, sizeof(params));
  Serial.print("HTD move target=");
  Serial.print(position);
  Serial.print(" time_ms=");
  Serial.println(timeMs);
}

void htdStopId(uint8_t servoId) {
  htdSend(servoId, HTD_CMD_MOVE_STOP, nullptr, 0);
  Serial.println("HTD stop sent");
}

bool htdReadPosition(int &position) {
  return htdReadPositionById(HTD_ID, position);
}

void htdPing() {
  htdPingId(HTD_ID);
}

void htdMoveTo(uint16_t position, uint16_t timeMs) {
  htdMoveToId(HTD_ID, position, timeMs);
}

void htdStop() {
  htdStopId(HTD_ID);
}

uint8_t hxChecksum(uint8_t id, uint8_t length, uint8_t instruction,
                   const uint8_t *params, uint8_t paramLen) {
  uint16_t sum = id + length + instruction;
  for (uint8_t i = 0; i < paramLen; ++i) sum += params[i];
  return static_cast<uint8_t>(~sum);
}

void hxSend(uint8_t id, uint8_t instruction, const uint8_t *params, uint8_t paramLen) {
  if (externalArm) return;
  const uint8_t length = paramLen + 2;
  HtdSerial.write(0xFF);
  HtdSerial.write(0xFF);
  HtdSerial.write(id);
  HtdSerial.write(length);
  HtdSerial.write(instruction);
  for (uint8_t i = 0; i < paramLen; ++i) HtdSerial.write(params[i]);
  HtdSerial.write(hxChecksum(id, length, instruction, params, paramLen));
  HtdSerial.flush();
}

bool hxReadStatus(uint8_t expectedId, uint8_t *params, size_t paramsCapacity,
                  uint8_t &paramLen, uint32_t timeoutMs = 100) {
  if (externalArm) { paramLen = 0; return false; }
  const uint32_t start = millis();
  uint8_t headerState = 0;
  while (millis() - start < timeoutMs) {
    if (!HtdSerial.available()) {
      delay(1);
      continue;
    }
    const uint8_t value = HtdSerial.read();
    if (headerState == 0) headerState = value == 0xFF ? 1 : 0;
    else if (value == 0xFF) {
      headerState = 2;
      break;
    } else headerState = 0;
  }
  if (headerState != 2) return false;

  while (HtdSerial.available() < 3 && millis() - start < timeoutMs) delay(1);
  if (HtdSerial.available() < 3) return false;
  const uint8_t id = HtdSerial.read();
  const uint8_t length = HtdSerial.read();
  const uint8_t error = HtdSerial.read();
  const uint8_t wireParamLen = length >= 2 ? length - 2 : 0;
  const bool payloadTooLarge = wireParamLen > paramsCapacity;
  uint16_t checksumSum = id + length + error;
  for (uint8_t i = 0; i < wireParamLen; ++i) {
    while (!HtdSerial.available() && millis() - start < timeoutMs) delay(1);
    if (!HtdSerial.available()) {
      paramLen = 0;
      return false;
    }
    const uint8_t value = HtdSerial.read();
    checksumSum += value;
    if (i < paramsCapacity) params[i] = value;
  }
  while (!HtdSerial.available() && millis() - start < timeoutMs) delay(1);
  if (!HtdSerial.available()) {
    paramLen = 0;
    return false;
  }
  const uint8_t checksum = HtdSerial.read();
  paramLen = payloadTooLarge ? 0 : wireParamLen;
  const uint8_t expectedChecksum = static_cast<uint8_t>(~checksumSum);
  return length >= 2 && id == expectedId && error == 0 && !payloadTooLarge &&
         checksum == expectedChecksum;
}

bool hxPing(uint8_t id) {
  if (externalArm) return false;
  armDrain();
  hxSend(id, 0x01, nullptr, 0);
  uint8_t params[8] = {0};
  uint8_t paramLen = 0;
  return hxReadStatus(id, params, sizeof(params), paramLen);
}

uint16_t hxEncodeSignedPosition(int32_t position) {
  const uint16_t magnitude = static_cast<uint16_t>(
      position < 0 ? -static_cast<int64_t>(position) : position);
  return position < 0 ? static_cast<uint16_t>(magnitude | 0x8000U) : magnitude;
}

int32_t hxDecodeSignedPosition(uint16_t raw) {
  const int32_t magnitude = raw & 0x7FFFU;
  return (raw & 0x8000U) ? -magnitude : magnitude;
}

bool hxReadPosition(uint8_t id, int32_t &position, uint32_t timeoutMs = 100) {
  if (externalArm) return false;
  const uint8_t request[] = {0x38, 0x02};
  armDrain();
  hxSend(id, 0x02, request, sizeof(request));
  uint8_t params[8] = {0};
  uint8_t paramLen = 0;
  if (!hxReadStatus(id, params, sizeof(params), paramLen, timeoutMs) || paramLen < 2) return false;
  const uint16_t raw = static_cast<uint16_t>(params[0]) |
                       (static_cast<uint16_t>(params[1]) << 8);
  position = hxDecodeSignedPosition(raw);
  const int index = servoIndex(id);
  if (index >= 0) {
    lastServoPosition[index] = position;
    lastServoPositionMs[index] = millis();
    lastServoPositionValid[index] = true;
    if (index < static_cast<int>(ARM_SERVO_COUNT)) {
      ++webArmPositionSequence[index];
      if (webArmPositionSequence[index] == 0) webArmPositionSequence[index] = 1;
    }
  }
  return true;
}

bool hxWrite(uint8_t id, uint8_t address, const uint8_t *data, uint8_t dataLen) {
  if (externalArm) return false;
  uint8_t params[10] = {address};
  if (dataLen > sizeof(params) - 1) return false;
  memcpy(&params[1], data, dataLen);
  armDrain();
  hxSend(id, 0x03, params, dataLen + 1);
  uint8_t response[8] = {0};
  uint8_t responseLen = 0;
  return hxReadStatus(id, response, sizeof(response), responseLen);
}

bool hxWriteFast(uint8_t id, uint8_t address, const uint8_t *data, uint8_t dataLen) {
  if (externalArm) return false;
  uint8_t params[10] = {address};
  if (dataLen > sizeof(params) - 1) return false;
  memcpy(&params[1], data, dataLen);
  armDrain();
  hxSend(id, 0x03, params, dataLen + 1);
  return true;
}

bool hxMoveTo(uint8_t id, int32_t position, uint16_t durationMs) {
  (void)durationMs;
  if (position < INT16_MIN || position > INT16_MAX) return false;
  const uint8_t torque = 1;
  if (!hxWrite(id, 0x28, &torque, 1)) return false;
  const uint16_t rawPosition = hxEncodeSignedPosition(position);
  const uint8_t target[] = {
      static_cast<uint8_t>(rawPosition & 0xFF), static_cast<uint8_t>(rawPosition >> 8),
      0, 0,
      static_cast<uint8_t>(HX_DEMO_SPEED & 0xFF), static_cast<uint8_t>(HX_DEMO_SPEED >> 8)};
  return hxWrite(id, 0x2A, target, sizeof(target));
}

bool hxMoveToFast(uint8_t id, int32_t position, uint16_t durationMs) {
  (void)durationMs;
  if (position < INT16_MIN || position > INT16_MAX) return false;
  const uint8_t torque = 1;
  if (!hxWriteFast(id, 0x28, &torque, 1)) return false;
  const uint16_t rawPosition = hxEncodeSignedPosition(position);
  const uint8_t target[] = {
      static_cast<uint8_t>(rawPosition & 0xFF), static_cast<uint8_t>(rawPosition >> 8),
      0, 0,
      static_cast<uint8_t>(HX_DEMO_SPEED & 0xFF), static_cast<uint8_t>(HX_DEMO_SPEED >> 8)};
  return hxWriteFast(id, 0x2A, target, sizeof(target));
}

bool hxMoveToProfileFast(uint8_t id, int32_t position, int16_t speed,
                           uint8_t acceleration) {
  if (position < INT16_MIN || position > INT16_MAX || speed < -3400 || speed > 3400) return false;
  const uint8_t torque = 1;
  // Match the verified STM32 arm controller: enable torque, then write the
  // whole acceleration/position/time/speed block in one acknowledged packet.
  // Splitting this block into back-to-back unacknowledged writes made the
  // assembled arm only twitch instead of executing the profile.
  if (!hxWrite(id, 0x28, &torque, 1)) return false;
  delay(5);
  const uint16_t rawPosition = hxEncodeSignedPosition(position);
  const uint16_t encodedSpeed = static_cast<uint16_t>(speed) & 0x7FFF;
  const uint8_t profile[] = {
      acceleration,
      static_cast<uint8_t>(rawPosition & 0xFF), static_cast<uint8_t>(rawPosition >> 8),
      0, 0,
      static_cast<uint8_t>(encodedSpeed & 0xFF), static_cast<uint8_t>(encodedSpeed >> 8)};
  return hxWrite(id, 0x29, profile, sizeof(profile));
}

bool hxHoldHx30AtPosition(uint8_t id, int32_t position) {
  if (position < INT16_MIN || position > INT16_MAX) return false;

  const uint16_t rawPosition = hxEncodeSignedPosition(position);
  const uint8_t target[] = {
      static_cast<uint8_t>(rawPosition & 0xFF),
      static_cast<uint8_t>(rawPosition >> 8)};

  // Keep torque loaded on the shoulder and replace the active position target
  // with the measured release position. Cycling torque here makes a loaded
  // HX-30HM joint drop before torque is restored.
  const bool targetWritten = hxWrite(id, 0x2A, target, sizeof(target));
  if (targetWritten) {
    Serial.printf("HX-30HM direct hold id=%u position=%ld\n",
                  static_cast<unsigned>(id), static_cast<long>(position));
  }
  return targetWritten;
}

void hxTorqueOff(uint8_t id) {
  if (externalArm) return;
  const uint8_t params[] = {0x28, 0x00};
  armDrain();
  hxSend(id, 0x03, params, sizeof(params));
}

const ServoDescriptor *findServo(uint8_t id) {
  for (size_t i = 0; i < SERVO_COUNT; ++i) {
    if (SERVOS[i].id == id) return &SERVOS[i];
  }
  return nullptr;
}

int servoIndex(uint8_t id) {
  for (size_t i = 0; i < SERVO_COUNT; ++i) {
    if (SERVOS[i].id == id) return static_cast<int>(i);
  }
  return -1;
}

bool servoPing(uint8_t id) {
  const ServoDescriptor *servo = findServo(id);
  if (!servo) return false;
  return servo->protocol == ServoProtocol::Htd ? htdPingId(id) : hxPing(id);
}

bool servoReadPosition(uint8_t id, int32_t &position) {
  const ServoDescriptor *servo = findServo(id);
  if (!servo) return false;
  if (servo->protocol == ServoProtocol::Hx) return hxReadPosition(id, position);
  int htdPosition = 0;
  if (!htdReadPositionById(id, htdPosition)) return false;
  position = htdPosition;
  const int index = servoIndex(id);
  if (index >= 0) {
    lastServoPosition[index] = position;
    lastServoPositionMs[index] = millis();
    lastServoPositionValid[index] = true;
  }
  return true;
}

bool servoMoveTo(uint8_t id, int32_t position, uint16_t durationMs) {
  const ServoDescriptor *servo = findServo(id);
  if (!servo) return false;
  if (servo->protocol == ServoProtocol::Hx) {
    return hxMoveToProfileFast(id, position, HX_DEMO_SPEED, HX_DEMO_ACCEL);
  }
  htdMoveToId(id, static_cast<uint16_t>(constrain(position, 0L, 1000L)), durationMs);
  return true;
}

bool servoMoveToFast(uint8_t id, int32_t position, uint16_t durationMs) {
  const ServoDescriptor *servo = findServo(id);
  if (!servo) return false;
  if (servo->protocol == ServoProtocol::Hx) {
    return hxMoveToProfileFast(id, position, HX_DEMO_SPEED, HX_DEMO_ACCEL);
  }
  htdMoveToId(id, static_cast<uint16_t>(constrain(position, 0L, 1000L)), durationMs);
  return true;
}

void servoStop(uint8_t id) {
  const ServoDescriptor *servo = findServo(id);
  if (!servo) return;
  if (servo->protocol == ServoProtocol::Htd) htdStopId(id);
}

void scheduleHxHold() {
  if (externalArm) return;
  hxHold.active = true;
  hxHold.servoCursor = 0;
}

void serviceHxHold() {
  if (externalArm) { hxHold.active = false; return; }
  if (!hxHold.active) return;
  while (hxHold.servoCursor < SERVO_COUNT) {
    const size_t index = hxHold.servoCursor++;
    if (SERVOS[index].protocol != ServoProtocol::Hx) continue;

    int32_t holdPosition = 0;
    bool havePosition = hxReadPosition(SERVOS[index].id, holdPosition, 15);
    if (havePosition) {
      hxHoldHx30AtPosition(SERVOS[index].id, holdPosition);
    } else {
      // Normal stop holds torque rather than dropping a loaded arm. An explicit
      // hx_torque_off_all command remains available only for an emergency.
      Serial.printf("HX ID %u hold_read_unavailable: keeping current torque state\n",
                    SERVOS[index].id);
    }
    return;
  }
  hxHold.active = false;
}

void emergencyTorqueOffAllHx() {
  hxHold.active = false;
  for (const ServoDescriptor &servo : SERVOS) {
    if (servo.protocol == ServoProtocol::Hx) hxTorqueOff(servo.id);
  }
}

void htdSmallTest() {
  Serial.println("HTD small position test starts");
  int current = 0;
  if (!htdReadPosition(current)) {
    Serial.println("HTD test aborted: cannot read position");
    return;
  }

  int target = current + HTD_SMALL_DELTA_UNITS;
  target = constrain(target, 0, 1000);
  htdMoveTo(static_cast<uint16_t>(target), HTD_MOVE_TIME_MS);
  delay(HTD_MOVE_TIME_MS + 200);
  htdReadPosition(current);
  htdStop();
  Serial.println("HTD small position test done");
}

void latchDrivePwmFault() {
  drivePwmFault = true;
  drivePwmReady = false;
  emergencyStopLatched = true;
  const int pins[] = {PIN_ONBOARD_M1_IN1, PIN_ONBOARD_M1_IN2,
                      PIN_ONBOARD_M2_IN1, PIN_ONBOARD_M2_IN2};
  for (int pin : pins) {
    ledcWrite(pin, 0);
    ledcDetach(pin);
    pinMode(pin, OUTPUT);
    gpio_set_level(static_cast<gpio_num_t>(pin), LOW);
  }
  driveLeftPwm = driveRightPwm = 0;
  baseMotion.active = false;
  webDriveActive = false;
  webMotionEnabled = false;
  Serial.println("DRV8870 PWM fault: outputs stopped; restart required");
}

void beginOnboardDrive() {
  const int inputPins[] = {
      PIN_ONBOARD_M1_IN1, PIN_ONBOARD_M1_IN2,
      PIN_ONBOARD_M2_IN1, PIN_ONBOARD_M2_IN2};
  for (size_t i = 0; i < 4; ++i) {
    pinMode(inputPins[i], OUTPUT);
    digitalWrite(inputPins[i], LOW);
    if (!ledcAttach(inputPins[i], 20000, 8) || !ledcWrite(inputPins[i], 0)) {
      latchDrivePwmFault();
      return;
    }
  }
  drivePwmReady = true;
  Serial.println("Onboard DRV8870 M1/M2 ready at 20 kHz drive/brake PWM; zero=coast; PSRAM must be disabled");
  Serial.println("M1=left-front+left-rear parallel; M2=right-front+right-rear parallel");
}

bool motorDriveInIn(int in1Pin, int in2Pin, int pwm) {
  if (!drivePwmReady || drivePwmFault) return false;
  pwm = constrain(pwm, -255, 255);
  const uint32_t current1 = ledcRead(in1Pin);
  const uint32_t current2 = ledcRead(in2Pin);
  // DRV8870: 10=forward, 01=reverse, 11=brake/slow decay, 00=coast.
  // Hold the direction input HIGH; PWM the other input with inverse duty.
  // Keep zero/STOP as coast, not continuous electrical braking.
  const int duty1 = pwm == 0 ? 0 : (pwm > 0 ? 255 : 255 + pwm);
  const int duty2 = pwm == 0 ? 0 : (pwm < 0 ? 255 : 255 - pwm);
  const uint32_t expected1 = duty1 == 255 ? 256U : duty1;
  const uint32_t expected2 = duty2 == 255 ? 256U : duty2;
  if (current1 == expected1 && current2 == expected2) return true;

  // Before swapping which input is held HIGH, settle in 11 (brake).
  // Do not reset both inputs to 00 on every joystick refresh.
  const bool reversing = (pwm > 0 && current2 == 256U && current1 < 256U) ||
                         (pwm < 0 && current1 == 256U && current2 < 256U);
  if (reversing) {
    if (!ledcWrite(in1Pin, 255) || !ledcWrite(in2Pin, 255)) {
      latchDrivePwmFault();
      return false;
    }
    esp_rom_delay_us(60);
    if (ledcRead(in1Pin) != 256U || ledcRead(in2Pin) != 256U) {
      latchDrivePwmFault();
      return false;
    }
  }

  // On start, write the intended direction input first. On stop, lower
  // the modulated input first. LEDC pair updates are not hardware-atomic;
  // this is not a guarantee of glitch-free pad transitions or shaft stop.
  // There is no intentional full-duty startup boost or blocking motor jog.
  const bool in1First = pwm > 0 || (pwm == 0 && current2 == 256U);
  const bool firstOk = ledcWrite(in1First ? in1Pin : in2Pin, in1First ? duty1 : duty2);
  if (!firstOk) {
    latchDrivePwmFault();
    return false;
  }
  if (!ledcWrite(in1First ? in2Pin : in1Pin, in1First ? duty2 : duty1)) {
    latchDrivePwmFault();
    return false;
  }
  // Arduino encodes 8-bit full scale as duty 256; updates latch next period.
  esp_rom_delay_us(60);
  const bool ok = ledcRead(in1Pin) == expected1 && ledcRead(in2Pin) == expected2;
  if (!ok) latchDrivePwmFault();
  return ok;
}

void chassisDriveLeft(int pwm) {
  pwm = constrain(pwm, -255, 255);
  const int m1Pwm = ONBOARD_M1_INVERTED ? -pwm : pwm;
  if (!motorDriveInIn(PIN_ONBOARD_M1_IN1, PIN_ONBOARD_M1_IN2, m1Pwm)) return;
  driveLeftPwm = pwm;
  if (ENABLE_DRIVE_PWM_TRACE && Serial) {
    Serial.print("Onboard M1 left pwm=");
    Serial.println(pwm);
  }
}

void chassisDriveRight(int pwm) {
  pwm = constrain(pwm, -255, 255);
  const int m2Pwm = ONBOARD_M2_INVERTED ? -pwm : pwm;
  if (!motorDriveInIn(PIN_ONBOARD_M2_IN1, PIN_ONBOARD_M2_IN2, m2Pwm)) return;
  driveRightPwm = pwm;
  if (ENABLE_DRIVE_PWM_TRACE && Serial) {
    Serial.print("Onboard M2 right pwm=");
    Serial.println(pwm);
  }
}

void chassisDriveBoth(int leftPwm, int rightPwm) {
  chassisDriveLeft(leftPwm);
  chassisDriveRight(rightPwm);
}

void chassisDrive(int pwm) {
  chassisDriveBoth(pwm, pwm);
}

void chassisStop() {
  chassisDriveBoth(0, 0);
  baseMotion.active = false;
  baseMotion.leftPwm = 0;
  baseMotion.rightPwm = 0;
}

bool startBaseMotion(int leftPwm, int rightPwm, uint32_t durationMs) {
  if (!drivePwmReady || drivePwmFault) return false;
  if (durationMs == 0 || durationMs > MAX_BASE_DURATION_SEC * 1000U) return false;
  chassisDriveBoth(leftPwm, rightPwm);
  if (!drivePwmReady || drivePwmFault) return false;
  baseMotion.active = true;
  baseMotion.leftPwm = leftPwm;
  baseMotion.rightPwm = rightPwm;
  baseMotion.deadlineMs = millis() + durationMs;
  return true;
}

void serviceBaseMotion() {
  if (baseMotion.active && static_cast<int32_t>(millis() - baseMotion.deadlineMs) >= 0) {
    chassisStop();
  }
}

void onboardDriveRawPulse(const char *name, int in1Pin, int in2Pin) {
  // Keep the legacy command name, but use the same checked, timed drive path.
  const bool left = in1Pin == PIN_ONBOARD_M1_IN1 && in2Pin == PIN_ONBOARD_M1_IN2;
  const bool right = in1Pin == PIN_ONBOARD_M2_IN1 && in2Pin == PIN_ONBOARD_M2_IN2;
  if ((!left && !right) ||
      !startBaseMotion(left ? DRIVE_TEST_PWM : 0, right ? DRIVE_TEST_PWM : 0, 300)) {
    Serial.println("drive_test_rejected");
    return;
  }
  Serial.printf("%s: timed low-PWM forward test 300ms (not a blocking raw test)\n", name);
}

void onboardM1RawTest() {
  onboardDriveRawPulse("Onboard M1 / left pair", PIN_ONBOARD_M1_IN1, PIN_ONBOARD_M1_IN2);
}

void onboardM2RawTest() {
  onboardDriveRawPulse("Onboard M2 / right pair", PIN_ONBOARD_M2_IN1, PIN_ONBOARD_M2_IN2);
}

void driveResetTicks() {
  noInterrupts();
  driveTicks = 0;
  interrupts();
  Serial.println("Chassis placeholder ticks reset (encoders are not connected)");
}

void drivePrintTicks() {
  noInterrupts();
  long ticks = driveTicks;
  interrupts();
  Serial.print("Chassis placeholder ticks=");
  Serial.println(ticks);
}

long driveReadTicks() {
  noInterrupts();
  long ticks = driveTicks;
  interrupts();
  return ticks;
}

void chassisRunFor(int pwm, uint16_t ms) {
  driveResetTicks();
  chassisDriveBoth(pwm, pwm);
  delay(ms);
  chassisStop();
  drivePrintTicks();
}

void chassisForwardTest() {
  Serial.println("Onboard DRV8870 forward low-PWM test starts");
  startBaseMotion(DRIVE_TEST_PWM, DRIVE_TEST_PWM, DRIVE_TEST_MS);
}

void chassisBackwardTest() {
  Serial.println("Onboard DRV8870 backward low-PWM test starts");
  startBaseMotion(-DRIVE_TEST_PWM, -DRIVE_TEST_PWM, DRIVE_TEST_MS);
}

void calibrationKey(char *key, size_t keySize, uint8_t id, const char *suffix) {
  snprintf(key, keySize, "j%02u_%s", id, suffix);
}

bool invalidateCalibrationFlagsForRegistryChange(Preferences &prefs) {
  bool writeOk = true;
  for (size_t i = 0; i < SERVO_COUNT; ++i) {
    char okKey[15];
    calibrationKey(okKey, sizeof(okKey), SERVOS[i].id, "ok");
    if (prefs.putBool(okKey, false) != sizeof(bool)) writeOk = false;
    jointCal[i].calibrated = false;
  }
  return writeOk;
}

bool isLegacyTorsoServo(uint8_t id) {
  return id == 11 || id == 15;
}

bool invalidateNewArmCalibrationFlags(Preferences &prefs) {
  bool writeOk = true;
  for (uint8_t id = 1; id <= 6; ++id) {
    char okKey[15];
    calibrationKey(okKey, sizeof(okKey), id, "ok");
    if (prefs.putBool(okKey, false) != sizeof(bool)) writeOk = false;
    const int index = servoIndex(id);
    if (index >= 0) jointCal[index].calibrated = false;
  }
  return writeOk;
}

int32_t servoHardMinimum(const ServoDescriptor &servo) {
  return servo.protocol == ServoProtocol::Htd ? 0 : HX30_TEMP_MIN_POSITION;
}

int32_t servoHardMaximum(const ServoDescriptor &servo) {
  return servo.protocol == ServoProtocol::Htd ? 1000 : HX30_TEMP_MAX_POSITION;
}

void loadCalibration() {
  if (!jointPrefs.begin("jointcal", true)) {
    Serial.println("Calibration NVS unavailable");
    return;
  }
  const uint32_t registry = servoRegistrySignature();
  const bool schemaMatches =
      jointPrefs.getUInt("schema", 0) == CALIBRATION_SCHEMA_VERSION &&
      jointPrefs.getUInt("registry", 0) == registry;
  for (size_t i = 0; i < SERVO_COUNT; ++i) {
    char key[15];
    calibrationKey(key, sizeof(key), SERVOS[i].id, "min");
    jointCal[i].minimum = jointPrefs.getInt(key, servoHardMinimum(SERVOS[i]));
    calibrationKey(key, sizeof(key), SERVOS[i].id, "ctr");
    jointCal[i].center = jointPrefs.getInt(key, 0);
    calibrationKey(key, sizeof(key), SERVOS[i].id, "max");
    jointCal[i].maximum = jointPrefs.getInt(key, servoHardMaximum(SERVOS[i]));
    calibrationKey(key, sizeof(key), SERVOS[i].id, "inv");
    jointCal[i].inverted = jointPrefs.getBool(key, false);
    calibrationKey(key, sizeof(key), SERVOS[i].id, "ok");
    // The arm registry replaces an old virtual-joint map. Keep the already
    // verified temporary folding calibration for IDs 11/15, but never carry an
    // old mapping over to the six new physical arm joints.
    jointCal[i].calibrated =
        (schemaMatches || isLegacyTorsoServo(SERVOS[i].id)) && jointPrefs.getBool(key, false);
    if (jointCal[i].calibrated) {
      const int32_t hardMin = servoHardMinimum(SERVOS[i]);
      const int32_t hardMax = servoHardMaximum(SERVOS[i]);
      jointCal[i].calibrated = jointCal[i].minimum >= hardMin && jointCal[i].maximum <= hardMax &&
                               jointCal[i].minimum < jointCal[i].center &&
                               jointCal[i].center < jointCal[i].maximum;
    }
  }
  jointPrefs.end();
}

bool saveCalibration(int index, String &reason) {
  if (index < 0 || index >= static_cast<int>(SERVO_COUNT)) {
    reason = "no_servo_selected";
    return false;
  }
  JointCalibration &cal = jointCal[index];
  cal.calibrated = false;
  const int32_t hardMin = servoHardMinimum(SERVOS[index]);
  const int32_t hardMax = servoHardMaximum(SERVOS[index]);
  if (cal.minimum < hardMin || cal.maximum > hardMax ||
      !(cal.minimum < cal.center && cal.center < cal.maximum)) {
    reason = "invalid_calibration_order";
    return false;
  }
  if (!jointPrefs.begin("jointcal", false)) {
    reason = "nvs_unavailable";
    return false;
  }
  const uint32_t registry = servoRegistrySignature();
  const bool metadataMatches =
      jointPrefs.getUInt("schema", 0) == CALIBRATION_SCHEMA_VERSION &&
      jointPrefs.getUInt("registry", 0) == registry;
  char minKey[15], ctrKey[15], maxKey[15], invKey[15], okKey[15];
  calibrationKey(minKey, sizeof(minKey), SERVOS[index].id, "min");
  calibrationKey(ctrKey, sizeof(ctrKey), SERVOS[index].id, "ctr");
  calibrationKey(maxKey, sizeof(maxKey), SERVOS[index].id, "max");
  calibrationKey(invKey, sizeof(invKey), SERVOS[index].id, "inv");
  calibrationKey(okKey, sizeof(okKey), SERVOS[index].id, "ok");
  // A registry migration must not silently make the physical arm movable from
  // the previous virtual-arm calibration. Preserve the temporary folding IDs
  // only; every arm joint must be calibrated explicitly.
  bool writeOk = metadataMatches || invalidateNewArmCalibrationFlags(jointPrefs);
  if (writeOk) writeOk = jointPrefs.putBool(okKey, false) == sizeof(bool);
  if (writeOk) writeOk = jointPrefs.putUInt("schema", CALIBRATION_SCHEMA_VERSION) == sizeof(uint32_t);
  if (writeOk) writeOk = jointPrefs.putUInt("registry", registry) == sizeof(uint32_t);
  if (writeOk) writeOk = jointPrefs.putInt(minKey, cal.minimum) == sizeof(int32_t);
  if (writeOk) writeOk = jointPrefs.putInt(ctrKey, cal.center) == sizeof(int32_t);
  if (writeOk) writeOk = jointPrefs.putInt(maxKey, cal.maximum) == sizeof(int32_t);
  if (writeOk) writeOk = jointPrefs.putBool(invKey, cal.inverted) == sizeof(bool);
  if (writeOk) writeOk = jointPrefs.putBool(okKey, true) == sizeof(bool);
  jointPrefs.end();
  if (!writeOk) {
    reason = "nvs_write_failed";
    return false;
  }

  if (!jointPrefs.begin("jointcal", true)) {
    if (jointPrefs.begin("jointcal", false)) {
      const bool invalidated = jointPrefs.putBool(okKey, false) == sizeof(bool);
      jointPrefs.end();
      if (!invalidated) Serial.println("Calibration NVS invalidation failed");
    }
    reason = "nvs_verify_failed";
    return false;
  }
  const bool verified = jointPrefs.getBool(okKey, false) &&
                        jointPrefs.getUInt("schema", 0) == CALIBRATION_SCHEMA_VERSION &&
                        jointPrefs.getUInt("registry", 0) == servoRegistrySignature() &&
                        jointPrefs.getInt(minKey, INT32_MIN) == cal.minimum &&
                        jointPrefs.getInt(ctrKey, INT32_MIN) == cal.center &&
                        jointPrefs.getInt(maxKey, INT32_MIN) == cal.maximum &&
                        jointPrefs.getBool(invKey, !cal.inverted) == cal.inverted;
  jointPrefs.end();
  if (!verified) {
    if (jointPrefs.begin("jointcal", false)) {
      const bool invalidated = jointPrefs.putBool(okKey, false) == sizeof(bool);
      jointPrefs.end();
      if (!invalidated) Serial.println("Calibration NVS invalidation failed");
    }
    reason = "nvs_verify_failed";
    return false;
  }
  cal.calibrated = true;
  reason = "saved";
  return true;
}

bool restoreVerifiedTorsoCalibration(String &reason) {
  struct TorsoCalibrationPreset {
    uint8_t id;
    int32_t minimum;
    int32_t center;
    int32_t maximum;
  };
  const TorsoCalibrationPreset presets[] = {
      {11, 38, 322, 719},
  };

  for (const TorsoCalibrationPreset &preset : presets) {
    const int index = servoIndex(preset.id);
    if (index < 0 || SERVOS[index].protocol != ServoProtocol::Htd) {
      reason = "torso_registry_mismatch";
      return false;
    }

    const JointCalibration previous = jointCal[index];
    jointCal[index].minimum = preset.minimum;
    jointCal[index].center = preset.center;
    jointCal[index].maximum = preset.maximum;
    jointCal[index].inverted = false;
    jointCal[index].calibrated = false;

    String saveReason;
    if (!saveCalibration(index, saveReason)) {
      jointCal[index] = previous;
      reason = "id" + String(preset.id) + "_" + saveReason;
      return false;
    }
  }

  reason = "restored";
  return true;
}

const PoseDefinition *findPose(const String &name) {
  for (const PoseDefinition &pose : POSES) {
    if (name == pose.name) return &pose;
  }
  return nullptr;
}

void poseKey(char *key, size_t keySize, const PoseDefinition &pose, uint8_t id) {
  snprintf(key, keySize, "%s_%02u", pose.prefix, id);
}

void poseValidKey(char *key, size_t keySize, const PoseDefinition &pose) {
  snprintf(key, keySize, "%s_ok", pose.prefix);
}

bool invalidatePoseFlagsForRegistryChange(Preferences &prefs) {
  bool writeOk = true;
  for (const PoseDefinition &storedPose : POSES) {
    char validKey[15];
    poseValidKey(validKey, sizeof(validKey), storedPose);
    if (prefs.putBool(validKey, false) != sizeof(bool)) writeOk = false;
  }
  return writeOk;
}

bool readPoseTarget(const PoseDefinition &pose, uint8_t id, int32_t &target) {
  Preferences posePrefs;
  if (!posePrefs.begin("poses", true)) return false;
  char key[15], validKey[15];
  poseValidKey(validKey, sizeof(validKey), pose);
  // Pose validity is protected by its calibrated joint limits. Do not discard
  // the two temporary torso poses merely because the separate arm registry was
  // added later.
  const bool poseValid = posePrefs.getUInt("schema", 0) == CALIBRATION_SCHEMA_VERSION &&
                         posePrefs.getBool(validKey, false);
  poseKey(key, sizeof(key), pose, id);
  const bool found = poseValid && posePrefs.isKey(key);
  if (found) target = posePrefs.getInt(key, 0);
  posePrefs.end();
  return found;
}

bool capturePose(const String &name, String &reason) {
  const PoseDefinition *pose = findPose(name);
  if (!pose) {
    reason = "unknown_pose";
    return false;
  }
  int32_t positions[6] = {0};
  for (uint8_t i = 0; i < pose->count; ++i) {
    const int index = servoIndex(pose->ids[i]);
    if (index < 0 || !jointCal[index].calibrated) {
      reason = "joint_not_calibrated";
      return false;
    }
    if (!servoReadPosition(pose->ids[i], positions[i])) {
      reason = "position_read_failed";
      return false;
    }
    if (positions[i] < jointCal[index].minimum || positions[i] > jointCal[index].maximum) {
      Serial.printf(
          "pose_capture out_of_range id=%u position=%ld min=%ld max=%ld\n",
          static_cast<unsigned>(pose->ids[i]), static_cast<long>(positions[i]),
          static_cast<long>(jointCal[index].minimum),
          static_cast<long>(jointCal[index].maximum));
      reason = "target_out_of_range";
      return false;
    }
  }
  Preferences posePrefs;
  if (!posePrefs.begin("poses", false)) {
    reason = "nvs_unavailable";
    return false;
  }
  const uint32_t registry = servoRegistrySignature();
  const bool metadataMatches = posePrefs.getUInt("schema", 0) == CALIBRATION_SCHEMA_VERSION;
  char validKey[15];
  poseValidKey(validKey, sizeof(validKey), *pose);
  bool writeOk = metadataMatches || invalidatePoseFlagsForRegistryChange(posePrefs);
  if (writeOk) writeOk = posePrefs.putBool(validKey, false) == sizeof(bool);
  if (writeOk) writeOk = posePrefs.putUInt("schema", CALIBRATION_SCHEMA_VERSION) == sizeof(uint32_t);
  if (writeOk) writeOk = posePrefs.putUInt("registry", registry) == sizeof(uint32_t);
  for (uint8_t i = 0; i < pose->count; ++i) {
    char key[15];
    poseKey(key, sizeof(key), *pose, pose->ids[i]);
    if (writeOk) writeOk = posePrefs.putInt(key, positions[i]) == sizeof(int32_t);
  }
  if (writeOk) writeOk = posePrefs.putBool(validKey, true) == sizeof(bool);
  posePrefs.end();
  if (!writeOk) {
    reason = "nvs_write_failed";
    return false;
  }

  if (!posePrefs.begin("poses", true)) {
    if (posePrefs.begin("poses", false)) {
      const bool invalidated = posePrefs.putBool(validKey, false) == sizeof(bool);
      posePrefs.end();
      if (!invalidated) Serial.println("Pose NVS invalidation failed");
    }
    reason = "nvs_verify_failed";
    return false;
  }
  bool verified = posePrefs.getUInt("schema", 0) == CALIBRATION_SCHEMA_VERSION &&
                  posePrefs.getBool(validKey, false);
  for (uint8_t i = 0; verified && i < pose->count; ++i) {
    char key[15];
    poseKey(key, sizeof(key), *pose, pose->ids[i]);
    verified = posePrefs.isKey(key) && posePrefs.getInt(key, INT32_MIN) == positions[i];
  }
  posePrefs.end();
  if (!verified) {
    if (posePrefs.begin("poses", false)) {
      const bool invalidated = posePrefs.putBool(validKey, false) == sizeof(bool);
      posePrefs.end();
      if (!invalidated) Serial.println("Pose NVS invalidation failed");
    }
    reason = "nvs_verify_failed";
    return false;
  }
  reason = "captured";
  return true;
}

bool isGroupCalibrated(const uint8_t *ids, uint8_t count) {
  for (uint8_t i = 0; i < count; ++i) {
    const int index = servoIndex(ids[i]);
    if (index < 0 || !jointCal[index].calibrated) return false;
  }
  return true;
}

bool validatePose(const PoseDefinition &pose, String &reason) {
  if (!isGroupCalibrated(pose.ids, pose.count)) {
    reason = "joint_not_calibrated";
    return false;
  }
  for (uint8_t i = 0; i < pose.count; ++i) {
    const int index = servoIndex(pose.ids[i]);
    int32_t target = 0;
    if (!readPoseTarget(pose, pose.ids[i], target)) {
      reason = "pose_not_recorded";
      return false;
    }
    if (target < jointCal[index].minimum || target > jointCal[index].maximum ||
        target < 0 || target > servoHardMaximum(SERVOS[index])) {
      reason = "target_out_of_range";
      return false;
    }
  }
  return true;
}

void finishRobotAction() {
  const ControlOwner owner = robotActionOwner;
  robotAction = RobotActionState();
  robotActionOwner = ControlOwner::None;
  if (!emergencyStopLatched) robotState = "stopped";
  if (owner == ControlOwner::Web) {
    webMarkMotionIdle();
  }
}

const char *foldStateName() {
  switch (foldState) {
    case FoldControlState::ZeroRequired: return "zero_required";
    case FoldControlState::Idle: return "idle";
    case FoldControlState::Calibrating: return "calibrating";
    case FoldControlState::Unfolding: return "unfolding";
    case FoldControlState::Folding: return "folding";
    case FoldControlState::Stopped: return "stopped";
    case FoldControlState::Fault: return "fault";
  }
  return "fault";
}

bool foldFormalActionActive() {
  return foldState == FoldControlState::Unfolding ||
         foldState == FoldControlState::Folding;
}

bool foldAnyMotionActive() {
  return foldStepper.isActive() || foldFormalActionActive() ||
         foldState == FoldControlState::Calibrating;
}

void foldSetError(const String &message) {
  foldError = message;
  if (message.length()) Serial.printf("FOLD error=%s\n", message.c_str());
}

bool foldInvalidatePersistedTravel(String &reason) {
  foldStepper.clearTravelSteps();
  if (!foldPreferencesReady) {
    reason = "fold_nvs_unavailable";
    foldState = FoldControlState::Fault;
    foldSetError(reason);
    return false;
  }
  const bool invalidated =
      foldPrefs.putBool(FOLD_NVS_KEY_TRAVEL_VALID, false) == sizeof(uint8_t);
  const bool cleared = !foldPrefs.isKey(FOLD_NVS_KEY_TRAVEL_STEPS) ||
                       foldPrefs.remove(FOLD_NVS_KEY_TRAVEL_STEPS);
  if (!invalidated || !cleared) {
    reason = "fold_nvs_travel_invalidate_failed";
    foldState = FoldControlState::Fault;
    foldSetError(reason);
    return false;
  }
  return true;
}

bool foldMigrateMotionSchema(String &reason) {
  if (!foldInvalidatePersistedTravel(reason)) return false;
  if (foldPrefs.putUInt(FOLD_NVS_KEY_SCHEMA, FOLD_MOTION_SCHEMA_VERSION) !=
      sizeof(uint32_t)) {
    reason = "fold_nvs_schema_write_failed";
    foldState = FoldControlState::Fault;
    foldSetError(reason);
    return false;
  }
  return true;
}

bool foldPersistTravel(int32_t steps, String &reason) {
  if (!foldPreferencesReady) {
    reason = "fold_nvs_unavailable";
    return false;
  }
  const bool invalidated =
      foldPrefs.putBool(FOLD_NVS_KEY_TRAVEL_VALID, false) == sizeof(uint8_t);
  const bool stepsWritten = invalidated &&
      foldPrefs.putInt(FOLD_NVS_KEY_TRAVEL_STEPS, steps) == sizeof(int32_t);
  const bool schemaWritten = stepsWritten &&
      foldPrefs.putUInt(FOLD_NVS_KEY_SCHEMA, FOLD_MOTION_SCHEMA_VERSION) ==
          sizeof(uint32_t);
  const bool validWritten = schemaWritten &&
      foldPrefs.putBool(FOLD_NVS_KEY_TRAVEL_VALID, true) == sizeof(uint8_t);
  if (!validWritten) {
    reason = "fold_nvs_travel_write_failed";
    foldState = FoldControlState::Fault;
    foldSetError(reason);
    return false;
  }
  return true;
}

void beginFoldMotion() {
  // Drive only ESP32 GPIO3/8/10 for the quarter-step external A4988.
  foldPulseEngineStarted = foldStepper.begin();
  if (!foldPulseEngineStarted) {
    foldState = FoldControlState::Fault;
    foldSetError("fold_pulse_engine_start_failed");
  }

  foldPreferencesReady = foldPrefs.begin(FOLD_NVS_NAMESPACE, false);
  if (!foldPreferencesReady) {
    foldStepper.restoreConfiguration(0, false, FOLD_STEPPER_PLAN_MS);
    foldState = FoldControlState::Fault;
    foldSetError("fold_nvs_begin_failed");
    return;
  }

  const bool savedDirection = foldPrefs.getBool(FOLD_NVS_KEY_DIRECTION, false);
  const uint32_t storedSchema = foldPrefs.getUInt(FOLD_NVS_KEY_SCHEMA, 0);
  bool schemaMigrated = false;
  if (storedSchema != FOLD_MOTION_SCHEMA_VERSION) {
    String reason;
    schemaMigrated = foldMigrateMotionSchema(reason);
    if (!schemaMigrated) {
      // Never restore old microstep counts after a failed schema migration.
      // Latch the configuration unavailable across STOP/zero until reinit.
      foldStepper.restoreConfiguration(0, savedDirection,
                                       FOLD_STEPPER_PLAN_MS);
      foldPreferencesReady = false;
      return;
    }
  }

  const bool travelValid = foldPrefs.getBool(FOLD_NVS_KEY_TRAVEL_VALID, false);
  const int32_t savedTravel = travelValid
      ? foldPrefs.getInt(FOLD_NVS_KEY_TRAVEL_STEPS, 0) : 0;
  const bool restored = foldPulseEngineStarted &&
      foldStepper.restoreConfiguration(savedTravel, savedDirection,
                                       FOLD_STEPPER_PLAN_MS);
  if (!restored) {
    foldStepper.restoreConfiguration(0, savedDirection,
                                     FOLD_STEPPER_PLAN_MS);
    String reason;
    foldInvalidatePersistedTravel(reason);
  } else if (schemaMigrated) {
    foldSetError("fold_motion_schema_migrated_recalibrate");
  } else if (!travelValid && foldPrefs.isKey(FOLD_NVS_KEY_TRAVEL_STEPS)) {
    String reason;
    foldInvalidatePersistedTravel(reason);
    foldSetError("fold_invalid_travel_record_recalibrate");
  }
}

bool foldConfirmZero(String &reason) {
  if (!foldPreferencesReady) {
    reason = "fold_nvs_unavailable";
    return false;
  }
  if (foldAnyMotionActive() || robotAction.active || baseMotion.active || webDriveActive || isWebJogActive()) {
    reason = "busy";
    return false;
  }
  if (foldState == FoldControlState::Fault) {
    reason = "fold_fault";
    return false;
  }
  if (!foldStepper.confirmFoldedZero()) {
    reason = foldStepper.lastError();
    return false;
  }
  foldState = FoldControlState::Idle;
  foldError = "";
  reason = "folded_zero_confirmed";
  return true;
}

bool foldConfirmUnfolded(String &reason) {
  if (motionSafetyLocked() || motionStopPending()) {
    reason = "motion_locked";
    return false;
  }
  if (!foldPreferencesReady) {
    reason = "fold_nvs_unavailable";
    return false;
  }
  if (!foldPulseEngineStarted || !foldStepper.pulseEngineReady() ||
      foldState == FoldControlState::Fault) {
    reason = "fold_configuration_not_ready";
    return false;
  }
  if (foldAnyMotionActive() || robotAction.active || baseMotion.active ||
      webDriveActive || isWebJogActive()) {
    reason = "busy";
    return false;
  }
  if (!foldStepper.confirmUnfoldedEndpoint()) {
    reason = foldStepper.lastError();
    return false;
  }
  foldState = FoldControlState::Idle;
  foldError = "";
  reason = "unfolded_endpoint_confirmed_no_motion";
  return true;
}

bool foldJog(int32_t steps, String &reason) {
  if (foldAnyMotionActive() || robotAction.active || baseMotion.active || webDriveActive || isWebJogActive()) {
    reason = "busy";
    return false;
  }
  if (foldState == FoldControlState::Fault) {
    reason = "fold_fault";
    return false;
  }
  if (!foldStepper.startJog(steps)) {
    reason = foldStepper.lastError();
    return false;
  }
  foldState = FoldControlState::Calibrating;
  foldError = "";
  reason = "jog_started";
  return true;
}

bool foldSaveTravel(String &reason) {
  if (foldAnyMotionActive() || robotAction.active || baseMotion.active || webDriveActive || isWebJogActive()) {
    reason = "busy";
    return false;
  }
  if (foldState == FoldControlState::Fault) {
    reason = "fold_fault";
    return false;
  }
  if (!foldStepper.setTravelSteps(foldStepper.currentSteps(),
                                  FOLD_STEPPER_PLAN_MS)) {
    reason = foldStepper.lastError();
    return false;
  }
  if (!foldPersistTravel(foldStepper.travelSteps(), reason)) return false;
  foldError = "";
  reason = "travel_saved";
  return true;
}

// Explicit USB recovery for this user's unchanged quarter-step mechanism.
// Never called at boot or by Web actions; never restores an open-loop position.
bool restoreVerifiedStepperConfiguration(String &reason) {
  if (motionSafetyLocked() || motionStopPending()) {
    reason = "stepper_restore_safety_locked";
    return false;
  }
  if (webMotionEnabled || foldAnyMotionActive() || robotAction.active ||
      baseMotion.active || webDriveActive || isWebJogActive()) {
    reason = "stepper_restore_requires_idle_and_web_motion_disabled";
    return false;
  }
  if (!foldPreferencesReady || !foldPulseEngineStarted ||
      foldState == FoldControlState::Fault || foldStepper.timerFaulted()) {
    reason = "fold_configuration_not_ready";
    return false;
  }
  if (foldStepper.hasTravel()) {
    reason = "stepper_travel_already_valid";
    return false;
  }

  // Disable STEP/ENABLE and invalidate any previous homing before NVS writes.
  // The operator must confirm physical folded zero separately after recovery.
  foldStepper.emergencyStopAndDisable();
  foldState = FoldControlState::ZeroRequired;
  if (foldStepper.timerFaulted()) {
    foldState = FoldControlState::Fault;
    reason = "stepper_restore_disable_failed";
    foldSetError(reason);
    return false;
  }
  if (!foldInvalidatePersistedTravel(reason)) return false;
  const bool directionWritten =
      foldPrefs.putBool(FOLD_NVS_KEY_DIRECTION, true) == sizeof(uint8_t);
  const bool travelWritten = directionWritten && foldPersistTravel(15200, reason);
  const bool readbackOk = travelWritten &&
      foldPrefs.getUInt(FOLD_NVS_KEY_SCHEMA, 0) == FOLD_MOTION_SCHEMA_VERSION &&
      foldPrefs.getBool(FOLD_NVS_KEY_TRAVEL_VALID, false) &&
      foldPrefs.getInt(FOLD_NVS_KEY_TRAVEL_STEPS, 0) == 15200 &&
      foldPrefs.getBool(FOLD_NVS_KEY_DIRECTION, false);
  if (!readbackOk || motionSafetyLocked() || motionStopPending()) {
    String cleanupReason;
    foldInvalidatePersistedTravel(cleanupReason);
    foldState = FoldControlState::Fault;
    reason = "stepper_restore_failed_or_interrupted_no_motion";
    foldSetError(reason);
    return false;
  }
  if (!foldStepper.restoreConfiguration(15200, true, FOLD_STEPPER_PLAN_MS)) {
    String cleanupReason;
    foldInvalidatePersistedTravel(cleanupReason);
    foldState = FoldControlState::Fault;
    reason = "stepper_restore_profile_rejected";
    foldSetError(reason);
    return false;
  }
  foldError = "";
  reason = "stepper_15200_restored_no_motion_zero_required";
  return true;
}

bool foldResetTravel(String &reason) {
  if (foldAnyMotionActive() || robotAction.active || baseMotion.active || webDriveActive || isWebJogActive()) {
    reason = "busy";
    return false;
  }
  if (foldState == FoldControlState::Fault) {
    reason = "fold_fault";
    return false;
  }
  if (!foldInvalidatePersistedTravel(reason)) return false;
  foldError = "";
  reason = "travel_reset";
  return true;
}

bool foldToggleDirection(String &reason) {
  if (foldAnyMotionActive() || robotAction.active || baseMotion.active || webDriveActive || isWebJogActive()) {
    reason = "busy";
    return false;
  }
  if (foldState == FoldControlState::Fault) {
    reason = "fold_fault";
    return false;
  }
  if (!foldStepper.isHomed() || foldStepper.currentSteps() != 0) {
    reason = "return_to_folded_zero_first";
    return false;
  }
  const bool next = !foldStepper.directionInverted();
  if (!foldInvalidatePersistedTravel(reason)) return false;
  if (foldPrefs.putBool(FOLD_NVS_KEY_DIRECTION, next) != sizeof(uint8_t) ||
      !foldStepper.setDirectionInverted(next)) {
    reason = "fold_direction_write_failed";
    foldState = FoldControlState::Fault;
    foldSetError(reason);
    return false;
  }
  foldError = "";
  reason = next ? "direction_inverted" : "direction_normal";
  return true;
}

bool foldRescan(String &reason) {
  if (localMotionActive() || isWebJogActive()) {
    reason = "busy";
    return false;
  }
  foldId11Online = servoPing(TORSO_SECOND_BOARD_ID);
  reason = foldId11Online ? "id11_online" : "id11_offline";
  return foldId11Online;
}

void serviceFoldMotion() {
  foldStepper.service();
  const uint32_t now = millis();
  if (foldStepper.timerFaulted() && foldState != FoldControlState::Fault) {
    allStop();
    foldState = FoldControlState::Fault;
    foldSetError("fold_pulse_timer_runtime_failed_restart_then_reconfirm_zero");
    return;
  }
  if (foldState == FoldControlState::Calibrating && !foldStepper.isActive()) {
    foldState = FoldControlState::Idle;
  }
  if (!foldFormalActionActive()) return;

  if (foldStepper.completedNormally() && !foldStepper.completedWithinLimit()) {
    allStop();
    foldState = FoldControlState::Fault;
    foldSetError("fold_action_timeout_reconfirm_zero");
    return;
  }
  if (foldStepper.completedNormally() && foldStepper.completedWithinLimit()) {
    foldFormalCompletionObserved = true;
  }
  if (static_cast<int32_t>(now - foldActionDeadlineMs) >= 0 &&
      !foldFormalCompletionObserved) {
    allStop();
    foldState = FoldControlState::Fault;
    foldSetError("fold_action_timeout_reconfirm_zero");
    return;
  }
  if (static_cast<int32_t>(now - foldActionFinishMs) >= 0 &&
      foldFormalCompletionObserved) {
    foldState = FoldControlState::Idle;
    foldError = "";
    Serial.printf("FOLD mode=%s software command complete; mechanical endpoints not verified\n",
                  foldActionStepperOnly ? "stepper_only" : "synchronized");
  }
}

bool validateFoldStepperAction(String &reason) {
  if (!foldPulseEngineStarted || !foldStepper.pulseEngineReady() ||
      !foldPreferencesReady || foldState == FoldControlState::Fault) {
    reason = "fold_configuration_not_ready";
    return false;
  }
  if (!foldStepper.isHomed()) {
    reason = "zero_not_confirmed";
    return false;
  }
  if (!foldStepper.hasTravel()) {
    reason = "travel_not_calibrated";
    return false;
  }
  return true;
}

bool validateTorsoLowerAction(String &reason, bool unfolding) {
  if (!validateFoldStepperAction(reason)) return false;
  const int index = servoIndex(TORSO_SECOND_BOARD_ID);
  if (index < 0 || !jointCal[index].calibrated) {
    reason = "id11_not_calibrated";
    return false;
  }
  if (FOLD_ID11_FOLDED_POSITION < jointCal[index].minimum ||
      FOLD_ID11_UNFOLDED_POSITION > jointCal[index].maximum) {
    reason = "id11_target_out_of_range";
    return false;
  }
  foldId11Online = servoPing(TORSO_SECOND_BOARD_ID);
  if (!foldId11Online) {
    reason = "id11_offline";
    return false;
  }
  return id11ValidateHardwareForMotion(reason, unfolding);
}

bool startTorsoLowerAction(bool unfolding) {
  const int32_t stepperTarget = unfolding ? foldStepper.travelSteps() : 0;
  const int32_t id11Target = unfolding ? FOLD_ID11_UNFOLDED_POSITION
                                       : FOLD_ID11_FOLDED_POSITION;
  if (!foldStepper.startEndpointMove(stepperTarget, FOLD_STEPPER_PLAN_MS,
                                     FOLD_COMPLETION_LIMIT_US)) {
    foldSetError(foldStepper.lastError());
    return false;
  }
  if (motionStopPending()) {
    foldStepper.emergencyStopAndDisable();
    foldState = FoldControlState::Stopped;
    foldSetError("stop_requested_reconfirm_zero");
    return false;
  }
  if (!servoMoveToFast(TORSO_SECOND_BOARD_ID, id11Target,
                       FOLD_ACTION_DURATION_MS)) {
    foldStepper.emergencyStopAndDisable();
    foldState = FoldControlState::Stopped;
    foldSetError("id11_start_failed_reconfirm_zero");
    return false;
  }
  foldActionStepperOnly = false;
  foldFormalCompletionObserved = false;
  foldActionFinishMs = millis() + FOLD_ACTION_DURATION_MS;
  foldActionDeadlineMs = foldActionFinishMs + FOLD_ACTION_TIMEOUT_MARGIN_MS;
  foldState = unfolding ? FoldControlState::Unfolding
                        : FoldControlState::Folding;
  foldError = "";
  Serial.printf("TORSO %s A4988_EXTERNAL->%ld ID11->%ld duration_ms=%u\n",
                unfolding ? "unfold" : "fold", static_cast<long>(stepperTarget),
                static_cast<long>(id11Target),
                static_cast<unsigned>(FOLD_ACTION_DURATION_MS));
  return true;
}

bool startStepperOnlyTest(bool unfolding, uint32_t durationMs, String &reason) {
  if (durationMs != 6000 && durationMs != 10000 &&
      durationMs != 20000 && durationMs != 30000) {
    reason = "stepper_test_duration_use_6_10_20_30_seconds";
    return false;
  }
  if (motionSafetyLocked() || motionStopPending()) {
    reason = "motion_locked";
    return false;
  }
  if (foldAnyMotionActive() || robotAction.active || baseMotion.active ||
      webDriveActive || isWebJogActive() || webOwnsMotion) {
    reason = "busy";
    return false;
  }
  if (!validateFoldStepperAction(reason)) return false;
  const int32_t target = unfolding ? foldStepper.travelSteps() : 0;
  const uint32_t planMs = durationMs - 200U;
  const uint64_t limitUs = static_cast<uint64_t>(durationMs) * 1000ULL;
  if (!foldStepper.startEndpointMove(target, planMs, limitUs)) {
    reason = foldStepper.lastError();
    return false;
  }
  if (motionStopPending()) {
    foldStepper.emergencyStopAndDisable();
    foldState = FoldControlState::Stopped;
    reason = "stop_requested_reconfirm_zero";
    foldSetError(reason);
    return false;
  }
  foldActionStepperOnly = true;
  foldFormalCompletionObserved = false;
  foldActionFinishMs = millis() + durationMs;
  foldActionDeadlineMs = foldActionFinishMs + FOLD_ACTION_TIMEOUT_MARGIN_MS;
  foldState = unfolding ? FoldControlState::Unfolding : FoldControlState::Folding;
  foldError = "";
  reason = "stepper_only_started";
  Serial.printf("STEPPER_ONLY %s target=%ld action_ms=%lu plan_ms=%lu hard_limit_us=%llu ID11_not_commanded=yes\n",
                unfolding ? "unfold" : "fold", static_cast<long>(target),
                static_cast<unsigned long>(durationMs), static_cast<unsigned long>(planMs),
                limitUs);
  return true;
}

bool handleStepperOnlyTestCommand(const String &command) {
  if (!command.startsWith("stepper_test")) return false;
  if (command == "stepper_test") {
    Serial.println("stepper_test unfold|fold 6|10|20|30 : calibrated A4988 only; keep ID11 power disconnected for isolated testing");
    return true;
  }
  // Exact whitelist: no toInt coercion, suffixes, extra tokens or implicit motion.
  for (uint32_t seconds : {6U, 10U, 20U, 30U}) {
    for (int direction = 0; direction < 2; ++direction) {
      const bool unfolding = direction == 0;
      char expected[48];
      snprintf(expected, sizeof(expected), "stepper_test %s %lu",
               unfolding ? "unfold" : "fold", static_cast<unsigned long>(seconds));
      if (command == expected) {
        String reason;
        const bool ok = startStepperOnlyTest(unfolding, seconds * 1000U, reason);
        Serial.printf("%s %s\n", ok ? "OK" : "ERR", reason.c_str());
        return true;
      }
    }
  }
  Serial.println("ERR use_stepper_test_unfold_or_fold_and_seconds_6_10_20_30");
  return true;
}

// Read-only diagnostic for the recorded web actions. This makes an NVS pose
// failure explicit instead of returning the generic "pose_not_recorded" from
// the serving action preflight.
void printPoseCheck() {
  Serial.println("POSE CHECK");
  for (const PoseDefinition &pose : POSES) {
    String reason;
    const bool valid = validatePose(pose, reason);
    Serial.printf("  %-15s %s", pose.name, valid ? "ready" : "missing");
    if (!valid) Serial.printf(" reason=%s", reason.c_str());
    Serial.println();
  }
}

bool startNamedAction(const String &name, int seq, ControlOwner owner, String &rejectReason,
                      uint32_t baseDurationMs) {
  const uint32_t controlGeneration = webGetControlGeneration();
  // SHOWCASE_ROUTE_BEGIN
  if (name == "showcase_demo") return startShowcaseAction(owner, seq, rejectReason);
  // SHOWCASE_ROUTE_END
  if (name == "stop_all" || name == "stop") {
    allStop();
    return true;
  }
  if (robotAction.active || baseMotion.active || webDriveActive || webOwnsMotion || isWebJogActive() ||
      foldAnyMotionActive()) {
    rejectReason = "busy";
    return false;
  }
  if (emergencyStopLatched) {
    rejectReason = "motion_locked";
    return false;
  }

  RobotActionKind kind = RobotActionKind::None;
  if (name == "base_forward") kind = RobotActionKind::BaseForward;
  else if (name == "base_backward") kind = RobotActionKind::BaseBackward;
  else if (name == "turn_left") kind = RobotActionKind::TurnLeft;
  else if (name == "turn_right") kind = RobotActionKind::TurnRight;
  else if (name == "torso_unfold") kind = RobotActionKind::TorsoUnfold;
  else if (name == "torso_fold") kind = RobotActionKind::TorsoFold;
  else {
    rejectReason = "unknown_action";
    return false;
  }

  const bool isBaseAction = kind >= RobotActionKind::BaseForward &&
                            kind <= RobotActionKind::TurnRight;
  uint32_t effectiveBaseDurationMs = baseDurationMs;
  if (isBaseAction && effectiveBaseDurationMs == 0) {
    effectiveBaseDurationMs =
        (kind == RobotActionKind::BaseForward || kind == RobotActionKind::BaseBackward)
            ? BASE_FORWARD_MS
            : BASE_TURN_MS;
  }

  const uint32_t preflightStartedAtMs = millis();
  if ((kind == RobotActionKind::TorsoUnfold || kind == RobotActionKind::TorsoFold) &&
      !validateTorsoLowerAction(rejectReason, kind == RobotActionKind::TorsoUnfold)) {
    return false;
  }

  // The servo preflight can wait on UART while HTTP receives STOP.
  if (motionStopPending() ||
      (owner == ControlOwner::Web && !webMotionStartAllowed(controlGeneration))) {
    rejectReason = "stop_requested";
    return false;
  }

  if (owner == ControlOwner::Web) {
    const uint32_t preflightMs = millis() - preflightStartedAtMs;
    const uint32_t leaseAgeMs = webLeaseAgeMs(controlGeneration);
    Serial.printf("WEB action preflight elapsed_ms=%lu lease_age_ms=%lu generation=%lu queued=%u\n",
                  static_cast<unsigned long>(preflightMs), static_cast<unsigned long>(leaseAgeMs),
                  static_cast<unsigned long>(controlGeneration),
                  webCommandQueue ? static_cast<unsigned>(uxQueueMessagesWaiting(webCommandQueue)) : 0U);
    // A real loss of heartbeat during preflight must reject BEFORE energizing
    // either actuator, not start the motion and stop it on the following loop.
    if (!webMotionStartAllowed(controlGeneration) || !webLeaseFresh(controlGeneration)) {
      webMotionEnabled = false;
      rejectReason = "heartbeat_timeout_before_start";
      return false;
    }
  }

  robotAction = RobotActionState();
  robotActionOwner = owner;
  robotAction.kind = kind;
  robotAction.seq = seq;
  robotAction.phase = 0;
  robotAction.active = true;
  robotAction.timeoutDeadlineMs = millis() +
      (isBaseAction ? effectiveBaseDurationMs + ACTION_TIMEOUT_GRACE_MS
                    : 15000);
  bool started = false;
  if (kind == RobotActionKind::BaseForward) started = startBaseMotion(BASE_DEMO_PWM, BASE_DEMO_PWM, effectiveBaseDurationMs);
  else if (kind == RobotActionKind::BaseBackward) started = startBaseMotion(-BASE_DEMO_PWM, -BASE_DEMO_PWM, effectiveBaseDurationMs);
  else if (kind == RobotActionKind::TurnLeft) started = startBaseMotion(-BASE_DEMO_PWM, BASE_DEMO_PWM, effectiveBaseDurationMs);
  else if (kind == RobotActionKind::TurnRight) started = startBaseMotion(BASE_DEMO_PWM, -BASE_DEMO_PWM, effectiveBaseDurationMs);
  else if (kind == RobotActionKind::TorsoUnfold) started = startTorsoLowerAction(true);
  else if (kind == RobotActionKind::TorsoFold) started = startTorsoLowerAction(false);

  if (!started) {
    allStop();
    rejectReason = "actuator_start_failed";
    return false;
  }
  if (baseMotion.active) {
    robotAction.deadlineMs = baseMotion.deadlineMs;
  } else if (kind == RobotActionKind::TorsoUnfold || kind == RobotActionKind::TorsoFold) {
    robotAction.deadlineMs = millis() + FOLD_ACTION_DURATION_MS;
  } else {
    robotAction.deadlineMs = 0;
  }
  robotState = "running";
  if (owner == ControlOwner::Web) {
    webMarkMotionActive();
  }
  return true;
}

void serviceRobotAction() {
  if (!robotAction.active) return;
  // SHOWCASE_SERVICE_BEGIN
  if (robotAction.kind == RobotActionKind::Showcase) { serviceShowcaseAction(); return; }
  // SHOWCASE_SERVICE_END
  const uint32_t now = millis();
  if (static_cast<int32_t>(now - robotAction.timeoutDeadlineMs) >= 0) {
    rejectActiveAction("action_timeout");
    return;
  }
  if (robotAction.kind <= RobotActionKind::TurnRight) {
    if (!baseMotion.active) finishRobotAction();
    return;
  }
  if (robotAction.kind == RobotActionKind::TorsoUnfold ||
      robotAction.kind == RobotActionKind::TorsoFold) {
    if (foldState == FoldControlState::Fault || foldState == FoldControlState::Stopped) {
      rejectActiveAction(foldError.length() ? foldError.c_str() : "fold_stopped");
      return;
    }
    if (foldState != FoldControlState::Idle ||
        static_cast<int32_t>(now - robotAction.deadlineMs) < 0) return;
    finishRobotAction();
    return;
  }
  if (static_cast<int32_t>(now - robotAction.deadlineMs) < 0) return;

  finishRobotAction();
}

void servoScan() {
  beginHtd(115200);
  Serial.println("Shared HTD/HX BusLinker scan at 115200 (registered IDs only)");
  Serial.println("HTD devices:");
  for (const ServoDescriptor &servo : SERVOS) {
    if (servo.protocol != ServoProtocol::Htd) continue;
    const bool online = servoPing(servo.id);
    Serial.printf("  ID %u %s %s\n", servo.id,
                  servo.protocol == ServoProtocol::Htd ? "HTD" : "HX",
                  online ? "online" : "no_response");
  }
  Serial.println("HX devices:");
  for (const ServoDescriptor &servo : SERVOS) {
    if (servo.protocol != ServoProtocol::Hx) continue;
    const bool online = servoPing(servo.id);
    Serial.printf("  ID %u HX %s\n", servo.id, online ? "online" : "no_response");
  }
}

void printCalibrationList() {
  for (size_t i = 0; i < SERVO_COUNT; ++i) {
    Serial.printf("ID %u %-24s calibrated=%s min=%ld center=%ld max=%ld inverted=%s\n",
                  SERVOS[i].id, SERVOS[i].name, jointCal[i].calibrated ? "yes" : "no",
                  static_cast<long>(jointCal[i].minimum), static_cast<long>(jointCal[i].center),
                  static_cast<long>(jointCal[i].maximum), jointCal[i].inverted ? "yes" : "no");
  }
}

bool motionSafetyLocked() {
  return emergencyStopLatched || drivePwmFault || foldStepper.levelTestMode();
}

bool localMotionActive() {
  return robotAction.active || baseMotion.active || webDriveActive || webOwnsMotion ||
         foldAnyMotionActive() || isWebJogActive();
}

const char *controlOwnerName(ControlOwner owner) {
  switch (owner) {
    case ControlOwner::Web: return "web";
    case ControlOwner::Usb: return "usb";
    default: return "none";
  }
}

bool isWebJogActive() {
  if (webArmWatchdogArmed || webArmHoldPendingMask != 0) return true;
  for (size_t i = 0; i < ARM_SERVO_COUNT; ++i) {
    if (webArmDirections[i] != 0) return true;
  }
  return false;
}

ControlOwner activeControlOwner() {
  if (robotAction.active) return robotActionOwner;
  if (webDriveActive || webOwnsMotion || isWebJogActive()) return ControlOwner::Web;
  if (baseMotion.active || foldAnyMotionActive()) return ControlOwner::Usb;
  return ControlOwner::None;
}

bool localTextCommandAllowedDuringMotion(const String &command) {
  return command == "fold_status" ||
         command == "step_diag" || command == "d_status" ||
         command == "all_stop" || command == "stop" || command == "d_stop" ||
         command == "h_stop" ||
         command == "hx_torque_off_all";
}

bool usbJsonCommandAllowedInFinalDemo(const String &command) {
  return command == "ping" || command == "heartbeat" ||
         command == "status" || command == "stop";
}

void allStop() {
  // SHOWCASE_STOP_BEGIN
  showcaseRuntime.active = false;
  // SHOWCASE_STOP_END
  // Discard every older queued enable/action, including USB-initiated STOP.
  webInvalidateControlCommands();
  const bool preserveSafetyState = motionSafetyLocked();
  const bool webWasInControl = webOwnsMotion || robotActionOwner == ControlOwner::Web;
  // Stop chassis before STEP diagnostics or UART traffic can delay it.
  chassisStop();
  foldStepper.emergencyStopAndDisable();
  foldState = FoldControlState::Stopped;
  foldFormalCompletionObserved = false;
  foldSetError("stopped_reconfirm_folded_zero");
  Serial.println("ALL STOP");
  robotAction = RobotActionState();
  robotActionOwner = ControlOwner::None;
  webMotionEnabled = false;
  webDriveActive = false;
  webOwnsMotion = false;
  webLinearCommand = 0;
  webTurnCommand = 0;
  for (size_t i = 0; i < ARM_SERVO_COUNT; ++i) {
    webArmDirections[i] = 0;
    webArmShadowValid[i] = false;
    webArmTorqueReady[i] = false;
  }
  webArmWatchdogArmed = false;
  webArmHoldPendingMask = 0;
  if (!preserveSafetyState) robotState = "stopped";
  for (const ServoDescriptor &servo : SERVOS) {
    if (servo.protocol == ServoProtocol::Htd) servoStop(servo.id);
  }
  scheduleHxHold();
  while (hxHold.active) serviceHxHold();
  (void)webWasInControl;
}

void webMarkMotionActive() {
  webOwnsMotion = true;
  if (!motionSafetyLocked()) robotState = "running";
}

void webMarkMotionIdle() {
  if (!webOwnsMotion) return;
  webOwnsMotion = false;
  if (!motionSafetyLocked() && !robotAction.active && !webDriveActive &&
      !isWebJogActive() && !foldAnyMotionActive()) {
    robotState = "stopped";
  }
}

void rejectActiveAction(const char *reason) {
  Serial.printf("ACTION rejected kind=%u phase=%u reason=%s\n",
                static_cast<unsigned>(robotAction.kind),
                static_cast<unsigned>(robotAction.phase), reason ? reason : "unknown");
  allStop();
}

void stopForWebControl() {
  allStop();
}

int jsonKeyIndex(const String &json, const char *key) {
  String pattern = "\"";
  pattern += key;
  pattern += "\"";
  return json.indexOf(pattern);
}

bool jsonGetString(const String &json, const char *key, String &value) {
  int keyPos = jsonKeyIndex(json, key);
  if (keyPos < 0) return false;

  int colon = json.indexOf(':', keyPos);
  if (colon < 0) return false;

  int quoteStart = json.indexOf('"', colon + 1);
  if (quoteStart < 0) return false;
  int quoteEnd = json.indexOf('"', quoteStart + 1);
  if (quoteEnd < 0) return false;

  value = json.substring(quoteStart + 1, quoteEnd);
  return true;
}

bool jsonGetFloat(const String &json, const char *key, float &value) {
  int keyPos = jsonKeyIndex(json, key);
  if (keyPos < 0) return false;

  int colon = json.indexOf(':', keyPos);
  if (colon < 0) return false;

  int pos = colon + 1;
  while (pos < json.length() && (json[pos] == ' ' || json[pos] == '\t' || json[pos] == '\r' || json[pos] == '\n')) {
    pos++;
  }

  int end = pos;
  while (end < json.length()) {
    char c = json[end];
    if ((c >= '0' && c <= '9') || c == '-' || c == '+' || c == '.' || c == 'e' || c == 'E') {
      end++;
    } else {
      break;
    }
  }

  if (end == pos) return false;
  value = json.substring(pos, end).toFloat();
  return true;
}

bool jsonGetInt(const String &json, const char *key, int &value) {
  float parsed = 0;
  if (!jsonGetFloat(json, key, parsed)) return false;
  value = static_cast<int>(parsed);
  return true;
}

bool jsonCommandAllowedDuringEstop(const String &command) {
  return command == "ping" || command == "heartbeat" || command == "status" || command == "stop";
}

bool textCommandAllowedDuringEstop(const String &command) {
  return command == "help" || command == "all_stop" ||
         command == "stop" || command == "fold_status" ||
         command == "step_diag" || command == "d_status" ||
         command == "h_ping" || command == "h_pos" || command == "h_stop" ||
         command == "d_ticks" || command == "d_zero" || command == "d_stop" ||
         command == "hx_torque_off_all" ||
         command == "servo_scan" || command.startsWith("servo_ping ") ||
         command.startsWith("servo_pos ") || command.startsWith("servo_baud ") ||
         command == "cal_list" || command == "cal_read";
}

void jsonPrintIntOrNull(bool ok, int value) {
  if (ok) {
    Serial.print(value);
  } else {
    Serial.print("null");
  }
}

void jsonOkStart(int seq) {
  Serial.print("{\"seq\":");
  Serial.print(seq);
  Serial.print(",\"ok\":true");
}

void jsonSendOk(int seq, const char *msg) {
  jsonOkStart(seq);
  Serial.print(",\"msg\":\"");
  Serial.print(msg);
  Serial.println("\"}");
}

void jsonSendErr(int seq, const char *err) {
  Serial.print("{\"seq\":");
  Serial.print(seq);
  Serial.print(",\"ok\":false,\"err\":\"");
  Serial.print(err);
  Serial.println("\"}");
}

void jsonSendStatus(int seq) {
  // A status query must never block the chassis/host watchdog on the servo bus.
  const int index = servoIndex(HTD_ID);
  const bool htdPosOk = index >= 0 && lastServoPositionValid[index] &&
      millis() - lastServoPositionMs[index] < ARM_POSITION_STALE_MS;
  const int htdPos = htdPosOk ? lastServoPosition[index] : 0;
  const long driveTickValue = driveReadTicks();

  jsonOkStart(seq);
  Serial.print(",\"protocol\":\"");
  Serial.print(PROTOCOL_VERSION);
  Serial.print("\",\"htd\":{\"pos\":");
  jsonPrintIntOrNull(htdPosOk, htdPos);
  Serial.print("},\"drive\":{\"ticks\":");
  Serial.print(driveTickValue);
  Serial.print(",\"left_pwm\":");
  Serial.print(driveLeftPwm);
  Serial.print(",\"right_pwm\":");
  Serial.print(driveRightPwm);
  Serial.println("}}");
}

bool protocolApplyDrive(const String &json) {
  int leftPwm = 0;
  int rightPwm = 0;
  int pwm = 0;
  bool hasLeftRight = jsonGetInt(json, "left_pwm", leftPwm) | jsonGetInt(json, "right_pwm", rightPwm);
  if (hasLeftRight) {
    leftPwm = constrain(leftPwm, -DRIVE_PROTOCOL_MAX_PWM, DRIVE_PROTOCOL_MAX_PWM);
    rightPwm = constrain(rightPwm, -DRIVE_PROTOCOL_MAX_PWM, DRIVE_PROTOCOL_MAX_PWM);
    chassisDriveBoth(leftPwm, rightPwm);
    return true;
  }
  if (!jsonGetInt(json, "pwm", pwm)) {
    return false;
  }
  pwm = constrain(pwm, -DRIVE_PROTOCOL_MAX_PWM, DRIVE_PROTOCOL_MAX_PWM);
  chassisDriveBoth(pwm, pwm);
  return true;
}

bool protocolApplyHtd(const String &json) {
  int target = 0;
  int delta = 0;
  int timeMs = HTD_MOVE_TIME_MS;
  jsonGetInt(json, "time_ms", timeMs);
  timeMs = constrain(timeMs, 100, 2000);

  bool hasTarget = jsonGetInt(json, "htd_pos", target) || jsonGetInt(json, "pos", target);
  bool hasDelta = jsonGetInt(json, "htd_delta", delta) || jsonGetInt(json, "delta", delta);
  if (!hasTarget && !hasDelta) {
    return false;
  }

  int current = 0;
  if (!htdReadPosition(current)) {
    return false;
  }

  if (hasDelta) {
    delta = constrain(delta, -HTD_SMALL_DELTA_UNITS, HTD_SMALL_DELTA_UNITS);
    target = current + delta;
  } else {
    int minTarget = current - HTD_SMALL_DELTA_UNITS;
    int maxTarget = current + HTD_SMALL_DELTA_UNITS;
    target = constrain(target, minTarget, maxTarget);
  }

  target = constrain(target, 0, 1000);
  htdMoveTo(static_cast<uint16_t>(target), static_cast<uint16_t>(timeMs));
  return true;
}

void handleJsonCommand(String json) {
  json.trim();
  int seq = 0;
  jsonGetInt(json, "seq", seq);

  String cmd;
  if (!jsonGetString(json, "cmd", cmd)) {
    jsonSendErr(seq, "missing_cmd");
    return;
  }
  cmd.toLowerCase();

  if (!ENABLE_BLOCKING_USB_DEBUG && !usbJsonCommandAllowedInFinalDemo(cmd)) {
    jsonSendErr(seq, "blocking_debug_disabled");
    return;
  }

  if (emergencyStopLatched && !jsonCommandAllowedDuringEstop(cmd)) {
    jsonSendErr(seq, "estop_latched");
    return;
  }

  if (motionSafetyLocked() && !jsonCommandAllowedDuringEstop(cmd)) {
    jsonSendErr(seq, "motion_locked");
    return;
  }

  if (ENABLE_BLOCKING_USB_DEBUG) {
    jsonProtocolActive = true;
    lastHostCommandMs = millis();
  }

  if (cmd == "ping") {
    jsonOkStart(seq);
    Serial.print(",\"msg\":\"pong\",\"protocol\":\"");
    Serial.print(PROTOCOL_VERSION);
    Serial.println("\"}");
  } else if (cmd == "heartbeat") {
    jsonSendOk(seq, "heartbeat");
  } else if (cmd == "status") {
    jsonSendStatus(seq);
  } else if (cmd == "stop") {
    allStop();
    jsonSendOk(seq, "stopped");
  } else if (cmd == "drive") {
    if (protocolApplyDrive(json)) jsonSendOk(seq, "drive_applied");
    else jsonSendErr(seq, "drive_missing_pwm");
  } else if (cmd == "htd") {
    if (protocolApplyHtd(json)) jsonSendOk(seq, "htd_applied");
    else jsonSendErr(seq, "htd_failed");
  } else if (cmd == "motion") {
    bool any = false;
    bool ok = true;
    if (jsonKeyIndex(json, "pwm") >= 0 || jsonKeyIndex(json, "left_pwm") >= 0 || jsonKeyIndex(json, "right_pwm") >= 0) {
      any = true;
      ok = protocolApplyDrive(json) && ok;
    }
    if (jsonKeyIndex(json, "htd") >= 0 || jsonKeyIndex(json, "htd_pos") >= 0 || jsonKeyIndex(json, "htd_delta") >= 0) {
      any = true;
      ok = protocolApplyHtd(json) && ok;
    }
    if (!any) jsonSendErr(seq, "motion_empty");
    else if (ok) jsonSendOk(seq, "motion_applied");
    else jsonSendErr(seq, "motion_partial_or_failed");
  } else {
    jsonSendErr(seq, "unknown_cmd");
  }
  if (ENABLE_BLOCKING_USB_DEBUG) lastHostCommandMs = millis();
}

void checkHostWatchdog() {
  if (!jsonProtocolActive) {
    return;
  }

  if (millis() - lastHostCommandMs > HOST_WATCHDOG_MS) {
    jsonProtocolActive = false;
    Serial.println("{\"event\":\"watchdog_timeout\",\"ok\":false,\"err\":\"host_timeout\"}");
    allStop();
  }
}

const char *webActionName() {
  switch (robotAction.kind) {
    case RobotActionKind::BaseForward: return "base_forward";
    case RobotActionKind::BaseBackward: return "base_backward";
    case RobotActionKind::TurnLeft: return "turn_left";
    case RobotActionKind::TurnRight: return "turn_right";
    case RobotActionKind::TorsoUnfold: return "torso_unfold";
    case RobotActionKind::TorsoFold: return "torso_fold";
    // SHOWCASE_NAME_BEGIN
    case RobotActionKind::Showcase: return "showcase_demo";
    // SHOWCASE_NAME_END
    default: return "idle";
  }
}

bool webSocketFdIsActive(int fd) {
  return webHttpServer != nullptr && fd >= 0 &&
         httpd_ws_get_fd_info(webHttpServer, fd) == HTTPD_WS_CLIENT_WEBSOCKET;
}

void webTransferComplete(esp_err_t err, int fd, void *arg) {
  auto *frame = static_cast<httpd_ws_frame_t *>(arg);
  free(frame->payload);
  free(frame);
  portENTER_CRITICAL(&webTxMux);
  --webPendingTx;
  portEXIT_CRITICAL(&webTxMux);
  if (err != ESP_OK && fd == webActiveClientFd) webDisconnectStopRequested = true;
}

esp_err_t webSendText(int fd, const char *payload) {
  if (!payload || !webSocketFdIsActive(fd)) return ESP_FAIL;
  portENTER_CRITICAL(&webTxMux);
  const bool full = webPendingTx >= 4;
  if (!full) ++webPendingTx;
  portEXIT_CRITICAL(&webTxMux);
  if (full) {
    webDisconnectStopRequested = true;
    return ESP_ERR_NO_MEM;
  }
  auto *frame = static_cast<httpd_ws_frame_t *>(calloc(1, sizeof(httpd_ws_frame_t)));
  if (frame) {
    frame->type = HTTPD_WS_TYPE_TEXT;
    frame->len = strlen(payload);
    frame->payload = static_cast<uint8_t *>(malloc(frame->len));
  }
  if (!frame || !frame->payload) {
    free(frame);
    portENTER_CRITICAL(&webTxMux);
    --webPendingTx;
    portEXIT_CRITICAL(&webTxMux);
    webDisconnectStopRequested = true;
    return ESP_ERR_NO_MEM;
  }
  memcpy(frame->payload, payload, frame->len);
  // HTTP task owns the copied frame until callback; loop never waits for TCP send.
  const esp_err_t err = httpd_ws_send_data_async(
      webHttpServer, fd, frame, webTransferComplete, frame);
  if (err != ESP_OK) webTransferComplete(err, fd, frame);
  return err;
}

void webSendAck(int fd, const char *command, bool ok, const char *reason = nullptr) {
  cJSON *root = cJSON_CreateObject();
  if (!root) return;
  cJSON_AddStringToObject(root, "type", ok ? "ack" : "error");
  cJSON_AddStringToObject(root, "command", command ? command : "unknown");
  cJSON_AddBoolToObject(root, "ok", ok);
  if (reason) cJSON_AddStringToObject(root, ok ? "message" : "reason", reason);
  char *payload = cJSON_PrintUnformatted(root);
  if (payload) {
    webSendText(fd, payload);
    cJSON_free(payload);
  }
  cJSON_Delete(root);
}

void webSendStatus(int fd) {
  if (!webSocketFdIsActive(fd)) return;
  cJSON *root = cJSON_CreateObject();
  if (!root) return;
  const uint32_t now = millis();
  cJSON_AddStringToObject(root, "type", "status");
  cJSON_AddStringToObject(root, "protocol", "suzhou-foldbot-web-v1");
  cJSON_AddNumberToObject(root, "rosControlVersion", 1);
  cJSON_AddStringToObject(root, "firmwareBuild", FIRMWARE_BUILD_ID);
  cJSON_AddNumberToObject(root, "controlSession", webControlSession);
  cJSON_AddNumberToObject(root, "statusSeq", ++webStatusSequence);
  cJSON_AddNumberToObject(root, "uptimeMs", now);
  cJSON_AddBoolToObject(root, "externalArm", externalArm);
  cJSON_AddNumberToObject(root, "armProtocol", 1);
  cJSON_AddBoolToObject(root, "armStepRangeGuard", true);
  cJSON_AddNumberToObject(root, "applied_jog_seq", webArmAppliedJogSequence);
  cJSON_AddNumberToObject(root, "jog_step_seq", webArmJogStepSequence);
  cJSON_AddStringToObject(root, "jog_step_status", webArmJogStepStatus);
  cJSON_AddBoolToObject(root, "watchdog_armed", webArmWatchdogArmed);
  cJSON_AddBoolToObject(root, "connected", true);
  cJSON_AddBoolToObject(root, "motionEnabled", webMotionEnabled);
  cJSON_AddBoolToObject(root, "estop", emergencyStopLatched);
  cJSON_AddStringToObject(root, "robotState", robotState.c_str());
  cJSON_AddStringToObject(root, "controlOwner", controlOwnerName(activeControlOwner()));
  cJSON_AddBoolToObject(root, "controlBusy", activeControlOwner() != ControlOwner::None);
  cJSON_AddStringToObject(root, "action", robotAction.active ? webActionName() : "idle");
  // SHOWCASE_STATUS_BEGIN
  const char *showcaseReason = "";
  cJSON_AddBoolToObject(root, "showcaseConfigured", showcaseSetupReady(showcaseReason));
  cJSON_AddStringToObject(root, "showcaseReason", showcaseReason);
  cJSON_AddNumberToObject(root, "showcaseStage", showcaseRuntime.stage);
  // SHOWCASE_STATUS_END
  cJSON_AddNumberToObject(root, "linear", webLinearCommand);
  cJSON_AddNumberToObject(root, "turn", webTurnCommand);
  cJSON_AddNumberToObject(root, "drivePowerPercent", webDrivePowerPercent);
  cJSON_AddNumberToObject(root, "leftPwm", driveLeftPwm);
  cJSON_AddNumberToObject(root, "rightPwm", driveRightPwm);
  cJSON_AddStringToObject(root, "ssid", WEB_AP_SSID);
  cJSON_AddStringToObject(root, "ip", WEB_AP_IP.toString().c_str());
  cJSON_AddNumberToObject(root, "stations", WiFi.softAPgetStationNum());

  cJSON *stepper = cJSON_AddObjectToObject(root, "stepper");
  if (stepper) {
    cJSON_AddStringToObject(stepper, "state", foldStateName());
    cJSON_AddNumberToObject(stepper, "microsteps", 4);
    cJSON_AddNumberToObject(stepper, "maxCalibrationSteps", 16000);
    cJSON_AddStringToObject(stepper, "lastExecutionMode",
                            foldActionStepperOnly ? "stepper_only" : "synchronized");
    cJSON_AddNumberToObject(stepper, "position", foldStepper.currentSteps());
    cJSON_AddNumberToObject(stepper, "target", foldStepper.targetSteps());
    cJSON_AddNumberToObject(stepper, "travel", foldStepper.travelSteps());
    cJSON_AddBoolToObject(stepper, "travelValid", foldStepper.hasTravel());
    cJSON_AddBoolToObject(stepper, "homed", foldStepper.isHomed());
    cJSON_AddBoolToObject(stepper, "active", foldStepper.isActive());
    cJSON_AddBoolToObject(stepper, "driverEnabled", foldStepper.driverEnabled());
    cJSON_AddBoolToObject(stepper, "enableControl", foldStepper.hasEnableControl());
    cJSON_AddBoolToObject(stepper, "directionInverted", foldStepper.directionInverted());
    cJSON_AddBoolToObject(stepper, "pulseEngineReady", foldStepper.pulseEngineReady());
    cJSON_AddBoolToObject(stepper, "nvsReady", foldPreferencesReady);
    cJSON_AddBoolToObject(stepper, "id11Online", foldId11Online);
    cJSON_AddStringToObject(stepper, "error",
                            foldError.length() ? foldError.c_str() : foldStepper.lastError());
  }

  bool armDegraded = false;
  cJSON *joints = cJSON_AddArrayToObject(root, "joints");
  if (joints) {
    for (size_t i = 0; i < SERVO_COUNT; ++i) {
      const bool externalJoint = externalArm && SERVOS[i].protocol == ServoProtocol::Hx;
      const bool online = !externalJoint && webServoOnline[i] && webServoLastConfirmedMs[i] != 0 &&
                          now - webServoLastConfirmedMs[i] <= WEB_JOINT_ONLINE_WINDOW_MS;
      if (SERVOS[i].protocol == ServoProtocol::Hx && !online) {
        armDegraded = true;
      }
      cJSON *joint = cJSON_CreateObject();
      if (!joint) continue;
      cJSON_AddNumberToObject(joint, "id", SERVOS[i].id);
      cJSON_AddStringToObject(joint, "name", SERVOS[i].name);
      cJSON_AddStringToObject(joint, "protocol", SERVOS[i].model);
      cJSON_AddStringToObject(joint, "model", SERVOS[i].model);
      if (lastServoPositionValid[i]) {
        cJSON_AddNumberToObject(joint, "position", lastServoPosition[i]);
      } else {
        cJSON_AddNullToObject(joint, "position");
      }
      const bool isArmJoint = i < ARM_SERVO_COUNT;
      const bool stale = externalJoint || !lastServoPositionValid[i] ||
                         now - lastServoPositionMs[i] >= ARM_POSITION_STALE_MS;
      cJSON_AddBoolToObject(joint, "online", online);
      cJSON_AddBoolToObject(joint, "calibrated", jointCal[i].calibrated);
      cJSON_AddBoolToObject(joint, "available",
                            online && (isArmJoint || jointCal[i].calibrated));
      cJSON_AddBoolToObject(joint, "position_valid", lastServoPositionValid[i]);
      cJSON_AddNumberToObject(joint, "position_seq",
                              isArmJoint ? webArmPositionSequence[i] : 0);
      cJSON_AddBoolToObject(joint, "stale", stale);
      cJSON_AddNumberToObject(joint, "direction",
                              isArmJoint ? webArmDirections[i] : 0);
      cJSON_AddBoolToObject(joint, "requires_neutral", false);
      cJSON_AddStringToObject(joint, "state",
          externalJoint ? "external" : (!online ? "fault" : (isArmJoint && webArmDirections[i] != 0 ? "moving" :
          (stale ? "waiting_position" : "ready"))));
      cJSON_AddStringToObject(joint, "fault", externalJoint ? "external_arm" : (online ? "none" : "offline"));
      cJSON_AddNumberToObject(joint, "min",
          jointCal[i].calibrated ? jointCal[i].minimum : servoHardMinimum(SERVOS[i]));
      cJSON_AddNumberToObject(joint, "center", jointCal[i].calibrated ? jointCal[i].center : 0);
      cJSON_AddNumberToObject(joint, "max",
                              jointCal[i].calibrated ? jointCal[i].maximum : servoHardMaximum(SERVOS[i]));
      cJSON_AddItemToArray(joints, joint);
    }
  }
  cJSON_AddBoolToObject(root, "armDegraded", armDegraded);

  char *payload = cJSON_PrintUnformatted(root);
  if (payload) {
    webSendText(fd, payload);
    cJSON_free(payload);
  }
  cJSON_Delete(root);
}

void webSendStatusToActiveClient() {
  const int fd = webActiveClientFd;
  if (fd >= 0) webSendStatus(fd);
}

void webCloseCallback(httpd_handle_t, int sockfd) {
  if (sockfd == webActiveClientFd) {
    webActiveClientFd = -1;
    webDisconnectStopRequested = true;
  }
  // ESP-IDF does not call close() when a custom close_fn is installed. The
  // descriptor may already be invalid if the network stack closed it, which
  // is harmless: close() simply returns -1 in that case.
  if (sockfd >= 0) close(sockfd);
}

void webWiFiEvent(arduino_event_id_t event, arduino_event_info_t info) {
  // After an AP reboot, iPadOS can retain the previous WPA/CCMP session and
  // send packets that the fresh ESP32 session rejects as replayed. Kick the
  // first association once per boot so the station performs a clean handshake.
  if (event == ARDUINO_EVENT_WIFI_AP_STACONNECTED && !webFreshAssociationForced) {
    webFreshAssociationAid = info.wifi_ap_staconnected.aid;
    webFreshAssociationForced = true;
    webFreshAssociationPending = true;
  }
  if (event == ARDUINO_EVENT_WIFI_AP_STADISCONNECTED) {
    webStationDisconnectEvent = true;
  }
}

void webResetClientSessions(const char *reason) {
  if (reason) {
    Serial.print("WEB session reset: ");
    Serial.println(reason);
  }

  const int activeFd = webActiveClientFd;
  webActiveClientFd = -1;
  webClientConnected = false;
  webResetArmControlSession();
  webDisconnectStopRequested = true;
  if (!webHttpServer) return;

  int clientFds[WEB_HTTP_MAX_OPEN_SOCKETS] = {};
  size_t clientCount = WEB_HTTP_MAX_OPEN_SOCKETS;
  const esp_err_t listResult = httpd_get_client_list(webHttpServer, &clientCount, clientFds);
  if (listResult == ESP_OK) {
    Serial.print("WEB closing HTTP sessions: ");
    Serial.println(clientCount);
    for (size_t i = 0; i < clientCount; ++i) {
      const esp_err_t closeResult = httpd_sess_trigger_close(webHttpServer, clientFds[i]);
      if (closeResult != ESP_OK && closeResult != ESP_ERR_NOT_FOUND) {
        Serial.print("WEB session close failed fd=");
        Serial.print(clientFds[i]);
        Serial.print(" result=");
        Serial.println(esp_err_to_name(closeResult));
      }
    }
    webLastHttpClientCount = UINT8_MAX;
    return;
  }

  Serial.print("WEB client list failed: ");
  Serial.println(esp_err_to_name(listResult));
  if (activeFd >= 0) {
    httpd_sess_trigger_close(webHttpServer, activeFd);
  }
}

void webAuditHttpSessions() {
  if (!webHttpServer) return;
  const uint32_t now = millis();
  if (now - lastWebHttpSessionAuditMs < WEB_HTTP_SESSION_AUDIT_MS) return;
  lastWebHttpSessionAuditMs = now;

  int clientFds[WEB_HTTP_MAX_OPEN_SOCKETS] = {};
  size_t clientCount = WEB_HTTP_MAX_OPEN_SOCKETS;
  if (httpd_get_client_list(webHttpServer, &clientCount, clientFds) != ESP_OK) return;
  if (clientCount == webLastHttpClientCount) return;

  size_t websocketCount = 0;
  for (size_t i = 0; i < clientCount; ++i) {
    if (httpd_ws_get_fd_info(webHttpServer, clientFds[i]) == HTTPD_WS_CLIENT_WEBSOCKET) {
      ++websocketCount;
    }
  }
  Serial.print("WEB HTTP sessions total=");
  Serial.print(clientCount);
  Serial.print(" websocket=");
  Serial.println(websocketCount);
  webLastHttpClientCount = static_cast<uint8_t>(clientCount);
}

bool webIsControlPageRequest(const char *uri) {
  if (!uri) return false;
  return strcmp(uri, "/") == 0 || strncmp(uri, "/?", 2) == 0 ||
         strcmp(uri, "/index.html") == 0 || strncmp(uri, "/index.html?", 12) == 0;
}

esp_err_t webPageHandler(httpd_req_t *req) {
  httpd_resp_set_hdr(req, "Connection", "close");
  if (!webIsControlPageRequest(req->uri)) {
    httpd_resp_set_status(req, "204 No Content");
    httpd_resp_set_type(req, "text/plain; charset=utf-8");
    httpd_resp_set_hdr(req, "Cache-Control", "no-store, max-age=0");
    return httpd_resp_send(req, "", 0);
  }
  httpd_resp_set_type(req, "text/html; charset=utf-8");
  httpd_resp_set_hdr(req, "Content-Encoding", "gzip");
  httpd_resp_set_hdr(req, "Cache-Control", "no-store, max-age=0");
  httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
  return httpd_resp_send(req, reinterpret_cast<const char *>(WEB_UI_GZ), WEB_UI_GZ_LEN);
}

bool webReadInteger(const cJSON *root, const char *key, int &value) {
  const cJSON *item = cJSON_GetObjectItemCaseSensitive(root, key);
  if (!cJSON_IsNumber(item) || !isfinite(item->valuedouble) ||
      floor(item->valuedouble) != item->valuedouble ||
      item->valuedouble < INT_MIN || item->valuedouble > INT_MAX) {
    return false;
  }
  value = static_cast<int>(item->valuedouble);
  return true;
}

bool webReadArmDirections(const cJSON *root,
                          int8_t directions[ARM_SERVO_COUNT],
                          bool requireSingleAxis) {
  const cJSON *items = cJSON_GetObjectItemCaseSensitive(root, "dir");
  if (!cJSON_IsArray(items) ||
      cJSON_GetArraySize(items) != static_cast<int>(ARM_SERVO_COUNT)) {
    return false;
  }
  uint8_t nonZeroCount = 0;
  for (size_t i = 0; i < ARM_SERVO_COUNT; ++i) {
    const cJSON *item = cJSON_GetArrayItem(items, static_cast<int>(i));
    if (!cJSON_IsNumber(item) || !isfinite(item->valuedouble) ||
        floor(item->valuedouble) != item->valuedouble ||
        item->valuedouble < -1 || item->valuedouble > 1) {
      return false;
    }
    const int value = static_cast<int>(item->valuedouble);
    if (value < -1 || value > 1) return false;
    directions[i] = static_cast<int8_t>(value);
    if (value != 0) ++nonZeroCount;
  }
  return !requireSingleAxis || nonZeroCount == 1;
}

esp_err_t webSendRequestError(httpd_req_t *req, const char *reason) {
  String json = "{\"type\":\"error\",\"ok\":false,\"reason\":\"";
  json += reason;
  json += "\"}";
  httpd_ws_frame_t frame = {};
  frame.type = HTTPD_WS_TYPE_TEXT;
  frame.payload = reinterpret_cast<uint8_t *>(const_cast<char *>(json.c_str()));
  frame.len = json.length();
  return httpd_ws_send_frame(req, &frame);
}

bool webQueueCommand(const WebControlCommand &command) {
  return webCommandQueue && xQueueSend(webCommandQueue, &command, 0) == pdTRUE;
}

// The HTTP task claims ownership; the loop expires it under the same lock.
// Rejected clients never reset arm state, generation, or the owner's stop flag.
bool webTryClaimOwner(int fd, uint32_t now) {
  portENTER_CRITICAL(&webLeaseMux);
  const bool accepted = webActiveClientFd < 0 || webActiveClientFd == fd;
  if (accepted && webActiveClientFd != fd) {
    webActiveClientFd = fd;
    webOwnerReceiptMs = now;
    ++webControlSession;
    webDisconnectStopRequested = true;
  }
  portEXIT_CRITICAL(&webLeaseMux);
  return accepted;
}

void webNoteOwnerReceipt(int fd, uint32_t receivedAtMs) {
  portENTER_CRITICAL(&webLeaseMux);
  if (fd == webActiveClientFd &&
      static_cast<int32_t>(receivedAtMs - webOwnerReceiptMs) >= 0) {
    webOwnerReceiptMs = receivedAtMs;
  }
  portEXIT_CRITICAL(&webLeaseMux);
}

int webExpireOwner(uint32_t now) {
  portENTER_CRITICAL(&webLeaseMux);
  int expired = -1;
  // Signed check prevents a concurrently newer receipt appearing expired.
  if (webActiveClientFd >= 0 && static_cast<int32_t>(now - webOwnerReceiptMs) >
      static_cast<int32_t>(WEB_OWNER_TIMEOUT_MS)) {
    expired = webActiveClientFd;
    webActiveClientFd = -1;
    webDisconnectStopRequested = true;
  }
  portEXIT_CRITICAL(&webLeaseMux);
  return expired;
}

bool webStatusDue(uint32_t now) {
  return webClientConnected && now - lastWebStatusMs >= WEB_STATUS_INTERVAL_MS;
}

bool webEnableExternalArm(bool enabled) {
  if (!enabled || webMotionEnabled || activeControlOwner() != ControlOwner::None ||
      robotAction.active || baseMotion.active || webDriveActive ||
      foldAnyMotionActive() || isWebJogActive() || hxHold.active) return false;
  externalArm = true;
  webResetArmControlSession();
  return true;
}

esp_err_t webSocketHandler(httpd_req_t *req) {
  const int fd = httpd_req_to_sockfd(req);
  if (req->method == HTTP_GET) {
    if (!webTryClaimOwner(fd, millis())) {
      // ESP-IDF may have already sent 101 Switching Protocols. Close this
      // session instead of attempting a second HTTP response after upgrade.
      httpd_sess_trigger_close(req->handle, fd);
      return ESP_OK;
    }
    webResetArmControlSession();
    return ESP_OK;
  }

  httpd_ws_frame_t frame = {};
  esp_err_t result = httpd_ws_recv_frame(req, &frame, 0);
  if (result != ESP_OK) return result;
  if (frame.type == HTTPD_WS_TYPE_CLOSE) {
    if (fd == webActiveClientFd) {
      webActiveClientFd = -1;
      webDisconnectStopRequested = true;
    }
    return ESP_OK;
  }
  if (frame.type != HTTPD_WS_TYPE_TEXT) return ESP_OK;
  if (frame.len == 0 || frame.len > WEB_MAX_FRAME_BYTES) {
    return webSendRequestError(req, "frame_too_large");
  }

  uint8_t payload[WEB_MAX_FRAME_BYTES + 1] = {0};
  frame.payload = payload;
  result = httpd_ws_recv_frame(req, &frame, WEB_MAX_FRAME_BYTES);
  if (result != ESP_OK) return result;
  payload[frame.len] = 0;

  cJSON *root = cJSON_Parse(reinterpret_cast<const char *>(payload));
  if (!root || !cJSON_IsObject(root)) {
    cJSON_Delete(root);
    return webSendRequestError(req, "invalid_json");
  }
  const cJSON *typeItem = cJSON_GetObjectItemCaseSensitive(root, "type");
  if (!cJSON_IsString(typeItem) || !typeItem->valuestring) {
    cJSON_Delete(root);
    return webSendRequestError(req, "missing_type");
  }
  if (fd != webActiveClientFd) {
    cJSON_Delete(root);
    return webSendRequestError(req, "controller_replaced");
  }

  WebControlCommand command;
  command.clientFd = fd;
  command.driveGeneration = __atomic_load_n(&webDriveGeneration, __ATOMIC_RELAXED);
  command.controlGeneration = webGetControlGeneration();
  command.receivedAtMs = millis();
  const String type = typeItem->valuestring;
  bool valid = true;
  if (type == "hello") {
    command.type = WebCommandType::Hello;
  } else if (type == "heartbeat") {
    command.type = WebCommandType::Heartbeat;
  } else if (type == "status_request") {
    command.type = WebCommandType::StatusRequest;
  } else if (type == "arm_external") {
    const cJSON *enabled = cJSON_GetObjectItemCaseSensitive(root, "enabled");
    valid = cJSON_IsBool(enabled);
    command.type = WebCommandType::ArmExternal;
    command.enabled = cJSON_IsTrue(enabled);
  } else if (type == "motion_enable") {
    const cJSON *enabled = cJSON_GetObjectItemCaseSensitive(root, "enabled");
    valid = cJSON_IsBool(enabled);
    command.type = WebCommandType::MotionEnable;
    command.enabled = cJSON_IsTrue(enabled);
  } else if (type == "drive_speed") {
    valid = webReadInteger(root, "percent", command.valueA) &&
            (command.valueA == WEB_DRIVE_POWER_LOW_PERCENT ||
             command.valueA == WEB_DRIVE_POWER_MEDIUM_PERCENT ||
             command.valueA == WEB_DRIVE_POWER_HIGH_PERCENT);
    command.type = WebCommandType::DriveSpeed;
  } else if (type == "drive") {
    valid = webReadInteger(root, "linear", command.valueA) &&
            webReadInteger(root, "turn", command.valueB);
    command.type = WebCommandType::Drive;
    command.valueA = constrain(command.valueA, -100, 100);
    command.valueB = constrain(command.valueB, -100, 100);
  } else if (type == "drive_stop") {
    command.type = WebCommandType::DriveStop;
    command.driveGeneration = __atomic_add_fetch(&webDriveGeneration, 1U, __ATOMIC_RELAXED);
    webImmediateDriveStopRequested = true;
  } else if (type == "action") {
    const cJSON *name = cJSON_GetObjectItemCaseSensitive(root, "name");
    valid = cJSON_IsString(name) && name->valuestring && strlen(name->valuestring) < sizeof(command.action);
    command.type = WebCommandType::Action;
    if (valid) strlcpy(command.action, name->valuestring, sizeof(command.action));
  } else if (type == "jog_state" || type == "jog_step") {
    int sequence = 0;
    valid = webReadInteger(root, "seq", sequence) && sequence > 0 &&
            webReadArmDirections(root, command.armDirections,
                                 type == "jog_step");
    command.type = type == "jog_step" ? WebCommandType::JogStep
                                      : WebCommandType::JogState;
    command.armSequence = valid ? static_cast<uint32_t>(sequence) : 0;
  } else if (type == "stepper_zero") {
    command.type = WebCommandType::FoldZero;
  } else if (type == "stepper_confirm_unfolded") {
    const cJSON *confirmed = cJSON_GetObjectItemCaseSensitive(root, "confirmed");
    valid = cJSON_IsBool(confirmed) && cJSON_IsTrue(confirmed);
    command.type = WebCommandType::FoldConfirmUnfolded;
    command.enabled = valid;
  } else if (type == "stepper_jog") {
    valid = webReadInteger(root, "steps", command.valueA) &&
            command.valueA != 0 && command.valueA >= -2000 && command.valueA <= 2000;
    command.type = WebCommandType::FoldJog;
  } else if (type == "stepper_save") {
    command.type = WebCommandType::FoldSave;
  } else if (type == "stepper_reset") {
    command.type = WebCommandType::FoldReset;
  } else if (type == "stepper_direction") {
    command.type = WebCommandType::FoldToggleDirection;
  } else if (type == "stepper_rescan") {
    command.type = WebCommandType::FoldRescan;
  } else if (type == "stop" || type == "stop_all") {
    command.type = WebCommandType::Stop;
  } else {
    valid = false;
  }
  cJSON_Delete(root);

  if (!valid) return webSendRequestError(req, "invalid_command");
  webNoteOwnerReceipt(fd, command.receivedAtMs);
  if (command.type == WebCommandType::Heartbeat) {
    // Receipt is proof of liveness; consuming a queued copy is not. This path
    // remains available even when the ordinary command queue is full.
    webNoteLeaseReceipt(command.clientFd, command.controlGeneration, command.receivedAtMs);
    return ESP_OK;
  }
  if (command.type == WebCommandType::Stop ||
      (command.type == WebCommandType::MotionEnable && !command.enabled)) {
    // Barrier at receipt cancels even an in-flight UART preflight. The loop
    // establishes a second barrier when applying the stop, so commands sent
    // before stop application cannot re-enable motion. No queue space needed.
    webInvalidateControlCommands();
    webImmediateStopRequested = true;
    return ESP_OK;
  }
  if (!webQueueCommand(command)) return webSendRequestError(req, "command_queue_full");
  return ESP_OK;
}

void webApplyDrive(int linear, int turn) {
  int left = linear - turn;
  int right = linear + turn;
  const int magnitude = max(abs(left), abs(right));
  if (magnitude > 100) {
    left = left * 100 / magnitude;
    right = right * 100 / magnitude;
  }
  // The selector limits PWM duty; the joystick remains proportional inside
  // that limit.  Round symmetrically so full-stick 30% maps to +/-77.
  const int leftNumerator = left * WEB_DRIVE_FULL_PWM * webDrivePowerPercent;
  const int rightNumerator = right * WEB_DRIVE_FULL_PWM * webDrivePowerPercent;
  const int leftPwm = (leftNumerator >= 0 ? leftNumerator + 5000 : leftNumerator - 5000) / 10000;
  const int rightPwm = (rightNumerator >= 0 ? rightNumerator + 5000 : rightNumerator - 5000) / 10000;
  webLinearCommand = linear;
  webTurnCommand = turn;
  if (leftPwm == 0 && rightPwm == 0) {
    chassisStop();
    webDriveActive = false;
  } else {
    chassisDriveBoth(leftPwm, rightPwm);
    webDriveActive = drivePwmReady && !drivePwmFault;
  }
}

void webLatchDriveTimeout(const char *message) {
  // Stop first, then invalidate every older drive command still waiting in the
  // FreeRTOS queue. Motion stays disabled until the operator enables it again.
  __atomic_add_fetch(&webDriveGeneration, 1U, __ATOMIC_RELAXED);
  webInvalidateControlCommands();
  webMotionEnabled = false;
  chassisStop();
  if (message) Serial.println(message);
  webDriveActive = false;
  webLinearCommand = 0;
  webTurnCommand = 0;
  lastWebStatusMs = 0;
  if (!robotAction.active && !isWebJogActive() && !foldAnyMotionActive()) webMarkMotionIdle();
  webSendAck(webActiveClientFd, "drive", false, "drive_timeout");
}

bool webArmAnyDirectionActive() {
  for (size_t i = 0; i < ARM_SERVO_COUNT; ++i) {
    if (webArmDirections[i] != 0) return true;
  }
  return false;
}

void webArmLimits(size_t index, int32_t &minimum, int32_t &maximum) {
  if (index < ARM_SERVO_COUNT && jointCal[index].calibrated) {
    minimum = jointCal[index].minimum;
    maximum = jointCal[index].maximum;
  } else {
    minimum = HX30_TEMP_MIN_POSITION;
    maximum = HX30_TEMP_MAX_POSITION;
  }
}

bool hxEnsureWebTorque(size_t index) {
  if (index >= ARM_SERVO_COUNT) return false;
  if (webArmTorqueReady[index]) return true;
  const uint8_t torque = 1;
  if (!hxWrite(SERVOS[index].id, 0x28, &torque, 1)) return false;
  webArmTorqueReady[index] = true;
  return true;
}

bool hxWriteWebTarget(size_t index, int32_t position) {
  if (index >= ARM_SERVO_COUNT || position < INT16_MIN || position > INT16_MAX) {
    return false;
  }
  const uint16_t raw = hxEncodeSignedPosition(position);
  const uint8_t target[] = {
      static_cast<uint8_t>(raw & 0xFF), static_cast<uint8_t>(raw >> 8),
      0, 0,
      static_cast<uint8_t>(HX_DEMO_SPEED & 0xFF),
      static_cast<uint8_t>(HX_DEMO_SPEED >> 8)};
  return hxWrite(SERVOS[index].id, 0x2A, target, sizeof(target));
}

void webArmQueueHold(size_t index) {
  if (index >= ARM_SERVO_COUNT) return;
  webArmDirections[index] = 0;
  webArmHoldPendingMask |= static_cast<uint8_t>(1U << index);
}

void webArmQueueAllHolds() {
  for (size_t i = 0; i < ARM_SERVO_COUNT; ++i) webArmQueueHold(i);
  webArmWatchdogArmed = false;
}

bool webApplyJogState(uint32_t sequence,
                      const int8_t directions[ARM_SERVO_COUNT],
                      String &reason) {
  if (sequence == 0 || sequence <= webArmLastJogSequence) {
    reason = "invalid_sequence";
    return false;
  }
  webArmLastJogSequence = sequence;
  bool anyNonZero = false;
  for (size_t i = 0; i < ARM_SERVO_COUNT; ++i) {
    if (webArmDirections[i] != 0 && directions[i] == 0) webArmQueueHold(i);
    webArmDirections[i] = directions[i];
    anyNonZero = anyNonZero || directions[i] != 0;
  }
  webArmAppliedJogSequence = sequence;
  if (anyNonZero) {
    webArmLastNonZeroCommandMs = millis();
    webArmWatchdogArmed = true;
    webMarkMotionActive();
  } else {
    webArmWatchdogArmed = false;
  }
  reason = anyNonZero ? "accepted" : "holding";
  return true;
}

bool webApplyJogStep(uint32_t sequence,
                     const int8_t directions[ARM_SERVO_COUNT],
                     String &reason) {
  const uint32_t controlGeneration = webGetControlGeneration();
  if (sequence == 0 || sequence <= webArmLastJogSequence) {
    reason = "invalid_sequence";
    webArmJogStepStatus = "invalid_sequence";
    return false;
  }
  size_t selected = ARM_SERVO_COUNT;
  for (size_t i = 0; i < ARM_SERVO_COUNT; ++i) {
    if (directions[i] == 0) continue;
    if (selected != ARM_SERVO_COUNT) {
      reason = "single_axis_required";
      webArmJogStepStatus = "invalid_direction";
      return false;
    }
    selected = i;
  }
  if (selected == ARM_SERVO_COUNT) {
    reason = "single_axis_required";
    webArmJogStepStatus = "invalid_direction";
    return false;
  }
  webArmLastJogSequence = sequence;
  webArmJogStepSequence = sequence;

  int32_t current = 0;
  if (!hxReadPosition(SERVOS[selected].id, current, 35)) {
    reason = "stale_position";
    webArmJogStepStatus = "stale_position";
    return false;
  }
  int32_t minimum = 0;
  int32_t maximum = 0;
  webArmLimits(selected, minimum, maximum);
  if (minimum >= maximum) {
    reason = "invalid_limits";
    webArmJogStepStatus = "invalid_limits";
    return false;
  }
  if (current < minimum || current > maximum) {
    reason = "position_outside_limits";
    webArmJogStepStatus = "position_outside_limits";
    return false;
  }
  int direction = directions[selected];
  if (jointCal[selected].calibrated && jointCal[selected].inverted) direction = -direction;
  const int32_t target = constrain(current + direction * HX30_JOG_STEP,
                                   minimum, maximum);
  if (target == current) {
    reason = "at_limit";
    webArmJogStepStatus = "at_limit";
    return false;
  }
  if (!webMotionStartAllowed(controlGeneration)) {
    reason = "stop_requested";
    webArmJogStepStatus = "stop_requested";
    return false;
  }
  if (!hxEnsureWebTorque(selected) ||
      !webMotionStartAllowed(controlGeneration) || !hxWriteWebTarget(selected, target)) {
    reason = "bus_rejected";
    webArmJogStepStatus = "bus_rejected";
    webArmTorqueReady[selected] = false;
    webServoOnline[selected] = false;
    return false;
  }
  webArmShadowTargets[selected] = target;
  webArmShadowValid[selected] = true;
  webConfirmJointOnline(static_cast<int>(selected));
  webArmJogStepStatus = "applied";
  reason = "applied";
  return true;
}

void serviceWebArmMotion() {
  if (externalArm) return;
  const uint32_t controlGeneration = webGetControlGeneration();
  const uint32_t now = millis();
  if (webArmWatchdogArmed &&
      now - webArmLastNonZeroCommandMs >= ARM_MOTION_WATCHDOG_MS) {
    Serial.println("WEB arm watchdog timeout: holding six HX-30HM joints");
    webArmQueueAllHolds();
    webArmJogStepStatus = "watchdog_hold";
  }

  if (webArmHoldPendingMask != 0) {
    for (size_t offset = 0; offset < ARM_SERVO_COUNT; ++offset) {
      const size_t index = (webArmMotionCursor + offset) % ARM_SERVO_COUNT;
      const uint8_t bit = static_cast<uint8_t>(1U << index);
      if ((webArmHoldPendingMask & bit) == 0) continue;
      int32_t holdPosition = 0;
      const bool havePosition = hxReadPosition(SERVOS[index].id, holdPosition, 25) ||
          (lastServoPositionValid[index] &&
           now - lastServoPositionMs[index] < ARM_POSITION_STALE_MS &&
           (holdPosition = lastServoPosition[index], true));
      if (havePosition && hxHoldHx30AtPosition(SERVOS[index].id, holdPosition)) {
        webConfirmJointOnline(static_cast<int>(index));
        webArmShadowTargets[index] = holdPosition;
        webArmShadowValid[index] = true;
      } else {
        webArmTorqueReady[index] = false;
      }
      webArmHoldPendingMask &= static_cast<uint8_t>(~bit);
      webArmMotionCursor = static_cast<uint8_t>((index + 1) % ARM_SERVO_COUNT);
      return;
    }
  }

  if (!webMotionStartAllowed(controlGeneration)) return;
  if (!webArmAnyDirectionActive()) {
    if (webOwnsMotion && !webDriveActive && !robotAction.active &&
        !foldAnyMotionActive()) webMarkMotionIdle();
    return;
  }
  if (now - webArmLastMotionTickMs < ARM_CONTROL_PERIOD_MS) return;
  webArmLastMotionTickMs = now;

  for (size_t offset = 0; offset < ARM_SERVO_COUNT; ++offset) {
    const size_t index = (webArmMotionCursor + offset) % ARM_SERVO_COUNT;
    if (webArmDirections[index] == 0) continue;
    if (webArmShadowValid[index] &&
        (!lastServoPositionValid[index] || now - lastServoPositionMs[index] >= 100)) {
      int32_t measured = 0;
      if (!hxReadPosition(SERVOS[index].id, measured, 25)) {
        webArmQueueHold(index);
        webServoOnline[index] = false;
        webArmJogStepStatus = "stale_position";
        return;
      }
    }
    if (!webArmShadowValid[index]) {
      int32_t current = 0;
      if (!hxReadPosition(SERVOS[index].id, current, 35)) {
        webArmQueueHold(index);
        webServoOnline[index] = false;
        webArmJogStepStatus = "stale_position";
        return;
      }
      webArmShadowTargets[index] = current;
      webArmShadowValid[index] = true;
      webConfirmJointOnline(static_cast<int>(index));
    }
    int32_t minimum = 0;
    int32_t maximum = 0;
    webArmLimits(index, minimum, maximum);
    int direction = webArmDirections[index];
    if (jointCal[index].calibrated && jointCal[index].inverted) direction = -direction;
    const int32_t target = constrain(
        webArmShadowTargets[index] + direction * HX30_JOG_STEP,
        minimum, maximum);
    if (target == webArmShadowTargets[index]) {
      webArmQueueHold(index);
      webArmJogStepStatus = "at_limit";
      return;
    }
    if (!webMotionStartAllowed(controlGeneration)) return;
    if (!hxEnsureWebTorque(index) ||
        !webMotionStartAllowed(controlGeneration) || !hxWriteWebTarget(index, target)) {
      webArmQueueHold(index);
      webArmTorqueReady[index] = false;
      webServoOnline[index] = false;
      webArmJogStepStatus = "bus_rejected";
      return;
    }
    webArmShadowTargets[index] = target;
    webConfirmJointOnline(static_cast<int>(index));
    webArmMotionCursor = static_cast<uint8_t>((index + 1) % ARM_SERVO_COUNT);
    return;
  }
}

void webProcessCommand(const WebControlCommand &command) {
  if (command.clientFd != webActiveClientFd) return;
  const bool safetyCommand = command.type == WebCommandType::Stop ||
      command.type == WebCommandType::DriveStop ||
      (command.type == WebCommandType::MotionEnable && !command.enabled);
  const bool readOnlyCommand = command.type == WebCommandType::Hello ||
      command.type == WebCommandType::Heartbeat || command.type == WebCommandType::StatusRequest;
  if (foldStepper.levelTestMode() && !safetyCommand && !readOnlyCommand) {
    webSendAck(command.clientFd, "command", false, "step_level_test_mode_reboot_required");
    return;
  }
  if (!safetyCommand && !readOnlyCommand) {
    if (command.controlGeneration != webGetControlGeneration() || motionStopPending()) return;
    if (millis() - command.receivedAtMs > WEB_CONTROL_LEASE_MS) {
      webSendAck(command.clientFd, "command", false, "stale_command");
      return;
    }
  }
  switch (command.type) {
    case WebCommandType::Hello:
      webSendAck(command.clientFd, "hello", true, "sophicar_ready");
      webSendStatus(command.clientFd);
      break;
    case WebCommandType::Heartbeat:
      webNoteLeaseReceipt(command.clientFd, command.controlGeneration, command.receivedAtMs);
      break;
    case WebCommandType::StatusRequest:
      // Cached software snapshot only; servo polling stays suspended in motion.
      webSendStatus(command.clientFd);
      break;
    case WebCommandType::ArmExternal: {
      const bool ok = webEnableExternalArm(command.enabled);
      webSendAck(command.clientFd, "arm_external", ok,
                 ok ? "external_arm_enabled" : "requires_enabled_true_disabled_idle");
      if (ok) webSendStatus(command.clientFd);
      break;
    }
    case WebCommandType::MotionEnable:
      if (command.enabled && motionSafetyLocked()) {
        webMotionEnabled = false;
        webSendAck(command.clientFd, "motion_enable", false, "motion_locked");
      } else if (command.enabled && activeControlOwner() != ControlOwner::None &&
                 activeControlOwner() != ControlOwner::Web) {
        webSendAck(command.clientFd, "motion_enable", false, "busy");
      } else {
        if (!command.enabled) {
          stopForWebControl();
        } else if (!webMotionEnabled) {
          chassisStop();
          webDriveActive = false;
          webLinearCommand = 0;
          webTurnCommand = 0;
          webMotionEnabled = drivePwmReady && !drivePwmFault;
          jsonProtocolActive = false;
        }
        // Repeated enable is idempotent: never orphan active PWM/watchdogs.
        webNoteLeaseReceipt(command.clientFd, command.controlGeneration, command.receivedAtMs);
        webSendAck(command.clientFd, "motion_enable", !command.enabled || webMotionEnabled,
                   !command.enabled ? "disabled" : (webMotionEnabled ? "enabled" : "drive_fault"));
      }
      break;
    case WebCommandType::DriveSpeed:
      webDrivePowerPercent = static_cast<uint8_t>(command.valueA);
      if (webDriveActive) {
        webApplyDrive(webLinearCommand, webTurnCommand);
      }
      webSendAck(command.clientFd, "drive_speed", true,
                 command.valueA == WEB_DRIVE_POWER_LOW_PERCENT ? "low_30" :
                 command.valueA == WEB_DRIVE_POWER_MEDIUM_PERCENT ? "medium_60" : "full_100");
      break;
    case WebCommandType::Drive:
      if (command.driveGeneration != __atomic_load_n(&webDriveGeneration, __ATOMIC_RELAXED)) {
        break;
      } else if (millis() - command.receivedAtMs > WEB_DRIVE_COMMAND_TIMEOUT_MS) {
        webLatchDriveTimeout("WEB stale queued drive command: latching motion disabled");
      } else if (motionSafetyLocked()) {
        webMotionEnabled = false;
        webSendAck(command.clientFd, "drive", false, "motion_locked");
      } else if (!webMotionEnabled) {
        webSendAck(command.clientFd, "drive", false, "motion_disabled");
      } else if (robotAction.active || baseMotion.active || isWebJogActive() || foldAnyMotionActive()) {
        webSendAck(command.clientFd, "drive", false, "busy");
      } else {
        webApplyDrive(command.valueA, command.valueB);
        webNoteLeaseReceipt(command.clientFd, command.controlGeneration, command.receivedAtMs);
        lastWebDriveCommandMs = command.receivedAtMs;
        if (webDriveActive) webMarkMotionActive();
        else webMarkMotionIdle();
      }
      break;
    case WebCommandType::DriveStop:
      chassisStop();
      webDriveActive = false;
      webLinearCommand = 0;
      webTurnCommand = 0;
      if (!robotAction.active && !isWebJogActive() && !foldAnyMotionActive()) webMarkMotionIdle();
      webSendAck(command.clientFd, "drive_stop", true, "stopped");
      break;
    case WebCommandType::Action: {
      if (motionSafetyLocked()) {
        webMotionEnabled = false;
        webSendAck(command.clientFd, "action", false, "motion_locked");
        break;
      }
      if (!webMotionEnabled) {
        webSendAck(command.clientFd, "action", false, "motion_disabled");
        break;
      }
      if (webDriveActive) {
        chassisStop();
        webDriveActive = false;
        webMarkMotionIdle();
      }
      String reason;
      webNoteLeaseReceipt(command.clientFd, command.controlGeneration, command.receivedAtMs);
      if (startNamedAction(String(command.action), 0, ControlOwner::Web, reason)) {
        webSendAck(command.clientFd, command.action, true, "started");
      } else {
        webSendAck(command.clientFd, command.action, false, reason.c_str());
      }
      break;
    }
    case WebCommandType::JogState: {
      if (externalArm) { webSendAck(command.clientFd, "jog_state", false, "external_arm"); break; }
      bool nonZero = false;
      for (size_t i = 0; i < ARM_SERVO_COUNT; ++i) nonZero |= command.armDirections[i] != 0;
      if (nonZero && millis() - command.receivedAtMs >= ARM_MOTION_WATCHDOG_MS) {
        webArmQueueAllHolds();
        webSendAck(command.clientFd, "jog_state", false, "stale_command");
        break;
      }
      if (motionSafetyLocked()) {
        webMotionEnabled = false;
        webSendAck(command.clientFd, "jog_state", false, "motion_locked");
        break;
      }
      if (!webMotionEnabled) {
        webSendAck(command.clientFd, "jog_state", false, "motion_disabled");
        break;
      }
      if (robotAction.active || baseMotion.active || webDriveActive || foldAnyMotionActive()) {
        webSendAck(command.clientFd, "jog_state", false, "busy");
        break;
      }
      String reason;
      if (webApplyJogState(command.armSequence, command.armDirections, reason)) {
        if (nonZero) webArmLastNonZeroCommandMs = command.receivedAtMs;
        webNoteLeaseReceipt(command.clientFd, command.controlGeneration, command.receivedAtMs);
        webSendAck(command.clientFd, "jog_state", true, reason.c_str());
      } else {
        webSendAck(command.clientFd, "jog_state", false, reason.c_str());
      }
      break;
    }
    case WebCommandType::JogStep: {
      if (externalArm) { webSendAck(command.clientFd, "jog_step", false, "external_arm"); break; }
      if (millis() - command.receivedAtMs >= ARM_MOTION_WATCHDOG_MS) {
        webSendAck(command.clientFd, "jog_step", false, "stale_command");
        break;
      }
      if (motionSafetyLocked()) {
        webMotionEnabled = false;
        webSendAck(command.clientFd, "jog_step", false, "motion_locked");
        break;
      }
      if (!webMotionEnabled) {
        webSendAck(command.clientFd, "jog_step", false, "motion_disabled");
        break;
      }
      if (robotAction.active || baseMotion.active || webDriveActive || isWebJogActive() || foldAnyMotionActive()) {
        webSendAck(command.clientFd, "jog_step", false, "busy");
        break;
      }
      String reason;
      if (webApplyJogStep(command.armSequence, command.armDirections, reason)) {
        webNoteLeaseReceipt(command.clientFd, command.controlGeneration, command.receivedAtMs);
        webSendAck(command.clientFd, "jog_step", true, reason.c_str());
      } else {
        webSendAck(command.clientFd, "jog_step", false, reason.c_str());
      }
      break;
    }
    case WebCommandType::FoldZero: {
      String reason;
      bool ok = false;
      if (motionSafetyLocked()) reason = "motion_locked";
      else if (!webMotionEnabled) reason = "motion_disabled";
      else ok = foldConfirmZero(reason);
      webSendAck(command.clientFd, "stepper_zero", ok, reason.c_str());
      break;
    }
    case WebCommandType::FoldConfirmUnfolded: {
      String reason;
      bool ok = false;
      if (!command.enabled) reason = "endpoint_confirmation_required";
      else if (motionSafetyLocked()) reason = "motion_locked";
      else if (!webMotionEnabled) reason = "motion_disabled";
      else ok = foldConfirmUnfolded(reason);
      webSendAck(command.clientFd, "stepper_confirm_unfolded", ok, reason.c_str());
      break;
    }
    case WebCommandType::FoldJog: {
      String reason;
      bool ok = false;
      if (motionSafetyLocked()) reason = "motion_locked";
      else if (!webMotionEnabled) reason = "motion_disabled";
      else ok = foldJog(command.valueA, reason);
      if (ok) {
        webMarkMotionActive();
        webNoteLeaseReceipt(command.clientFd, command.controlGeneration, command.receivedAtMs);
      }
      webSendAck(command.clientFd, "stepper_jog", ok, reason.c_str());
      break;
    }
    case WebCommandType::FoldSave: {
      String reason;
      const bool ok = foldSaveTravel(reason);
      webSendAck(command.clientFd, "stepper_save", ok, reason.c_str());
      break;
    }
    case WebCommandType::FoldReset: {
      String reason;
      const bool ok = foldResetTravel(reason);
      webSendAck(command.clientFd, "stepper_reset", ok, reason.c_str());
      break;
    }
    case WebCommandType::FoldToggleDirection: {
      String reason;
      const bool ok = foldToggleDirection(reason);
      webSendAck(command.clientFd, "stepper_direction", ok, reason.c_str());
      break;
    }
    case WebCommandType::FoldRescan: {
      String reason;
      const bool ok = foldRescan(reason);
      webSendAck(command.clientFd, "stepper_rescan", ok, reason.c_str());
      break;
    }
    case WebCommandType::Stop:
      stopForWebControl();
      webSendAck(command.clientFd, "stop", true, "all_stopped");
      break;
  }
  lastWebStatusMs = 0;
}

void webPollOneJoint() {
  if (!WEB_JOINT_POLL_ENABLED) return;
  if (!webClientConnected || robotAction.active || baseMotion.active || webDriveActive || hxHold.active ||
      foldAnyMotionActive() || isWebJogActive()) return;
  const uint32_t now = millis();
  if (now - lastWebJointPollMs < WEB_JOINT_POLL_INTERVAL_MS) return;
  lastWebJointPollMs = now;

  const uint8_t servoId = WEB_JOINT_POLL_IDS[
      webJointPollCursor++ % (sizeof(WEB_JOINT_POLL_IDS) / sizeof(WEB_JOINT_POLL_IDS[0]))];
  const int index = servoIndex(servoId);
  if (index < 0) return;
  int32_t position = 0;
  bool ok = false;
  if (SERVOS[index].protocol == ServoProtocol::Hx) {
    if (externalArm) return;
    ok = hxReadPosition(SERVOS[index].id, position, 25);
  } else {
    int htdPosition = 0;
    ok = htdReadPositionById(SERVOS[index].id, htdPosition, 40, false);
    position = htdPosition;
    if (ok) {
      lastServoPosition[index] = position;
      lastServoPositionMs[index] = millis();
      lastServoPositionValid[index] = true;
    }
  }
  if (ok) {
    webConfirmJointOnline(index);
  } else {
    // Shared-bus feedback can miss one response; only mark the joint offline
    // after its last confirmed response is outside the online window.
    const bool recentlyConfirmed = webServoLastConfirmedMs[index] != 0 &&
                                   now - webServoLastConfirmedMs[index] <=
                                       WEB_JOINT_ONLINE_WINDOW_MS;
    if (!recentlyConfirmed) webServoOnline[index] = false;
  }
}

void beginWebControl() {
  webCommandQueue = xQueueCreate(WEB_COMMAND_QUEUE_DEPTH, sizeof(WebControlCommand));
  if (!webCommandQueue) {
    Serial.println("WEB control disabled: command queue allocation failed");
    return;
  }

  WiFi.mode(WIFI_AP);
  WiFi.setSleep(false);
  WiFi.onEvent(webWiFiEvent);
  WiFi.softAPConfig(WEB_AP_IP, WEB_AP_GATEWAY, WEB_AP_SUBNET);
  if (!WiFi.softAP(WEB_AP_SSID, WEB_AP_PASSWORD, 1, false, 4)) {
    Serial.println("WEB control disabled: Wi-Fi AP start failed");
    return;
  }
  webDnsServer.start(53, "*", WEB_AP_IP);

  httpd_config_t config = HTTPD_DEFAULT_CONFIG();
  config.server_port = WEB_HTTP_PORT;
  // LWIP has 16 descriptors total and HTTPD reserves three more internally.
  // Four client slots are sufficient for one control page/WebSocket plus
  // captive probes while preserving descriptor headroom for DNS and HTTPD.
  config.max_open_sockets = WEB_HTTP_MAX_OPEN_SOCKETS;
  config.max_uri_handlers = 4;
  config.stack_size = 8192;
  config.lru_purge_enable = true;
  config.keep_alive_enable = true;
  config.keep_alive_idle = WEB_HTTP_KEEPALIVE_IDLE_S;
  config.keep_alive_interval = WEB_HTTP_KEEPALIVE_INTERVAL_S;
  config.keep_alive_count = WEB_HTTP_KEEPALIVE_COUNT;
  config.recv_wait_timeout = 5;
  config.send_wait_timeout = 1;
  config.close_fn = webCloseCallback;
  config.uri_match_fn = httpd_uri_match_wildcard;
  if (httpd_start(&webHttpServer, &config) != ESP_OK) {
    webHttpServer = nullptr;
    Serial.println("WEB control disabled: HTTP server start failed");
    return;
  }

  httpd_uri_t wsUri = {};
  wsUri.uri = "/ws";
  wsUri.method = HTTP_GET;
  wsUri.handler = webSocketHandler;
  wsUri.is_websocket = true;
  httpd_register_uri_handler(webHttpServer, &wsUri);

  httpd_uri_t pageUri = {};
  pageUri.uri = "/*";
  pageUri.method = HTTP_GET;
  pageUri.handler = webPageHandler;
  httpd_register_uri_handler(webHttpServer, &pageUri);

  Serial.println("WEB controller ready");
  Serial.print("  Wi-Fi: ");
  Serial.println(WEB_AP_SSID);
  Serial.print("  Password: ");
  Serial.println(WEB_AP_PASSWORD);
  Serial.print("  Open: http://");
  Serial.println(WEB_AP_IP);
  webLastStationCount = WiFi.softAPgetStationNum();
}

void serviceWebControl() {
  if (!webCommandQueue) return;
  webDnsServer.processNextRequest();

  if (webFreshAssociationPending) {
    const uint16_t aid = webFreshAssociationAid;
    webFreshAssociationPending = false;
    const esp_err_t result = esp_wifi_deauth_sta(aid);
    Serial.print("WEB Wi-Fi fresh association requested for AID ");
    Serial.print(aid);
    Serial.print(": ");
    Serial.println(result == ESP_OK ? "ok" : esp_err_to_name(result));
  }

  const uint8_t stationCount = WiFi.softAPgetStationNum();
  const bool stationJustDisconnected = webLastStationCount > 0 && stationCount == 0;
  if ((webStationDisconnectEvent || stationJustDisconnected) && stationCount == 0) {
    webStationDisconnectEvent = false;
    webResetClientSessions("AP station disconnected");
  } else if (webStationDisconnectEvent && stationCount > 0) {
    // The disconnect event belongs to an older association. Do not close a
    // page that has already reconnected on a fresh station session.
    webStationDisconnectEvent = false;
  }
  webLastStationCount = stationCount;
  webAuditHttpSessions();

  const int expiredOwner = webExpireOwner(millis());
  if (expiredOwner >= 0 && webHttpServer) {
    httpd_sess_trigger_close(webHttpServer, expiredOwner);
  }
  if (webDisconnectStopRequested) {
    webDisconnectStopRequested = false;
    webMotionEnabled = false;
    webClientConnected = false;
    webDriveActive = false;
    webLinearCommand = 0;
    webTurnCommand = 0;
    stopForWebControl();
  }
  if (webImmediateStopRequested) {
    webImmediateStopRequested = false;
    webMotionEnabled = false;
    webDriveActive = false;
    webLinearCommand = 0;
    webTurnCommand = 0;
    stopForWebControl();
    webSendAck(webActiveClientFd, "stop", true, "all_stopped");
  }
  if (webImmediateDriveStopRequested) {
    webImmediateDriveStopRequested = false;
    chassisStop();
    webDriveActive = false;
    webLinearCommand = 0;
    webTurnCommand = 0;
    if (!robotAction.active && !isWebJogActive() && !foldAnyMotionActive()) webMarkMotionIdle();
    if (webActiveClientFd >= 0) {
      webSendAck(webActiveClientFd, "drive_stop", true, "stopped");
    }
  }
  const int activeFd = webActiveClientFd;
  webClientConnected = webSocketFdIsActive(activeFd);
  if (activeFd >= 0 && !webClientConnected) {
    webActiveClientFd = -1;
    webDisconnectStopRequested = true;
  }

  uint32_t now = millis();
  if (webDriveActive && now - lastWebDriveCommandMs > WEB_DRIVE_COMMAND_TIMEOUT_MS) {
    webLatchDriveTimeout("WEB drive command timeout: latching motion disabled");
  }

  WebControlCommand command;
  uint8_t commandsProcessed = 0;
  while (commandsProcessed < 8 && xQueueReceive(webCommandQueue, &command, 0) == pdTRUE) {
    webProcessCommand(command);
    ++commandsProcessed;
    // A tap can query/write the bus. Yield to watchdogs before another tap.
    if (command.type == WebCommandType::JogStep || webImmediateStopRequested || webImmediateDriveStopRequested ||
        webDisconnectStopRequested) break;
  }

  now = millis();
  if (webDriveActive && now - lastWebDriveCommandMs > WEB_DRIVE_COMMAND_TIMEOUT_MS) {
    webLatchDriveTimeout("WEB drive command timeout: latching motion disabled");
  }
  serviceWebArmMotion();
  const bool webJogActive = isWebJogActive();
  const uint32_t leaseAgeMs = webLeaseAgeMs(webGetControlGeneration());
  if (webOwnsMotion && (webDriveActive || robotAction.active || webJogActive ||
                        foldAnyMotionActive()) &&
      leaseAgeMs > WEB_CONTROL_LEASE_MS) {
    webMotionEnabled = false;
    webDriveActive = false;
    webLinearCommand = 0;
    webTurnCommand = 0;
    stopForWebControl();
    Serial.printf("WEB control lease timeout: stopping all actuators; lease_age_ms=%lu processed=%u queued=%u\n",
                  static_cast<unsigned long>(leaseAgeMs), static_cast<unsigned>(commandsProcessed),
                  webCommandQueue ? static_cast<unsigned>(uxQueueMessagesWaiting(webCommandQueue)) : 0U);
    webSendAck(webActiveClientFd, "lease", false, "heartbeat_timeout");
  }
  if (webOwnsMotion && !webDriveActive && !robotAction.active && !webJogActive &&
      !foldAnyMotionActive()) {
    webMarkMotionIdle();
  }

  webPollOneJoint();
  if (webStatusDue(now)) {
    webSendStatusToActiveClient();
    lastWebStatusMs = now;
  }
}

bool selectedServoPosition(int32_t &position) {
  if (selectedServoIndex < 0 || selectedServoIndex >= static_cast<int>(SERVO_COUNT)) {
    Serial.println("no_servo_selected");
    return false;
  }
  if (!servoReadPosition(SERVOS[selectedServoIndex].id, position)) {
    Serial.println("position_read_failed");
    return false;
  }
  return true;
}

void printFoldStatus() {
  Serial.printf("FOLD_CONFIG microsteps=4 max_calibration_steps=16000 formal_ms=6000 stepper_plan_ms=5800 last_mode=%s\n",
                foldActionStepperOnly ? "stepper_only" : "synchronized");
  Serial.printf(
      "FOLD state=%s position=%ld target=%ld travel=%ld travel_valid=%s "
      "homed=%s active=%s pulse_command_enabled=%s enable_control=%s direction_inverted=%s "
      "pulse_engine=%s nvs=%s id11=%s error=%s\n",
      foldStateName(), static_cast<long>(foldStepper.currentSteps()),
      static_cast<long>(foldStepper.targetSteps()),
      static_cast<long>(foldStepper.travelSteps()),
      foldStepper.hasTravel() ? "yes" : "no",
      foldStepper.isHomed() ? "yes" : "no",
      foldStepper.isActive() ? "yes" : "no",
      foldStepper.driverEnabled() ? "yes" : "no",
      foldStepper.hasEnableControl() ? "yes" : "no",
      foldStepper.directionInverted() ? "yes" : "no",
      foldStepper.pulseEngineReady() ? "ready" : "fault",
      foldPreferencesReady ? "ready" : "fault",
      foldId11Online ? "online" : "unknown/offline",
      foldError.length() ? foldError.c_str() : foldStepper.lastError());
}

void printSelectedCalibration() {
  if (selectedServoIndex < 0 || selectedServoIndex >= static_cast<int>(SERVO_COUNT)) {
    Serial.println("no_servo_selected");
    return;
  }
  const ServoDescriptor &servo = SERVOS[selectedServoIndex];
  const JointCalibration &cal = jointCal[selectedServoIndex];
  Serial.printf("selected ID %u %s calibrated=%s min=%ld center=%ld max=%ld inverted=%s\n",
                servo.id, servo.name, cal.calibrated ? "yes" : "no",
                static_cast<long>(cal.minimum), static_cast<long>(cal.center),
                static_cast<long>(cal.maximum), cal.inverted ? "yes" : "no");
}

void calibrationSetPoint(const char *point) {
  int32_t position = 0;
  if (!selectedServoPosition(position)) return;
  JointCalibration &cal = jointCal[selectedServoIndex];
  if (strcmp(point, "min") == 0) cal.minimum = position;
  else if (strcmp(point, "center") == 0) cal.center = position;
  else cal.maximum = position;
  cal.calibrated = false;
  Serial.printf("calibration %s=%ld (run cal_save after all three points are set)\n",
                point, static_cast<long>(position));
}

void calibrationJog(int delta) {
  if (selectedServoIndex < 0 || selectedServoIndex >= static_cast<int>(SERVO_COUNT)) {
    Serial.println("no_servo_selected");
    return;
  }
  const ServoDescriptor &servo = SERVOS[selectedServoIndex];
  const int maxDelta = servo.protocol == ServoProtocol::Htd ? 20 : 80;
  if (delta == 0 || delta < -maxDelta || delta > maxDelta) {
    Serial.println("cal_jog_delta_rejected");
    return;
  }
  int32_t current = 0;
  if (!selectedServoPosition(current)) return;
  if (jointCal[selectedServoIndex].inverted) delta = -delta;
  const int32_t target = constrain(current + delta,
                                   servoHardMinimum(servo),
                                   servoHardMaximum(servo));
  if (!servoMoveTo(servo.id, target, 500)) {
    Serial.println("cal_jog_failed");
    return;
  }
  Serial.printf("cal_jog target=%ld duration_ms=500\n", static_cast<long>(target));
}

bool stepLevelTestTextAllowed(const String &command) {
  return command == "help" || command == "step_diag" ||
         command == "fold_status" || command == "status_fold" ||
         command == "d_status" || command == "stop" || command == "all_stop";
}

bool handleStepLevelTestCommand(const String &command) {
  if (command == "step_level_test stop") {
    allStop();  // STEP falls before any serial or servo-bus output.
    foldStepper.printLevelTestStatus();
    return true;
  }
  if (command == "step_level_test drivers_removed_power_off") {
    // This is an explicit human acknowledgement, NOT a hardware power sensor.
    if (emergencyStopLatched || drivePwmFault || motionStopPending() ||
        localMotionActive() || foldStepper.levelTestActive() ||
        driveLeftPwm != 0 || driveRightPwm != 0) {
      Serial.println("ERR step_level_test_busy_or_safety_locked");
      return true;
    }
    allStop();  // Revoke old Web commands and disable other motion owners.
    if (foldStepper.startLevelTest()) {
      Serial.println("OK step_level_test_started; BOTH drivers removed, ALL motor power disconnected, USB only; mode locked until reboot");
      foldStepper.printLevelTestStatus();
    } else {
      Serial.println("ERR step_level_test_start_failed");
    }
    return true;
  }
  if (command.startsWith("step_level_test")) {
    foldStepper.printLevelTestStatus();
    Serial.println("USB ONLY: remove BOTH stepper driver modules; unplug XT60 and ALL battery/motor power. Firmware CANNOT verify this.");
    Serial.println("Start: step_level_test drivers_removed_power_off | Stop: step_level_test stop");
    return true;
  }
  if (foldStepper.levelTestMode() && !stepLevelTestTextAllowed(command)) {
    Serial.println("ERR step_level_test_mode_reboot_required");
    return true;
  }
  return false;
}

void printHelp() {
  Serial.println();
  Serial.println("LOCAL USB DEBUG ONLY: legacy actuator tests below may block.");
  Serial.println("Commands:");
  Serial.println("  help        - print commands");
  Serial.println("  all_stop    - folding + arm + onboard chassis stop");
  Serial.println("  h_ping      - HTD read ID response");
  Serial.println("  h_pos       - HTD read position");
  Serial.println("  h_diag [ID] - read-only HTD position/limits/torque diagnostic (default ID 11)");
  Serial.println("  h_prepare_calibration ID - set registered HTD ID to limits 0..1000 and torque on");
  Serial.println("  h_apply_calibration_limits ID - persist saved calibration limits and torque on");
  Serial.println("  h_apply_id11_limits - legacy alias for h_apply_calibration_limits 11");
  Serial.println("  h_id11_setup_check - read-only ID11 position/limits/mode/torque; web motion must be disabled");
  Serial.println("  h_id11_limits_write - ID11 saved limits ONLY, requires unloaded stable servo; never torque on");
  Serial.println("  WARNING: legacy h_apply* and h_prepare_calibration still TORQUE ON; do not use for no-motion setup");
  Serial.println("  h_baud_115k - set HTD UART to 115200");
  Serial.println("  h_baud_1m   - unavailable; HTD bus is fixed at 115200");
  Serial.println("  h_test      - HTD small position test, then stop");
  Serial.println("  h_stop      - HTD stop");
  Serial.println("  d_fwd       - onboard M1/M2 low-PWM forward 0.5s");
  Serial.println("  d_back      - onboard M1/M2 low-PWM backward 0.5s");
  Serial.println("  d_pwm N     - both tracks PWM -255..255");
  Serial.println("  d_left N    - onboard M1 / left pair PWM -255..255");
  Serial.println("  d_right N   - onboard M2 / right pair PWM -255..255");
  Serial.println("  d_drive L R - left/right PWM -255..255");
  Serial.println("  d_m1_raw    - left pair timed low-PWM forward test 300ms");
  Serial.println("  d_m2_raw    - right pair timed low-PWM forward test 300ms");
  Serial.println("  d_ticks     - print placeholder ticks (encoders not connected)");
  Serial.println("  d_zero      - clear placeholder ticks");
  Serial.println("  d_stop      - onboard chassis stop");
  Serial.println("  base_forward/base_backward/turn_left/turn_right - timed base actions");
  Serial.println("  servo_scan             - read-only scan: HTD bus + ARM HX bus");
  Serial.println("  servo_ping ID           - read-only query of one registered ID");
  Serial.println("  servo_pos ID            - read position of one registered ID");
  Serial.println("  servo_baud 115200       - select HTD query baud; never writes servo baud");
  Serial.println("  hx_torque_off_all      - EMERGENCY/DEBUG only; loaded HX joints may drop");
  Serial.println("  cal_list / cal_select ID / cal_read / cal_jog DELTA");
  Serial.println("  cal_set_min / cal_set_center / cal_set_max / cal_invert 0|1 / cal_save");
  Serial.println("  restore_torso_calibration - restore verified ID11 NVS values; no motion");
  Serial.println("  pose_capture torso_fold|torso_unfold - capture approved torso pose to NVS");
  Serial.println("  pose_check              - read-only check of the two torso poses");
  Serial.println("  fold_status / zero / jog +/-100|+/-500 / save / reset_travel (quarter-step; max travel=16000)");
  Serial.println("  stepper_test unfold|fold 6|10|20|30 - calibrated A4988 only, no ID11 move; begin with 20 seconds");
  Serial.println("  step_diag / d_status - read-only GPIO and PWM diagnostics");
  Serial.println("  step_level_test - USB-only meter-test instructions/status; BOTH drivers removed and ALL motor power unplugged first");
  Serial.println("  step_level_test drivers_removed_power_off - fixed 30s STEP LOW/HIGH test; motion locked until reboot");
  Serial.println("  step_level_test stop - stop test immediately; keeps reboot-required lock");
  Serial.println("  fold_direction / rescan - stepper calibration and ID11 diagnostics");
  Serial.println("  restore_stepper_15200 - USB only; unchanged quarter-step hardware confirmed; Web motion OFF; restore 15200/inverted, no motion, ZERO still required");
  Serial.println("  torso_unfold/torso_fold (aliases: unfold/fold) - synchronized folding actions");
  Serial.println("  HX-30HM arm IDs 1..6 are controlled from the web press/hold buttons");
  Serial.println();
  Serial.println("JSON protocol over USB Serial, 115200, one object per line:");
  Serial.println("  {\"seq\":1,\"cmd\":\"ping\"}");
  Serial.println("  {\"seq\":2,\"cmd\":\"status\"}");
  Serial.println("  {\"seq\":3,\"cmd\":\"heartbeat\"}");
  Serial.println("  {\"seq\":4,\"cmd\":\"stop\"}");
  Serial.println("  {\"seq\":5,\"cmd\":\"drive\",\"left_pwm\":60,\"right_pwm\":60}");
  Serial.println("  {\"seq\":6,\"cmd\":\"htd\",\"htd_delta\":10,\"time_ms\":800}");
  Serial.println("  {\"seq\":7,\"cmd\":\"motion\",\"htd_delta\":10,\"left_pwm\":50,\"right_pwm\":50}");
  Serial.println();
}

void handleCommand(String command) {
  command.trim();
  if (command.startsWith("{")) {
    handleJsonCommand(command);
    return;
  }

  command.toLowerCase();
  if (command.length() == 0) {
    return;
  }

  // Explicit USB-only diagnostic gate precedes legacy commands and calibration.
  if (handleStepLevelTestCommand(command)) return;

  if (emergencyStopLatched && !textCommandAllowedDuringEstop(command)) {
    Serial.print("Command rejected while estop is latched: ");
    Serial.println(command);
    return;
  }

  if (localMotionActive() && !localTextCommandAllowedDuringMotion(command)) {
    Serial.println("command_rejected_while_motion_active");
    return;
  }

  if (!ENABLE_BLOCKING_USB_DEBUG &&
      (command == "h_test" ||
       command.startsWith("d_pwm ") || command.startsWith("d_left ") ||
       command.startsWith("d_right ") || command.startsWith("d_drive "))) {
    Serial.println("blocking_debug_disabled");
    return;
  }

  if (handleId11SetupCommand(command)) return;

  if (handleStepperOnlyTestCommand(command)) {
    return;
  } else if (command == "help") {
    printHelp();
  } else if (command == "all_stop" || command == "stop") {
    allStop();
  } else if (command == "fold_status" || command == "status_fold") {
    printFoldStatus();
  } else if (command == "step_diag") {
    foldStepper.printDiagnostics();
  } else if (command == "d_status") {
    Serial.println("DRIVE mode=drive_brake pwm_hz=20000 zero=coast (duty registers are not measured speed)");
    Serial.printf("DRIVE ready=%s fault=%s left=%d right=%d\n",
                  drivePwmReady ? "yes" : "no", drivePwmFault ? "yes" : "no",
                  driveLeftPwm, driveRightPwm);
    const int pins[] = {PIN_ONBOARD_M1_IN1, PIN_ONBOARD_M1_IN2,
                        PIN_ONBOARD_M2_IN1, PIN_ONBOARD_M2_IN2};
    for (int pin : pins) {
      Serial.printf("GPIO%d duty_register=%lu frequency_when_active=%lu\n", pin,
                    static_cast<unsigned long>(ledcRead(pin)),
                    static_cast<unsigned long>(ledcReadFreq(pin)));
    }
  } else if (command == "zero") {
    String reason;
    Serial.println(foldConfirmZero(reason) ? "OK folded_zero_confirmed" : ("ERR " + reason));
  } else if (command.startsWith("jog ")) {
    String reason;
    const int32_t steps = command.substring(4).toInt();
    Serial.println(foldJog(steps, reason) ? "OK jog_started" : ("ERR " + reason));
  } else if (command == "save") {
    String reason;
    Serial.println(foldSaveTravel(reason) ? "OK travel_saved" : ("ERR " + reason));
  } else if (command == "reset_travel") {
    String reason;
    Serial.println(foldResetTravel(reason) ? "OK travel_reset" : ("ERR " + reason));
  } else if (command == "restore_stepper_15200") {
    String reason;
    const bool ok = restoreVerifiedStepperConfiguration(reason);
    Serial.println((ok ? String("OK ") : String("ERR ")) + reason);
    printFoldStatus();
  } else if (command == "fold_direction") {
    String reason;
    Serial.println(foldToggleDirection(reason) ? ("OK " + reason) : ("ERR " + reason));
  } else if (command == "rescan") {
    String reason;
    Serial.println(foldRescan(reason) ? "OK id11_online" : ("ERR " + reason));
  } else if (command == "unfold" || command == "fold") {
    String reason;
    const String action = command == "unfold" ? "torso_unfold" : "torso_fold";
    if (!startNamedAction(action, 0, ControlOwner::Usb, reason)) {
      Serial.println("ERR " + reason);
    } else {
      Serial.println("OK " + action + "_started");
    }
  } else if (command == "h_ping") {
    htdPing();
  } else if (command == "h_pos") {
    int position = 0;
    htdReadPosition(position);
  } else if (command == "h_diag" || command.startsWith("h_diag ")) {
    const int parsedId = command == "h_diag" ? TORSO_LOWER_ID : command.substring(7).toInt();
    if (parsedId <= 0 || parsedId > 255 || !isRegisteredHtdServoId(static_cast<uint8_t>(parsedId))) {
      Serial.println("unknown_or_non_htd_servo");
    } else {
      htdDiagnoseById(static_cast<uint8_t>(parsedId));
    }
  } else if (command.startsWith("h_prepare_calibration ")) {
    const int parsedId = command.substring(22).toInt();
    if (parsedId <= 0 || parsedId > 255) Serial.println("unknown_or_non_htd_servo");
    else prepareHtdCalibrationById(static_cast<uint8_t>(parsedId));
  } else if (command.startsWith("h_apply_calibration_limits ")) {
    const int parsedId = command.substring(27).toInt();
    if (parsedId <= 0 || parsedId > 255) Serial.println("unknown_or_non_htd_servo");
    else applyHtdHardwareSafetyLimitsFromCalibration(static_cast<uint8_t>(parsedId));
  } else if (command == "h_apply_id11_limits") {
    applyTorsoLowerHardwareSafetyLimits();
  } else if (command == "h_baud_115k") {
    beginHtd(115200);
  } else if (command == "h_baud_1m") {
    Serial.println("Shared HTD/HX BusLinker remains fixed at 115200");
  } else if (command == "h_test") {
    htdSmallTest();
  } else if (command == "h_stop") {
    htdStop();
  } else if (command == "d_fwd") {
    chassisForwardTest();
  } else if (command == "d_back") {
    chassisBackwardTest();
  } else if (command.startsWith("d_pwm ")) {
    int pwm = command.substring(6).toInt();
    chassisDrive(pwm);
  } else if (command.startsWith("d_left ")) {
    int pwm = command.substring(7).toInt();
    chassisDriveLeft(pwm);
  } else if (command.startsWith("d_right ")) {
    int pwm = command.substring(8).toInt();
    chassisDriveRight(pwm);
  } else if (command.startsWith("d_drive ")) {
    int spaceIndex = command.indexOf(' ', 8);
    if (spaceIndex < 0) {
      Serial.println("Usage: d_drive L R");
    } else {
      int leftPwm = command.substring(8, spaceIndex).toInt();
      int rightPwm = command.substring(spaceIndex + 1).toInt();
      chassisDriveBoth(leftPwm, rightPwm);
    }
  } else if (command == "d_m1_raw") {
    onboardM1RawTest();
  } else if (command == "d_m2_raw") {
    onboardM2RawTest();
  } else if (command == "d_ticks") {
    drivePrintTicks();
  } else if (command == "d_zero") {
    driveResetTicks();
  } else if (command == "d_stop") {
    chassisStop();
  } else if (command == "hx_torque_off_all") {
    emergencyTorqueOffAllHx();
    Serial.println("HX torque disabled by explicit emergency/debug command");
  } else if (command == "base_forward" || command == "base_backward" ||
             command == "turn_left" || command == "turn_right" ||
             command == "torso_unfold" || command == "torso_fold") {
    String reason;
    if (!startNamedAction(command, 0, ControlOwner::Usb, reason)) {
      Serial.print("Named action rejected: ");
      Serial.println(reason);
    }
  } else if (command == "servo_scan") {
    servoScan();
  } else if (command.startsWith("servo_ping ")) {
    const int id = command.substring(11).toInt();
    if (!findServo(id)) Serial.println("unknown_servo_id");
    else Serial.println(servoPing(id) ? "servo_online" : "servo_no_response");
  } else if (command.startsWith("servo_pos ")) {
    const int id = command.substring(10).toInt();
    int32_t position = 0;
    if (!findServo(id)) Serial.println("unknown_servo_id");
    else if (servoReadPosition(id, position)) Serial.printf("servo ID %d position=%ld\n", id, static_cast<long>(position));
    else Serial.println("position_read_failed");
  } else if (command.startsWith("servo_baud ")) {
    const uint32_t baud = static_cast<uint32_t>(command.substring(11).toInt());
    if (baud == 115200) beginHtd(baud);
    else Serial.println("servo_baud only accepts the shared 115200 bus baud");
  } else if (command == "cal_list") {
    printCalibrationList();
  } else if (command.startsWith("cal_select ")) {
    selectedServoIndex = servoIndex(command.substring(11).toInt());
    if (selectedServoIndex < 0) Serial.println("unknown_servo_id");
    else printSelectedCalibration();
  } else if (command == "cal_read") {
    int32_t position = 0;
    if (selectedServoPosition(position)) Serial.printf("cal_read position=%ld\n", static_cast<long>(position));
  } else if (command.startsWith("cal_jog ")) {
    calibrationJog(command.substring(8).toInt());
  } else if (command == "cal_set_min") {
    calibrationSetPoint("min");
  } else if (command == "cal_set_center") {
    calibrationSetPoint("center");
  } else if (command == "cal_set_max") {
    calibrationSetPoint("max");
  } else if (command.startsWith("cal_invert ")) {
    const int inverted = command.substring(11).toInt();
    if (selectedServoIndex < 0) Serial.println("no_servo_selected");
    else if (inverted != 0 && inverted != 1) Serial.println("cal_invert must be 0 or 1");
    else {
      jointCal[selectedServoIndex].inverted = inverted == 1;
      jointCal[selectedServoIndex].calibrated = false;
      printSelectedCalibration();
    }
  } else if (command == "cal_save") {
    String reason;
    if (saveCalibration(selectedServoIndex, reason)) printSelectedCalibration();
    else Serial.println(reason);
  } else if (command == "restore_torso_calibration") {
    String reason;
    if (restoreVerifiedTorsoCalibration(reason)) {
      Serial.println("torso_calibration_restored no_motion");
      const int id11Index = servoIndex(11);
      Serial.printf("ID11 calibrated=%s min=%ld center=%ld max=%ld\n",
                    jointCal[id11Index].calibrated ? "yes" : "no",
                    static_cast<long>(jointCal[id11Index].minimum),
                    static_cast<long>(jointCal[id11Index].center),
                    static_cast<long>(jointCal[id11Index].maximum));
    } else {
      Serial.print("restore_torso_calibration_failed: ");
      Serial.println(reason);
    }
  } else if (command.startsWith("pose_capture ")) {
    String reason;
    const String poseName = command.substring(13);
    if (capturePose(poseName, reason)) Serial.println("pose_captured");
    else Serial.println(reason);
  } else if (command == "pose_check") {
    printPoseCheck();
  } else {
    Serial.print("Unknown command: ");
    Serial.println(command);
    printHelp();
  }
}

void setup() {
  Serial.begin(115200);
  // Keep disconnected/stalled USB logging from delaying motor watchdogs.
  // HWCDC requires a positive timeout; zero can underflow its retry counter.
#if ARDUINO_USB_CDC_ON_BOOT
  Serial.setTxTimeoutMs(1);
#endif
  beginFoldMotion();
  beginOnboardDrive();
  delay(1200);

  Serial.println();
  Serial.println("Sophicar ESP32-S3-N16R8 + onboard DRV8870 staged controller");
  Serial.println(FIRMWARE_BUILD_ID);
  foldStepper.printDiagnostics();
  Serial.print("HTD UART pins TX/RX = ");
  Serial.print(PIN_HTD_TX);
  Serial.print("/");
  Serial.println(PIN_HTD_RX);
  Serial.println("Shared HTD/HX BusLinker uses the HTD UART pins above at 115200");
  Serial.printf("A4988 external pins DIR=%u STEP=%u ENABLE=%d active_low=yes\n",
                PIN_FOLD_DIR, PIN_FOLD_STEP, PIN_FOLD_ENABLE);
  Serial.printf("A4988 single-output GPTimer=%s resolution=1MHz step_high_us=4 rising_only=yes fixed_microstep=1/4 MS1=GND MS2=3V3 MS3=GND\n",
                foldStepper.pulseEngineReady() ? "ready" : "fault");
  Serial.printf("Onboard DRV8870 M1 IN1/IN2 = %d/%d (left motor pair)\n",
                PIN_ONBOARD_M1_IN1, PIN_ONBOARD_M1_IN2);
  Serial.printf("Onboard DRV8870 M2 IN1/IN2 = %d/%d (right motor pair)\n",
                PIN_ONBOARD_M2_IN1, PIN_ONBOARD_M2_IN2);
  Serial.println("Build requirement: PSRAM Disabled; chassis encoders are not connected");
  beginHtd(HTD_DEFAULT_BAUD);
  beginArmBus();
  loadCalibration();
  foldId11Online = servoPing(TORSO_SECOND_BOARD_ID);

  beginWebControl();
  printHelp();

  Serial.println("Boot complete. A4988 STEP is LOW and ENABLE is HIGH (disabled); external pull-up still required.");
  Serial.println("Place the first folding joint at the folded mechanical endpoint, then run ZERO.");
}

void serviceSerialInput() {
  uint16_t consumed = 0;
  while (consumed < SERIAL_RX_BYTES_PER_LOOP && Serial.available()) {
    ++consumed;
    char c = Serial.read();
    if (c == '\n' || c == '\r') {
      if (serialLineDiscarding) {
        serialLineDiscarding = false;
        inputLine = "";
        Serial.println("serial_line_too_long_discarded");
        return;
      }
      if (inputLine.length()) {
        handleCommand(inputLine);
        inputLine = "";
        return;  // Always service watchdogs between complete USB commands.
      }
    } else if (!serialLineDiscarding) {
      inputLine += c;
      if (inputLine.length() > 360) {
        inputLine = "";
        serialLineDiscarding = true;  // Discard the WHOLE line, not its prefix.
      }
    }
  }
}

void loop() {
  serviceWebControl();
  serviceFoldMotion();
  serviceSerialInput();
  checkHostWatchdog();
  serviceHxHold();
  serviceBaseMotion();
  serviceRobotAction();
}
