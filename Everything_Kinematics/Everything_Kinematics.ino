#include <Arduino.h>
#include <Servo.h>
#include <AccelStepper.h>

// --- Config Includes ---
#if __has_include("pin_config.user.h")
  #include "pin_config.user.h"
  #include "pin_config.h"
#else
  #include "pin_config.h"
#endif

#if __has_include("general_config.user.h")
  #include "general_config.user.h"
  #include "general_config.h"
#else
  #include "general_config.h"
#endif

#ifdef USE_NATIVE_LEDS
  #include <Adafruit_NeoPixel.h>  
  // Construct the 4 independent strip objects using your defined macros
  Adafruit_NeoPixel deckL(DECK_LED_LENGTH_L, DECK_PIN_L, NEO_GRB + NEO_KHZ800);
  Adafruit_NeoPixel deckR(DECK_LED_LENGTH_R, DECK_PIN_R, NEO_GRB + NEO_KHZ800);
  Adafruit_NeoPixel laneL(LANE_LED_LENGTH_L, LANE_PIN_L, NEO_GRB + NEO_KHZ800);
  Adafruit_NeoPixel laneR(LANE_LED_LENGTH_R, LANE_PIN_R, NEO_GRB + NEO_KHZ800);
#endif


#if DEBUG_TURRET
  #define DEBUG_PRINT(x) Serial.println(x)
  #define tPrint(x) Serial.println(String("TURRET: ") + x)
#else
  #define DEBUG_PRINT(x) 
  #define tPrint(x) 
#endif

#if SCOREMORE_USER == 1
void waitForScoreMore() {
    Serial.println("WAITING_FOR_SCOREMORE");
    while (true) {
        if (Serial && Serial.available() > 0) {
            Serial.println("SCOREMORE_CONNECTED");
            return;
        }
        delay(5);
    }
}
#endif

// ==========================================
// GLOBAL HARDWARE & TELEMETRY
// ==========================================
Servo LeftRaiseServo, RightRaiseServo, SlideServo, ScissorsServo, LeftSweepServo, RightSweepServo, BallReturnServo;
AccelStepper stepper1(1, STEP_PIN, DIR_PIN);

// ==========================================
// 1. BUTTON CONTROLLER (Non-Blocking Debounce)
// ==========================================
class ButtonController {
public:
    enum Event { NONE, SHORT_PRESS, LONG_PRESS };
    
    ButtonController(int pin) : _pin(pin), _buttonState(HIGH), _lastFlickerState(HIGH), 
                                _lastDebounceTime(0), _pressStartTime(0), 
                                _isPressed(false), _longPressFired(false) {}
    
    void begin() { pinMode(_pin, INPUT_PULLUP); }
    
    Event update() {
        int reading = digitalRead(_pin);
        Event e = NONE;
        
        if (reading != _lastFlickerState) {
            _lastDebounceTime = millis();
        }
        
        if ((millis() - _lastDebounceTime) > 20) {
            if (reading != _buttonState) {
                _buttonState = reading;
                if (_buttonState == LOW) {
                    _isPressed = true;
                    _pressStartTime = millis();
                    _longPressFired = false;
                } else {
                    _isPressed = false;
                    if (!_longPressFired && (millis() - _pressStartTime > 50)) {
                        e = SHORT_PRESS; 
                    }
                }
            }
        }
        
        if (_isPressed && !_longPressFired && (millis() - _pressStartTime >= 1000)) {
            _longPressFired = true;
            e = LONG_PRESS;
        }
        
        _lastFlickerState = reading;
        return e;
    }
private:
    int _pin, _buttonState, _lastFlickerState;
    unsigned long _lastDebounceTime, _pressStartTime;
    bool _isPressed, _longPressFired;
};

// ==========================================
// 2. TURRET CONTROLLER FSM (RESTORED DEBUGS)
// ==========================================
class TurretController {
public:
    enum State {
        UNHOMED, HOMING_PROBE_CCW, HOMING_BLIND_ESCAPE,
        HOMING_FAST_FWD, HOMING_BACKOFF, HOMING_CREEP,
        IDLE_EMPTY, 
        WAITING_FOR_PIN, CATCH_DELAY_PHASE, INDEXING_TO_NEXT, 
        PAUSED_AT_9, 
        FETCH_10TH, 
        MOVING_TO_DROP, DWELLING_AT_DROP, 
        MOVING_TO_PURGE,
        SLEEP_STARVED_WAIT, SLEEP_STARVED_FETCH,
        HALTED 
    };

    TurretController(AccelStepper& stp, int ir, int hall, int relay, int enablePin)
        : _stepper(stp), _irPin(ir), _hallPin(hall), _relayPin(relay), _enablePin(enablePin),
          _state(UNHOMED), _loadedCount(0), _targetPos(0), _dropComplete(false), 
          _queuedPins(0), _lastPinDetectTime(0), _isPurging(false) {} 

    void begin() {
        pinMode(_irPin, INPUT_PULLUP); pinMode(_hallPin, INPUT_PULLUP); pinMode(_relayPin, OUTPUT);
        pinMode(_enablePin, OUTPUT);
        enableStepper();
        conveyorOff();
        _stepper.setMaxSpeed(TURRET_NORMAL_MAXSPEED); _stepper.setAcceleration(TURRET_NORMAL_ACCEL);
        _irStableState = HIGH; _irLastRead = HIGH;
        tPrint("Booting: Entering UNHOMED state.");
        changeState(UNHOMED);
    }

    void update() {
        if (_state == HALTED) return; 
        
        _stepper.run();
        updateIRSensor(); 
        unsigned long elapsed = millis() - _stateStart;

        switch (_state) {
            case UNHOMED:
                conveyorOff();
                if (digitalRead(_hallPin) == LOW) { _stepper.setMaxSpeed(150); _stepper.move(150); changeState(HOMING_PROBE_CCW); }
                else { _stepper.move(10000); changeState(HOMING_FAST_FWD); }
                break;
            case HOMING_PROBE_CCW:
                if (_stepper.distanceToGo() == 0) {
                    if (digitalRead(_hallPin) == HIGH) { _stepper.setMaxSpeed(TURRET_NORMAL_MAXSPEED); _stepper.move(10000); changeState(HOMING_FAST_FWD); }
                    else { _stepper.move(-150); changeState(HOMING_BACKOFF); }
                }
                break;
            case HOMING_FAST_FWD: if (digitalRead(_hallPin) == LOW) { _stepper.stop(); changeState(HOMING_BACKOFF); } break;
            case HOMING_BACKOFF: if (_stepper.distanceToGo() == 0) { _stepper.move(-150); _stepper.setMaxSpeed(100); changeState(HOMING_CREEP); } break;
            case HOMING_CREEP:
                if (_stepper.distanceToGo() == 0) {
                    if (digitalRead(_hallPin) == LOW) { 
                        _stepper.setCurrentPosition(TURRET_HOME_ADJUSTER); 
                        _stepper.setMaxSpeed(TURRET_NORMAL_MAXSPEED); 
                        _loadedCount = 0; _queuedPins = 0; 
                        tPrint("Homing Complete. Entering IDLE_EMPTY.");
                        changeState(IDLE_EMPTY); 
                    }
                    else { _stepper.move(2); } 
                }
                break;

            case IDLE_EMPTY: 
                conveyorOff(); 
                break;
                
            case WAITING_FOR_PIN:
                conveyorOn(); 
                if (millis() - _lastPinDetectTime > NO_CATCH_TIMEOUT_MS) {
                    conveyorOff();
                    tPrint("Starved Timeout Reached. Sleeping.");
                    changeState(SLEEP_STARVED_WAIT);
                    break;
                }
                if (_queuedPins > 0) {
                    _queuedPins--; 
                    _loadedCount++; 
                    tPrint("Caught Pin " + String(_loadedCount) + ". Queued remaining: " + String(_queuedPins));
                    changeState(CATCH_DELAY_PHASE, CATCH_DELAY_MS); 
                }
                break;

            case SLEEP_STARVED_WAIT: conveyorOff(); break;

            case CATCH_DELAY_PHASE:
                conveyorOn();
                if (elapsed >= _timeout) {
                    if (_loadedCount == 9) { 
                        tPrint("9 Pins Loaded. Hard Locking Turret.");
                        changeState(PAUSED_AT_9); 
                    } 
                    else { 
                        _targetPos = getPositionForSlot(_loadedCount + 1); 
                        _stepper.moveTo(_targetPos); 
                        tPrint("Indexing to slot " + String(_loadedCount + 1));
                        changeState(INDEXING_TO_NEXT); 
                    }
                }
                break;

            case INDEXING_TO_NEXT: 
                conveyorOn();
                if (_stepper.distanceToGo() == 0) { changeState(WAITING_FOR_PIN); } 
                break;

            case PAUSED_AT_9: conveyorOff(); break;

            case FETCH_10TH:
                conveyorOn(); 
                if (millis() - _lastPinDetectTime > NO_CATCH_TIMEOUT_MS) {
                    conveyorOff();
                    tPrint("10th Pin Starved. Sleeping.");
                    changeState(SLEEP_STARVED_FETCH);
                    break;
                }
                if (_queuedPins > 0) {
                    _queuedPins--; 
                    _loadedCount = 10;
                    tPrint("10th Pin Sensed! Pushing and Moving to Drop...");
                    _stepper.setMaxSpeed(TURRET_SPRING_MAXSPEED);
                    _stepper.setAcceleration(TURRET_SPRING_ACCEL);
                    _targetPos = getPositionForSlot(10) + TURRET_PIN10_RELEASE_OFFSET;
                    _stepper.moveTo(_targetPos);
                    changeState(MOVING_TO_DROP); 
                }
                break;

            case SLEEP_STARVED_FETCH: conveyorOff(); break;

            case MOVING_TO_DROP:
                conveyorOn(); 
                if (_stepper.distanceToGo() == 0) { changeState(DWELLING_AT_DROP, RELEASE_DWELL_MS); }
                break;

            case MOVING_TO_PURGE:
                if (_stepper.distanceToGo() == 0) { changeState(DWELLING_AT_DROP, RELEASE_DWELL_MS); }
                break;

            case DWELLING_AT_DROP:
                if (!_isPurging && elapsed < RELEASE_FEED_ASSIST_MS) { conveyorOn(); } else { conveyorOff(); }
                if (elapsed >= _timeout) {
                    tPrint("Drop Complete. Escaping Blindly.");
                    _dropComplete = true; 
                    _stepper.setMaxSpeed(TURRET_NORMAL_MAXSPEED);
                    _stepper.move(800); 
                    changeState(HOMING_BLIND_ESCAPE); 
                }
                break;

            case HOMING_BLIND_ESCAPE:
                if (_stepper.distanceToGo() == 0) {
                    if (digitalRead(_hallPin) == HIGH) { _stepper.move(10000); changeState(HOMING_FAST_FWD); }
                    else { _stepper.move(100); }
                }
                break;
        }
    }

    bool commandStartRefill() {
        if (_state != IDLE_EMPTY) return false;
        tPrint("Command: Start Refill");
        _dropComplete = false; _loadedCount = 0; _queuedPins = 0; 
        _isPurging = false; // <-- NEW: Ensure feed assist is active for gameplay       
        _lastPinDetectTime = millis() - IR_PIN_LOCKOUT_MS - 1;        
        _targetPos = getPositionForSlot(1); _stepper.moveTo(_targetPos); changeState(INDEXING_TO_NEXT); 
        return true;
    }

    bool commandExecuteDropSequence() {
        if (_state != PAUSED_AT_9) return false;
        tPrint("Command: Execute Drop Sequence (Waiting on 10th Pin)");
        _dropComplete = false;        
        _isPurging = false; // <-- NEW: Ensure feed assist is active for gameplay
        _lastPinDetectTime = millis() - IR_PIN_LOCKOUT_MS - 1;        
        changeState(FETCH_10TH);
        return true;
    }

    bool commandWake() {
        if (_state == SLEEP_STARVED_WAIT) { 
            tPrint("Waking up to WAITING_FOR_PIN"); 
            // FIX: Remove blind spot
            _lastPinDetectTime = millis() - IR_PIN_LOCKOUT_MS - 1; 
            changeState(WAITING_FOR_PIN); 
            return true; 
        } 
        else if (_state == SLEEP_STARVED_FETCH) { 
            tPrint("Waking up to FETCH_10TH"); 
            // FIX: Remove blind spot
            _lastPinDetectTime = millis() - IR_PIN_LOCKOUT_MS - 1; 
            changeState(FETCH_10TH); 
            return true; 
        }
        return false; 
    }

    bool commandPurge() {
        if (_state != IDLE_EMPTY) return false;
        tPrint("Command: Purge Turret");
        _dropComplete = false; 
        _isPurging = true; // <-- NEW: Suppress feed assist during purge
        _stepper.setMaxSpeed(TURRET_SPRING_MAXSPEED); _stepper.setAcceleration(TURRET_SPRING_ACCEL);
        _targetPos = getPositionForSlot(10) + TURRET_PIN10_RELEASE_OFFSET + TURRET_EMPTY_EXTRA_OFFSET; 
        _stepper.moveTo(_targetPos); changeState(MOVING_TO_PURGE);
        return true;
    }

    bool commandSuspendFetch() {
        if (_state == FETCH_10TH || _state == SLEEP_STARVED_FETCH) {
            tPrint("Command: Suspending Fetch. Returning to PAUSED_AT_9");
            changeState(PAUSED_AT_9); _lastPinDetectTime = millis(); return true;
        }
        return false; 
    }

    void emergencyHalt() {
        conveyorOff();
        _stepper.stop();
        _queuedPins = 0;
        tPrint("EMERGENCY HALT TRIGGERED");
        changeState(HALTED);
    }
    
    void disableStepper() { digitalWrite(_enablePin, HIGH); } 
    void enableStepper()  { digitalWrite(_enablePin, LOW); }
    
    bool isIdleEmpty() const { return _state == IDLE_EMPTY; }
    bool isPausedAt9() const { return _state == PAUSED_AT_9; }
    bool isDropComplete() const { return _dropComplete; }
    bool isHoming() const { return (_state == UNHOMED || _state == HOMING_PROBE_CCW || _state == HOMING_FAST_FWD || _state == HOMING_BACKOFF || _state == HOMING_CREEP || _state == HOMING_BLIND_ESCAPE); }
    bool isCommittedToDrop() const { return (_state == MOVING_TO_DROP || _state == DWELLING_AT_DROP || _state == MOVING_TO_PURGE); }

private:
    AccelStepper& _stepper; int _irPin, _hallPin, _relayPin, _enablePin; State _state; 
    unsigned long _stateStart, _timeout; int _loadedCount; long _targetPos;
    int _irLastRead, _irStableState; unsigned long _irLastChange;
    bool _dropComplete; int _queuedPins; unsigned long _lastPinDetectTime; 
    bool _isPurging;
    const unsigned long PIN_TRANSIT_LOCKOUT_MS = IR_PIN_LOCKOUT_MS; 

    void changeState(State s, unsigned long to = 0) { _state = s; _stateStart = millis(); _timeout = to; }
    void conveyorOn() { digitalWrite(_relayPin, CONVEYOR_ACTIVE_HIGH ? HIGH : LOW); }
    void conveyorOff() { digitalWrite(_relayPin, CONVEYOR_ACTIVE_HIGH ? LOW : HIGH); }

    void updateIRSensor() {
        int raw = digitalRead(_irPin);
        if (raw != _irLastRead) { _irLastChange = millis(); _irLastRead = raw; }
        if ((millis() - _irLastChange) > DEBOUNCE_MS) {
            if (_irStableState != raw) {
                _irStableState = raw;
                if (_irStableState == LOW) {
                    if ((millis() - _lastPinDetectTime) > PIN_TRANSIT_LOCKOUT_MS) {
                        _lastPinDetectTime = millis(); 
                        if (_state != UNHOMED && _state != HOMING_PROBE_CCW && _state != HOMING_BLIND_ESCAPE && _state != HOMING_FAST_FWD && 
                            _state != HOMING_BACKOFF && _state != HOMING_CREEP && _state != DWELLING_AT_DROP && _state != MOVING_TO_PURGE &&
                            _state != SLEEP_STARVED_WAIT && _state != SLEEP_STARVED_FETCH && _state != HALTED) { 
                            _queuedPins++;
                            tPrint("IR TRIGGERED. Queued: " + String(_queuedPins));
                        }
                    } else {
                        tPrint("IR Flapped (Lockout Active). Ignored.");
                    }
                }
            }
        }
    }

    long getPositionForSlot(int slot) {
        switch(slot) {
            case 1: return PIN_POS_1; case 2: return PIN_POS_2; case 3: return PIN_POS_3; case 4: return PIN_POS_4;
            case 5: return PIN_POS_5; case 6: return PIN_POS_6; case 7: return PIN_POS_7; case 8: return PIN_POS_8;
            case 9: return PIN_POS_9; case 10: return PIN_POS_10; default: return 0;
        }
    }
};

// ==========================================
// 3. SWEEP CONTROLLER FSM
// ==========================================

class SweepController {
public:
    enum State { IDLE, TWEENING }; enum Pose { GUARD, UP, BACK, UNKNOWN };
    SweepController(Servo& left, Servo& right) : _left(left), _right(right), _state(IDLE), _currentPose(UNKNOWN) {}

    void begin() {
        _left.write(SWEEP_UP_ANGLE); _right.write(180 - SWEEP_UP_ANGLE);
        _currentPose = UP; _startL = _targetL = _curL = SWEEP_UP_ANGLE;
    }

    void update() {
        if (_state == TWEENING) {
            unsigned long elapsed = millis() - _tweenStart;
            if (elapsed >= SWEEP_TWEEN_MS) {
                _curL = _targetL; _left.write(_curL); _right.write(180 - _curL);
                _state = IDLE; _currentPose = _targetPose;
            } else {
                float t = (float)elapsed / (float)SWEEP_TWEEN_MS;
                _curL = _startL + (int)((_targetL - _startL) * t);
                _left.write(_curL); _right.write(180 - _curL);
            }
        }
    }

    bool commandPose(Pose target) {
        if (_state != IDLE) return false;
        if (_currentPose == target) return true;
        _startL = _curL;
        if (target == GUARD) _targetL = SWEEP_GUARD_ANGLE;
        else if (target == UP) _targetL = SWEEP_UP_ANGLE;
        else if (target == BACK) _targetL = SWEEP_BACK_ANGLE;
        _targetPose = target; _tweenStart = millis(); _state = TWEENING;
        return true;
    }
    bool isIdle() const { return _state == IDLE; }
    Pose getCurrentPose() const { return _currentPose; }
private:
    Servo& _left; Servo& _right; State _state; Pose _currentPose; Pose _targetPose;
    unsigned long _tweenStart; int _startL, _targetL, _curL;
};

// ==========================================
// 4. DECK CONTROLLER FSM
// ==========================================
class DeckController {
public:
    enum State { IDLE_UP, SETTLING_PINS_DOWN, SETTLING_PINS_UP, GRABBING_DOWN, GRABBING_UP, DROPPING_DOWN, DROPPING_UP, MAINT_PARKED };
    DeckController(Servo& lr, Servo& rr, Servo& sl, Servo& sc) : _lr(lr), _rr(rr), _slide(sl), _scissors(sc), _state(IDLE_UP) {}

    void begin() {
        attachSlide(); _scissors.write(SCISSOR_DROP_ANGLE);
        setRaise(RAISE_UP_ANGLE); _state = IDLE_UP;
    }

    void update() {
        if (_state == MAINT_PARKED) return;
        
        unsigned long elapsed = millis() - _stateStart;
        switch (_state) {
            case SETTLING_PINS_DOWN:
                if (elapsed >= PDROP_RAISE_SETTLE_MS) { _slide.write(SLIDER_RELEASE_ANGLE); changeState(SETTLING_PINS_UP, PDROP_DROP_MS); }
                break;
            case SETTLING_PINS_UP:
                if (elapsed >= _timeout) { setRaise(RAISE_UP_ANGLE); changeState(IDLE_UP, PDROP_RAISE_SETTLE_MS); }
                break;
            case GRABBING_DOWN:
                if (elapsed >= PDROP_RAISE_SETTLE_MS) { _scissors.write(SCISSOR_GRAB_ANGLE); changeState(GRABBING_UP, 800); }
                break;
            case GRABBING_UP:
                if (elapsed >= _timeout) { setRaise(RAISE_UP_ANGLE); changeState(IDLE_UP, PDROP_RAISE_SETTLE_MS); }
                break;
            case DROPPING_DOWN:
                if (elapsed >= PDROP_RAISE_SETTLE_MS) { _scissors.write(SCISSOR_DROP_ANGLE); changeState(DROPPING_UP, 800); }
                break;
            case DROPPING_UP:
                if (elapsed >= _timeout) { setRaise(RAISE_UP_ANGLE); changeState(IDLE_UP, PDROP_RAISE_SETTLE_MS); }
                break;
            case IDLE_UP:
            default: break;
        }
        if (_state == IDLE_UP && _timeout != 0) {
           if (elapsed >= _timeout) { _slide.write(SLIDER_HOME_ANGLE); _timeout = 0; }
        }
    }

    bool commandSetPins() { if (_state != IDLE_UP) return false; setRaise(RAISE_DOWN_ANGLE); changeState(SETTLING_PINS_DOWN); return true; }
    bool commandGrabPins() { if (_state != IDLE_UP) return false; setRaise(RAISE_GRAB_ANGLE); changeState(GRABBING_DOWN); return true; }
    bool commandDropPins() { if (_state != IDLE_UP) return false; setRaise(RAISE_DROP_ANGLE); changeState(DROPPING_DOWN); return true; }
    bool isIdle() const { return _state == IDLE_UP && _timeout == 0; }

    void forceScissorsOpen() { _scissors.write(SCISSOR_DROP_ANGLE); }
    void forceMaintenanceHeight() { setRaise(RAISE_DROP_ANGLE); _state = MAINT_PARKED; }
    void detachSlide() { _slide.detach(); }
    void attachSlide() { _slide.attach(SLIDE_PIN); _slide.write(SLIDER_HOME_ANGLE); }

private:
    Servo& _lr; Servo& _rr; Servo& _slide; Servo& _scissors; State _state; unsigned long _stateStart, _timeout;
    void changeState(State s, unsigned long to = 0) { _state = s; _stateStart = millis(); _timeout = to; }
    void setRaise(int ang) { _lr.write(ang); _rr.write(180 - ang); }
};

// ==========================================
// 5. BALL RETURN CONTROLLER FSM (FIXED SAFETY HAZARD)
// ==========================================
class BallReturnController {
public:
    enum State { CLOSED, WAITING_PIN_CLEAR, OPEN };
    BallReturnController(Servo& servo, int relayPin) : _servo(servo), _relayPin(relayPin), _state(CLOSED), _stateStart(0) {}
    void begin() { closeDoor(); }
    void update() {
        unsigned long elapsed = millis() - _stateStart;
        //bool conveyorRunning = (digitalRead(_relayPin) == (CONVEYOR_ACTIVE_HIGH ? HIGH : LOW));
        switch (_state) {
            case CLOSED: break;
            case WAITING_PIN_CLEAR:
                if (elapsed >= BALL_DOOR_WAIT_MS) { openDoor(); changeState(OPEN); }
                break;
            case OPEN:
                // FIX APPLIED: Removed || !conveyorRunning. 
                // The door will now stay open for the full 15 seconds regardless of turret indexing.
                if (elapsed >= BALL_DOOR_OPEN_MS) { closeDoor(); changeState(CLOSED); }
                break;
        }
    }
    void triggerCycle() { if (_state == CLOSED) { changeState(WAITING_PIN_CLEAR); } }
    void emergencyClose() { if (_state != CLOSED) { closeDoor(); changeState(CLOSED); } }
private:
    Servo& _servo; int _relayPin; State _state; unsigned long _stateStart;
    void changeState(State s) { _state = s; _stateStart = millis(); }
    void openDoor() { _servo.write(BALL_DOOR_OPEN_ANGLE); }
    void closeDoor() { _servo.write(BALL_DOOR_CLOSED_ANGLE); }
};

// ==========================================
// 6. DUAL BALL SENSOR CONTROLLER (DECOUPLED & SHIELDED)
// ==========================================
volatile bool _isrSpeedTripped = false;
volatile bool _isrTriggerTripped = false;
volatile unsigned long _isrSpeedTime = 0;
volatile unsigned long _isrTriggerTime = 0;

#if USE_HARDWARE_INTERRUPTS == 1
// SENSOR 1: SPEED (Clean hardware signal expected)
void ballSensorSpeedISR() {
    unsigned long currentMicros = micros(); 
    if (currentMicros - _isrSpeedTime > (BALL_REARM_MS * 1000UL)) {
        _isrSpeedTripped = true;
        _isrSpeedTime = currentMicros;
    }
}

// SENSOR 2: TRIGGER (Clean hardware signal expected)
void ballSensorTriggerISR() {
    unsigned long currentMicros = micros();
    if (currentMicros - _isrTriggerTime > (BALL_REARM_MS * 1000UL)) {
        _isrTriggerTripped = true;
        _isrTriggerTime = currentMicros;
    }
}
#endif

class BallSensorController {
public:
    BallSensorController(int pinSpeed, int pinTrigger) : _pinSpeed(pinSpeed), _pinTrigger(pinTrigger), 
        _smSpeedPulseActive(false), _smTriggerPulseActive(false), 
        _smSpeedPulseStart(0), _smTriggerPulseStart(0),
        _wasReady(false), _readyStartTime(0),
        _lastFlickerSpeed(HIGH), _lastFlickerTrigger(HIGH),
        _lastSpeedRead(HIGH), _lastTriggerRead(HIGH),
        _speedDebounceTime(0), _triggerDebounceTime(0) {}
    
    void begin() { 
        pinMode(_pinSpeed, INPUT_PULLUP);
        pinMode(_pinTrigger, INPUT_PULLUP); 

        #if USE_HARDWARE_INTERRUPTS == 1
        attachInterrupt(digitalPinToInterrupt(_pinSpeed), ballSensorSpeedISR, FALLING);
        attachInterrupt(digitalPinToInterrupt(_pinTrigger), ballSensorTriggerISR, FALLING);
        #endif
    }
    
    bool updateAndCheck(bool systemReady) {
        // --- 1. SCOREMORE PULSE TIMEOUTS ---
        if (_smSpeedPulseActive && (millis() - _smSpeedPulseStart >= SCOREMORE_BALL_PULSE_MS)) {
            Serial.print("INPUT_CHANGE:");
            Serial.print(SM_SPEED_SENSOR); Serial.println(":1");
            _smSpeedPulseActive = false;
        }

        if (_smTriggerPulseActive && (millis() - _smTriggerPulseStart >= SCOREMORE_BALL_PULSE_MS)) {
            Serial.print("INPUT_CHANGE:");
            Serial.print(SM_BALL_TRIGGER); Serial.println(":1");
            _smTriggerPulseActive = false;
        }

        // --- 2. SWEEP OCCLUSION ---
        if (!systemReady) {
            _wasReady = false;
            _isrSpeedTripped = false;
            _isrTriggerTripped = false;
            return false;
        }

        // --- 3. INDUCTIVE SETTLE ---
        if (!_wasReady) {
            _wasReady = true;
            _readyStartTime = millis();
            _isrSpeedTripped = false;
            _isrTriggerTripped = false;
            return false;
        }
        if (millis() - _readyStartTime < 500) {
            _isrSpeedTripped = false;
            _isrTriggerTripped = false;
            return false; 
        }

        bool triggeredGame = false;

        #if USE_HARDWARE_INTERRUPTS == 0
        // --- 4. SOFTWARE POLLING (EMI RESISTANT) ---
        // Simulates an ISR falling-edge trigger but with a 5ms physical debounce requirement
        
        int currentSpeedRead = digitalRead(_pinSpeed);
        if (currentSpeedRead != _lastFlickerSpeed) {
            _speedDebounceTime = millis();
        }
        if ((millis() - _speedDebounceTime) > 5) { // Requires 5ms continuous block
            if (currentSpeedRead != _lastSpeedRead) {
                _lastSpeedRead = currentSpeedRead;
                if (_lastSpeedRead == LOW) _isrSpeedTripped = true;
            }
        }
        _lastFlickerSpeed = currentSpeedRead;

        int currentTriggerRead = digitalRead(_pinTrigger);
        if (currentTriggerRead != _lastFlickerTrigger) {
            _triggerDebounceTime = millis();
        }
        if ((millis() - _triggerDebounceTime) > 5) { // Requires 5ms continuous block
            if (currentTriggerRead != _lastTriggerRead) {
                _lastTriggerRead = currentTriggerRead;
                if (_lastTriggerRead == LOW) _isrTriggerTripped = true;
            }
        }
        _lastFlickerTrigger = currentTriggerRead;
        #endif

        // --- 5. EVALUATE TRIPS ---
        if (_isrSpeedTripped) {
            _isrSpeedTripped = false;
            Serial.print("INPUT_CHANGE:"); Serial.print(SM_SPEED_SENSOR); Serial.println(":0");
            _smSpeedPulseActive = true;
            _smSpeedPulseStart = millis();
        }

        if (_isrTriggerTripped) {
            _isrTriggerTripped = false;
            triggeredGame = true; 
            Serial.print("INPUT_CHANGE:"); Serial.print(SM_BALL_TRIGGER); Serial.println(":0");
            _smTriggerPulseActive = true;
            _smTriggerPulseStart = millis();
        }

        return triggeredGame; 
    }

private:
    int _pinSpeed, _pinTrigger; 
    bool _smSpeedPulseActive, _smTriggerPulseActive;
    unsigned long _smSpeedPulseStart, _smTriggerPulseStart;
    bool _wasReady; unsigned long _readyStartTime;
    
    // Software polling state variables
    int _lastFlickerSpeed, _lastFlickerTrigger;
    int _lastSpeedRead, _lastTriggerRead;
    unsigned long _speedDebounceTime, _triggerDebounceTime;
};

// ==========================================
// 7. LED CONTROLLER (Native or WLED API)
// ==========================================
#ifdef USE_NATIVE_LEDS
class NativeLedController {
public:
    enum State { NORMAL, STRIKE, PAUSED, FILL, COMET };
    NativeLedController() : _state(NORMAL), _animStart(0), _lastFrame(0) {}

    void begin() {
        deckL.begin(); deckR.begin(); laneL.begin(); laneR.begin();
        deckL.setBrightness(DECK_LED_BRIGHTNESS);
        deckR.setBrightness(DECK_LED_BRIGHTNESS);
        laneL.setBrightness(LED_BRIGHTNESS_NORMAL);
        laneR.setBrightness(LED_BRIGHTNESS_NORMAL);
        changeState(NORMAL);
    }

    void triggerPreset(int presetId) {
        if (presetId == WLED_PRESET_NORMAL) changeState(NORMAL);
        else if (presetId == WLED_PRESET_STRIKE) changeState(STRIKE);
        else if (presetId == WLED_PRESET_PAUSE) changeState(PAUSED);
        else if (presetId == WLED_PRESET_FILL) changeState(FILL);
        else if (presetId == WLED_PRESET_THROW) changeState(COMET);
        // Fallback for BOOTING/MAINTENANCE/SPARE is to do nothing (maintain current state)
    }
    
    void setPower(bool on) {
        if (!on) { setAll(0, 0, 0); showAll(); }
        else { changeState(_state); } // Restore current state
    }

    void update() {
        unsigned long now = millis();
        
        if (_state == STRIKE) {
            unsigned long elapsed = now - _animStart;
            if (elapsed >= 1000) { 
                changeState(NORMAL); 
            } else {
                // 3 flashes over 1000ms = 6 toggle phases of ~166ms each
                int phase = elapsed / (1000 / 6);
                if (phase % 2 == 0) setAll(255, 0, 0); // Red
                else setAll(0, 0, 0);                  // Off
                showAll();
            }
        }
        else if (_state == COMET) {
            unsigned long elapsed = now - _animStart;
            if (elapsed >= BALL_COMET_MS) { 
                changeState(NORMAL); 
            } else if (now - _lastFrame >= BALL_COMET_FRAME_MS) {
                _lastFrame = now;
                float t = (float)elapsed / (float)BALL_COMET_MS;
                
                // Clear lanes (Off)
                for(int i = 0; i < LANE_LED_LENGTH_L; i++) laneL.setPixelColor(i, 0);
                for(int i = 0; i < LANE_LED_LENGTH_R; i++) laneR.setPixelColor(i, 0);
                
                // Keep decks white
                for(int i = 0; i < DECK_LED_LENGTH_L; i++) deckL.setPixelColor(i, deckL.Color(255, 255, 255));
                for(int i = 0; i < DECK_LED_LENGTH_R; i++) deckR.setPixelColor(i, deckR.Color(255, 255, 255));

                int maxIndexL = LANE_LED_LENGTH_L + COMET_LEN;
                int maxIndexR = LANE_LED_LENGTH_R + COMET_LEN;
                int headL = (int)(t * maxIndexL + 0.5f);
                int headR = (int)(t * maxIndexR + 0.5f);

                for(int k = 0; k < COMET_LEN; k++) {
                    int idxL = headL - k;
                    int idxR = headR - k;
                    if(idxL >= 0 && idxL < LANE_LED_LENGTH_L) laneL.setPixelColor(idxL, laneL.Color(255, 255, 255));
                    if(idxR >= 0 && idxR < LANE_LED_LENGTH_R) laneR.setPixelColor(idxR, laneR.Color(255, 255, 255));
                }
                showAll();
            }
        }
    }

private:
    State _state;
    unsigned long _animStart;
    unsigned long _lastFrame;

    void changeState(State s) {
        _state = s;
        _animStart = millis();
        _lastFrame = 0;
        
        if (s == NORMAL) { setAll(255, 255, 255); showAll(); }      // All White
        else if (s == PAUSED) { setAll(0, 0, 50); showAll(); }      // Dim Blue
        else if (s == FILL) { setAll(0, 255, 0); showAll(); }       // All Green
    }

    void setAll(uint8_t r, uint8_t g, uint8_t b) {
        for(int i=0; i<DECK_LED_LENGTH_L; i++) deckL.setPixelColor(i, deckL.Color(r,g,b));
        for(int i=0; i<DECK_LED_LENGTH_R; i++) deckR.setPixelColor(i, deckR.Color(r,g,b));
        for(int i=0; i<LANE_LED_LENGTH_L; i++) laneL.setPixelColor(i, laneL.Color(r,g,b));
        for(int i=0; i<LANE_LED_LENGTH_R; i++) laneR.setPixelColor(i, laneR.Color(r,g,b));
    }
    
    void showAll() {
        deckL.show(); deckR.show(); laneL.show(); laneR.show();
    }
};

#else
// WLED Fallback Class (Fire & Forget JSON API)
// ==========================================
// Configurable Hardware Serial implementation.
// Recommended wiring: Mega TX1 (Pin 18) -> ESP8266 RX
// Preset definitions are located in general_config.h
// ==========================================
class WledController {
public:
    WledController(HardwareSerial& serialPort) : _serial(serialPort) {}

    void begin() {
        _serial.begin(115200);
        tPrint("WLED Controller Initialized.");
    }

    void triggerPreset(int presetId) {
        _serial.print("{\"ps\":");
        _serial.print(presetId);
        _serial.println("}");
        tPrint("WLED: Fired Preset " + String(presetId));
    }
    
    void setPower(bool on) {
        if (on) _serial.println("{\"on\":true}");
        else _serial.println("{\"on\":false}");
    }
    
    void update() { 
        // WLED handles its own animations, so this is non-blocking empty logic
    }

private:
    HardwareSerial& _serial; 
};
#endif


// ==========================================
// CONTROLLER INSTANTIATIONS
// ==========================================
TurretController Turret(stepper1, IR_SENSOR_PIN, HALL_EFFECT_PIN, MOTOR_RELAY_PIN, STEPPER_ENABLE_PIN);
SweepController Sweep(LeftSweepServo, RightSweepServo);
DeckController Deck(LeftRaiseServo, RightRaiseServo, SlideServo, ScissorsServo);
BallReturnController Door(BallReturnServo, MOTOR_RELAY_PIN);
BallSensorController BallSensor(BALL_SPEED_PIN, BALL_SENSOR_PIN);
ButtonController ResetBtn(PINSETTER_RESET_PIN);
#ifdef USE_NATIVE_LEDS
NativeLedController Wled;
#else
WledController Wled(Serial1);
#endif

// ==========================================
// GAME ORCHESTRATOR FSM
// ==========================================
class GameOrchestrator {
public:
    enum OpMode { MODE_NORMAL, MODE_MAINT_ENTERING, MODE_MAINT_ACTIVE };
    enum MaintState { MAINT_START, MAINT_WAIT_SWEEP, MAINT_WAIT_DECK };

    enum State {
        BOOT_INIT, BOOT_SWEEP_1_GUARD, BOOT_SWEEP_1_BACK, BOOT_SWEEP_1_GUARD_POST,
        BOOT_DECK_CLEAR_1, BOOT_DECK_CLEAR_1_WAIT, BOOT_SWEEP_2_BACK, BOOT_SWEEP_2_GUARD,
        BOOT_WAIT_TURRET_HOME, BOOT_TURRET_PURGE_WAIT, BOOT_PURGE_DWELL,
        BOOT_DECK_CLEAR_2, BOOT_DECK_CLEAR_2_WAIT, BOOT_SWEEP_3_BACK, BOOT_SWEEP_3_GUARD,
        
        BOOT_WAIT_TURRET_IDLE_1, 
        BOOT_START_PRIME, BOOT_WAIT_TURRET_9, BOOT_TURRET_PRIME_DROP, BOOT_PRIME_DWELL, BOOT_DECK_SET_LANE, 
        BOOT_FINAL_SWEEP_UP, 
        BOOT_WAIT_TURRET_IDLE_2, 
        
        WAITING_FOR_BALL,
        
        T1_SWEEP_GUARD_FIRST, T1_PIN_SETTLE_WAIT, T1_DECK_GRAB, T1_SWEEP_BACK, T1_SWEEP_GUARD_POST, T1_DECK_DROP, T1_SWEEP_UP, T1_WAIT_TURRET_IDLE,
        T2_SWEEP_GUARD_FIRST, T2_PIN_SETTLE_WAIT, T2_SWEEP_BACK, RESET_SWEEP_GUARD_POST, 
        
        RESET_DECK_SET, RESET_SWEEP_UP, RESET_WAIT_TURRET_IDLE, 
        RESET_WAIT_BACKGROUND_DROP, 
        RESET_WAIT_TURRET_9, RESET_TURRET_DROPPING, RESET_BUFFER_DWELL,

        MANUAL_RESET_START, MANUAL_RESET_BACK 
    };

    GameOrchestrator() : _state(BOOT_INIT), _throwCount(1), _stateStart(0), 
                         _deckHasPins(false), _backgroundDropActive(false), _backgroundRestartPending(false),
                         _opMode(MODE_NORMAL), _isPaused(false), _lastActivityMs(0) {}

    void update() {

        if (_opMode == MODE_MAINT_ENTERING) {
            switch (_maintState) {
                case MAINT_START:
                    Turret.emergencyHalt(); 
                    Door.emergencyClose();
                    Deck.forceScissorsOpen(); 
                    Sweep.commandPose(SweepController::GUARD);
                    _maintState = MAINT_WAIT_SWEEP;
                    break;
                case MAINT_WAIT_SWEEP:
                    if (Sweep.isIdle()) {
                        Deck.forceMaintenanceHeight();
                        _maintState = MAINT_WAIT_DECK;
                        _stateStart = millis(); 
                    }
                    break;
                case MAINT_WAIT_DECK:
                    if (millis() - _stateStart >= 1000) { 
                        Deck.detachSlide();
                        Turret.disableStepper();
                        _opMode = MODE_MAINT_ACTIVE;
                        DEBUG_PRINT("Maintenance Mode ACTIVE. Stepper/Slide detached. Safe to clear jams.");
                    }
                    break;
            }
            return; 
        }

        if (_opMode == MODE_MAINT_ACTIVE) return; 

        if (_state == WAITING_FOR_BALL && !Turret.isHoming() && !_isPaused) {
            if (millis() - _lastActivityMs > 120000) { 
                Sweep.commandPose(SweepController::GUARD);
                _isPaused = true;
                Wled.triggerPreset(WLED_PRESET_PAUSE);
                DEBUG_PRINT("Idle timeout. System Paused. Sweep Guard deployed.");
            }
        }
        if (_isPaused) return; 
        
        if (!_deckHasPins && !_backgroundDropActive && !_backgroundRestartPending && Turret.isPausedAt9() && Deck.isIdle() && _state == WAITING_FOR_BALL) {
            Turret.commandExecuteDropSequence();
            _backgroundDropActive = true; 
        }
        if (_backgroundDropActive && Turret.isDropComplete()) {
            _deckHasPins = true;
            _backgroundDropActive = false; 
            _backgroundRestartPending = true; 
        }
        if (_backgroundRestartPending && Turret.isIdleEmpty()) {
            Turret.commandStartRefill();
            _backgroundRestartPending = false; 
        }

        switch (_state) {
            case BOOT_INIT:
                if (Sweep.isIdle() && Deck.isIdle()) { Sweep.commandPose(SweepController::GUARD); Wled.triggerPreset(WLED_PRESET_BOOTING); changeState(BOOT_SWEEP_1_GUARD); DEBUG_PRINT("Boot: Step 1 - Initial Sweep Guard"); }
                break;
            case BOOT_SWEEP_1_GUARD:
                if (Sweep.isIdle()) { Sweep.commandPose(SweepController::BACK); changeState(BOOT_SWEEP_1_BACK); DEBUG_PRINT("Boot: Step 2 - Sweep Back"); }
                break;
            case BOOT_SWEEP_1_BACK:
                if (Sweep.isIdle()) { Sweep.commandPose(SweepController::GUARD); changeState(BOOT_SWEEP_1_GUARD_POST); }
                break;
            case BOOT_SWEEP_1_GUARD_POST:
                if (Sweep.isIdle()) { Deck.commandSetPins(); changeState(BOOT_DECK_CLEAR_1_WAIT); DEBUG_PRINT("Boot: Step 3 - Deck Clear 1"); }
                break;
            case BOOT_DECK_CLEAR_1_WAIT:
                if (Deck.isIdle()) { Sweep.commandPose(SweepController::BACK); changeState(BOOT_SWEEP_2_BACK); DEBUG_PRINT("Boot: Step 4 - Sweep Clear 1"); }
                break;
            case BOOT_SWEEP_2_BACK:
                if (Sweep.isIdle()) { Sweep.commandPose(SweepController::GUARD); changeState(BOOT_SWEEP_2_GUARD); }
                break;
            case BOOT_SWEEP_2_GUARD:
                if (Sweep.isIdle()) { changeState(BOOT_WAIT_TURRET_HOME); DEBUG_PRINT("Boot: Step 5 - Waiting for Turret Home"); }
                break;
            case BOOT_WAIT_TURRET_HOME:
                if (Turret.isIdleEmpty()) { Turret.commandPurge(); changeState(BOOT_TURRET_PURGE_WAIT); DEBUG_PRINT("Boot: Step 6 - Turret Purge"); }
                break;
            case BOOT_TURRET_PURGE_WAIT:
                if (Turret.isDropComplete()) { changeState(BOOT_PURGE_DWELL); }
                break;
            case BOOT_PURGE_DWELL:
                if (millis() - _stateStart >= 1500) { Deck.commandSetPins(); changeState(BOOT_DECK_CLEAR_2_WAIT); DEBUG_PRINT("Boot: Step 7 - Deck Clear 2"); }
                break;
            case BOOT_DECK_CLEAR_2_WAIT:
                if (Deck.isIdle()) { Sweep.commandPose(SweepController::BACK); changeState(BOOT_SWEEP_3_BACK); DEBUG_PRINT("Boot: Step 8 - Sweep Clear 2"); }
                break;
            case BOOT_SWEEP_3_BACK:
                if (Sweep.isIdle()) { Sweep.commandPose(SweepController::GUARD); changeState(BOOT_SWEEP_3_GUARD); }
                break;
            case BOOT_SWEEP_3_GUARD:
                if (Sweep.isIdle()) { changeState(BOOT_WAIT_TURRET_IDLE_1); DEBUG_PRINT("Boot: Step 9 - Waiting for Turret to finish Homing"); }
                break;

            case BOOT_WAIT_TURRET_IDLE_1:
                if (Turret.isIdleEmpty()) {
                    Turret.commandStartRefill(); 
                    changeState(BOOT_WAIT_TURRET_9); 
                }
                break;

            case BOOT_WAIT_TURRET_9:
                if (Turret.isPausedAt9()) { Turret.commandExecuteDropSequence(); changeState(BOOT_TURRET_PRIME_DROP); DEBUG_PRINT("Boot: Dropping 10 into empty Deck."); }
                break;
            case BOOT_TURRET_PRIME_DROP:
                if (Turret.isDropComplete()) { changeState(BOOT_PRIME_DWELL); }
                break;
            case BOOT_PRIME_DWELL:
                if (millis() - _stateStart >= 1500) { 
                    Deck.commandSetPins(); 
                    _deckHasPins = false; 
                    changeState(BOOT_DECK_SET_LANE); 
                }
                break;

            case BOOT_DECK_SET_LANE:
                if (Deck.isIdle()) { 
                    Sweep.commandPose(SweepController::UP); 
                    changeState(BOOT_FINAL_SWEEP_UP); 
                }
                break;
            case BOOT_FINAL_SWEEP_UP:
                if (Sweep.isIdle()) { 
                    changeState(BOOT_WAIT_TURRET_IDLE_2); 
                }
                break;

            case BOOT_WAIT_TURRET_IDLE_2:
                if (Turret.isIdleEmpty()) {
                    Turret.commandStartRefill();
                    _throwCount = 1; 
                    changeState(WAITING_FOR_BALL); 
                    updateFrameLights();
                    DEBUG_PRINT("Boot Complete: Ready for Throw 1."); 
                }
                break;

            case WAITING_FOR_BALL: break;

            case T1_SWEEP_GUARD_FIRST:
                if (Sweep.isIdle()) { changeState(T1_PIN_SETTLE_WAIT); }
                break;
            case T1_PIN_SETTLE_WAIT:
                if (millis() - _stateStart >= 3000) { 
                    if (Turret.isCommittedToDrop()) break; 
                    Deck.commandGrabPins(); 
                    changeState(T1_DECK_GRAB); 
                }
                break;
            case T1_DECK_GRAB:
                if (Deck.isIdle()) { Sweep.commandPose(SweepController::BACK); changeState(T1_SWEEP_BACK); }
                break;
            case T1_SWEEP_BACK:
                if (Sweep.isIdle()) { Sweep.commandPose(SweepController::GUARD); changeState(T1_SWEEP_GUARD_POST); }
                break;
            case T1_SWEEP_GUARD_POST:
                if (Sweep.isIdle()) { Deck.commandDropPins(); changeState(T1_DECK_DROP); }
                break;
            case T1_DECK_DROP:
                if (Deck.isIdle()) { Sweep.commandPose(SweepController::UP); changeState(T1_SWEEP_UP); }
                break;
            case T1_SWEEP_UP:
                if (Sweep.isIdle()) { changeState(T1_WAIT_TURRET_IDLE); }
                break;
            case T1_WAIT_TURRET_IDLE:
                if (_backgroundRestartPending) {
                    Door.triggerCycle(); 
                    _throwCount = 2; 
                    changeState(WAITING_FOR_BALL); 
                }
                else if (Turret.isIdleEmpty()) {
                    Turret.commandStartRefill(); 
                    Door.triggerCycle(); 
                    _throwCount = 2; 
                    changeState(WAITING_FOR_BALL); 
                }
                else if (!Turret.isHoming() && !Turret.isCommittedToDrop()) {
                    Turret.commandWake(); 
                    Door.triggerCycle(); 
                    _throwCount = 2; 
                    changeState(WAITING_FOR_BALL); 
                }
                updateFrameLights();
                break;

            case T2_SWEEP_GUARD_FIRST:
                if (Sweep.isIdle()) { changeState(T2_PIN_SETTLE_WAIT); }
                break;
            case T2_PIN_SETTLE_WAIT:
                if (millis() - _stateStart >= 3000) { Sweep.commandPose(SweepController::BACK); changeState(T2_SWEEP_BACK); }
                break;
            case T2_SWEEP_BACK:
                if (Sweep.isIdle()) { Sweep.commandPose(SweepController::GUARD); changeState(RESET_SWEEP_GUARD_POST); }
                break;

            case RESET_SWEEP_GUARD_POST:
                if (Sweep.isIdle() && Deck.isIdle()) { 
                    if (_deckHasPins) {
                        if (Turret.isCommittedToDrop()) break; 
                        Deck.commandSetPins(); 
                        _deckHasPins = false; 
                        changeState(RESET_DECK_SET); 
                    } else if (_backgroundDropActive) {
                        changeState(RESET_WAIT_BACKGROUND_DROP); 
                    } else {
                        changeState(RESET_WAIT_TURRET_9); 
                    }
                }
                break;

            case RESET_WAIT_BACKGROUND_DROP:
                if (_deckHasPins) {
                    Deck.commandSetPins();
                    _deckHasPins = false;
                    changeState(RESET_DECK_SET);
                }
                break;

            case RESET_DECK_SET:
                if (Deck.isIdle()) { 
                    Sweep.commandPose(SweepController::UP); 
                    changeState(RESET_SWEEP_UP); 
                }
                break;
            case RESET_SWEEP_UP:
                if (Sweep.isIdle()) {
                    changeState(RESET_WAIT_TURRET_IDLE); 
                }
                break;
                
            case RESET_WAIT_TURRET_IDLE:
                if (_backgroundRestartPending) {
                    Door.triggerCycle(); _throwCount = 1; changeState(WAITING_FOR_BALL); 
                }
                else if (Turret.isIdleEmpty()) {
                    Turret.commandStartRefill(); Door.triggerCycle(); _throwCount = 1; clearFillBall(); changeState(WAITING_FOR_BALL); 
                }
                else if (!Turret.isHoming() && !Turret.isCommittedToDrop()) {
                    Turret.commandWake(); Door.triggerCycle(); _throwCount = 1; changeState(WAITING_FOR_BALL); 
                }
                updateFrameLights();
                break;

            case RESET_WAIT_TURRET_9:
                if (Turret.isPausedAt9()) { Turret.commandExecuteDropSequence(); changeState(RESET_TURRET_DROPPING); }
                break;
            case RESET_TURRET_DROPPING:
                if (Turret.isDropComplete()) { changeState(RESET_BUFFER_DWELL); }
                break;
            case RESET_BUFFER_DWELL:
                if (millis() - _stateStart >= 1500) { Deck.commandSetPins(); _deckHasPins = false; changeState(RESET_DECK_SET); }
                break;

            case MANUAL_RESET_START:
                if (Sweep.isIdle()) { Sweep.commandPose(SweepController::BACK); changeState(MANUAL_RESET_BACK); }
                break;
            case MANUAL_RESET_BACK:
                if (Sweep.isIdle()) { Sweep.commandPose(SweepController::GUARD); changeState(RESET_SWEEP_GUARD_POST); }
                break;
        }
    }

    void updateFrameLights() {
        // 1. Set physical hardware lights based strictly on throw count
        if (_throwCount == 2) {
            digitalWrite(FRAME_LED1_PIN, HIGH);
            digitalWrite(FRAME_LED2_PIN, HIGH);
        } else {
            digitalWrite(FRAME_LED1_PIN, HIGH);
            digitalWrite(FRAME_LED2_PIN, LOW);
        }
        // 2. Transmit exactly one WLED preset based on priority
        if (_fillBallArmed) {
            Wled.triggerPreset(WLED_PRESET_FILL);
        } else if (_throwCount == 2) {
            Wled.triggerPreset(WLED_PRESET_BALL2);
        } else {
            Wled.triggerPreset(WLED_PRESET_BALL1);
        }
    }

    void updateActivity() { _lastActivityMs = millis(); }
    bool isPaused() const { return _isPaused; }

    void wakeFromPause() {
        if (_isPaused) {
            Sweep.commandPose(SweepController::UP);
            _isPaused = false;
            updateActivity();
            updateFrameLights();
            DEBUG_PRINT("System Waking. Sweep Raised.");
        }
    }

    // --> NEW: Arm the fill ball when commanded by ScoreMore
    void armFillBall() { 
        _fillBallArmed = true; 
        updateFrameLights(); // Fire the preset immediately
        DEBUG_PRINT("Fill Ball Armed by ScoreMore: Next throw will clear the deck."); 
    }

    void clearFillBall() {
        _fillBallArmed = false;
    }

    void triggerBall() {
        updateActivity();
        if (_state != WAITING_FOR_BALL) return;
        Wled.triggerPreset(WLED_PRESET_THROW);

        if (_backgroundDropActive) {
            if (Turret.commandSuspendFetch()) {
                _backgroundDropActive = false;
            }
        }

        // Let the machine behave naturally based on _throwCount.
        // Strike aborts are handled asynchronously by triggerStrike().
        if (_throwCount == 1) { 
            Sweep.commandPose(SweepController::GUARD);
            changeState(T1_SWEEP_GUARD_FIRST); 
        } 
        else { 
            Sweep.commandPose(SweepController::GUARD);
            changeState(T2_SWEEP_GUARD_FIRST); 
        }
    }
    
    void triggerStrike() {
        if (_opMode != MODE_NORMAL) return;
        
        DEBUG_PRINT("Strike Detected by ScoreMore! Aborting Throw 1 and Re-racking...");
        _throwCount = 1; 
        // REMOVED _fillBallArmed = false; // Reset fill state if a strike interrupts
        Sweep.commandPose(SweepController::GUARD);
        changeState(MANUAL_RESET_START); 
    }

    void triggerLaneReset() {
        if (_opMode != MODE_NORMAL || _state != WAITING_FOR_BALL) {
            DEBUG_PRINT("Short Press Ignored: System is actively cycling or not in Normal Mode.");
            return;
        }
        
        updateActivity();
        _throwCount = 1; 
        _fillBallArmed = false;
        Sweep.commandPose(SweepController::GUARD);
        changeState(MANUAL_RESET_START);
        DEBUG_PRINT("Short Press: Aborting Frame. Commencing Lane Reset Sweep.");
    }

    void triggerMaintenanceMode() {
        if (_opMode != MODE_NORMAL) return;
        _opMode = MODE_MAINT_ENTERING;
        _maintState = MAINT_START;
        Wled.triggerPreset(WLED_PRESET_MAINTENANCE);
        DEBUG_PRINT("Long Press: Entering Maintenance Mode...");
    }

    void exitMaintenanceMode() {
        if (_opMode != MODE_MAINT_ACTIVE) return;
        DEBUG_PRINT("Short Press: Exiting Maintenance Mode. Re-homing System...");
        
        Turret.enableStepper();
        Deck.attachSlide();
        Turret.begin(); Deck.begin(); Sweep.begin(); Door.begin();
        
        _deckHasPins = false; _backgroundDropActive = false; _backgroundRestartPending = false;
        _throwCount = 1; _fillBallArmed = false;

        // Reset the inactivity timer to prevent immediate pausing
        updateActivity();
        
        _opMode = MODE_NORMAL;
        changeState(BOOT_INIT);
    }
    
    bool isMaintenanceActive() const { return _opMode == MODE_MAINT_ACTIVE; }
    bool isWaitingForBall() const { return _state == WAITING_FOR_BALL && !_isPaused; }

private:
    State _state; 
    int _throwCount;
    unsigned long _stateStart; 
    bool _deckHasPins; 
    bool _backgroundDropActive; 
    bool _backgroundRestartPending; 
    bool _fillBallArmed; // <-- NEW
    
    OpMode _opMode;
    MaintState _maintState;
    bool _isPaused;
    unsigned long _lastActivityMs;

    void changeState(State s) { 
        _state = s; 
        _stateStart = millis(); 
    }
};

GameOrchestrator Game;

// ==========================================
// 8. SCOREMORE SERIAL CONTROLLER
// ==========================================
class ScoreMoreController {
public:
    ScoreMoreController() : _index(0) {
        _buffer[0] = '\0';
    }

    void update() {
        if ((_pendingStrike || _pendingSpare) && (millis() - _animWindowStart > EVAL_WINDOW_MS)) {
            if (_pendingStrike) {
                DEBUG_PRINT("Executing Buffered STRIKE");
                Game.triggerStrike();
                Wled.triggerPreset(WLED_PRESET_STRIKE);
            } else if (_pendingSpare) {
                DEBUG_PRINT("Executing Buffered SPARE");
                Game.triggerLaneReset();
                Wled.triggerPreset(WLED_PRESET_SPARE);
            }
            // Clear the buffer
            _pendingStrike = false;
            _pendingSpare = false;
        }

        while (Serial.available()) {
            char c = Serial.read();

            if (c == '\n') {
                _buffer[_index] = '\0'; 
                String cmd = String(_buffer);
                cmd.trim();
                
                if (cmd.length() > 0) {
                    handleCommand(cmd);
                }
                _index = 0;
            } 
            else if (c >= 32 && c <= 126) {
                if (_index < sizeof(_buffer) - 1) {
                    _buffer[_index++] = c;
                } else {
                    _index = 0; // Overflow protection
                }
            }
        }
    }

    void sendResetPulse() {
        // Sends the hardware reset command back to the PC to clear the screen
        Serial.print("INPUT_CHANGE:"); 
        Serial.print(SM_PINSETTER_RESET); 
        Serial.println(":0");
        
        delay(50); // Minor blocking is safe here since the machine is halting to reset
        
        Serial.print("INPUT_CHANGE:"); 
        Serial.print(SM_PINSETTER_RESET); 
        Serial.println(":1");
    }

private:
    char _buffer[100];
    uint8_t _index;
    unsigned long _animWindowStart = 0;
    bool _pendingStrike = false;
    bool _pendingSpare = false;
    const unsigned long EVAL_WINDOW_MS = 150;

    void handleCommand(String cmd) {
        if (cmd.length() == 1) {
            char c = cmd[0];
            if (c == 't') {
                Game.updateActivity();
                if (Game.isPaused()) Game.wakeFromPause();
                else { Door.emergencyClose(); Game.triggerBall(); }
            }
            else if (c == 'r') {
                Game.triggerLaneReset();
                sendResetPulse(); 
            }
            else if (c == 'm') Game.triggerMaintenanceMode();
            return;
        }

        if (cmd.startsWith("WRITE:")) {
            int firstColon = cmd.indexOf(':');
            int secondColon = cmd.indexOf(':', firstColon + 1);
            
            if (firstColon != -1 && secondColon != -1) {
                int smPin = cmd.substring(firstColon + 1, secondColon).toInt();
                int val = cmd.substring(secondColon + 1).toInt();

                // --> NEW: Explicit physical mapping for Frame LEDs
                if (smPin == SM_FIRST_BALL) {
                    // Decoupled: Now Managed by GameOrchestrator
                    //digitalWrite(FRAME_LED1_PIN, val);
                    //if (val == 1) Wled.triggerPreset(WLED_PRESET_BALL1);
                }
                else if (smPin == SM_SECOND_BALL) {
                    // Decoupled: Now Managed by GameOrchestrator
                    //digitalWrite(FRAME_LED2_PIN, val);
                    //if (val == 1) Wled.triggerPreset(WLED_PRESET_BALL2);
                }
                else if (smPin == SM_AUTO_RESET) {
                    if (val == 1) {
                        Game.armFillBall(); // Arm the Throw 2 cycle for the 5th/10th frame
                        Wled.triggerPreset(WLED_PRESET_FILL);
                    }
                }
                else if (smPin == SM_STRIKE_LIGHT) {
                    digitalWrite(smPin, val);
                    if (val == 1) {
                        // Start the timer if it isn't already running
                        if (!_pendingStrike && !_pendingSpare) _animWindowStart = millis();
                        _pendingStrike = true;
                    }
                }
                else {
                    // Standard hardware pin drive for Spare, Miss, etc.
                    digitalWrite(smPin, val);
                    if (val == 1) {
                        // Start the timer if it isn't already running
                        if (!_pendingStrike && !_pendingSpare) _animWindowStart = millis();
                        _pendingSpare = true;
                    }
                }
                
                Serial.print("ACK_WRITE:");
                Serial.print(smPin);
                Serial.print(":");
                Serial.println(val);
            }
        } 
        else if (cmd.startsWith("SET_INPUT:") || cmd.startsWith("SET_OUTPUT:")) {
            int firstColon = cmd.indexOf(':');
            int smPin = cmd.substring(firstColon + 1).toInt();
            
            if (cmd.startsWith("SET_INPUT:")) {
                pinMode(smPin, INPUT_PULLUP);
                Serial.print("ACK_SET_INPUT:");
            }
            else {
                pinMode(smPin, OUTPUT);
                Serial.print("ACK_SET_OUTPUT:");
            }
            Serial.println(smPin);
        }
        else if (cmd == "RESET") {
            digitalWrite(SM_STRIKE_LIGHT, LOW);
            digitalWrite(SM_SPARE_LIGHT, LOW);
            // Decoupled: Managed by GameOrchestrator
            //digitalWrite(FRAME_LED1_PIN, LOW); 
            //digitalWrite(FRAME_LED2_PIN, LOW); 
            Game.clearFillBall();
            Serial.println("ACK_RESET");
        }
        else if (cmd == "CHECK_READY") {
            Serial.println("READY");
        } 
        else if (cmd == "VERSION") {
            Serial.println("1.0.0"); 
        }
        else {
            Serial.println("ACK_UNKNOWN_COMMAND");
        }
    }
};

ScoreMoreController ScoreMoreSerial;

// ==========================================
// MAIN ARDUINO HOOKS
// ==========================================
void setup() {
    Serial.begin(SCOREMORE_BAUD);
    #ifdef USE_NATIVE_LEDS
    // Todo: Initialize Native Pins
    //deckL.begin(); deckR.begin(); laneL.begin(); laneR.begin();    
    // Push the initial 'off' state to clear any residual data
    //deckL.show(); deckR.show(); laneL.show(); laneR.show();
    #else
      // Initialize WLED Serial Logic
      Serial1.begin(115200);
    #endif
    delay(1000);
    
    #if SCOREMORE_USER == 1
        waitForScoreMore();
    #endif

    Serial.println("READY"); // CRITICAL: ScoreMore Handshake

    pinMode(FRAME_LED1_PIN, OUTPUT);
    pinMode(FRAME_LED2_PIN, OUTPUT);
    digitalWrite(FRAME_LED1_PIN, LOW);
    digitalWrite(FRAME_LED2_PIN, LOW);
    
    LeftRaiseServo.attach(RAISE_LEFT_PIN); RightRaiseServo.attach(RAISE_RIGHT_PIN);
    SlideServo.attach(SLIDE_PIN); ScissorsServo.attach(SCISSOR_PIN);
    LeftSweepServo.attach(LEFT_SWEEP_PIN); RightSweepServo.attach(RIGHT_SWEEP_PIN);
    BallReturnServo.attach(BALL_RETURN_PIN);
    
    Sweep.begin(); Deck.begin(); Turret.begin(); Door.begin(); BallSensor.begin(); ResetBtn.begin();
    Wled.begin();
    
    DEBUG_PRINT("Waiting 3 seconds for WLED to Boot...");
    delay(3000); // Give ESP32 time to wake up
    Wled.triggerPreset(WLED_PRESET_BOOTING);
    DEBUG_PRINT("Automated Kinematics Booted. Starting Dance.");
}

void loop() {
    Turret.update(); Sweep.update(); Deck.update(); Door.update(); Game.update(); 
    // Update the LEDs either with neopixel natively or with a WLED controller:
    Wled.update();
    // Process incoming serial strings non-stop
    ScoreMoreSerial.update(); 

    ButtonController::Event btnEvent = ResetBtn.update();
    if (btnEvent == ButtonController::SHORT_PRESS) {
        if (Game.isMaintenanceActive()) {
            Game.exitMaintenanceMode();
        } else if (Game.isPaused()) {
            Game.wakeFromPause();
        } else {
            Game.triggerLaneReset();
            ScoreMoreSerial.sendResetPulse(); // Tell PC that lane was reset
        }
    } else if (btnEvent == ButtonController::LONG_PRESS) {
        Game.triggerMaintenanceMode();
    }

    if (BallSensor.updateAndCheck(Game.isWaitingForBall())) {
        Game.updateActivity();
        if (Game.isPaused()) {
            Game.wakeFromPause();
        } else {
            DEBUG_PRINT("HARDWARE TRIGGER: IR Beam Broken by Ball!");
            Door.emergencyClose(); 
            Game.triggerBall();
        }
    }
}
