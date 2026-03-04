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

// ==========================================
// GLOBAL HARDWARE & TELEMETRY
// ==========================================
Servo LeftRaiseServo, RightRaiseServo, SlideServo, ScissorsServo, LeftSweepServo, RightSweepServo, BallReturnServo;
AccelStepper stepper1(1, STEP_PIN, DIR_PIN);

void tPrint(const String& msg) {
    Serial.print("["); Serial.print(millis()); Serial.print(" ms] "); Serial.println(msg);
}

// ==========================================
// 1. TURRET CONTROLLER FSM (CONCURRENT READY)
// Features Dynamic 10th Pin Drop Rotation
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
        MOVING_TO_DROP, DWELLING_AT_DROP, // FIX: Pushing and Dwelling 10th states removed!
        MOVING_TO_PURGE,
        SLEEP_STARVED_WAIT, SLEEP_STARVED_FETCH
    };

    TurretController(AccelStepper& stp, int ir, int hall, int relay)
        : _stepper(stp), _irPin(ir), _hallPin(hall), _relayPin(relay),
          _state(UNHOMED), _loadedCount(0), _targetPos(0), _dropComplete(false), 
          _queuedPins(0), _lastPinDetectTime(0) {}

    void begin() {
        pinMode(_irPin, INPUT_PULLUP); pinMode(_hallPin, INPUT_PULLUP); pinMode(_relayPin, OUTPUT);
        conveyorOff();
        _stepper.setMaxSpeed(TURRET_NORMAL_MAXSPEED); _stepper.setAcceleration(TURRET_NORMAL_ACCEL);
        _irStableState = HIGH; _irLastRead = HIGH;
        changeState(UNHOMED);
    }

    void update() {
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
                    if (digitalRead(_hallPin) == LOW) { _stepper.setCurrentPosition(TURRET_HOME_ADJUSTER); _stepper.setMaxSpeed(TURRET_NORMAL_MAXSPEED); _loadedCount = 0; _queuedPins = 0; changeState(IDLE_EMPTY); }
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
                    tPrint("Turret Sleep: Pin Starvation. Awaiting Orchestrator Wake.");
                    changeState(SLEEP_STARVED_WAIT);
                    break;
                }

                if (_queuedPins > 0) {
                    _queuedPins--; 
                    _loadedCount++; 
                    tPrint("Caught Pin " + String(_loadedCount) + " | In Queue: " + String(_queuedPins));
                    changeState(CATCH_DELAY_PHASE, CATCH_DELAY_MS); 
                }
                break;

            case SLEEP_STARVED_WAIT:
                conveyorOff(); 
                break;

            case CATCH_DELAY_PHASE:
                conveyorOn();
                if (elapsed >= _timeout) {
                    if (_loadedCount == 9) {
                        tPrint("9 Pins Caught. Safely Pausing at Slot 9.");
                        changeState(PAUSED_AT_9);
                    } else {
                        _targetPos = getPositionForSlot(_loadedCount + 1);
                        _stepper.moveTo(_targetPos);
                        changeState(INDEXING_TO_NEXT);
                    }
                }
                break;

            case INDEXING_TO_NEXT: 
                conveyorOn();
                if (_stepper.distanceToGo() == 0) { 
                    changeState(WAITING_FOR_PIN); 
                } 
                break;

            case PAUSED_AT_9: 
                conveyorOff(); 
                break;

            case FETCH_10TH:
                conveyorOn(); 

                if (millis() - _lastPinDetectTime > NO_CATCH_TIMEOUT_MS) {
                    conveyorOff();
                    tPrint("Turret Sleep: 10th Pin Starved. Awaiting Orchestrator Wake.");
                    changeState(SLEEP_STARVED_FETCH);
                    break;
                }

                if (_queuedPins > 0) {
                    _queuedPins--; 
                    _loadedCount = 10;
                    tPrint("10th Pin Sensed! Instantly rotating to drop while shoving 10th pin...");
                    
                    // =======================================================
                    // THE FIX: Immediate dynamic rotation. No software wait!
                    // =======================================================
                    _stepper.setMaxSpeed(TURRET_SPRING_MAXSPEED);
                    _stepper.setAcceleration(TURRET_SPRING_ACCEL);
                    _targetPos = getPositionForSlot(10) + TURRET_PIN10_RELEASE_OFFSET;
                    _stepper.moveTo(_targetPos);
                    
                    changeState(MOVING_TO_DROP); 
                }
                break;

            case SLEEP_STARVED_FETCH:
                conveyorOff(); 
                break;

            case MOVING_TO_DROP:
                // Keep the conveyor running while rotating to shove Pin 10 in!
                conveyorOn(); 
                if (_stepper.distanceToGo() == 0) { 
                    changeState(DWELLING_AT_DROP, RELEASE_DWELL_MS); 
                }
                break;

            case MOVING_TO_PURGE:
                if (_stepper.distanceToGo() == 0) { changeState(DWELLING_AT_DROP, RELEASE_DWELL_MS); }
                break;

            case DWELLING_AT_DROP:
                // Feed assist: ensure the 10th pin falls completely off the belt
                if (elapsed < RELEASE_FEED_ASSIST_MS) {
                    conveyorOn();
                } else {
                    conveyorOff(); 
                }

                if (elapsed >= _timeout) {
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
        _dropComplete = false; _loadedCount = 0; _queuedPins = 0;
        _lastPinDetectTime = millis(); 
        
        _targetPos = getPositionForSlot(1);
        _stepper.moveTo(_targetPos); 
        changeState(INDEXING_TO_NEXT); 
        return true;
    }

    bool commandExecuteDropSequence() {
        if (_state != PAUSED_AT_9) return false;
        _dropComplete = false;
        _lastPinDetectTime = millis(); 
        
        changeState(FETCH_10TH);
        tPrint("Drop Triggered: Waiting for 10th Pin...");
        return true;
    }

    bool commandWake() {
        if (_state == SLEEP_STARVED_WAIT) {
            _lastPinDetectTime = millis();
            changeState(WAITING_FOR_PIN);
            tPrint("Turret Woken: Conveyor resuming refill.");
            return true;
        } else if (_state == SLEEP_STARVED_FETCH) {
            _lastPinDetectTime = millis();
            changeState(FETCH_10TH);
            tPrint("Turret Woken: Conveyor resuming 10th pin fetch.");
            return true;
        }
        return false; 
    }

    bool commandPurge() {
        if (_state != IDLE_EMPTY) return false;
        _dropComplete = false;
        _stepper.setMaxSpeed(TURRET_SPRING_MAXSPEED);
        _stepper.setAcceleration(TURRET_SPRING_ACCEL);
        _targetPos = getPositionForSlot(10) + TURRET_PIN10_RELEASE_OFFSET + TURRET_EMPTY_EXTRA_OFFSET;
        _stepper.moveTo(_targetPos);
        changeState(MOVING_TO_PURGE);
        return true;
    }

    bool commandSuspendFetch() {
        if (_state == FETCH_10TH || _state == SLEEP_STARVED_FETCH) {
            changeState(PAUSED_AT_9);
            _lastPinDetectTime = millis();
            tPrint("Turret: Fetch aborted by Orchestrator. Reverting to safe Pause.");
            return true;
        }
        return false; 
    }
    
    bool isIdleEmpty() const { return _state == IDLE_EMPTY; }
    bool isPausedAt9() const { return _state == PAUSED_AT_9; }
    bool isDropComplete() const { return _dropComplete; }

    bool isHoming() const { 
        return (_state == UNHOMED || _state == HOMING_PROBE_CCW || 
                _state == HOMING_FAST_FWD || _state == HOMING_BACKOFF || 
                _state == HOMING_CREEP || _state == HOMING_BLIND_ESCAPE); 
    }

    // FIX: Updated shield states
    bool isCommittedToDrop() const {
        return (_state == MOVING_TO_DROP || _state == DWELLING_AT_DROP || _state == MOVING_TO_PURGE);
    }

private:
    AccelStepper& _stepper; int _irPin, _hallPin, _relayPin; State _state; 
    unsigned long _stateStart, _timeout; int _loadedCount; long _targetPos;
    int _irLastRead, _irStableState; unsigned long _irLastChange;
    bool _dropComplete; int _queuedPins;
    
    unsigned long _lastPinDetectTime; 
    const unsigned long PIN_TRANSIT_LOCKOUT_MS = 400; 

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
                        
                        if (_state != UNHOMED && _state != HOMING_PROBE_CCW && 
                            _state != HOMING_BLIND_ESCAPE && _state != HOMING_FAST_FWD && 
                            _state != HOMING_BACKOFF && _state != HOMING_CREEP && 
                            _state != DWELLING_AT_DROP && _state != MOVING_TO_PURGE &&
                            _state != SLEEP_STARVED_WAIT && _state != SLEEP_STARVED_FETCH) {
                            
                            _queuedPins++;
                        }
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
// 2. SWEEP CONTROLLER FSM
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
private:
    Servo& _left; Servo& _right; State _state; Pose _currentPose; Pose _targetPose;
    unsigned long _tweenStart; int _startL, _targetL, _curL;
};

// ==========================================
// 3. DECK CONTROLLER FSM
// ==========================================
class DeckController {   
public:
    enum State { IDLE_UP, SETTLING_PINS_DOWN, SETTLING_PINS_UP, GRABBING_DOWN, GRABBING_UP, DROPPING_DOWN, DROPPING_UP };
    DeckController(Servo& lr, Servo& rr, Servo& sl, Servo& sc) : _lr(lr), _rr(rr), _slide(sl), _scissors(sc), _state(IDLE_UP) {}

    void begin() {
        _slide.write(SLIDER_HOME_ANGLE); _scissors.write(SCISSOR_DROP_ANGLE);
        setRaise(RAISE_UP_ANGLE); _state = IDLE_UP;
    }

    void update() {
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

private:
    Servo& _lr; Servo& _rr; Servo& _slide; Servo& _scissors; State _state; unsigned long _stateStart, _timeout;
    void changeState(State s, unsigned long to = 0) { _state = s; _stateStart = millis(); _timeout = to; }
    void setRaise(int ang) { _lr.write(ang); _rr.write(180 - ang); }
};

// ==========================================
// 4. BALL RETURN CONTROLLER FSM
// ==========================================
class BallReturnController {
public:
    enum State { CLOSED, WAITING_PIN_CLEAR, OPEN };

    BallReturnController(Servo& servo, int relayPin) 
        : _servo(servo), _relayPin(relayPin), _state(CLOSED), _stateStart(0) {}

    void begin() {
        closeDoor();
    }

    void update() {
        unsigned long elapsed = millis() - _stateStart;
        bool conveyorRunning = (digitalRead(_relayPin) == (CONVEYOR_ACTIVE_HIGH ? HIGH : LOW));

        switch (_state) {
            case CLOSED: 
                break;
                
            case WAITING_PIN_CLEAR:
                // User calibrated: 3 seconds for pin dam to clear
                if (elapsed >= 3000) {
                    openDoor();
                    changeState(OPEN);
                    tPrint("Ball Return: Pins cleared (3s). Door OPEN.");
                }
                break;
                
            case OPEN:
                // User calibrated: 15 seconds max open window
                if (elapsed >= 15000) {
                    closeDoor();
                    changeState(CLOSED);
                    tPrint("Ball Return: 15s timeout. Door CLOSED.");
                } 
                // Hardware override: Conveyor shut off
                else if (!conveyorRunning) {
                    closeDoor();
                    changeState(CLOSED);
                    tPrint("Ball Return: Conveyor halted early. Door CLOSED.");
                }
                break;
        }
    }

    void triggerCycle() {
        if (_state == CLOSED) {
            changeState(WAITING_PIN_CLEAR);
            tPrint("Ball Return: Cycle triggered. Waiting 3s for pin dam to clear...");
        }
    }

    void emergencyClose() {
        if (_state != CLOSED) {
            closeDoor();
            changeState(CLOSED);
            tPrint("Ball Return: Fast throw detected! Emergency door close.");
        }
    }

private:
    Servo& _servo;
    int _relayPin;
    State _state;
    unsigned long _stateStart;

    void changeState(State s) { _state = s; _stateStart = millis(); }
    void openDoor() { _servo.write(BALL_DOOR_OPEN_ANGLE); }
    void closeDoor() { _servo.write(BALL_DOOR_CLOSED_ANGLE); }
};

// ==========================================
// 5. BALL SENSOR CONTROLLER
// Microsecond-level debounce and rearm logic
// ==========================================
class BallSensorController {
public:
    BallSensorController(int pin) : _pin(pin), _ballPrev(HIGH), _ballPending(false), _ballRearmed(true), _ballLowStartUs(0), _lastBallHighMs(0) {}

    void begin() {
        pinMode(_pin, INPUT_PULLUP);
        _ballPrev = HIGH;
    }

    bool updateAndCheck() {
        int rawBall = digitalRead(_pin);
        bool trippedThisFrame = false;

        // Detect the exact falling edge
        if (rawBall == LOW && _ballPrev == HIGH) {
            _ballPending = true;
            _ballLowStartUs = micros();
        }

        // Validate the low signal duration
        if (_ballPending) {
            if (rawBall == LOW) {
                // Has it been low long enough to be a ball (not a flying pin)?
                if ((micros() - _ballLowStartUs) >= BALL_LOW_CONFIRM_US && _ballRearmed) {
                    trippedThisFrame = true;
                    _ballRearmed = false;
                    _ballPending = false;
                    _lastBallHighMs = millis(); 
                }
            } else {
                // It bounced high too quickly. It was noise/debris.
                _ballPending = false; 
            }
        }

        // Rearm logic: Ensure the beam is clear for a solid duration before allowing the next throw
        if (!_ballRearmed && rawBall == HIGH) {
            if (millis() - _lastBallHighMs >= BALL_REARM_MS) {
                _ballRearmed = true;
            }
        } else if (rawBall == LOW) {
            // Keep resetting the rearm clock if a dead pin is just sitting in the beam
            _lastBallHighMs = millis(); 
        }

        _ballPrev = rawBall;
        return trippedThisFrame; // Returns true EXACTLY once per valid ball
    }

private:
    int _pin;
    int _ballPrev;
    bool _ballPending;
    bool _ballRearmed;
    unsigned long _ballLowStartUs;
    unsigned long _lastBallHighMs;
};

// ==========================================
// CONTROLLER INSTANTIATIONS
// ==========================================
TurretController Turret(stepper1, IR_SENSOR_PIN, HALL_EFFECT_PIN, MOTOR_RELAY_PIN);
SweepController Sweep(LeftSweepServo, RightSweepServo);
DeckController Deck(LeftRaiseServo, RightRaiseServo, SlideServo, ScissorsServo);
BallReturnController Door(BallReturnServo, MOTOR_RELAY_PIN);
BallSensorController BallSensor(BALL_SENSOR_PIN); // NEW

// ==========================================
// GAME ORCHESTRATOR FSM (CONCURRENT MANAGER)
// True Brunswick A2 Continuous Buffering with Fetch-Abort
// ==========================================
class GameOrchestrator {
public:
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
        RESET_WAIT_TURRET_9, RESET_TURRET_DROPPING, RESET_BUFFER_DWELL
    };

    GameOrchestrator() : _state(BOOT_INIT), _throwCount(1), _stateStart(0), 
                         _deckHasPins(false), _backgroundDropActive(false), _backgroundRestartPending(false) {}

    void update() {
        
        // =======================================================
        // BACKGROUND DECK BUFFER MANAGER (Unrestricted Continuous)
        // =======================================================
        // Restored: Fills the buffer during Throw 1 OR Throw 2
        if (!_deckHasPins && !_backgroundDropActive && !_backgroundRestartPending && Turret.isPausedAt9() && Deck.isIdle() && _state == WAITING_FOR_BALL) {
            Turret.commandExecuteDropSequence();
            _backgroundDropActive = true; 
            tPrint("Background: Turret dropping 10 into Deck Buffer.");
        }
        if (_backgroundDropActive && Turret.isDropComplete()) {
            _deckHasPins = true;
            _backgroundDropActive = false; 
            _backgroundRestartPending = true; 
            tPrint("Background: Deck Buffer is now FULL. Waiting for Turret to home.");
        }
        if (_backgroundRestartPending && Turret.isIdleEmpty()) {
            Turret.commandStartRefill();
            _backgroundRestartPending = false; 
            tPrint("Background: Turret homed. Resuming catch for next frame.");
        }
        // =======================================================

        switch (_state) {
            case BOOT_INIT:
                if (Sweep.isIdle() && Deck.isIdle()) { Sweep.commandPose(SweepController::GUARD); changeState(BOOT_SWEEP_1_GUARD); tPrint("Boot: Step 1 - Initial Sweep Guard"); }
                break;
            case BOOT_SWEEP_1_GUARD:
                if (Sweep.isIdle()) { Sweep.commandPose(SweepController::BACK); changeState(BOOT_SWEEP_1_BACK); tPrint("Boot: Step 2 - Sweep Back"); }
                break;
            case BOOT_SWEEP_1_BACK:
                if (Sweep.isIdle()) { Sweep.commandPose(SweepController::GUARD); changeState(BOOT_SWEEP_1_GUARD_POST); }
                break;
            case BOOT_SWEEP_1_GUARD_POST:
                if (Sweep.isIdle()) { Deck.commandSetPins(); changeState(BOOT_DECK_CLEAR_1_WAIT); tPrint("Boot: Step 3 - Deck Clear 1"); }
                break;
            case BOOT_DECK_CLEAR_1_WAIT:
                if (Deck.isIdle()) { Sweep.commandPose(SweepController::BACK); changeState(BOOT_SWEEP_2_BACK); tPrint("Boot: Step 4 - Sweep Clear 1"); }
                break;
            case BOOT_SWEEP_2_BACK:
                if (Sweep.isIdle()) { Sweep.commandPose(SweepController::GUARD); changeState(BOOT_SWEEP_2_GUARD); }
                break;
            case BOOT_SWEEP_2_GUARD:
                if (Sweep.isIdle()) { changeState(BOOT_WAIT_TURRET_HOME); tPrint("Boot: Step 5 - Waiting for Turret Home"); }
                break;
            case BOOT_WAIT_TURRET_HOME:
                if (Turret.isIdleEmpty()) { Turret.commandPurge(); changeState(BOOT_TURRET_PURGE_WAIT); tPrint("Boot: Step 6 - Turret Purge"); }
                break;
            case BOOT_TURRET_PURGE_WAIT:
                if (Turret.isDropComplete()) { changeState(BOOT_PURGE_DWELL); }
                break;
            case BOOT_PURGE_DWELL:
                if (millis() - _stateStart >= 1500) { Deck.commandSetPins(); changeState(BOOT_DECK_CLEAR_2_WAIT); tPrint("Boot: Step 7 - Deck Clear 2"); }
                break;
            case BOOT_DECK_CLEAR_2_WAIT:
                if (Deck.isIdle()) { Sweep.commandPose(SweepController::BACK); changeState(BOOT_SWEEP_3_BACK); tPrint("Boot: Step 8 - Sweep Clear 2"); }
                break;
            case BOOT_SWEEP_3_BACK:
                if (Sweep.isIdle()) { Sweep.commandPose(SweepController::GUARD); changeState(BOOT_SWEEP_3_GUARD); }
                break;
            case BOOT_SWEEP_3_GUARD:
                if (Sweep.isIdle()) { changeState(BOOT_WAIT_TURRET_IDLE_1); tPrint("Boot: Step 9 - Waiting for Turret to finish Homing"); }
                break;

            case BOOT_WAIT_TURRET_IDLE_1:
                if (Turret.isIdleEmpty()) {
                    Turret.commandStartRefill(); 
                    changeState(BOOT_WAIT_TURRET_9); 
                    tPrint("Boot: Step 10 - Start Lane Prime"); 
                }
                break;

            case BOOT_WAIT_TURRET_9:
                if (Turret.isPausedAt9()) { Turret.commandExecuteDropSequence(); changeState(BOOT_TURRET_PRIME_DROP); tPrint("Boot: Dropping 10 into empty Deck."); }
                break;
            case BOOT_TURRET_PRIME_DROP:
                if (Turret.isDropComplete()) { changeState(BOOT_PRIME_DWELL); tPrint("Boot: Pins in deck. Dwelling 1.5s for gravity settle."); }
                break;
            case BOOT_PRIME_DWELL:
                if (millis() - _stateStart >= 1500) { 
                    Deck.commandSetPins(); 
                    _deckHasPins = false; 
                    changeState(BOOT_DECK_SET_LANE); 
                    tPrint("Boot: Deck is setting the Lane."); 
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
                    tPrint("Boot: Lane visually clear. Waiting for Turret to finish Homing before starting background buffer."); 
                }
                break;

            case BOOT_WAIT_TURRET_IDLE_2:
                if (Turret.isIdleEmpty()) {
                    Turret.commandStartRefill();
                    _throwCount = 1; 
                    changeState(WAITING_FOR_BALL); 
                    tPrint("Boot Complete: Lane Set! Turret buffering in background. Ready for Throw 1."); 
                }
                break;

            case WAITING_FOR_BALL: break;

            case T1_SWEEP_GUARD_FIRST:
                if (Sweep.isIdle()) { changeState(T1_PIN_SETTLE_WAIT); tPrint("Orchestrator: Guard deployed. Waiting 3s for pins to settle."); }
                break;
            case T1_PIN_SETTLE_WAIT:
                if (millis() - _stateStart >= 3000) { 
                    if (Turret.isCommittedToDrop()) break; // SHIELD UPDATED
                    Deck.commandGrabPins(); 
                    changeState(T1_DECK_GRAB); 
                    tPrint("Orchestrator: T1_DECK_GRAB"); 
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
                    tPrint("Orchestrator: Throw 1 clear. Background manager handling Turret homing. WAITING FOR BALL 2"); 
                }
                else if (Turret.isIdleEmpty()) {
                    Turret.commandStartRefill(); 
                    Door.triggerCycle(); 
                    _throwCount = 2; 
                    changeState(WAITING_FOR_BALL); 
                    tPrint("Orchestrator: Throw 1 clear. Refilling Turret. WAITING FOR BALL 2"); 
                }
                else if (!Turret.isHoming() && !Turret.isCommittedToDrop()) {
                    Turret.commandWake(); 
                    Door.triggerCycle(); 
                    _throwCount = 2; 
                    changeState(WAITING_FOR_BALL); 
                    tPrint("Orchestrator: Throw 1 clear. Turret active in background. WAITING FOR BALL 2"); 
                }
                break;

            case T2_SWEEP_GUARD_FIRST:
                if (Sweep.isIdle()) { changeState(T2_PIN_SETTLE_WAIT); tPrint("Orchestrator: Guard deployed. Waiting 3s for pins to settle."); }
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
                        if (Turret.isCommittedToDrop()) break; // SHIELD UPDATED
                        Deck.commandSetPins(); 
                        _deckHasPins = false; 
                        changeState(RESET_DECK_SET); 
                        tPrint("Orchestrator: Setting Lane from full Deck Buffer."); 
                    } else if (_backgroundDropActive) {
                        changeState(RESET_WAIT_BACKGROUND_DROP); 
                        tPrint("Orchestrator: Fast Bowler! Waiting for active background drop to finish."); 
                    } else {
                        changeState(RESET_WAIT_TURRET_9); 
                        tPrint("Orchestrator: Fast Bowler! Taking manual control of Turret."); 
                    }
                }
                break;

            case RESET_WAIT_BACKGROUND_DROP:
                if (_deckHasPins) {
                    Deck.commandSetPins();
                    _deckHasPins = false;
                    changeState(RESET_DECK_SET);
                    tPrint("Orchestrator: Background drop finished. Setting Lane.");
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
                    Door.triggerCycle(); 
                    _throwCount = 1; 
                    changeState(WAITING_FOR_BALL); 
                    tPrint("Orchestrator: Frame Complete. Background manager handling Turret homing. Ready for Throw 1."); 
                }
                else if (Turret.isIdleEmpty()) {
                    Turret.commandStartRefill(); 
                    Door.triggerCycle(); 
                    _throwCount = 1; 
                    changeState(WAITING_FOR_BALL); 
                    tPrint("Orchestrator: Frame Complete. Refilling Turret. Ready for Throw 1."); 
                }
                else if (!Turret.isHoming() && !Turret.isCommittedToDrop()) {
                    Turret.commandWake(); 
                    Door.triggerCycle(); 
                    _throwCount = 1; 
                    changeState(WAITING_FOR_BALL); 
                    tPrint("Orchestrator: Frame Complete. Turret active in background. Ready for Throw 1."); 
                }
                break;

            case RESET_WAIT_TURRET_9:
                if (Turret.isPausedAt9()) { 
                    Turret.commandExecuteDropSequence(); 
                    changeState(RESET_TURRET_DROPPING); 
                    tPrint("Orchestrator: Turret Ready. Dropping into empty Deck Buffer.");
                }
                break;
            case RESET_TURRET_DROPPING:
                if (Turret.isDropComplete()) { changeState(RESET_BUFFER_DWELL); tPrint("Orchestrator: Drop complete. Dwelling 1.5s."); }
                break;
            case RESET_BUFFER_DWELL:
                if (millis() - _stateStart >= 1500) { 
                    Deck.commandSetPins(); 
                    _deckHasPins = false; 
                    changeState(RESET_DECK_SET); 
                }
                break;
        }
    }

    void triggerBall() {
        if (_state != WAITING_FOR_BALL) {
            tPrint("Orchestrator: Ball Ignored - System is still cycling.");
            return;
        }

        // =======================================================
        // THE INTERLOCK: Fast Bowler Abort
        // =======================================================
        if (_backgroundDropActive) {
            if (Turret.commandSuspendFetch()) {
                // The FSM cleanly forgets the drop ever started and proceeds safely
                _backgroundDropActive = false;
                tPrint("Orchestrator: Fast Bowler override! Background drop aborted to protect moving Deck.");
            }
        }
        // =======================================================

        if (_throwCount == 1) { Sweep.commandPose(SweepController::GUARD); changeState(T1_SWEEP_GUARD_FIRST); } 
        else { Sweep.commandPose(SweepController::GUARD); changeState(T2_SWEEP_GUARD_FIRST); }
    }
    
    void triggerReset() {
        if (Deck.isIdle()) { changeState(BOOT_INIT); tPrint("Orchestrator: Commencing Safe System Reset Dance"); }
    }
private:
    State _state; 
    int _throwCount;
    unsigned long _stateStart; 
    bool _deckHasPins; 
    bool _backgroundDropActive; 
    bool _backgroundRestartPending; 

    void changeState(State s) { 
        _state = s; 
        _stateStart = millis(); 
    }
};

GameOrchestrator Game;

// ==========================================
// MAIN ARDUINO HOOKS
// ==========================================
void setup() {
    Serial.begin(115200);
    delay(1000);
    
    LeftRaiseServo.attach(RAISE_LEFT_PIN); RightRaiseServo.attach(RAISE_RIGHT_PIN);
    SlideServo.attach(SLIDE_PIN); ScissorsServo.attach(SCISSOR_PIN);
    LeftSweepServo.attach(LEFT_SWEEP_PIN); RightSweepServo.attach(RIGHT_SWEEP_PIN);
    BallReturnServo.attach(BALL_RETURN_PIN);
    
    Sweep.begin(); Deck.begin(); Turret.begin(); Door.begin(); BallSensor.begin(); // Added BallSensor
    
    tPrint("Automated Kinematics Booted. Starting Dance.");
}

void loop() {
    Turret.update(); Sweep.update(); Deck.update(); Door.update(); Game.update(); 

    // --- NEW: Physical Hardware Trigger ---
    if (BallSensor.updateAndCheck()) {
        tPrint("HARDWARE TRIGGER: IR Beam Broken by Ball!");
        Door.emergencyClose(); 
        Game.triggerBall();
    }

    // --- KEEP: Serial Override for Testing ---
    if (Serial.available() > 0) {
        char c = Serial.read();
        if (c == 't') {
            Door.emergencyClose(); 
            Game.triggerBall();
        }
        if (c == 'r') Game.triggerReset();
    }
}
