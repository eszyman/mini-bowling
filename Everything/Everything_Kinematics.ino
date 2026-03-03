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
Servo LeftRaiseServo, RightRaiseServo, SlideServo, ScissorsServo, LeftSweepServo, RightSweepServo;
AccelStepper stepper1(1, STEP_PIN, DIR_PIN);

void tPrint(const String& msg) {
    Serial.print("["); Serial.print(millis()); Serial.print(" ms] "); Serial.println(msg);
}

// ==========================================
// 1. TURRET CONTROLLER FSM (HARD LOCKED 10TH)
// ==========================================
class TurretController {
public:
    enum State {
        UNHOMED, HOMING_PROBE_CCW, HOMING_BLIND_ESCAPE,
        HOMING_FAST_FWD, HOMING_BACKOFF, HOMING_CREEP,
        IDLE_EMPTY, 
        WAITING_FOR_PIN, CATCH_DELAY_PHASE, INDEXING_TO_NEXT, NINTH_SETTLE_PHASE,
        PAUSED_AT_9, 
        FETCH_10TH_AT_9, TENTH_PIN_TRANSIT, MOVING_TO_10_DYNAMIC, 
        PUSHING_10TH_OFF, DWELLING_10TH,
        MOVING_TO_DROP, DWELLING_AT_DROP,
        MOVING_TO_PURGE 
    };

    TurretController(AccelStepper& stp, int ir, int hall, int relay)
        : _stepper(stp), _irPin(ir), _hallPin(hall), _relayPin(relay),
          _state(UNHOMED), _loadedCount(0), _targetPos(0), _dropComplete(false), _queuedPins(0) {}

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
            // ... (Homing cases UNHOMED through HOMING_CREEP remain identical and verified) ...
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
                if (_queuedPins > 0) {
                    _queuedPins--; 
                    _loadedCount++; 
                    tPrint("Caught Pin " + String(_loadedCount));
                    if (_loadedCount >= 9) { 
                        conveyorOff(); 
                        changeState(NINTH_SETTLE_PHASE, 1500); 
                    } else { 
                        changeState(CATCH_DELAY_PHASE, 500); 
                    }
                }
                break;

            case CATCH_DELAY_PHASE:
                if (elapsed >= _timeout) {
                    _targetPos = getPositionForSlot(_loadedCount + 1);
                    _stepper.moveTo(_targetPos);
                    changeState(INDEXING_TO_NEXT);
                }
                break;

            case INDEXING_TO_NEXT: 
                if (_stepper.distanceToGo() == 0) { changeState(WAITING_FOR_PIN); } 
                break;

            case NINTH_SETTLE_PHASE: 
                conveyorOff(); 
                if (elapsed >= _timeout) { changeState(PAUSED_AT_9); } 
                break;

            case PAUSED_AT_9: 
                conveyorOff(); 
                break;

            // --- THE FIXED 10TH PIN LOCK ---
            case FETCH_10TH_AT_9:
                conveyorOn(); 
                // HARD CHECK: We do not move until the IR sensor definitively sees a pin
                if (_queuedPins > 0) {
                    _queuedPins--; 
                    _loadedCount = 10;
                    tPrint("10th Pin Sensed! Entering transit delay...");
                    changeState(TENTH_PIN_TRANSIT, 200); 
                }
                break;

            case TENTH_PIN_TRANSIT:
                conveyorOn(); 
                if (elapsed >= _timeout) {
                    tPrint("Transit Done. Moving to Slot 10.");
                    _targetPos = getPositionForSlot(10); 
                    _stepper.moveTo(_targetPos);
                    changeState(MOVING_TO_10_DYNAMIC);
                }
                break;

            case MOVING_TO_10_DYNAMIC:
                conveyorOn(); 
                if (_stepper.distanceToGo() == 0) { 
                    changeState(PUSHING_10TH_OFF, 1500); 
                }
                break;

            case PUSHING_10TH_OFF:
                conveyorOn();
                if (elapsed >= _timeout) { 
                    conveyorOff(); 
                    changeState(DWELLING_10TH, 2000); 
                }
                break;

            case DWELLING_10TH:
                if (elapsed >= _timeout) {
                    tPrint("Initiating Final Mass Drop Sequence.");
                    _stepper.setMaxSpeed(TURRET_SPRING_MAXSPEED);
                    _stepper.setAcceleration(TURRET_SPRING_ACCEL);
                    _targetPos = getPositionForSlot(10) + TURRET_PIN10_RELEASE_OFFSET;
                    _stepper.moveTo(_targetPos);
                    changeState(MOVING_TO_DROP);
                }
                break;

            case MOVING_TO_DROP:
                if (_stepper.distanceToGo() == 0) { changeState(DWELLING_AT_DROP, RELEASE_DWELL_MS); }
                break;

            case MOVING_TO_PURGE:
                if (_stepper.distanceToGo() == 0) { changeState(DWELLING_AT_DROP, RELEASE_DWELL_MS); }
                break;

            case DWELLING_AT_DROP:
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
        _targetPos = getPositionForSlot(1);
        _stepper.moveTo(_targetPos); 
        changeState(INDEXING_TO_NEXT); 
        return true;
    }

    bool commandExecuteDropSequence() {
        // Only allow this if we are actually waiting for the 10th pin!
        if (_state != PAUSED_AT_9) return false;
        _dropComplete = false;
        _queuedPins = 0; // Clear any noise before waiting for the real 10th pin
        changeState(FETCH_10TH_AT_9);
        tPrint("Drop Triggered: Waiting for 10th Pin to cross IR...");
        return true;
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
    
    bool isIdleEmpty() const { return _state == IDLE_EMPTY; }
    bool isPausedAt9() const { return _state == PAUSED_AT_9; }
    bool isDropComplete() const { return _dropComplete; }

private:
    AccelStepper& _stepper; int _irPin, _hallPin, _relayPin; State _state; 
    unsigned long _stateStart, _timeout; int _loadedCount; long _targetPos;
    int _irLastRead, _irStableState; unsigned long _irLastChange;
    bool _dropComplete; int _queuedPins;

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
                    // Only count pins if we are in a state that expects a pin arrival
                    if (_state == WAITING_FOR_PIN || _state == FETCH_10TH_AT_9) {
                        _queuedPins++;
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
// CONTROLLER INSTANTIATIONS
// ==========================================
TurretController Turret(stepper1, IR_SENSOR_PIN, HALL_EFFECT_PIN, MOTOR_RELAY_PIN);
SweepController Sweep(LeftSweepServo, RightSweepServo);
DeckController Deck(LeftRaiseServo, RightRaiseServo, SlideServo, ScissorsServo);

// ==========================================
// 4. GAME ORCHESTRATOR FSM
// ==========================================
class GameOrchestrator {
public:
    enum State {
        BOOT_INIT, BOOT_SWEEP_1_GUARD, BOOT_SWEEP_1_BACK, BOOT_SWEEP_1_GUARD_POST,
        BOOT_DECK_CLEAR_1, BOOT_DECK_CLEAR_1_WAIT, BOOT_SWEEP_2_BACK, BOOT_SWEEP_2_GUARD,
        BOOT_WAIT_TURRET_HOME, BOOT_TURRET_PURGE_WAIT,
        BOOT_DECK_CLEAR_2, BOOT_DECK_CLEAR_2_WAIT, BOOT_SWEEP_3_BACK, BOOT_SWEEP_3_GUARD,
        BOOT_START_PRIME, BOOT_WAIT_TURRET_9, BOOT_TURRET_PRIME_DROP,
        BOOT_DECK_SET_LANE, BOOT_FINAL_SWEEP_UP,
        WAITING_FOR_BALL,
        T1_SWEEP_GUARD_FIRST, T1_DECK_GRAB, T1_SWEEP_BACK, T1_SWEEP_GUARD_POST, T1_DECK_DROP, T1_SWEEP_UP,
        T2_SWEEP_GUARD_FIRST, T2_SWEEP_BACK, RESET_SWEEP_GUARD_POST, 
        RESET_WAIT_TURRET_9, RESET_TURRET_DROPPING, 
        RESET_DECK_SET, RESET_SWEEP_UP
    };

    GameOrchestrator() : _state(BOOT_INIT), _throwCount(1) {}

    void update() {
        switch (_state) {
            case BOOT_INIT:
                if (Sweep.isIdle() && Deck.isIdle()) { Sweep.commandPose(SweepController::GUARD); _state = BOOT_SWEEP_1_GUARD; tPrint("Boot: Step 1 - Initial Sweep Guard"); }
                break;
            case BOOT_SWEEP_1_GUARD:
                if (Sweep.isIdle()) { Sweep.commandPose(SweepController::BACK); _state = BOOT_SWEEP_1_BACK; tPrint("Boot: Step 2 - Sweep Back"); }
                break;
            case BOOT_SWEEP_1_BACK:
                if (Sweep.isIdle()) { Sweep.commandPose(SweepController::GUARD); _state = BOOT_SWEEP_1_GUARD_POST; }
                break;
            case BOOT_SWEEP_1_GUARD_POST:
                if (Sweep.isIdle()) { Deck.commandSetPins(); _state = BOOT_DECK_CLEAR_1_WAIT; tPrint("Boot: Step 3 - Deck Clear 1"); }
                break;
            case BOOT_DECK_CLEAR_1_WAIT:
                if (Deck.isIdle()) { Sweep.commandPose(SweepController::BACK); _state = BOOT_SWEEP_2_BACK; tPrint("Boot: Step 4 - Sweep Clear 1"); }
                break;
            case BOOT_SWEEP_2_BACK:
                if (Sweep.isIdle()) { Sweep.commandPose(SweepController::GUARD); _state = BOOT_SWEEP_2_GUARD; }
                break;
            case BOOT_SWEEP_2_GUARD:
                if (Sweep.isIdle()) { _state = BOOT_WAIT_TURRET_HOME; tPrint("Boot: Step 5 - Waiting for Turret Home"); }
                break;
            case BOOT_WAIT_TURRET_HOME:
                if (Turret.isIdleEmpty()) { Turret.commandPurge(); _state = BOOT_TURRET_PURGE_WAIT; tPrint("Boot: Step 6 - Turret Purge"); }
                break;
            case BOOT_TURRET_PURGE_WAIT:
                if (Turret.isDropComplete() && Turret.isIdleEmpty()) { Deck.commandSetPins(); _state = BOOT_DECK_CLEAR_2_WAIT; tPrint("Boot: Step 7 - Deck Clear 2"); }
                break;
            case BOOT_DECK_CLEAR_2_WAIT:
                if (Deck.isIdle()) { Sweep.commandPose(SweepController::BACK); _state = BOOT_SWEEP_3_BACK; tPrint("Boot: Step 8 - Sweep Clear 2"); }
                break;
            case BOOT_SWEEP_3_BACK:
                if (Sweep.isIdle()) { Sweep.commandPose(SweepController::GUARD); _state = BOOT_SWEEP_3_GUARD; }
                break;
            case BOOT_SWEEP_3_GUARD:
                if (Sweep.isIdle()) { Turret.commandStartRefill(); _state = BOOT_WAIT_TURRET_9; tPrint("Boot: Step 9 - Start 10-Pin Prime"); }
                break;
            case BOOT_WAIT_TURRET_9:
                if (Turret.isPausedAt9()) { Turret.commandExecuteDropSequence(); _state = BOOT_TURRET_PRIME_DROP; tPrint("Boot: Primer 9 reached. Executing Drop Sequence."); }
                break;
            case BOOT_TURRET_PRIME_DROP:
                if (Turret.isDropComplete() && Turret.isIdleEmpty()) { Deck.commandSetPins(); _state = BOOT_DECK_SET_LANE; tPrint("Boot: Primer setting lane"); }
                break;
            case BOOT_DECK_SET_LANE:
                if (Deck.isIdle()) { Sweep.commandPose(SweepController::UP); Turret.commandStartRefill(); _state = BOOT_FINAL_SWEEP_UP; }
                break;
            case BOOT_FINAL_SWEEP_UP:
                if (Sweep.isIdle()) { _throwCount = 1; _state = WAITING_FOR_BALL; tPrint("Boot Complete: Ready for Throw 1"); }
                break;

            case WAITING_FOR_BALL: break;

            case T1_SWEEP_GUARD_FIRST:
                if (Sweep.isIdle()) { Deck.commandGrabPins(); _state = T1_DECK_GRAB; tPrint("Orchestrator: T1_DECK_GRAB"); }
                break;
            case T1_DECK_GRAB:
                if (Deck.isIdle()) { Sweep.commandPose(SweepController::BACK); _state = T1_SWEEP_BACK; }
                break;
            case T1_SWEEP_BACK:
                if (Sweep.isIdle()) { Sweep.commandPose(SweepController::GUARD); _state = T1_SWEEP_GUARD_POST; }
                break;
            case T1_SWEEP_GUARD_POST:
                if (Sweep.isIdle()) { Deck.commandDropPins(); _state = T1_DECK_DROP; }
                break;
            case T1_DECK_DROP:
                if (Deck.isIdle()) { Sweep.commandPose(SweepController::UP); _state = T1_SWEEP_UP; }
                break;
            case T1_SWEEP_UP:
                if (Sweep.isIdle()) { _throwCount = 2; _state = WAITING_FOR_BALL; tPrint("Orchestrator: WAITING FOR BALL 2"); }
                break;

            case T2_SWEEP_GUARD_FIRST:
                if (Sweep.isIdle()) { Sweep.commandPose(SweepController::BACK); _state = T2_SWEEP_BACK; }
                break;
            case T2_SWEEP_BACK:
                if (Sweep.isIdle()) { Sweep.commandPose(SweepController::GUARD); _state = RESET_SWEEP_GUARD_POST; }
                break;
            case RESET_SWEEP_GUARD_POST:
                if (Sweep.isIdle() && Deck.isIdle()) { _state = RESET_WAIT_TURRET_9; tPrint("Orchestrator: Waiting for 9 Pins in Turret"); }
                break;
            case RESET_WAIT_TURRET_9:
                if (Turret.isPausedAt9()) { Turret.commandExecuteDropSequence(); _state = RESET_TURRET_DROPPING; tPrint("Orchestrator: 9 Pins reached. Executing Drop Sequence.");}
                break;
            case RESET_TURRET_DROPPING:
                if (Turret.isDropComplete() && Turret.isIdleEmpty()) { Deck.commandSetPins(); _state = RESET_DECK_SET; }
                break;
            case RESET_DECK_SET:
                if (Deck.isIdle()) { Sweep.commandPose(SweepController::UP); Turret.commandStartRefill(); _state = RESET_SWEEP_UP; }
                break;
            case RESET_SWEEP_UP:
                if (Sweep.isIdle()) { _throwCount = 1; _state = WAITING_FOR_BALL; tPrint("Orchestrator: Frame Complete. Ready."); }
                break;
        }
    }

    void triggerBall() {
        if (_state != WAITING_FOR_BALL) return;
        if (_throwCount == 1) { Sweep.commandPose(SweepController::GUARD); _state = T1_SWEEP_GUARD_FIRST; } 
        else { Sweep.commandPose(SweepController::GUARD); _state = T2_SWEEP_GUARD_FIRST; }
    }
    
    void triggerReset() {
        if (Deck.isIdle()) { _state = BOOT_INIT; tPrint("Orchestrator: Commencing Safe System Reset Dance"); }
    }
private:
    State _state; int _throwCount;
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
    
    Sweep.begin(); Deck.begin(); Turret.begin(); 
    
    tPrint("Automated Kinematics Booted. Starting Dance.");
}

void loop() {
    Turret.update(); Sweep.update(); Deck.update(); Game.update();

    if (Serial.available() > 0) {
        char c = Serial.read();
        if (c == 't') Game.triggerBall();
        if (c == 'r') Game.triggerReset();
    }
}