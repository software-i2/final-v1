// Copyright by BeeX [2026]
#pragma once

namespace final_v1 {

// COLLECT and PROCESS run twice: once to choose where to park, once to choose what to grab from there.
enum class State {
    READY,      // waiting for start
    STREAM,     // waiting for the camera 
    COLLECT,    // picking up cloud/frames frames
    PROCESS,    // deriving obstacles and grasp candidates from the frames
    PICKSPOT,   // searching for parking spot
    RESURVEY,   // cant find a spot; take new frames and try again
    PICKGRASP,  // planning a path to a handle
    GOTOGRASP,  // following to handle
    CLOSEJAW,   // closing on the handle
    RETARGET,   // cant find a path to handle; take new frames and try again
    REPARK,     // used up all retargets; move to new parking spot
    MANUAL,     // going where a manual command sent the arm
    SUCCESS,    // sequence complete, regardless if jaw caught anything or not
    FAIL,       // something went wrong
    ESTOP       // contact or a stop request; the arm is released
};

enum class Event {
    NONE,
    START,
    STOP,
    GOTO,            // a manual command; takes the arm from any state
    STREAMING,
    NO_STREAM,
    FRAMES_IN,
    CANDIDATES_SPOT,
    CANDIDATES_GRASP,
    NO_CANDIDATES_SPOT,
    NO_CANDIDATES_GRASP,
    SURVEY_AGAIN,
    OUT_OF_TIME,
    SPOT_UNCHANGED, 
    NO_SPOT,
    PLAN_FOUND,
    PLAN_ONLY,
    NO_PLAN,
    REACHED,
    STALLED,         // out of arrival time without contact
    JAW_SETTLED,
    LOOK_AGAIN,
    OUT_OF_LOOKS,
    PARK_AGAIN,
    OUT_OF_PARKS,
    COLLIDED,
    FAILURE
};

const char *stateName(State s);
bool        isWorking(State s);  // part way through a pick
State       nextState(State s, Event e);  // `s` itself when the event does not apply

}  // namespace final_v1
