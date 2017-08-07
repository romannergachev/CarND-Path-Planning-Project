#ifndef PATH_PLANNING_FSM_H
#define PATH_PLANNING_FSM_H

/*
 * Finite State Machine states definition as enum
 */
enum class CarStates : int {
  KEEP_LANE, PREPARE_TO_CHANGE, CHANGE_LEFT, CHANGE_RIGHT
};


#endif //PATH_PLANNING_FSM_H
