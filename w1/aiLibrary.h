#pragma once

#include "stateMachine.h"

// states
State *create_attack_enemy_state();
State *create_move_to_enemy_state();
State *create_flee_from_enemy_state();
State *create_patrol_state(float patrol_dist);
State *create_nop_state();
State* create_heal_state();
State* create_heal_friend_player_state();
State* create_follow_friend_player_state();
State* create_nested_statemachine_state(const StateMachine& sm);
State* create_spawn_heal_state();
State* create_spawn_monster_state();
State* create_immortal_state();
State* create_move_to_target_state();

// transitions
StateTransition *create_enemy_available_transition(float dist);
StateTransition* create_friend_player_available_transition(float dist);
StateTransition *create_enemy_reachable_transition();
StateTransition *create_hitpoints_less_than_transition(float thres);
StateTransition* create_friend_player_hitpoints_less_than_transition(float thres);
StateTransition *create_negate_transition(StateTransition *in);
StateTransition *create_and_transition(StateTransition *lhs, StateTransition *rhs);
StateTransition* create_target_pos_reached_transition();
StateTransition* create_check_monster_spawn_transition();
