#include "aiLibrary.h"
#include <flecs.h>
#include "ecsTypes.h"
#include "raylib.h"
#include <cfloat>
#include <cmath>

class AttackEnemyState : public State
{
public:
  void enter() const override {}
  void exit() const override {}
  void act(float/* dt*/, flecs::world &/*ecs*/, flecs::entity /*entity*/) const override {}
};

template<typename T>
T sqr(T a){ return a*a; }

template<typename T, typename U>
static float dist_sq(const T &lhs, const U &rhs) { return float(sqr(lhs.x - rhs.x) + sqr(lhs.y - rhs.y)); }

template<typename T, typename U>
static float dist(const T &lhs, const U &rhs) { return sqrtf(dist_sq(lhs, rhs)); }

template<typename T, typename U>
static int move_towards(const T &from, const U &to)
{
  int deltaX = to.x - from.x;
  int deltaY = to.y - from.y;
  if (abs(deltaX) > abs(deltaY))
    return deltaX > 0 ? EA_MOVE_RIGHT : EA_MOVE_LEFT;
  return deltaY < 0 ? EA_MOVE_UP : EA_MOVE_DOWN;
}

static int inverse_move(int move)
{
  return move == EA_MOVE_LEFT ? EA_MOVE_RIGHT :
         move == EA_MOVE_RIGHT ? EA_MOVE_LEFT :
         move == EA_MOVE_UP ? EA_MOVE_DOWN :
         move == EA_MOVE_DOWN ? EA_MOVE_UP : move;
}


template<typename Callable>
static void on_closest_enemy_pos(flecs::world &ecs, flecs::entity entity, Callable c)
{
  static auto enemiesQuery = ecs.query<const Position, const Team, const Hitpoints>();
  entity.set([&](const Position &pos, const Team &t, Action &a)
  {
    flecs::entity closestEnemy;
    float closestDist = FLT_MAX;
    Position closestPos;
    enemiesQuery.each([&](flecs::entity enemy, const Position &epos, const Team &et, const Hitpoints& hp)
    {
      if (t.team == et.team)
        return;
      if (hp.hitpoints == 1)
          return;

      float curDist = dist(epos, pos);
      if (curDist < closestDist)
      {
        closestDist = curDist;
        closestPos = epos;
        closestEnemy = enemy;
      }
    });
    if (ecs.is_valid(closestEnemy))
      c(a, pos, closestPos);
  });
}

template<typename Callable>
static void on_closest_player_teammate_pos(flecs::world& ecs, flecs::entity entity, Callable c)
{
    static auto enemiesQuery = ecs.query<const Position, const Team, Hitpoints, IsPlayer>();
    entity.set([&](const Position& pos, const Team& t, Action& a)
        {
            flecs::entity closestTeammate;
            float closestDist = FLT_MAX;
            Position closestPos;
            Hitpoints playerHp;
            enemiesQuery.each([&](flecs::entity teammate, const Position& epos, const Team& et, Hitpoints& hp, IsPlayer)
                {
                    if (t.team != et.team)
                        return;

                    float curDist = dist(epos, pos);
                    if (curDist < closestDist)
                    {
                        closestDist = curDist;
                        closestPos = epos;
                        closestTeammate = teammate;
                        playerHp = hp;
                    }
                });
            if (ecs.is_valid(closestTeammate))
                c(a, pos, closestPos, playerHp);
        });
}

class MoveToEnemyState : public State
{
public:
  void enter() const override {}
  void exit() const override {}
  void act(float/* dt*/, flecs::world &ecs, flecs::entity entity) const override
  {
    on_closest_enemy_pos(ecs, entity, [&](Action &a, const Position &pos, const Position &enemy_pos)
    {
      a.action = move_towards(pos, enemy_pos);
    });
  }
};

class MoveToTargetPosState : public State
{
public:
    void enter() const override {}
    void exit() const override {}
    void act(float/* dt*/, flecs::world& ecs, flecs::entity entity) const override
    {
        entity.set([&](const Position& pos, const TargetPosition& tpos, Action& a)
            {
                if (tpos.x != pos.x || tpos.y != pos.y)
                {
                    a.action = move_towards(pos, tpos);
                }
                else
                {
                    a.action = EA_NOP;
                }
            });
    }
};


class FleeFromEnemyState : public State
{
public:
  FleeFromEnemyState() {}
  void enter() const override {}
  void exit() const override {}
  void act(float/* dt*/, flecs::world &ecs, flecs::entity entity) const override
  {
    on_closest_enemy_pos(ecs, entity, [&](Action &a, const Position &pos, const Position &enemy_pos)
    {
      a.action = inverse_move(move_towards(pos, enemy_pos));
    });
  }
};

class FollowFriendPlayerState : public State
{
public:
    FollowFriendPlayerState() {}
    void enter() const override {}
    void exit() const override {}
    void act(float/* dt*/, flecs::world& ecs, flecs::entity entity) const override
    {
        on_closest_player_teammate_pos(ecs, entity, [&](Action& a, const Position& pos,
            const Position& enemy_pos, Hitpoints& playerHP)
            {
                if (dist(pos, enemy_pos) > 2.5f)
                    a.action = move_towards(pos, enemy_pos);
                else
                    a.action = EA_NOP;
            });
    }
};

class PatrolState : public State
{
  float patrolDist;
public:
  PatrolState(float dist) : patrolDist(dist) {}
  void enter() const override {}
  void exit() const override {}
  void act(float/* dt*/, flecs::world &ecs, flecs::entity entity) const override
  {
    entity.set([&](const Position &pos, const PatrolPos &ppos, Action &a)
    {
      if (dist(pos, ppos) > patrolDist)
        a.action = move_towards(pos, ppos); // do a recovery walk
      else
      {
        // do a random walk
        a.action = GetRandomValue(EA_MOVE_START, EA_MOVE_END - 1);
      }
    });
  }
};

class NopState : public State
{
public:
  void enter() const override {}
  void exit() const override {}
  void act(float/* dt*/, flecs::world &ecs, flecs::entity entity) const override {}
};

class HealState : public State
{
public:
    void enter() const override {}
    void exit() const override {}
    void act(float/* dt*/, flecs::world& ecs, flecs::entity entity) const override
    {
        entity.set([&](Action& a, Hitpoints& hp, const HealAmount& healAmount, CoolDown& healCD)
            {
                if (healCD.turnsWithoutCastCounter >= healCD.turnLimit)
                {
                    healCD.turnsWithoutCastCounter = 0;
                    hp.hitpoints += healAmount.amount;
                    a.action = EA_HEAL;
                }
                else
                    a.action = EA_NOP;

            });
    }
};

class HealFriendPlayerState : public State
{
public:
    void enter() const override {}
    void exit() const override {}
    void act(float/* dt*/, flecs::world& ecs, flecs::entity entity) const override
    {
        
        on_closest_player_teammate_pos(ecs, entity, [&](Action& a, const Position& pos, 
            const Position& player_pos, Hitpoints& playerHP)
            {
                static auto playersQuery = ecs.query<const Position, Hitpoints, IsPlayer>();

                entity.set([&](Action& a, const HealAmount& healAmount, CoolDown& healCD)
                    {
                        if (healCD.turnsWithoutCastCounter >= healCD.turnLimit)
                        {
                            playersQuery.each([&](flecs::entity player, const Position playerPos, Hitpoints& hp, IsPlayer)
                                {
                                    if (dist_sq(pos, player_pos) == dist_sq(pos, playerPos))
                                    {
                                        healCD.turnsWithoutCastCounter = 0;
                                        hp.hitpoints += healAmount.amount;
                                        a.action = EA_HEAL;
                                       // std::cout << hp.hitpoints << " " << healCD.turnsWithoutCastCounter << std::endl;
                                    }
                                });
                        }
                        else
                            a.action = EA_NOP;
                    });
                
            });
            
    }
};

class SpawnHealState : public State
{
    void enter() const override {}
    void exit() const override {}
    void act(float/* dt*/, flecs::world& ecs, flecs::entity entity) const override
    {
        static auto healsQuery = ecs.query<const Position, HealAmount, const IsSummon, Color>();
        entity.set([&](const Position& pos, TargetPosition& tpos, Action& a)
            {
                bool checkTargetPosition = false;
                healsQuery.each([&](const Position& ppos, HealAmount& amt, const IsSummon, Color& cl)
                    {
                        if (tpos.x == ppos.x && tpos.y == ppos.y)
                        {
                            checkTargetPosition = true;
                            amt.amount = 50.f;
                            cl = GetColor(0x44ff44ff);
                            a.action = EA_SPAWN_HEAL;
                        }
                    });
                    
                if (!checkTargetPosition)
                {
                    a.action = EA_NOP;
                }

                if(tpos.x == pos.x && tpos.y == pos.y)
                    tpos.y *= -1;
            });
    }
};

class ImmortalState : public State
{
public:
    void enter() const override {}
    void exit() const override {}
    void act(float/* dt*/, flecs::world& ecs, flecs::entity entity) const override
    {
        entity.set([&](Action& a, Color& cl, DeadFlag& dead, const Hitpoints& hp)
            {
                a.action = EA_NOP;
                if (hp.hitpoints <= 1)
                {
                    cl = GetColor(0x440000ff);
                    dead.isDead = true;
                }
            });
    }
};


class SpawnMonsterState : public State
{
    void enter() const override {}
    void exit() const override {}
    void act(float/* dt*/, flecs::world& ecs, flecs::entity entity) const override
    {
        
        static auto allSummons = ecs.query<DeadFlag, Hitpoints, Position, Color, const IsSummon>();
        bool spawnedMonster = false;
        entity.set([&](const Position& pos, Action& a, CoolDown& cd)
            {
                if (cd.turnsWithoutCastCounter >= cd.turnLimit)
                {
                    allSummons.each([&](flecs::entity, DeadFlag& dead, Hitpoints& hp, Position& ppos, Color& cl, const IsSummon)
                        {
                            if (dead.isDead)
                            {
                                hp.hitpoints += 100.f;
                                ppos.x = pos.x + GetRandomValue(-2, 2);
                                ppos.y = pos.y + GetRandomValue(-2, 2);
                                cl = GetColor(0x880000ff);
                                dead.isDead = false;
                                spawnedMonster = true;
                            }
                        });
                    if(spawnedMonster)
                        a.action = EA_SPAWN_MONSTER;
                    else
                        a.action = EA_NOP;
                    cd.turnsWithoutCastCounter = 0;
                }
                else
                    a.action = EA_NOP;
            });
        
    }
};


class NestedStateMachine : public State
{
    StateMachine *sm;
public:
    NestedStateMachine(const StateMachine& stateMachine)
    {
        sm = new StateMachine(stateMachine);
    }
    void enter() const override { sm->resetStateMachine(); }
    void exit() const override { }
    void act(float dt, flecs::world& ecs, flecs::entity entity) const override 
    {
        sm->act(dt, ecs, entity);
    }
};

class EnemyAvailableTransition : public StateTransition
{
  float triggerDist;
public:
  EnemyAvailableTransition(float in_dist) : triggerDist(in_dist) {}
  bool isAvailable(flecs::world &ecs, flecs::entity entity) const override
  {
    static auto enemiesQuery = ecs.query<const Position, const Team>();
    bool enemiesFound = false;
    entity.get([&](const Position &pos, const Team &t)
    {
      enemiesQuery.each([&](flecs::entity enemy, const Position &epos, const Team &et)
      {
        if (t.team == et.team)
          return;
        float curDist = dist(epos, pos);
        enemiesFound |= curDist <= triggerDist;
      });
    });
    return enemiesFound;
  }
};

class FriendPlayerAvailableTransition : public StateTransition
{
    float triggerDist;
public:
    FriendPlayerAvailableTransition(float in_dist) : triggerDist(in_dist) {}
    bool isAvailable(flecs::world& ecs, flecs::entity entity) const override
    {
        static auto enemiesQuery = ecs.query<const Position, const Team, IsPlayer>();
        bool playersFound = false;
        entity.get([&](const Position& pos, const Team& t)
            {
                enemiesQuery.each([&](flecs::entity enemy, const Position& epos, const Team& et, IsPlayer)
                    {
                        if (t.team != et.team)
                            return;
                        float curDist = dist(epos, pos);
                        playersFound |= curDist <= triggerDist;
                    });
            });
        return playersFound;
    }
};

class FriendPlayerHitpointsLessThanTransition : public StateTransition
{
  float threshold;
public:
    FriendPlayerHitpointsLessThanTransition(float in_thres) : threshold(in_thres) {}
  bool isAvailable(flecs::world &ecs, flecs::entity entity) const override
  {
    bool hitpointsThresholdReached = false;
    
    static auto enemiesQuery = ecs.query<Hitpoints, const Team, IsPlayer>();
    entity.get([&](const Team& t)
        {
            enemiesQuery.each([&](flecs::entity enemy, Hitpoints &hp, const Team& et, IsPlayer)
                {
                    if (t.team != et.team)
                        return;
                    hitpointsThresholdReached |= hp.hitpoints < threshold;
                });
        });
        
    return hitpointsThresholdReached;
  }
};

class HitpointsLessThanTransition : public StateTransition
{
    float threshold;
public:
    HitpointsLessThanTransition(float in_thres) : threshold(in_thres) {}
    bool isAvailable(flecs::world& ecs, flecs::entity entity) const override
    {
        bool hitpointsThresholdReached = false;
        entity.get([&](const Hitpoints& hp)
            {
                hitpointsThresholdReached |= hp.hitpoints < threshold;
            });
        
        return hitpointsThresholdReached;
    }
};

class CheckTargetPosTransition : public StateTransition
{
public:
    CheckTargetPosTransition() {}
    bool isAvailable(flecs::world& ecs, flecs::entity entity) const override
    {
        bool targetPosReached = false;
        entity.get([&](const Position& pos, const TargetPosition& tpos )
            {
                if(tpos.x == pos.x && tpos.y == pos.y)
                    targetPosReached = true;
            });
        return  targetPosReached;
    }
};

class CheckMonsterSpawnTransition : public StateTransition
{
public:
    CheckMonsterSpawnTransition() {}
    bool isAvailable(flecs::world& ecs, flecs::entity entity) const override
    {
        bool monsterSpawnAvailable = false;
        entity.get([&](const Position& pos, const CoolDown& cd)
            {
                if (cd.turnsWithoutCastCounter >= cd.turnLimit)
                    monsterSpawnAvailable = true;
            });
        return  monsterSpawnAvailable;
    }
};

class EnemyReachableTransition : public StateTransition
{
public:
  bool isAvailable(flecs::world &ecs, flecs::entity entity) const override
  {
    return false;
  }
};

class NegateTransition : public StateTransition
{
  const StateTransition *transition; // we own it
public:
  NegateTransition(const StateTransition *in_trans) : transition(in_trans) {}
  ~NegateTransition() override { delete transition; }

  bool isAvailable(flecs::world &ecs, flecs::entity entity) const override
  {
    return !transition->isAvailable(ecs, entity);
  }
};

class AndTransition : public StateTransition
{
  const StateTransition *lhs; // we own it
  const StateTransition *rhs; // we own it
public:
  AndTransition(const StateTransition *in_lhs, const StateTransition *in_rhs) : lhs(in_lhs), rhs(in_rhs) {}
  ~AndTransition() override
  {
    delete lhs;
    delete rhs;
  }

  bool isAvailable(flecs::world &ecs, flecs::entity entity) const override
  {
    return lhs->isAvailable(ecs, entity) && rhs->isAvailable(ecs, entity);
  }
};


// states
State *create_attack_enemy_state()
{
  return new AttackEnemyState();
}
State *create_move_to_enemy_state()
{
  return new MoveToEnemyState();
}

State *create_flee_from_enemy_state()
{
  return new FleeFromEnemyState();
}

State *create_patrol_state(float patrol_dist)
{
  return new PatrolState(patrol_dist);
}

State *create_nop_state()
{
  return new NopState();
}

State* create_heal_state()
{
    return new HealState();
}

State* create_heal_friend_player_state()
{
    return new HealFriendPlayerState();
}

State* create_follow_friend_player_state()
{
    return new FollowFriendPlayerState();
}

State* create_nested_statemachine_state(const StateMachine& sm)
{
    return new NestedStateMachine(sm);
}

State* create_spawn_heal_state()
{
    return new SpawnHealState();
}

State* create_spawn_monster_state()
{
    return new SpawnMonsterState();
}

State* create_immortal_state()
{
    return new ImmortalState();
}

State* create_move_to_target_state()
{
    return new MoveToTargetPosState();
}

// transitions
StateTransition *create_enemy_available_transition(float dist)
{
  return new EnemyAvailableTransition(dist);
}

StateTransition* create_friend_player_available_transition(float dist)
{
    return new FriendPlayerAvailableTransition(dist);
}

StateTransition *create_enemy_reachable_transition()
{
  return new EnemyReachableTransition();
}

StateTransition *create_hitpoints_less_than_transition(float thres)
{
  return new HitpointsLessThanTransition(thres);
}

StateTransition* create_friend_player_hitpoints_less_than_transition(float thres)
{
    return new FriendPlayerHitpointsLessThanTransition(thres);
}

StateTransition *create_negate_transition(StateTransition *in)
{
  return new NegateTransition(in);
}

StateTransition *create_and_transition(StateTransition *lhs, StateTransition *rhs)
{
  return new AndTransition(lhs, rhs);
}

StateTransition* create_target_pos_reached_transition()
{
    return new CheckTargetPosTransition();
}

StateTransition* create_check_monster_spawn_transition()
{
    return new CheckMonsterSpawnTransition();
}