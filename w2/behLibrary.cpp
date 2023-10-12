#include "aiLibrary.h"
#include "ecsTypes.h"
#include "aiUtils.h"
#include "math.h"
#include "raylib.h"
#include "blackboard.h"

struct CompoundNode : public BehNode
{
  std::vector<BehNode*> nodes;

  virtual ~CompoundNode()
  {
    for (BehNode *node : nodes)
      delete node;
    nodes.clear();
  }

  CompoundNode &pushNode(BehNode *node)
  {
    nodes.push_back(node);
    return *this;
  }
};

struct Sequence : public CompoundNode
{
  BehResult update(flecs::world &ecs, flecs::entity entity, Blackboard &bb) override
  {
    for (BehNode *node : nodes)
    {
      BehResult res = node->update(ecs, entity, bb);
      if (res != BEH_SUCCESS)
        return res;
    }
    return BEH_SUCCESS;
  }
};

struct Selector : public CompoundNode
{
  BehResult update(flecs::world &ecs, flecs::entity entity, Blackboard &bb) override
  {
    for (BehNode *node : nodes)
    {
      BehResult res = node->update(ecs, entity, bb);
      if (res != BEH_FAIL)
        return res;
    }
    return BEH_FAIL;
  }
};

struct Parallel : public CompoundNode
{
    BehResult update(flecs::world& ecs, flecs::entity entity, Blackboard& bb) override
    {
        for (BehNode* node : nodes)
        {
            BehResult res = node->update(ecs, entity, bb);
            if (res != BEH_RUNNING)
                return res;
        }
        return BEH_RUNNING;
    }
};

struct Not : public BehNode
{
    BehNode* node;
    Not(BehNode* n)
    {
        node = n;
    }

    ~Not()
    {
        delete node;
    }

    BehResult update(flecs::world& ecs, flecs::entity entity, Blackboard& bb) override
    {
        BehResult res = node->update(ecs, entity, bb);
        return res == BEH_FAIL ? BEH_SUCCESS : (res == BEH_SUCCESS ? BEH_FAIL : BEH_RUNNING);
    }

};

struct FindPickUp : public BehNode
{
    size_t entityBb = size_t(-1);
    FindPickUp(flecs::entity entity, const char* bb_name)
    {
        entityBb = reg_entity_blackboard_var<flecs::entity>(entity, bb_name);
    }
    BehResult update(flecs::world& ecs, flecs::entity entity, Blackboard& bb) override
    {
        BehResult res = BEH_FAIL;
        static auto boostsQuery = ecs.query<const Position, const IsBoost>();
        entity.set([&](const Position& pos)
            {
                flecs::entity closestEnemy;
                float closestDist = FLT_MAX;
                Position closestPos;
                boostsQuery.each([&](flecs::entity enemy, const Position& epos, const IsBoost)
                    {
                        float curDist = dist(epos, pos);
                        if (curDist < closestDist)
                        {
                            closestDist = curDist;
                            closestPos = epos;
                            closestEnemy = enemy;
                        }
                    });

                if (ecs.is_valid(closestEnemy))
                {
                    bb.set<flecs::entity>(entityBb, closestEnemy);
                    res = BEH_SUCCESS;
                }
            });
        return res;
    }
};

struct MoveToEntity : public BehNode
{
  size_t entityBb = size_t(-1); // wraps to 0xff...
  MoveToEntity(flecs::entity entity, const char *bb_name)
  {
    entityBb = reg_entity_blackboard_var<flecs::entity>(entity, bb_name);
  }

  BehResult update(flecs::world &, flecs::entity entity, Blackboard &bb) override
  {
    BehResult res = BEH_RUNNING;
    entity.set([&](Action &a, const Position &pos)
    {
      flecs::entity targetEntity = bb.get<flecs::entity>(entityBb);
      if (!targetEntity.is_alive())
      {
        res = BEH_FAIL;
        return;
      }
      targetEntity.get([&](const Position &target_pos)
      {
        if (pos != target_pos)
        {
          a.action = move_towards(pos, target_pos);
          res = BEH_RUNNING;
        }
        else
          res = BEH_SUCCESS;
      });
    });
    return res;
  }
};

struct OnWayPoint : public BehNode
{
    size_t entityBb = size_t(-1); // wraps to 0xff...
    OnWayPoint(flecs::entity entity, const char* bb_name)
    {
        entityBb = reg_entity_blackboard_var<flecs::entity>(entity, bb_name);
    }

    BehResult update(flecs::world& ecs, flecs::entity entity, Blackboard& bb) override
    {
        BehResult res = BEH_SUCCESS;
        static auto wayQuery = ecs.query<const WayPoint&, const Position&>();
        entity.set([&](Action& a, const Position& pos)
            {
                flecs::entity targetEntity = bb.get<flecs::entity>(entityBb);
                if (!targetEntity.is_alive())
                {
                    flecs::entity closestWaypoint;
                    float closestDist = FLT_MAX;
                    Position closestPos;
                    wayQuery.each([&](flecs::entity waypoint, const WayPoint& way, const Position& wpos)
                        {
                            float curDist = dist(wpos, pos);
                            if (curDist < closestDist)
                            {
                                closestDist = curDist;
                                closestPos = wpos;
                                closestWaypoint = waypoint;
                            }
                        });
                    
                    if (!closestWaypoint.is_alive())
                    {
                        res = BEH_FAIL;
                        return;
                    }
                    a.action = EA_SWITCH_WAY_POINT;
                    bb.set<flecs::entity>(entityBb, closestWaypoint);
                    res = BEH_SUCCESS;
                    return;
                }
                targetEntity.get([&](const Position& target_pos, const WayPoint& way )
                    {
                        if (pos == target_pos)
                        {
                            a.action = EA_SWITCH_WAY_POINT;
                            bb.set<flecs::entity>(entityBb, way.nextPoint);
                            res = BEH_SUCCESS;
                        }
                    });
            });
        return res;
    }
};

struct IsLowHp : public BehNode
{
  float threshold = 0.f;
  IsLowHp(float thres) : threshold(thres) {}

  BehResult update(flecs::world &, flecs::entity entity, Blackboard &) override
  {
    BehResult res = BEH_SUCCESS;
    entity.get([&](const Hitpoints &hp)
    {
      res = hp.hitpoints < threshold ? BEH_SUCCESS : BEH_FAIL;
    });
    return res;
  }
};

struct FindEnemy : public BehNode
{
  size_t entityBb = size_t(-1);
  float distance = 0;
  FindEnemy(flecs::entity entity, float in_dist, const char *bb_name) : distance(in_dist)
  {
    entityBb = reg_entity_blackboard_var<flecs::entity>(entity, bb_name);
  }
  BehResult update(flecs::world &ecs, flecs::entity entity, Blackboard &bb) override
  {
    BehResult res = BEH_FAIL;
    static auto enemiesQuery = ecs.query<const Position, const Team>();
    entity.set([&](const Position &pos, const Team &t)
    {
      flecs::entity closestEnemy;
      float closestDist = FLT_MAX;
      Position closestPos;
      enemiesQuery.each([&](flecs::entity enemy, const Position &epos, const Team &et)
      {
        if (t.team == et.team)
          return;
        float curDist = dist(epos, pos);
        if (curDist < closestDist)
        {
          closestDist = curDist;
          closestPos = epos;
          closestEnemy = enemy;
        }
      });
      if (ecs.is_valid(closestEnemy) && closestDist <= distance)
      {
        bb.set<flecs::entity>(entityBb, closestEnemy);
        res = BEH_SUCCESS;
      }
    });
    return res;
  }
};

struct Flee : public BehNode
{
  size_t entityBb = size_t(-1);
  Flee(flecs::entity entity, const char *bb_name)
  {
    entityBb = reg_entity_blackboard_var<flecs::entity>(entity, bb_name);
  }

  BehResult update(flecs::world &, flecs::entity entity, Blackboard &bb) override
  {
    BehResult res = BEH_RUNNING;
    entity.set([&](Action &a, const Position &pos)
    {
      flecs::entity targetEntity = bb.get<flecs::entity>(entityBb);
      if (!targetEntity.is_alive())
      {
        res = BEH_FAIL;
        return;
      }
      targetEntity.get([&](const Position &target_pos)
      {
        a.action = inverse_move(move_towards(pos, target_pos));
      });
    });
    return res;
  }
};

struct Patrol : public BehNode
{
  size_t pposBb = size_t(-1);
  float patrolDist = 1.f;
  Patrol(flecs::entity entity, float patrol_dist, const char *bb_name)
    : patrolDist(patrol_dist)
  {
    pposBb = reg_entity_blackboard_var<Position>(entity, bb_name);
    entity.set([&](Blackboard &bb, const Position &pos)
    {
      bb.set<Position>(pposBb, pos);
    });
  }

  BehResult update(flecs::world &, flecs::entity entity, Blackboard &bb) override
  {
    BehResult res = BEH_RUNNING;
    entity.set([&](Action &a, const Position &pos)
    {
      Position patrolPos = bb.get<Position>(pposBb);
      if (dist(pos, patrolPos) > patrolDist)
        a.action = move_towards(pos, patrolPos);
      else
        a.action = GetRandomValue(EA_MOVE_START, EA_MOVE_END - 1); // do a random walk
    });
    return res;
  }
};


BehNode *sequence(const std::vector<BehNode*> &nodes)
{
  Sequence *seq = new Sequence;
  for (BehNode *node : nodes)
    seq->pushNode(node);
  return seq;
}

BehNode *selector(const std::vector<BehNode*> &nodes)
{
  Selector *sel = new Selector;
  for (BehNode *node : nodes)
    sel->pushNode(node);
  return sel;
}

BehNode* parallel(const std::vector<BehNode*>& nodes)
{
    Selector* par = new Selector;
    for (BehNode* node : nodes)
        par->pushNode(node);
    return par;
}

BehNode *move_to_entity(flecs::entity entity, const char *bb_name)
{
  return new MoveToEntity(entity, bb_name);
}

BehNode *is_low_hp(float thres)
{
  return new IsLowHp(thres);
}

BehNode *find_enemy(flecs::entity entity, float dist, const char *bb_name)
{
  return new FindEnemy(entity, dist, bb_name);
}

BehNode *flee(flecs::entity entity, const char *bb_name)
{
  return new Flee(entity, bb_name);
}

BehNode *patrol(flecs::entity entity, float patrol_dist, const char *bb_name)
{
  return new Patrol(entity, patrol_dist, bb_name);
}

BehNode* _not(BehNode* node)
{
    return new Not(node);
}

BehNode* find_boost(flecs::entity entity, const char* bb_name)
{
    return new FindPickUp(entity, bb_name);
}

BehNode* on_way(flecs::entity entity, const char* bb_name)
{
    return new OnWayPoint(entity, bb_name);
}