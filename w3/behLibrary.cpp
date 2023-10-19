#include "aiLibrary.h"
#include "ecsTypes.h"
#include "aiUtils.h"
#include "math.h"
#include "raylib.h"
#include "blackboard.h"
#include <algorithm>
#include <random>

static std::random_device dev;
static std::default_random_engine engine(dev());

static float get_random_float(float a, float b)
{
    std::uniform_real_distribution<float> dist(a, b);
    return dist(engine);
}

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

struct UtilitySelector : public BehNode
{
  std::vector<std::pair<BehNode*, utility_function>> utilityNodes;

  BehResult update(flecs::world &ecs, flecs::entity entity, Blackboard &bb) override
  {
    std::vector<std::pair<float, size_t>> utilityScores;
    for (size_t i = 0; i < utilityNodes.size(); ++i)
    {
      const float utilityScore = utilityNodes[i].second(bb);
      utilityScores.push_back(std::make_pair(utilityScore, i));
    }
    std::sort(utilityScores.begin(), utilityScores.end(), [](auto &lhs, auto &rhs)
    {
      return lhs.first > rhs.first;
    });
    for (const std::pair<float, size_t> &node : utilityScores)
    {
      size_t nodeIdx = node.second;
      BehResult res = utilityNodes[nodeIdx].first->update(ecs, entity, bb);
      if (res != BEH_FAIL)
        return res;
    }
    return BEH_FAIL;
  }
};

struct RandomUtilitySelector : public UtilitySelector
{
    BehResult update(flecs::world& ecs, flecs::entity entity, Blackboard& bb) override
    {
        std::vector<float> utilityScores;
        float sum = 0;
        for (size_t i = 0; i < utilityNodes.size(); ++i)
        {
            const float utilityScore = exp(utilityNodes[i].second(bb));
            utilityScores.push_back(utilityScore);
            sum += utilityScore;
        }
        for (size_t i = 0; i < utilityNodes.size(); ++i)
        {
            float r = get_random_float(0.f, sum);

            size_t nodeIdx = 0;
            while(r > 0.f) 
            {
                r -= utilityScores[nodeIdx];
                nodeIdx++;
            }
            nodeIdx -= 1;

            BehResult res = utilityNodes[nodeIdx].first->update(ecs, entity, bb);
            if (res != BEH_FAIL)
                return res;

            sum -= utilityScores[nodeIdx];
            utilityScores[nodeIdx] = 0;
        }
        return BEH_FAIL;
    }
};

struct InertialUtilitySelector : public UtilitySelector
{
    std::vector<float> inert;

private:
    float inertia_amount = 5.f;
    float cooldown = 10.f;

    BehResult update(flecs::world& ecs, flecs::entity entity, Blackboard& bb) override
    {
        std::vector<std::pair<float, size_t>> utilityScores;
        for (size_t i = 0; i < utilityNodes.size(); ++i)
        {
            const float utilityScore = utilityNodes[i].second(bb) + inert[i];
            utilityScores.push_back(std::make_pair(utilityScore, i));
        }
        std::sort(utilityScores.begin(), utilityScores.end(), [](auto& lhs, auto& rhs)
            {
                return lhs.first > rhs.first;
            });
        for (const std::pair<float, size_t>& node : utilityScores)
        {
            size_t nodeIdx = node.second;
            BehResult res = utilityNodes[nodeIdx].first->update(ecs, entity, bb);
            if (res != BEH_FAIL) {
                float prev = inert[nodeIdx];
                std::ranges::fill(inert, 0);
                if (prev > 0)
                    inert[nodeIdx] = prev - cooldown;
                else
                    inert[nodeIdx] = prev + inertia_amount;
                return res;
            }
        }

        return BEH_FAIL;
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

struct GroupPatrol : public BehNode
{
    int dirChance[4][2] = { {EA_MOVE_LEFT, 100}, {EA_MOVE_RIGHT, 100}, {EA_MOVE_DOWN, 100}, {EA_MOVE_UP, 100} };
    GroupPatrol()
    { }

    BehResult update(flecs::world&, flecs::entity entity, Blackboard& bb) override
    {
        BehResult res = BEH_RUNNING;
        entity.set([&](Action& a, const Position& pos, GroupPartoler& patrol)
            {
                int move_dir = EA_MOVE_START;
                int sum = 0;
                for (int i = 0; i < 4; ++i)
                    sum += dirChance[i][1];

                int rand = GetRandomValue(1, sum);

                sum = 0;
                
                for (int i = 0; i < 4; ++i)
                {
                    if (sum <= rand && rand <= dirChance[i][1] + sum)
                    {
                        move_dir = dirChance[i][0];
                        break;
                    }
                    sum += dirChance[i][1];
                }

                a.action = move_dir;
                patrol.last_move = move_dir;
                switch (move_dir)
                {
                    case EA_MOVE_LEFT:
                        dirChance[0][1] = 100;
                        dirChance[1][1] = 0;
                        dirChance[2][1] = 30;
                        dirChance[3][1] = 30;
                        break;
                    case EA_MOVE_RIGHT:
                        dirChance[0][1] = 0;
                        dirChance[1][1] = 100;
                        dirChance[2][1] = 30;
                        dirChance[3][1] = 30;
                        break;
                    case EA_MOVE_DOWN:
                        dirChance[0][1] = 30;
                        dirChance[1][1] = 30;
                        dirChance[2][1] = 100;
                        dirChance[3][1] = 0;
                        break;
                    case EA_MOVE_UP:
                        dirChance[0][1] = 30;
                        dirChance[1][1] = 30;
                        dirChance[2][1] = 0;
                        dirChance[3][1] = 100;
                        break;
                }
            });
        return res;
    }
};


struct MoveToPos : public BehNode
{
    char* _bb_name;
    MoveToPos(flecs::entity entity, const char* bb_name) { }

    BehResult update(flecs::world&, flecs::entity entity, Blackboard& bb) override
    {
        BehResult res = BEH_RUNNING;
        entity.set([&](Action& a, const Position& pos)
            {
                Position patrolPos = bb.get<Position>("base_position");
                if(pos != patrolPos)
                    a.action = move_towards(pos, patrolPos);
            });
        return res;
    }
};


struct PatchUp : public BehNode
{
  float hpThreshold = 100.f;
  PatchUp(float threshold) : hpThreshold(threshold) {}

  BehResult update(flecs::world &, flecs::entity entity, Blackboard &) override
  {
    BehResult res = BEH_SUCCESS;
    entity.set([&](Action &a, Hitpoints &hp)
    {
      if (hp.hitpoints >= hpThreshold)
        return;
      res = BEH_RUNNING;
      a.action = EA_HEAL_SELF;
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

BehNode *utility_selector(const std::vector<std::pair<BehNode*, utility_function>> &nodes)
{
  UtilitySelector *usel = new UtilitySelector;
  usel->utilityNodes = std::move(nodes);
  return usel;
}

BehNode* random_utility_selector(const std::vector<std::pair<BehNode*, utility_function>>& nodes)
{
    RandomUtilitySelector* rusel = new RandomUtilitySelector;
    rusel->utilityNodes = std::move(nodes);
    return rusel;
}

BehNode* inertial_utility_selector(const std::vector<std::pair<BehNode*, utility_function>>& nodes)
{
    InertialUtilitySelector* iusel = new InertialUtilitySelector;
    std::vector<float> inertia(nodes.size(), 0.f);
    iusel->utilityNodes = std::move(nodes);
    iusel->inert = std::move(inertia);
    return iusel;
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

BehNode *patch_up(float thres)
{
  return new PatchUp(thres);
}

BehNode* group_patrol()
{
    return new GroupPatrol();
}

BehNode* move_to_pos(flecs::entity entity, const char* bb_name)
{
    return new MoveToPos(entity, bb_name);
}