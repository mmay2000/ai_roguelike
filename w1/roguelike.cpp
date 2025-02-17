#include "roguelike.h"
#include "ecsTypes.h"
#include "raylib.h"
#include "stateMachine.h"
#include "aiLibrary.h"

static void add_patrol_attack_flee_sm(flecs::entity entity)
{
  entity.get([](StateMachine &sm)
  {
    int patrol = sm.addState(create_patrol_state(3.f));
    int moveToEnemy = sm.addState(create_move_to_enemy_state());
    int fleeFromEnemy = sm.addState(create_flee_from_enemy_state());

    sm.addTransition(create_enemy_available_transition(3.f), patrol, moveToEnemy);
    sm.addTransition(create_negate_transition(create_enemy_available_transition(5.f)), moveToEnemy, patrol);

    sm.addTransition(create_and_transition(create_hitpoints_less_than_transition(60.f), create_enemy_available_transition(5.f)),
                     moveToEnemy, fleeFromEnemy);
    sm.addTransition(create_and_transition(create_hitpoints_less_than_transition(60.f), create_enemy_available_transition(3.f)),
                     patrol, fleeFromEnemy);

    sm.addTransition(create_negate_transition(create_enemy_available_transition(7.f)), fleeFromEnemy, patrol);
  });
}

static void add_patrol_flee_sm(flecs::entity entity)
{
  entity.get([](StateMachine &sm)
  {
    int patrol = sm.addState(create_patrol_state(3.f));
    int fleeFromEnemy = sm.addState(create_flee_from_enemy_state());

    sm.addTransition(create_enemy_available_transition(3.f), patrol, fleeFromEnemy);
    sm.addTransition(create_negate_transition(create_enemy_available_transition(5.f)), fleeFromEnemy, patrol);
  });
}

static void add_attack_sm(flecs::entity entity)
{
  entity.get([](StateMachine &sm)
  {
    sm.addState(create_move_to_enemy_state());
  });
}

static void add_berserk_sm(flecs::entity entity)
{
    entity.get([](StateMachine& sm)
        {
            int patrol = sm.addState(create_patrol_state(3.f));
            int moveToEnemy = sm.addState(create_move_to_enemy_state());
            int infinityMoveToEnemy = sm.addState(create_move_to_enemy_state());

            sm.addTransition(create_enemy_available_transition(3.f), patrol, moveToEnemy);
            sm.addTransition(create_negate_transition(create_enemy_available_transition(5.f)), moveToEnemy, patrol);

            sm.addTransition(create_and_transition(create_hitpoints_less_than_transition(60.f), create_enemy_available_transition(5.f)),
                moveToEnemy, infinityMoveToEnemy);
            sm.addTransition(create_and_transition(create_hitpoints_less_than_transition(60.f), create_enemy_available_transition(3.f)),
                patrol, infinityMoveToEnemy);
        });
}

static void add_healer_sm(flecs::entity entity)
{
    entity.get([](StateMachine& sm)
        {
            int patrol = sm.addState(create_patrol_state(3.f));
            int moveToEnemy = sm.addState(create_move_to_enemy_state());
            int healing = sm.addState(create_heal_state());
            int fleeFromEnemy = sm.addState(create_flee_from_enemy_state());

            sm.addTransition(create_enemy_available_transition(3.f), patrol, moveToEnemy);
            sm.addTransition(create_negate_transition(create_enemy_available_transition(5.f)), moveToEnemy, patrol);

            sm.addTransition(create_and_transition(create_hitpoints_less_than_transition(60.f), create_enemy_available_transition(5.f)),
                moveToEnemy, healing);
            sm.addTransition(create_and_transition(create_hitpoints_less_than_transition(60.f), create_enemy_available_transition(3.f)),
                patrol, healing);

            sm.addTransition(create_enemy_available_transition(3.f), healing, moveToEnemy);
            sm.addTransition(create_enemy_available_transition(5.f), healing, patrol);
            sm.addTransition(create_and_transition(create_hitpoints_less_than_transition(60.f), create_enemy_available_transition(5.f)),
                healing, fleeFromEnemy);
            sm.addTransition(create_and_transition(create_hitpoints_less_than_transition(60.f), create_enemy_available_transition(3.f)),
                healing, fleeFromEnemy);
            sm.addTransition(create_negate_transition(create_enemy_available_transition(5.f)), fleeFromEnemy, healing);
        });
}

static void add_teammate_healer_sm(flecs::entity entity)
{
    entity.get([](StateMachine& sm)
        {
            int follow = sm.addState(create_follow_friend_player_state());
            int healFriend = sm.addState(create_heal_friend_player_state());
            int moveToEnemy = sm.addState(create_move_to_enemy_state());

            sm.addTransition(create_negate_transition(create_friend_player_available_transition(4.f)), 
                healFriend, follow);
            sm.addTransition(create_negate_transition(create_friend_player_available_transition(4.f)), 
                moveToEnemy, follow);
            
            sm.addTransition(create_and_transition(create_friend_player_available_transition(4.f), 
                create_enemy_available_transition(3.f)), follow, moveToEnemy);

            sm.addTransition(create_and_transition(create_friend_player_available_transition(4.f),
                create_friend_player_hitpoints_less_than_transition(100)), follow, healFriend);

            sm.addTransition(create_and_transition(create_friend_player_available_transition(4.f),
                create_friend_player_hitpoints_less_than_transition(100)), moveToEnemy, healFriend);

            sm.addTransition(create_and_transition(create_negate_transition(create_friend_player_hitpoints_less_than_transition(100)),
                create_negate_transition(create_enemy_available_transition(3.f))),
               healFriend, follow);

            sm.addTransition(create_and_transition(create_negate_transition(create_friend_player_hitpoints_less_than_transition(100)),
                (create_enemy_available_transition(3.f))),
                healFriend, moveToEnemy);
        });
}

static const StateMachine& add_phase0_sm_nested()
{
    StateMachine *sm = new StateMachine();

    int moveToTeargetPos = sm->addState(create_move_to_target_state());
    int spawnHeal = sm->addState(create_spawn_heal_state());

    sm->addTransition(create_target_pos_reached_transition(), moveToTeargetPos, spawnHeal);
    sm->addTransition(create_negate_transition(create_target_pos_reached_transition()), spawnHeal, moveToTeargetPos);

    return std::move(*sm);
}

static const StateMachine& add_phase1_sm_nested()
{
    StateMachine* sm = new StateMachine();

    int patrol = sm->addState(create_patrol_state(3.f));
    int moveToEnemy = sm->addState(create_move_to_enemy_state());
    int monsterSpawn = sm->addState(create_spawn_monster_state());

    sm->addTransition(create_enemy_available_transition(2.5f), patrol, moveToEnemy);  

    sm->addTransition(create_enemy_available_transition(2.5f), monsterSpawn, moveToEnemy);
    sm->addTransition(create_negate_transition(create_enemy_available_transition(3.f)), monsterSpawn, patrol);

    sm->addTransition(create_check_monster_spawn_transition(), patrol, monsterSpawn);

    sm->addTransition(create_check_monster_spawn_transition(), moveToEnemy, monsterSpawn);

    return std::move(*sm);
}

static const StateMachine& add_phase2_sm_nested()
{
    StateMachine* sm = new StateMachine();

    int patrol = sm->addState(create_patrol_state(3.f));
    int moveToEnemy = sm->addState(create_move_to_enemy_state());

    sm->addTransition(create_enemy_available_transition(8.f), patrol, moveToEnemy);
    sm->addTransition(create_negate_transition(create_enemy_available_transition(8.f)), moveToEnemy, patrol);

    return std::move(*sm);
}

static void add_summon_monster_sm(flecs::entity entity)
{
    entity.get([](StateMachine& sm)
        {
            int attack = sm.addState(create_move_to_enemy_state());
            int dead = sm.addState(create_immortal_state());

            sm.addTransition(create_hitpoints_less_than_transition(10), attack, dead);
            sm.addTransition(create_negate_transition(create_hitpoints_less_than_transition(10)), dead, attack);

        });
}

static void add_crafter_boss_sm(flecs::entity entity)
{
    entity.get([](StateMachine& sm)
        {
            int phase0 = sm.addState(create_nested_statemachine_state(add_phase0_sm_nested()));
            int phase1 = sm.addState(create_nested_statemachine_state(add_phase1_sm_nested()));
            int phase2 = sm.addState(create_nested_statemachine_state(add_phase2_sm_nested()));

            sm.addTransition(create_and_transition(create_hitpoints_less_than_transition(6000.f), create_enemy_available_transition(10.f)),
                phase0, phase2);
            sm.addTransition(create_and_transition(create_negate_transition(create_hitpoints_less_than_transition(6000.f)),
                create_enemy_available_transition(10.f)), phase0, phase1);

            sm.addTransition((create_hitpoints_less_than_transition(6000.f)),
                phase1, phase2);

            sm.addTransition(create_negate_transition(create_enemy_available_transition(10.f)),
                phase1, phase0);

            sm.addTransition(create_negate_transition(create_enemy_available_transition(10.f)),
                phase2, phase0);
        });
}

static flecs::entity create_monster(flecs::world &ecs, int x, int y, Color color)
{
  return ecs.entity()
    .set(Position{x, y})
    .set(MovePos{x, y})
    .set(PatrolPos{x, y})
    .set(Hitpoints{100.f})
    .set(Action{EA_NOP})
    .set(Color{color})
    .set(StateMachine{})
    .set(Team{1})
    .set(NumActions{1, 0})
    .set(MeleeDamage{20.f});
}

static flecs::entity create_healing_monster(flecs::world& ecs, int x, int y, Color color, float healAmount, int healCD)
{
    return ecs.entity()
        .set(Position{ x, y })
        .set(MovePos{ x, y })
        .set(PatrolPos{ x, y })
        .set(Hitpoints{ 100.f })
        .set(Action{ EA_NOP })
        .set(Color{ color })
        .set(StateMachine{})
        .set(Team{ 1 })
        .set(NumActions{ 1, 0 })
        .set(MeleeDamage{ 20.f })
        .set(HealAmount{ healAmount })
        .set(CoolDown{ healCD, healCD });
}

static flecs::entity create_healing_teammate(flecs::world& ecs, int x, int y, Color color, float healAmount, int healCD)
{
    return ecs.entity()
        .set(Position{ x, y })
        .set(MovePos{ x, y })
        .set(PatrolPos{ x, y })
        .set(Hitpoints{ 10000.f })
        .set(Action{ EA_NOP })
        .set(Color{ color })
        .set(StateMachine{})
        .set(Team{ 0 })
        .set(NumActions{ 1, 0 })
        .set(MeleeDamage{ 5.f })
        .set(HealAmount{ healAmount })
        .set(CoolDown{ healCD, healCD });
}

static void create_player(flecs::world &ecs, int x, int y)
{
  ecs.entity("player")
    .set(Position{x, y})
    .set(MovePos{x, y})
    .set(Hitpoints{100.f})
    .set(GetColor(0xeeeeeeff))
    .set(Action{EA_NOP})
    .add<IsPlayer>()
    .set(Team{0})
    .set(PlayerInput{})
    .set(NumActions{2, 0})
    .set(MeleeDamage{50.f});
}

static flecs::entity create_boss_monster(flecs::world& ecs, int x, int y, Color color)
{
    return ecs.entity()
        .set(Position{ x, y })
        .set(MovePos{ x, y })
        .set(PatrolPos{ x, y })
        .set(Hitpoints{ 15000.f })
        .set(Action{ EA_NOP })
        .set(Color{ color })
        .set(StateMachine{})
        .set(Team{ 1 })
        .set(NumActions{ 1, 0 })
        .set(TargetPosition{ 1, 5 })
        .add<IsHealSpawner>()
        .add<IsMonsterSpawner>()
        .set(CoolDown{ 25, 1 })
        .set(MeleeDamage{ 25.f });
}

static void create_heal(flecs::world &ecs, int x, int y, float amount)
{
  ecs.entity()
    .set(Position{x, y})
    .set(HealAmount{amount})
    .add<IsBoost>()
    .set(GetColor(0x44ff44ff));
}

static void create_summon_heal(flecs::world& ecs, int x, int y, float amount)
{
    ecs.entity()
        .set(Position{ x, y })
        .set(HealAmount{ amount })
        .add<IsSummon>()
        .set(GetColor(0x44ff44ff));
}

static flecs::entity create_summon_monster(flecs::world& ecs, int x, int y, Color color)
{
    return ecs.entity()
        .set(Position{ x, y })
        .set(MovePos{ x, y })
        .set(PatrolPos{ x, y })
        .set(Hitpoints{ 50.f })
        .set(Action{ EA_NOP })
        .set(Color{ color })
        .set(StateMachine{})
        .set(Team{ 1 })
        .set(NumActions{ 1, 0 })
        .add<IsSummon>()
        .set(DeadFlag{false})
        .set(MeleeDamage{ 2.f });
}

static void create_powerup(flecs::world &ecs, int x, int y, float amount)
{
  ecs.entity()
    .set(Position{x, y})
    .set(PowerupAmount{amount})
    .add<IsBoost>()
    .set(Color{255, 255, 0, 255});
}

static void register_roguelike_systems(flecs::world &ecs)
{
  ecs.system<PlayerInput, Action, const IsPlayer>()
    .each([&](PlayerInput &inp, Action &a, const IsPlayer)
    {
      bool left = IsKeyDown(KEY_LEFT);
      bool right = IsKeyDown(KEY_RIGHT);
      bool up = IsKeyDown(KEY_UP);
      bool down = IsKeyDown(KEY_DOWN);
      if (left && !inp.left)
        a.action = EA_MOVE_LEFT;
      if (right && !inp.right)
        a.action = EA_MOVE_RIGHT;
      if (up && !inp.up)
        a.action = EA_MOVE_UP;
      if (down && !inp.down)
        a.action = EA_MOVE_DOWN;
      inp.left = left;
      inp.right = right;
      inp.up = up;
      inp.down = down;
    });
  ecs.system<const Position, const Color>()
    .term<TextureSource>(flecs::Wildcard).not_()
    .each([&](const Position &pos, const Color color)
    {
      const Rectangle rect = {float(pos.x), float(pos.y), 1, 1};
      DrawRectangleRec(rect, color);
    });
  ecs.system<const Position, const Color>()
    .term<TextureSource>(flecs::Wildcard)
    .each([&](flecs::entity e, const Position &pos, const Color color)
    {
      const auto textureSrc = e.target<TextureSource>();
      DrawTextureQuad(*textureSrc.get<Texture2D>(),
          Vector2{1, 1}, Vector2{0, 0},
          Rectangle{float(pos.x), float(pos.y), 1, 1}, color);
    });
}


void init_roguelike(flecs::world &ecs)
{
  register_roguelike_systems(ecs);

  // default enemies & first task enemies
  /* 
  add_patrol_attack_flee_sm(create_monster(ecs, 5, 5, GetColor(0xee00eeff)));
  add_patrol_attack_flee_sm(create_monster(ecs, 10, -5, GetColor(0xee00eeff)));
  add_patrol_flee_sm(create_monster(ecs, -5, -5, GetColor(0x111111ff)));
  add_attack_sm(create_monster(ecs, -5, 5, GetColor(0x880000ff)));
  add_berserk_sm(create_monster(ecs, -5, 7, GetColor(0x00FFFFFF)));
  add_healer_sm(create_healing_monster(ecs, 5, 7, GetColor(0xCD853Fff), 100.f, 1));
  
  add_teammate_healer_sm(create_healing_teammate(ecs, 1, 1, GetColor(0x9932CCff), 10.f, 1));
  */
  
  add_crafter_boss_sm(create_boss_monster(ecs, 6, 7, GetColor(0x0000FFff)));

  create_summon_heal(ecs, 1, -5, 50.f);
  create_summon_heal(ecs, 1, 5, 50.f);
  add_summon_monster_sm(create_summon_monster(ecs, 7, 5, GetColor(0x880000ff)));
  add_summon_monster_sm(create_summon_monster(ecs, 7, 6, GetColor(0x880000ff)));
  add_summon_monster_sm(create_summon_monster(ecs, 7, 7, GetColor(0x880000ff)));

  create_player(ecs, 0, 0);

  create_powerup(ecs, 7, 7, 10.f);
  create_powerup(ecs, 10, -6, 10.f);
  create_powerup(ecs, 10, -4, 10.f);

  create_heal(ecs, -5, -5, 50.f);
  create_heal(ecs, -5, 5, 50.f);
}

static bool is_player_acted(flecs::world &ecs)
{
  static auto processPlayer = ecs.query<const IsPlayer, const Action>();
  bool playerActed = false;
  processPlayer.each([&](const IsPlayer, const Action &a)
  {
    playerActed = a.action != EA_NOP;
  });
  return playerActed;
}

static bool upd_player_actions_count(flecs::world &ecs)
{
  static auto updPlayerActions = ecs.query<const IsPlayer, NumActions>();
  bool actionsReached = false;
  updPlayerActions.each([&](const IsPlayer, NumActions &na)
  {
    na.curActions = (na.curActions + 1) % na.numActions;
    actionsReached |= na.curActions == 0;
  });
  return actionsReached;
}

static Position move_pos(Position pos, int action)
{
  if (action == EA_MOVE_LEFT)
    pos.x--;
  else if (action == EA_MOVE_RIGHT)
    pos.x++;
  else if (action == EA_MOVE_UP)
    pos.y--;
  else if (action == EA_MOVE_DOWN)
    pos.y++;
  return pos;
}

static void process_actions(flecs::world &ecs)
{
  static auto processActions = ecs.query<Action, Position, MovePos, const MeleeDamage, const Team>();
  static auto checkAttacks = ecs.query<const MovePos, Hitpoints, const Team>();
  // Process all actions
  ecs.defer([&]
  {
    processActions.each([&](flecs::entity entity, Action &a, Position &pos, MovePos &mpos, const MeleeDamage &dmg, const Team &team)
    {
      Position nextPos = move_pos(pos, a.action);
      bool blocked = false;
      checkAttacks.each([&](flecs::entity enemy, const MovePos &epos, Hitpoints &hp, const Team &enemy_team)
      {
        if (entity != enemy && epos == nextPos)
        {
          blocked = true;
          if (team.team != enemy_team.team)
            hp.hitpoints -= dmg.damage;
        }
      });
      if (blocked)
        a.action = EA_NOP;
      else
        mpos = nextPos;
    });
    // now move
    processActions.each([&](flecs::entity entity, Action &a, Position &pos, MovePos &mpos, const MeleeDamage &, const Team&)
    {
      pos = mpos;
      a.action = EA_NOP;
    });
  });

  
  static auto saveAllSummons = ecs.query< Hitpoints, DeadFlag, const IsSummon>();
  ecs.defer([&]
      {
          saveAllSummons.each([&](flecs::entity entity, Hitpoints& hp, DeadFlag& dead, const IsSummon)
              {
                  if (hp.hitpoints <= 0.f)
                  {
                    hp.hitpoints = 1.f;
                  }
              });
      });

  static auto deleteAllDead = ecs.query<const Hitpoints>();
  ecs.defer([&]
  {
    deleteAllDead.each([&](flecs::entity entity, const Hitpoints &hp)
    {
            if (hp.hitpoints <= 0.f)
                entity.destruct();
    });
  });

  static auto playerPickup = ecs.query<const IsPlayer, const Position, Hitpoints, MeleeDamage>();
  static auto healPickup = ecs.query<const Position, const HealAmount, const IsBoost>();
  static auto powerupPickup = ecs.query<const Position, const PowerupAmount>();
  static auto bossHealPickup = ecs.query<const Position, HealAmount, Color, const IsSummon>();
  ecs.defer([&]
  {
    playerPickup.each([&](const IsPlayer&, const Position &pos, Hitpoints &hp, MeleeDamage &dmg)
    {
      healPickup.each([&](flecs::entity entity, const Position &ppos, const HealAmount &amt, const IsBoost)
      {
        if (pos == ppos)
        {
          hp.hitpoints += amt.amount;
          entity.destruct();
        }
      });
      powerupPickup.each([&](flecs::entity entity, const Position &ppos, const PowerupAmount &amt)
      {
        if (pos == ppos)
        {
          dmg.damage += amt.amount;
          entity.destruct();
        }
      });
      bossHealPickup.each([&](flecs::entity entity, const Position& ppos,
          HealAmount& amt, Color& cl, const IsSummon)
          {
              if (ppos == pos)
              {
                  dmg.damage += amt.amount;
                  amt.amount = 0;
                  cl = GetColor(0x228822ff);
              }
          });
     });
  });
}

void process_turn(flecs::world &ecs)
{
  static auto stateMachineAct = ecs.query<StateMachine>();
  static auto tickCountAct = ecs.query<CoolDown>();
  if (is_player_acted(ecs))
  {
      ecs.defer([&]
          {
              tickCountAct.each([&](CoolDown& cd)
                  {
                      cd.turnsWithoutCastCounter++;
                  });
          });
    if (upd_player_actions_count(ecs))
    {
      // Plan action for NPCs
      ecs.defer([&]
      {
        stateMachineAct.each([&](flecs::entity e, StateMachine &sm)
        {
          sm.act(0.f, ecs, e);
        });
      });
    }
    process_actions(ecs);
  }
}

void print_stats(flecs::world &ecs)
{
  static auto playerStatsQuery = ecs.query<const IsPlayer, const Hitpoints, const MeleeDamage>();
  playerStatsQuery.each([&](const IsPlayer &, const Hitpoints &hp, const MeleeDamage &dmg)
  {
    DrawText(TextFormat("hp: %d", int(hp.hitpoints)), 20, 20, 20, WHITE);
    DrawText(TextFormat("power: %d", int(dmg.damage)), 20, 40, 20, WHITE);
  });
}

