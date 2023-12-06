#include "pathfinder.h"
#include "dungeonUtils.h"
#include "math.h"
#include <algorithm>

float heuristic(IVec2 lhs, IVec2 rhs)
{
  return sqrtf(sqr(float(lhs.x - rhs.x)) + sqr(float(lhs.y - rhs.y)));
};

template<typename T>
static size_t coord_to_idx(T x, T y, size_t w)
{
  return size_t(y) * w + size_t(x);
}

template<typename Item>
static void concat(std::vector<Item>& a, std::vector<Item>& b) {
    a.insert(
        a.end(),
        std::make_move_iterator(b.begin()),
        std::make_move_iterator(b.end())
    );
}

static std::vector<IVec2> reconstruct_path(std::vector<IVec2> prev, IVec2 to, size_t width)
{
  IVec2 curPos = to;
  std::vector<IVec2> res = {curPos};
  while (prev[coord_to_idx(curPos.x, curPos.y, width)] != IVec2{-1, -1})
  {
    curPos = prev[coord_to_idx(curPos.x, curPos.y, width)];
    res.insert(res.begin(), curPos);
  }
  return res;
}

static std::vector<IVec2> find_path_a_star(const DungeonData &dd, IVec2 from, IVec2 to,
                                           IVec2 lim_min, IVec2 lim_max)
{
  if (from.x < 0 || from.y < 0 || from.x >= int(dd.width) || from.y >= int(dd.height))
    return std::vector<IVec2>();
  size_t inpSize = dd.width * dd.height;

  std::vector<float> g(inpSize, std::numeric_limits<float>::max());
  std::vector<float> f(inpSize, std::numeric_limits<float>::max());
  std::vector<IVec2> prev(inpSize, {-1,-1});

  auto getG = [&](IVec2 p) -> float { return g[coord_to_idx(p.x, p.y, dd.width)]; };
  auto getF = [&](IVec2 p) -> float { return f[coord_to_idx(p.x, p.y, dd.width)]; };

  g[coord_to_idx(from.x, from.y, dd.width)] = 0;
  f[coord_to_idx(from.x, from.y, dd.width)] = heuristic(from, to);

  std::vector<IVec2> openList = {from};
  std::vector<IVec2> closedList;

  while (!openList.empty())
  {
    size_t bestIdx = 0;
    float bestScore = getF(openList[0]);
    for (size_t i = 1; i < openList.size(); ++i)
    {
      float score = getF(openList[i]);
      if (score < bestScore)
      {
        bestIdx = i;
        bestScore = score;
      }
    }
    if (openList[bestIdx] == to)
      return reconstruct_path(prev, to, dd.width);
    IVec2 curPos = openList[bestIdx];
    openList.erase(openList.begin() + bestIdx);
    if (std::find(closedList.begin(), closedList.end(), curPos) != closedList.end())
      continue;
    size_t idx = coord_to_idx(curPos.x, curPos.y, dd.width);
    closedList.emplace_back(curPos);
    auto checkNeighbour = [&](IVec2 p)
    {
      // out of bounds
      if (p.x < lim_min.x || p.y < lim_min.y || p.x >= lim_max.x || p.y >= lim_max.y)
        return;
      size_t idx = coord_to_idx(p.x, p.y, dd.width);
      // not empty
      if (dd.tiles[idx] == dungeon::wall)
        return;
      float edgeWeight = 1.f;
      float gScore = getG(curPos) + 1.f * edgeWeight; // we're exactly 1 unit away
      if (gScore < getG(p))
      {
        prev[idx] = curPos;
        g[idx] = gScore;
        f[idx] = gScore + heuristic(p, to);
      }
      bool found = std::find(openList.begin(), openList.end(), p) != openList.end();
      if (!found)
        openList.emplace_back(p);
    };
    checkNeighbour({curPos.x + 1, curPos.y + 0});
    checkNeighbour({curPos.x - 1, curPos.y + 0});
    checkNeighbour({curPos.x + 0, curPos.y + 1});
    checkNeighbour({curPos.x + 0, curPos.y - 1});
  }
  // empty path
  return std::vector<IVec2>();
}

void prebuild_map(flecs::world &ecs)
{
  auto mapQuery = ecs.query<const DungeonData>();

  constexpr size_t splitTiles = 10;
  ecs.defer([&]()
  {
    mapQuery.each([&](flecs::entity e, const DungeonData &dd)
    {
      // go through each super tile
      const size_t width = dd.width / splitTiles;
      const size_t height = dd.height / splitTiles;

      auto check_border = [&](size_t xx, size_t yy,
                              size_t dir_x, size_t dir_y,
                              int offs_x, int offs_y,
                              std::vector<PathPortal> &portals)
      {
        int spanFrom = -1;
        int spanTo = -1;
        for (size_t i = 0; i < splitTiles; ++i)
        {
          size_t x = xx * splitTiles + i * dir_x;
          size_t y = yy * splitTiles + i * dir_y;
          size_t nx = x + offs_x;
          size_t ny = y + offs_y;
          if (dd.tiles[y * dd.width + x] != dungeon::wall &&
              dd.tiles[ny * dd.width + nx] != dungeon::wall)
          {
            if (spanFrom < 0)
              spanFrom = i;
            spanTo = i;
          }
          else if (spanFrom >= 0)
          {
            // write span
            portals.push_back({xx * splitTiles + spanFrom * dir_x + offs_x,
                               yy * splitTiles + spanFrom * dir_y + offs_y,
                               xx * splitTiles + spanTo * dir_x,
                               yy * splitTiles + spanTo * dir_y});
            spanFrom = -1;
          }
        }
        if (spanFrom >= 0)
        {
          portals.push_back({xx * splitTiles + spanFrom * dir_x + offs_x,
                             yy * splitTiles + spanFrom * dir_y + offs_y,
                             xx * splitTiles + spanTo * dir_x,
                             yy * splitTiles + spanTo * dir_y});
        }
      };

      std::vector<PathPortal> portals;
      std::vector<std::vector<size_t>> tilePortalsIndices;

      auto push_portals = [&](size_t x, size_t y,
                              int offs_x, int offs_y,
                              const std::vector<PathPortal> &new_portals)
      {
        for (const PathPortal &portal : new_portals)
        {
          size_t idx = portals.size();
          portals.push_back(portal);
          tilePortalsIndices[y * width + x].push_back(idx);
          tilePortalsIndices[(y + offs_y) * width + x + offs_x].push_back(idx);
        }
      };
      for (size_t y = 0; y < height; ++y)
        for (size_t x = 0; x < width; ++x)
        {
          tilePortalsIndices.push_back(std::vector<size_t>{});
          // check top
          if (y > 0)
          {
            std::vector<PathPortal> topPortals;
            check_border(x, y, 1, 0, 0, -1, topPortals);
            push_portals(x, y, 0, -1, topPortals);
          }
          // left
          if (x > 0)
          {
            std::vector<PathPortal> leftPortals;
            check_border(x, y, 0, 1, -1, 0, leftPortals);
            push_portals(x, y, -1, 0, leftPortals);
          }
        }
      for (size_t tidx = 0; tidx < tilePortalsIndices.size(); ++tidx)
      {
        const std::vector<size_t> &indices = tilePortalsIndices[tidx];
        size_t x = tidx % width;
        size_t y = tidx / width;
        IVec2 limMin{int((x + 0) * splitTiles), int((y + 0) * splitTiles)};
        IVec2 limMax{int((x + 1) * splitTiles), int((y + 1) * splitTiles)};
        for (size_t i = 0; i < indices.size(); ++i)
        {
          PathPortal &firstPortal = portals[indices[i]];
          for (size_t j = i + 1; j < indices.size(); ++j)
          {
            PathPortal &secondPortal = portals[indices[j]];
            // check path from i to j
            // check each position (to find closest dist) (could be made more optimal)
            bool noPath = false;
            size_t minDist = 0xffffffff;
            for (size_t fromY = std::max(firstPortal.startY, size_t(limMin.y));
                        fromY <= std::min(firstPortal.endY, size_t(limMax.y - 1)) && !noPath; ++fromY)
            {
              for (size_t fromX = std::max(firstPortal.startX, size_t(limMin.x));
                          fromX <= std::min(firstPortal.endX, size_t(limMax.x - 1)) && !noPath; ++fromX)
              {
                for (size_t toY = std::max(secondPortal.startY, size_t(limMin.y));
                            toY <= std::min(secondPortal.endY, size_t(limMax.y - 1)) && !noPath; ++toY)
                {
                  for (size_t toX = std::max(secondPortal.startX, size_t(limMin.x));
                              toX <= std::min(secondPortal.endX, size_t(limMax.x - 1)) && !noPath; ++toX)
                  {
                    IVec2 from{int(fromX), int(fromY)};
                    IVec2 to{int(toX), int(toY)};
                    std::vector<IVec2> path = find_path_a_star(dd, from, to, limMin, limMax);
                    if (path.empty() && from != to)
                    {
                      noPath = true; // if we found that there's no path at all - we can break out
                      break;
                    }
                    minDist = std::min(minDist, path.size());
                  }
                }
              }
            }
            // write pathable data and length
            if (noPath)
              continue;
            firstPortal.conns.push_back({indices[j], float(minDist)});
            secondPortal.conns.push_back({indices[i], float(minDist)});
          }
        }
      }
      e.set(DungeonPortals{splitTiles, portals, tilePortalsIndices});
    });
  });
}


static std::vector<size_t> find_portal_path_a_star(size_t from, size_t to, const DungeonPortals& dp)
{

    std::vector<int> g(dp.portals.size(), INT_MAX);
    std::vector<int> f(dp.portals.size(), INT_MAX);
    g[from] = 0;

    auto portal_heuristic([&](size_t idx0, size_t idx1) -> float {
        return heuristic({ (int)(dp.portals[idx0].startX + dp.portals[idx0].endX + 1) / 2,
        (int)(dp.portals[idx0].startY + dp.portals[idx0].endY + 1) / 2 },
            { (int)(dp.portals[idx1].startX + dp.portals[idx1].endX + 1) / 2,
            (int)(dp.portals[idx1].startY + dp.portals[idx1].endY + 1) / 2 });
        });

    f[from] = portal_heuristic(from, to);

    std::vector<size_t> open_list = { from };
    std::vector<size_t> closed_list;
    std::vector<size_t> prev(dp.portals.size(), INT_MAX);
    
    while (!open_list.empty())
    {
        size_t bestIdx = 0;
        for (int i = 1; i < open_list.size(); ++i)
        {
            if (f[open_list[i]] < f[bestIdx])
                bestIdx = i;
        }

        if (open_list[bestIdx] == to)
        {
            //reconstruct path
            size_t curPort = to;
            std::vector<size_t> res = { curPort };
            while (prev[curPort] != INT_MAX)
            {
                curPort = prev[curPort];
                res.insert(res.begin(), curPort);
            }
            return res;
        }

        size_t curPortIdx = open_list[bestIdx];
        open_list.erase(open_list.begin() + bestIdx);
        if (std::find(closed_list.begin(), closed_list.end(), curPortIdx) != closed_list.end())
            continue;

        closed_list.emplace_back(curPortIdx);
        
        for (const PortalConnection& conn : dp.portals[curPortIdx].conns)
        {
            int edgeWeight = 1;
            int gScore = g[curPortIdx] + conn.score * edgeWeight; 
            if (gScore < g[conn.connIdx])
            {
                prev[conn.connIdx] = curPortIdx;
                g[conn.connIdx] = gScore;
                f[conn.connIdx] = gScore + 3 * portal_heuristic(conn.connIdx, to);
            }
            bool found = std::find(open_list.begin(), open_list.end(), conn.connIdx) != open_list.end();
            if (!found)
                open_list.emplace_back(conn.connIdx);
        }
        
    }
    
    return std::vector<size_t>();
}

void find_path_(Path &p, const DungeonPortals &dp, const DungeonData &dd)
{
    p.path.clear();
    size_t w = dd.width;
    size_t ts = dp.tileSplit;

    size_t fromTileIdx = (((int)p.from.x - (int)p.from.x % (ts))
        + (w / ts) * ((int)p.from.y - (int)p.from.y % (ts)))/ts;

    size_t toTileIdx = (((int)p.to.x - (int)p.to.x % (ts))
        + (w / ts) * ((int)p.to.y - (int)p.to.y % (ts)))/ts;

    PathPortal fromPortal;
    PathPortal toPortal;

    auto find_nearest_portal = ([&](Position pos)
        {
            size_t xIdx = ((int)pos.x - (int)pos.x % (ts)) / ts;
            size_t yIdx = ((int)pos.y - (int)pos.y % (ts)) / ts;
            size_t posTileIdx = xIdx + (w / ts) * yIdx;
            std::vector<int> cells_in_tile;
            cells_in_tile.resize(ts * ts);
            std::fill(cells_in_tile.begin(), cells_in_tile.end(), INT_MAX);
            
            cells_in_tile[(int)pos.x % ts + ts * ((int)pos.y % (ts))] = 0;
            
            bool done = false;
            auto getMapAt = [&](size_t x, size_t y, int def)
            {
                if (x >= xIdx * ts && x < ts * xIdx + (ts) && y >= ts * yIdx && y < ts * yIdx + (ts) && dd.tiles[y * w + x] == dungeon::floor)
                    return cells_in_tile[y % ts * (ts) + x % ts];
                return def;
            };

            auto getMinNei = [&](size_t x, size_t y)
            {
                int val = cells_in_tile[(y % ts) * (ts) + (x % ts)];
                val = std::min(val, getMapAt(x - 1, y + 0, val));
                val = std::min(val, getMapAt(x + 1, y + 0, val));
                val = std::min(val, getMapAt(x + 0, y - 1, val));
                val = std::min(val, getMapAt(x + 0, y + 1, val));
                return val;
            };
            
            while (!done)
            {
                done = true;
                for (size_t y = yIdx * ts; y < yIdx * ts + ts; ++y)
                    for (size_t x = xIdx * ts; x < xIdx * ts + ts; ++x)
                    {
                        const size_t i = y * w + x;
                        if (dd.tiles[i] != dungeon::floor)
                            continue;
                        const int myVal = getMapAt(x, y, INT_MAX);
                        const int minVal = getMinNei(x, y);
                        if (minVal < myVal - 1.f)
                        {
                            cells_in_tile[(y % ts) * (ts) + (x % ts)] = minVal + 1.f;
                            done = false;
                        }
                    }
            }
            

            int min_val = INT_MAX;
            size_t min_portal_idx = dp.tilePortalsIndices[posTileIdx][0];
            for (const size_t& idx : dp.tilePortalsIndices[posTileIdx])
            {
                for(int x = dp.portals[idx].startX; x <= dp.portals[idx].endX; ++x)
                    for (int y = dp.portals[idx].startY; y <= dp.portals[idx].endY; ++y)
                    {
                        if (min_val > getMapAt(x, y, INT_MAX) + heuristic({ x, y }, { (int)p.to.x, (int)p.to.y }))
                        {
                            min_val = getMapAt(x, y, INT_MAX) + heuristic({ x, y }, { (int)p.to.x, (int)p.to.y });
                            min_portal_idx = idx;
                        }
                    }
            }
            
            return min_portal_idx;
           
        });
        
    //find nearest fromPortal & toPortal
    size_t fromPortalIdx = find_nearest_portal(p.from);
    fromPortal = dp.portals[fromPortalIdx];
    size_t toPortalIdx = find_nearest_portal(p.to);

    //A* for portals
    std::vector<size_t> portals_path = find_portal_path_a_star(fromPortalIdx, toPortalIdx, dp);
    
    //path from "from" and firts portal

    size_t x = ((int)p.from.x - (int)p.from.x % (ts));
    size_t y = ((int)p.from.y - (int)p.from.y % (ts));

    IVec2 limMin{ int((x + 0)), int((y + 0)) };
    IVec2 limMax{ int((x + ts)), int((y + ts)) };


    int fromVecX = ((int)p.from.x);
    int fromVecY = ((int)p.from.y);


    int fromPortalVecX = std::clamp<int>(fromPortal.startX + (fromPortal.endX - fromPortal.startX) / 2, limMin.x, limMax.x);
    int fromPortalVecY = std::clamp<int>(fromPortal.startY + (fromPortal.endY - fromPortal.startY) / 2, limMin.y, limMax.y);
    
    std::vector<IVec2> curPath = find_path_a_star(dd, { fromVecX , fromVecY }, { fromPortalVecX , fromPortalVecY }, limMin, limMax);
    std::vector<Position> cur_path;
    auto IVec2_2_Pos([&](std::vector<Position> &a, std::vector<IVec2> &b) {
        for (IVec2& vec : b)
        {
            float xx = vec.x;
            float yy = vec.y;
            a.push_back(Position{ xx, yy });
        }
        });
    IVec2_2_Pos(cur_path, curPath);
    concat<Position>(p.path, cur_path);

    //path in tiles between portals on portal way
    
    size_t prev_p = portals_path[0];
    int prev_pX = dp.portals[prev_p].startX + (dp.portals[prev_p].endX - dp.portals[prev_p].startX) / 2;
    int prev_pY = dp.portals[prev_p].startY + (dp.portals[prev_p].endY - dp.portals[prev_p].startY) / 2;

    for (int i = 1; i < portals_path.size(); ++i)
    {
        int cur_pX = dp.portals[portals_path[i]].startX + (dp.portals[portals_path[i]].endX - dp.portals[portals_path[i]].startX) / 2;
        int cur_pY = dp.portals[portals_path[i]].startY + (dp.portals[portals_path[i]].endY - dp.portals[portals_path[i]].startY) / 2;

        int centerX = (prev_pX + cur_pX) / 2 + 1;
        int centerY = (prev_pY + cur_pY) / 2 + 1;

        size_t xx = (centerX - centerX % (ts));
        size_t yy = (centerY - centerY % (ts));

        limMin = { int((xx + 0)), int((yy + 0)) };
        limMax = { int((xx + ts)), int((yy + ts)) };

        cur_pX = std::clamp<int>(cur_pX, limMin.x, limMax.x);
        cur_pY = std::clamp<int>(cur_pY, limMin.y, limMax.y);
        prev_pX = std::clamp<int>(prev_pX, limMin.x, limMax.x);
        prev_pY = std::clamp<int>(prev_pY, limMin.y, limMax.y);

        curPath.clear();
        cur_path.clear();
        curPath = find_path_a_star(dd, { prev_pX , prev_pY }, { cur_pX , cur_pY }, limMin, limMax);
        IVec2_2_Pos(cur_path, curPath);
        concat<Position>(p.path, cur_path);

        prev_p = portals_path[i];
        prev_pX = cur_pX;
        prev_pY = cur_pY;
    }

    //path from "to" and last portal

    x = ((int)p.to.x - (int)p.to.x % (ts));
    y = ((int)p.to.y - (int)p.to.y % (ts));

    limMin = { int((x + 0)), int((y + 0)) };
    limMax = { int((x + ts)), int((y + ts)) };

    int toVecX = ((int)p.to.x);
    int toVecY = ((int)p.to.y);
    curPath.clear();
    cur_path.clear();
    curPath = find_path_a_star(dd, { prev_pX , prev_pY }, { toVecX , toVecY }, limMin, limMax);
    IVec2_2_Pos(cur_path, curPath);
    concat<Position>(p.path, cur_path);

}