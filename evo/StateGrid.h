//  This file is part of Empirical, https://github.com/devosoft/Empirical/
//  Copyright (C) Michigan State University, 2017.
//  Released under the MIT Software license; see doc/LICENSE
//
//
//  StateGrid maintains a rectilinear grid that agents can traverse.
//
//
//  Developer Notes:
//  * Functions such as Load() should throw exceptions (or equilv.), not use asserts.
//  * Need to figure out a default mapping for how outputs translate to moves around a state grid.
//    -1 = Back up
//     0 = Turn left
//     1 = Move fast-forwards
//     2 = Turn right

#ifndef EMP_EVO_STATE_GRID_H
#define EMP_EVO_STATE_GRID_H

#include <map>
#include <string>

#include "../base/vector.h"
#include "../tools/File.h"
#include "../tools/map_utils.h"

namespace emp {
namespace evo {

  /// Full information about a state grid and meanings of each state.
  class StateGridInfo {
  protected:

    /// Information about what a particular state type means in a state grid.
    struct StateInfo {
      int state_id;       // Ordinal id for this state.
      char symbol;        // Symbol for printing this state.
      double score_mult;  // Change factor for organism score by stepping on this squre.
      std::string name;   // Name of this state.
      std::string desc;   // Explanation of this state.

      StateInfo(int _id, char _sym, double _mult,
                const std::string & _name, const std::string & _desc)
      : state_id(_id), symbol(_sym), score_mult(_mult), name(_name), desc(_desc) { ; }
    };

    emp::vector<StateInfo> states;         // All available states.  Position is key ID

    std::map<int, size_t> state_map;          // Map of state_id to key ID (state_id can be < 0)
    std::map<char, size_t> symbol_map;        // Map of symbols to associated key ID
    std::map<std::string, size_t> name_map;   // Map of names to associated key ID

    size_t GetKey(int state_id) const { return Find(state_map, state_id, 0); }
    size_t GetKey(char symbol) const { return Find(symbol_map, symbol, 0); }
    size_t GetKey(const std::string & name) const { return Find(name_map, name, 0); }
  public:
    StateGridInfo() : states(), state_map(), symbol_map(), name_map() { ; }
    ~StateGridInfo() { ; }

    size_t GetNumStates() const { return states.size(); }

    // Convert from state ids...
    char GetSymbol(int state_id) const { return states[ GetKey(state_id) ].symbol; }
    double GetScoreMult(int state_id) const { return states[ GetKey(state_id) ].score_mult; }
    const std::string & GetName(int state_id) const { return states[ GetKey(state_id) ].name; }
    const std::string & GetDesc(int state_id) const { return states[ GetKey(state_id) ].desc; }

    // Convert to state ids...
    int GetState(char symbol) const { return states[ GetKey(symbol) ].state_id; }
    int GetState(const std::string & name) const { return states[ GetKey(name) ].state_id; }

    void AddState(int id, char symbol, double mult=1.0, std::string name="", std::string desc="") {
      size_t key_id = states.size();
      states.emplace_back(id, symbol, mult, name, desc);
      state_map[id] = key_id;
      symbol_map[symbol] = key_id;
      name_map[name] = key_id;
    }
  };

  /// A StateGrid describes a map of grid positions to the current state of each position.
  class StateGrid {
  protected:
    size_t width;
    size_t height;
    emp::vector<int> states;

    StateGridInfo & info;

  public:
    StateGrid(StateGridInfo & _i, size_t _w=1, size_t _h=1, int init_val=0)
      : width(_w), height(_h), states(_w*_h,init_val), info(_i) { ; }
    StateGrid(StateGridInfo & _i, const std::string & filename)
      : width(1), height(1), states(), info(_i) { Load(filename); }
    StateGrid(const StateGrid &) = default;
    StateGrid(StateGrid &&) = default;
    ~StateGrid() { ; }

    StateGrid & operator=(const StateGrid &) = default;
    StateGrid & operator=(StateGrid &&) = default;

    size_t GetWidth() const { return width; }
    size_t GetHeight() const { return height; }
    size_t GetSize() const { return states.size(); }
    const StateGridInfo & GetInfo() const { return info; }

    int & operator()(size_t x, size_t y) { return states[y*width+x]; }
    int operator()(size_t x, size_t y) const { return states[y*width+x]; }
    int GetState(size_t x, size_t y) const { return states[y*width+x]; }
    StateGrid & SetState(size_t x, size_t y, int in) { states[y*width+x] = in; return *this; }

    char GetSymbol(size_t x, size_t y) const { return info.GetSymbol(GetState(x,y)); }
    double GetScoreMult(size_t x, size_t y) const { return info.GetScoreMult(GetState(x,y)); }
    const std::string & GetName(size_t x, size_t y) const { return info.GetName(GetState(x,y)); }

    template <typename... Ts>
    StateGrid & Load(Ts &&... args) {
      // Load this data from a stream or a file.
      File file(std::forward<Ts>(args)...);
      file.RemoveWhitespace();

      // Determine the size of the new grid.
      height = file.GetNumLines();
      emp_assert(height > 0);
      width = file[0].size();
      emp_assert(width > 0);

      // Now that we have the new size, resize the state grid.
      size_t size = width * height;
      states.resize(size);

      // Load in the specific states.
      for (size_t row = 0; row < height; row++) {
        emp_assert(file[row].size() == width);  // Make sure all rows are the same size.
        for (size_t col = 0; col < width; col++) {
          states[row*width+col] = info.GetState(file[row][col]);
        }
      }

      return *this;
    }

    template <typename... Ts>
    StateGrid & Write(Ts &&... args) {
      File file;
      std::string out;
      for (size_t i = 0; i < height; i++) {
        out.resize(0);
        out += info.GetSymbol( states[i*width] );
        for (size_t j = 1; j < width; j++) {
          out += ' ';
          out +=info.GetSymbol( states[i*width+j] );
        }
        file.Append(out);
      }
      file.Write(std::forward<Ts>(args)...);
      return *this;
    }
  };

  class StateGridStatus {
  protected:
    size_t x;
    size_t y;
    size_t facing;  // 0=UL, 1=Up, 2=UR, 3=Right, 4=DR, 5=Down, 6=DL, 7=Left (+=Clockwise)
  public:
    StateGridStatus() : x(0), y(0), facing(1) { ; }
    StateGridStatus(const StateGridStatus &) = default;
    StateGridStatus(StateGridStatus &&) = default;

    StateGridStatus & operator=(const StateGridStatus &) = default;
    StateGridStatus & operator=(StateGridStatus &&) = default;

    size_t GetX() const { return x; }
    size_t GetY() const { return y; }
    size_t GetFacing() const { return facing; }

    StateGridStatus & SetX(size_t _x) { x = _x; return *this; }
    StateGridStatus & SetY(size_t _y) { y = _y; return *this; }
    StateGridStatus & SetPos(size_t _x, size_t _y) { x = _x; y = _y; return *this; }
    StateGridStatus & SetFacing(size_t _f) { facing = _f; return *this; }

    void MoveX(const StateGrid & grid, int steps=1) {
      int tmp_x = (steps + (int) x) % (int) grid.GetWidth();
      x = (size_t) ((tmp_x < 0) ? (tmp_x + (int) grid.GetWidth()) : tmp_x);
    }

    void MoveY(const StateGrid & grid, int steps=1) {
      int tmp_y = (steps + (int) y) % (int) grid.GetHeight();
      y = (size_t) ((tmp_y < 0) ? (tmp_y + (int) grid.GetHeight()) : tmp_y);
    }

    void Move(const StateGrid & grid, int steps=1) {
      switch (facing) {
        case 0: MoveX(grid, -steps); MoveY(grid, -steps); break;
        case 1:                      MoveY(grid, -steps); break;
        case 2: MoveX(grid, +steps); MoveY(grid, -steps); break;
        case 3: MoveX(grid, +steps);                      break;
        case 4: MoveX(grid, +steps); MoveY(grid, +steps); break;
        case 5:                      MoveY(grid, +steps); break;
        case 6: MoveX(grid, -steps); MoveY(grid, +steps); break;
        case 7: MoveX(grid, -steps);                      break;
      }
    }

    void Rotate(int turns=1) {
      facing = (facing + turns % 8 + 8) % 8;
    }

    int Scan(const StateGrid & grid) {
      return grid(x,y);
    }
  };

}
}

#endif
