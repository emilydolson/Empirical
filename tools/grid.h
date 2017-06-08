//  This file is part of Empirical, https://github.com/devosoft/Empirical
//  Copyright (C) Michigan State University, 2015-2017.
//  Released under the MIT Software license; see doc/LICENSE
//
//
//  Tools for building grids that can be easily navigated and have state associated with
//  grid cells, edges, and/or intersection points.
//  Status: BETA
//
//
//  Grid::Board objects are templated based on which of the components they work with.
//  They include three template arguments for the state types associated with cells,
//  edges, and instersection points.  The most commonly use types are:
//
//    int    -- full state; this or an enumerated type should be used for discrete states.
//    bool   -- binary state (on/off)
//    void   -- no state (or extra memory) associated with this component
//
//    A Sudoku puzzle (which only uses cells and regions) might be defined as:
//    Grid::Board<int, void, void>
//
//    A Sliterlink puzzle has binary states at edges and possibly cells (inside/outside):
//    Grid::Board<bool, bool, void>
//
//  Grid::Layout describes the layout of a grid, including its size and which cells
//  should be grouped together into a region.
//
//  Grid::StateSet is a helper templated class the holds a collection of states, or is
//  empty if given a template type of void.

#ifndef EMP_GRID_H
#define EMP_GRID_H

#include <type_traits>

#include "../base/assert.h"
#include "../base/vector.h"
#include "BitVector.h"

#include "../meta/meta.h"

namespace emp {
namespace Grid {

  template <typename CELL_TYPE> class Cell;
  template <typename EDGE_TYPE> class VEdge;
  template <typename EDGE_TYPE> class HEdge;
  template <typename POINT_TYPE> class Point;

  class Layout {
  private:
    int width;
    int height;
    emp::vector< emp::vector<int> > regions;
  public:
    Layout(int w, int h) : width(w), height(h) { ; }
    Layout(const Layout &) = default;
    ~Layout() { ; }
    Layout & operator=(const Layout &) = default;

    int GetWidth() const { return width; }
    int GetHeight() const { return height; }
    int GetNumRegions() const { return (int) regions.size(); }
    const emp::vector<int> & GetRegion(int id) { return regions[id]; }

    void AddRegion(const emp::vector<int> & in_region) { regions.push_back(in_region); }

    // Helper functions
    int GetX(int id) const { return id % width; }
    int GetY(int id) const { return id / width; }
    int GetID(int x, int y) const { return y*width + x; }

    int GetTopID(int id) const { return id; }                              // ID of edge at top
    int GetBottomID(int id) const { return id + width; }                   // ID of edge at bottom
    int GetLeftID(int id) const { return GetY(id)*(width+1) + GetX(id); }  // ID of edge at left
    int GetRightID(int id) const { return GetLeftID(id)+1; }               // ID of edge at right
  };

  template <typename STATE_TYPE>
  class StateSet {
  private:
    int width;
    emp::vector<STATE_TYPE> states;

  public:
    StateSet(int _w, int _h) : width(_w), states(_w*_h) { ; }
    StateSet(const StateSet &) = default;
    ~StateSet() { ; }
    StateSet & operator=(const StateSet &) = default;

    int GetWidth() const { return width; }
    int GetHeight() const { return states.size() / width; }
    int GetSize() const { return states.size(); }

    STATE_TYPE& operator()(int x, int y) {
        emp_assert(x >= 0 && x < width && "x coordinate out of range",x, width);
        emp_assert(y >= 0 && y < GetHeight()
                    && "y coordinate out of range", y, GetHeight());
        return states[y*width+x];
    }
    STATE_TYPE& operator()(int id) { return states[id]; }
    STATE_TYPE& operator[](int id) { return states[id]; }

    STATE_TYPE operator()(int x, int y) const {
    emp_assert(x >= 0 && x < width && "x coordinate out of range",x, width);
    emp_assert(y >= 0 && y < GetHeight()
                && "y coordinate out of range", y, GetHeight());
        return states[y*width+x];
    }
    STATE_TYPE operator()(int id) const { return states[id]; }
    STATE_TYPE operator[](int id) const { return states[id]; }


  };

  template <>
  class StateSet<bool> {
  private:
    int width;
    BitVector states;

  public:
    StateSet(int _w, int _h) : width(_w), states(_w*_h) { ; }
    StateSet(const StateSet &) = default;
    ~StateSet() { ; }
    StateSet & operator=(const StateSet &) = default;

    int GetWidth() const { return width; }
    int GetHeight() const { return states.size() / width; }
    int GetSize() const { return states.size(); }

    auto operator()(int x, int y) {
        emp_assert(x >= 0 && x < width && "x coordinate out of range",x, width);
        emp_assert(y >= 0 && y < GetHeight()
                && "y coordinate out of range", y, GetHeight());
        return states[y*width+x];
    }
    auto operator()(int id) { return states[id]; }
    auto operator[](int id) { return states[id]; }

    const bool operator()(int x, int y) const {
        emp_assert(x >= 0 && x < width && "x coordinate out of range",x, width);
        emp_assert(y >= 0 && y < GetHeight()
                && "y coordinate out of range", y, GetHeight());
        return states[y*width+x];
    }
    const bool operator()(int id) const { return states[id]; }
    const bool operator[](int id) const { return states[id]; }


  };

  // StateSet is specialized on void: no data is stored.
  template <>
  class StateSet<void> {
  public:
    StateSet(int _w, int _h) { (void) _w; (void) _h; }
    StateSet(const StateSet &) { ; }
    ~StateSet() { ; }
    StateSet & operator=(const StateSet &) { return *this; }

    int GetWidth() const { return -1; }
    int GetHeight() const { return -1; }
    int GetSize() const { return -1; }

    void operator()(int x, int y) { (void) x; (void) y; }
    void operator()(int id) { (void) id; }
    void operator[](int id) { (void) id; }
  };

  template <typename CELL_TYPE=int, typename EDGE_TYPE=void, class POINT_TYPE=void>
  class Board {
  private:
    const Layout & layout;
    StateSet<CELL_TYPE> cell_states;    // States of cells in grid.
    StateSet<EDGE_TYPE> edge_states_h;  // States of horizontal edges.
    StateSet<EDGE_TYPE> edge_states_v;  // States of vertical edges.
    StateSet<POINT_TYPE> point_states;  // States of points (where edges cross)

  public:

    Board(const Layout & in_layout)
      : layout(in_layout)
      , cell_states(layout.GetWidth(), layout.GetHeight())
      , edge_states_h(layout.GetWidth(), layout.GetHeight()+1)
      , edge_states_v(layout.GetWidth()+1, layout.GetHeight())
      , point_states(layout.GetWidth()+1, layout.GetHeight()+1)
    {
      // std::cout << "Built Board with " << cell_states.GetSize() << " cells!" << std::endl;
      // std::cout << "Built Board with " << edge_states_h.GetSize() << " h edges!" << std::endl;
      // std::cout << "Built Board with " << edge_states_v.GetSize() << " v edges!" << std::endl;
      // std::cout << "Built Board with " << point_states.GetSize() << " points!" << std::endl
      //           << std::endl;
    }

    const Layout & GetLayout() const { return layout; }

    CELL_TYPE GetCellValue(int id) const { return cell_states[id]; }
    EDGE_TYPE GetEdgeHValue(int id) const { return edge_states_h[id]; }
    EDGE_TYPE GetEdgeVValue(int id) const { return edge_states_v[id]; }
    POINT_TYPE GetPointValue(int id) const { return point_states[id]; }

    CELL_TYPE GetCellValue(int x, int y) const {
        emp_assert(x >= 0 && x < cell_states.GetWidth()
                                 && "x coordinate out of range",
                                 x, cell_states.GetWidth());
        emp_assert(y >= 0 && y < cell_states.GetHeight()
                                 && "y coordinate out of range",
                                 y, cell_states.GetHeight());
        return cell_states(x, y);
    }
    EDGE_TYPE GetEdgeHValue(int x, int y) const {
        emp_assert(x >= 0 && x < edge_states_h.GetWidth()
                                 && "x coordinate out of range",
                                 x, edge_states_h.GetWidth());
        emp_assert(y >= 0 && y < edge_states_h.GetHeight()
                                 && "y coordinate out of range",
                                 y, edge_states_h.GetHeight());
        return edge_states_h(x, y);
    }
    EDGE_TYPE GetEdgeVValue(int x, int y) const {
        emp_assert(x >= 0 && x < edge_states_v.GetWidth()
                                && "x coordinate out of range",
                                x, edge_states_v.GetWidth());
        emp_assert(y >= 0 && y < edge_states_v.GetHeight()
                                && "y coordinate out of range",
                                y, edge_states_v.GetHeight());
        return edge_states_v(x, y);
    }
    POINT_TYPE GetPointValue(int x, int y) const {
        emp_assert(x >= 0 && x < point_states.GetWidth()
                                 && "x coordinate out of range",
                                 x, point_states.GetWidth());
        emp_assert(y >= 0 && y < point_states.GetHeight()
                                 && "y coordinate out of range",
                                 y, point_states.GetHeight());
        return point_states(x, y);
    }


    // T is always going to be CELL_TYPE but we need to do it this way to use
    // enable_if to avoid building these methods when CELL_TYPE is void
    template <typename T>
    typename std::enable_if<!std::is_void<T>::value, void>::type
    SetCellValue(int id, T value) { cell_states[id] = value; }

    template <typename T>
    typename std::enable_if<std::is_void<T>::value, void>::type
    SetCellValue(int id, T value) {
        emp_assert(false &&
        "Attempting to call SetCellValue but CELL_TYPE is void");
    }

    template <typename T>
    typename std::enable_if<!std::is_void<T>::value, void>::type
    SetCellValue(int x, int y, T value) {
        emp_assert(x >= 0 && x < cell_states.GetWidth()
                                 && "x coordinate out of range",
                                 x, cell_states.GetWidth());
        emp_assert(y >= 0 && y < cell_states.GetHeight()
                                 && "y coordinate out of range",
                                 y, cell_states.GetHeight());
        cell_states(x, y) = value;
    }

    template <typename T>
    typename std::enable_if<std::is_void<T>::value, void>::type
    SetCellValue(int x, int y, T value) {
        emp_assert(false &&
        "Attempting to call SetCellValue but CELL_TYPE is void");
    }


    // T is always going to be POINT_TYPE but we need to do it this way to use
    // enable_if to avoid building these methods when CELL_TYPE is void
    template <typename T>
    typename std::enable_if<!std::is_void<T>::value, void>::type
    SetPointValue(int id, T value) { point_states[id] = value; }

    template <typename T>
    typename std::enable_if<std::is_void<T>::value, void>::type
    SetPointValue(int id, T value) {
        emp_assert(false &&
        "Attempting to call SetPointValue but POINT_TYPE is void");
    }

    template <typename T>
    typename std::enable_if<!std::is_void<T>::value, void>::type
    SetPointValue(int x, int y, T value) {
        emp_assert(x >= 0 && x < point_states.GetWidth()
                                && "x coordinate out of range",
                                x, point_states.GetWidth());
        emp_assert(y >= 0 && y < point_states.GetHeight()
                                && "y coordinate out of range",
                                y, point_states.GetHeight());

        point_states(x, y) = value;
    }

    template <typename T>
    typename std::enable_if<std::is_void<T>::value, void>::type
    SetPointValue(int x, int y, T value) {
        emp_assert(false &&
        "Attempting to call SetPointValue but POINT_TYPE is void");
    }


    void SetEdgeHValue(int id, EDGE_TYPE value) { edge_states_h[id] = value; }
    void SetEdgeHValue(int x, int y, EDGE_TYPE value) {
        emp_assert(x >= 0 && x < edge_states_h.GetWidth()
                                 && "x coordinate out of range",
                                 x, edge_states_h.GetWidth());
        emp_assert(y >= 0 && y < edge_states_h.GetHeight()
                                 && "y coordinate out of range",
                                 y, edge_states_h.GetHeight());

        edge_states_h(x, y) = value;
    }

    void SetEdgeVValue(int id, EDGE_TYPE value) { edge_states_v[id] = value; }
    void SetEdgeVValue(int x, int y, EDGE_TYPE value) {
        emp_assert(x >= 0 && x < edge_states_v.GetWidth()
                                && "x coordinate out of range",
                                x, edge_states_v.GetWidth());
        emp_assert(y >= 0 && y < edge_states_v.GetHeight()
                                && "y coordinate out of range",
                                y, edge_states_v.GetHeight());

        edge_states_v(x, y) = value;
    }

  };


  // template <typename CELL_TYPE>
  // class Cell {
  // private:
  //   Board<CELL_TYPE> & board;
  //   int id;
  // public:
  //   Cell(Board<CELL_TYPE> &b, int in_id) : board(b), id(in_id) { ; }
  //   Cell(const Cell &) = default;
  //   Cell & operator=(const Cell &) = default;
  //
  //   CELL_TYPE GetValue() const { return board.GetCellValue(id); }
  //   void SetValue(CELL_TYPE value) { board.SetCellValue(id, value); }
  // };
  //
  // template <typename EDGE_TYPE>
  // class VEdge {
  // private:
  //   Board<EDGE_TYPE> & board;
  //   int id;
  // public:
  //   VEdge(Board<EDGE_TYPE> & b, int in_id) : board(b), id(in_id) { ; }
  //   VEdge(const VEdge &) = default;
  //   VEdge & operator=(const VEdge &) = default;
  //
  //   EDGE_TYPE GetValue() const { return board.GetEdgeVValue(id); }
  //   void SetValue(EDGE_TYPE value) { board.SetEdgeVValue(id, value); }
  // };
  //
  // template <typename EDGE_TYPE>
  // class HEdge {
  // private:
  //   Board<EDGE_TYPE> & board;
  //   int id;
  // public:
  //   HEdge(Board<EDGE_TYPE> & b, int in_id) : board(b), id(in_id) { ; }
  //   HEdge(const HEdge &) = default;
  //   HEdge & operator=(const HEdge &) = default;
  //
  //   EDGE_TYPE GetValue() const { return board.GetEdgeHValue(id); }
  //   void SetValue(EDGE_TYPE value) { board.SetEdgeHValue(id, value); }
  // };
  //
  // template <typename POINT_TYPE>
  // class Point {
  // private:
  //   Board<POINT_TYPE> & board;
  //   int id;
  // public:
  //   Point(Board<> & b, int in_id) : board(b), id(in_id) { ; }
  //   Point(const Point &) = default;
  //   Point & operator=(const Point &) = default;
  //
  //   POINT_TYPE GetValue() const { return board.GetPointValue(id); }
  //   void SetValue(POINT_TYPE value) { board.SetPointValue(id, value); }
  // };

}
}


#endif
