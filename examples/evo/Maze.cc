#include "evo/World.h"

#include "base/vector.h"
#include "base/array.h"

#include "../../hardware/AvidaGP.h"
#include "geometry/Point2D.h"
#include "geometry/Circle2D.h"
#include "geometry/Rectangle2D.h"
#include "physics/Physics2D.h"
#include "physics/PhysicsBody2D.h"
#include "physics/PhysicsBodyOwner.h"

#include "tools/Random.h"
#include "tools/grid.h"

#include <queue>
#include <math.h>
#include <iostream>
#include <list>

#include "web/web.h"
#include "web/Document.h"
#include "web/Animate.h"
#include "web/Canvas.h"
#include "web/canvas_utils.h"
#include "web/emfunctions.h"

namespace web = emp::web;

using point_pair_t = emp::array<emp::Point2D<int>, 2>;
using line_t = emp::array<emp::Point2D<int>, 2>;

using maze_genome_t = emp::vector<emp::array<double, 2> >;


class Maze {
protected:
    // Use a list because we're going to need to give Physics pointers to each
    std::list<emp::PhysicsBody2D<emp::Rect> > bodies;
    std::list<emp::PhysicsBodyOwner_Base<emp::PhysicsBody2D<emp::Rect> > > body_owners;
public:
    maze_genome_t genome;
    emp::Grid::Board<void, bool, void> grid;

    std::list<emp::PhysicsBody2D<emp::Rect> > & GetBodies(){return bodies;}
    std::list<emp::PhysicsBodyOwner_Base<emp::PhysicsBody2D<emp::Rect> > > & GetBodyOwners(){return body_owners;}


    Maze(emp::Grid::Layout layout) :
        grid(emp::Grid::Board<void, bool, void>(layout)){;}

    void DrawWalls() {
          for (int i = 0; i < grid.GetLayout().GetWidth(); i++) {
              for (int j = 0; j < grid.GetLayout().GetHeight()+1; j++) {
                  if (grid.GetEdgeHValue(i, j)){
                      bodies.emplace_back(emp::Point(i*100,(j-.1)*100), emp::Point((i+1)*100, (j+.1)*100));
                      body_owners.emplace_back();
                      body_owners.back().AttachBody(&(bodies.back()));
                      bodies.back().SetImmobile(true);
                  }
              }
          }
          for (int i = 0; i < grid.GetLayout().GetWidth()+1; i++) {
              for (int j = 0; j < grid.GetLayout().GetHeight(); j++) {
                  if (grid.GetEdgeVValue(i, j)){
                      bodies.emplace_back(emp::Point((i-.1)*100,j*100), emp::Point((i+.1)*100, (j+1)*100));
                      body_owners.emplace_back();
                      body_owners.back().AttachBody(&(bodies.back()));
                      bodies.back().SetImmobile(true);

                  }
              }
          }

    }

};

class Solver : public emp::PhysicsBodyOwner_Base<emp::PhysicsBody2D<emp::Circle> > {
public:
    using Body_t = emp::PhysicsBody2D<emp::Circle>;
    emp::AvidaGP genome;
    using emp::PhysicsBodyOwner_Base<Body_t>::body;
    using emp::PhysicsBodyOwner_Base<Body_t>::has_body;
    Solver(emp::AvidaGP genome) : genome(genome) {
        body = nullptr;
        has_body = false;
        AttachBody(new Body_t(20, 20, 10));
        SetBodyCleanup(false);
    }
    ~Solver(){;}
};


using maze_t = Maze;
using solver_t = Solver;

using Physics_t = emp::MixedPhysics2D<emp::PhysicsBodyOwner_Base<emp::PhysicsBody2D<emp::Rect> >, solver_t>;

void AddMazeToPhysics(Physics_t * physics, maze_t * maze) {
    for (auto body : maze->GetBodyOwners()) {
        physics->AddBody(&body);
    }
}

double MAZE_SIZE = 10;
constexpr size_t POP_SIZE = 1;
constexpr size_t GENOME_SIZE = 50;

emp::Grid::Layout layout(MAZE_SIZE, MAZE_SIZE);

maze_t GenomeToMaze(maze_genome_t genome) {

    maze_t maze(layout);
    maze.genome = genome;
    std::queue<point_pair_t> boxes;

    point_pair_t curr;
    curr[0] = emp::Point2D<int>(0,0);
    curr[1] = emp::Point2D<int>(MAZE_SIZE, MAZE_SIZE);

    for (int i = 0; i < MAZE_SIZE; i++) {
        maze.grid.SetEdgeHValue(i, 0, true);
        maze.grid.SetEdgeHValue(i, MAZE_SIZE, true);
        maze.grid.SetEdgeVValue(0, i, true);
        maze.grid.SetEdgeVValue(MAZE_SIZE, i, true);
    }

    for (auto gene : genome) {
        int curr_width = curr[1].XDistance(curr[0]);
        int curr_height = curr[1].YDistance(curr[0]);

        // std::cout << "Curr: " << curr[0] << " " << curr[1] << " curr_width: " << curr_width << " curr_height: " << curr_height << std::endl;

        if (curr_width < curr_height) {
            // Subtract 1 because we can't draw over an external wall
            double loc = (curr_height - 1) * gene[0];
            double passage = (curr_width  - 1) * gene[1];

            int loc_i = ceil(loc);
            int pass_i = ceil(passage);

            // std::cout << "loc_i " << loc_i << " pass_i: " << pass_i << std::endl;

            for (int i = 0; i < curr_width; i++) {
                if (i != pass_i) {
                    // std::cout << "Setting edgeH " << i << " " << loc_i << std::endl;
                    maze.grid.SetEdgeHValue(curr[0].GetX()+i, loc_i, true);
                }
            }

            point_pair_t new_ul, new_lr;
            new_ul[0] = curr[0];
            new_ul[1].SetX(curr[1].GetX());
            new_ul[1].SetY(loc_i);

            new_lr[0].SetX(curr[0].GetX());
            new_lr[0].SetY(loc_i);
            new_lr[1] = curr[1];

            if (new_ul[1].XDistance(new_ul[0]) > 1 && new_ul[1].YDistance(new_ul[0]) > 1) {
                boxes.push(new_ul);
            }
            if (new_lr[1].XDistance(new_lr[0]) > 1 && new_lr[1].YDistance(new_lr[0]) > 1) {
                boxes.push(new_lr);
            }

        } else {
            // Subtract 1 because we can't draw over an external wall
            double loc = (curr_width - 1) * gene[0];
            double passage = (curr_height - 1) * gene[1];

            int loc_i = ceil(loc);
            int pass_i = ceil(passage);

            // std::cout << "loc_i " << loc_i << " pass_i: " << pass_i << std::endl;

            for (int i = 0; i < curr_height; i++) {
                if (i != pass_i) {
                    // std::cout << "Setting edgeV " << i << " " << loc_i << std::endl;
                    maze.grid.SetEdgeVValue(loc_i, curr[0].GetY()+i, true);
                }
            }

            point_pair_t new_ul, new_lr;
            new_ul[0] = curr[0];
            new_ul[1].SetX(loc_i);
            new_ul[1].SetY(curr[1].GetY());

            new_lr[0].SetX(loc_i);
            new_lr[0].SetY(curr[0].GetY());
            new_lr[1] = curr[1];

            if (new_ul[1].XDistance(new_ul[0]) > 1 && new_ul[1].YDistance(new_ul[0]) > 1) {
                boxes.push(new_ul);
            }
            if (new_lr[1].XDistance(new_lr[0]) > 1 && new_lr[1].YDistance(new_lr[0]) > 1) {
                boxes.push(new_lr);
            }
        }
        curr = boxes.front();
        boxes.pop();
    }
    maze.DrawWalls();
    return maze;
}

void PrintMaze(maze_t maze) {
    for (int i = 0; i < MAZE_SIZE; i++) {
        std::cout << " ";
        for (int j = 0; j < MAZE_SIZE; j++) {
            if (maze.grid.GetEdgeHValue(j, i)) {
                std::cout << "_";
            } else {
                std::cout << " ";
            }
            std::cout << " ";
        }
        std::cout << std::endl;
        for (int j = 0; j < MAZE_SIZE+1; j++) {
            if (maze.grid.GetEdgeVValue(j, i)) {
                std::cout << "|";
            } else {
                std::cout << " ";
                // std::cout << "     ";
            }
            std::cout << " ";
        }
        std::cout << std::endl;
    }

    std::cout << " ";
    for (int j = 0; j < MAZE_SIZE; j++) {
        if (maze.grid.GetEdgeHValue(j, MAZE_SIZE)) {
            std::cout << "_";
        } else {
            std::cout << " ";
        }
        std::cout << " ";
    }
    std::cout << std::endl;
}

void Draw(web::Canvas & canvas,
          Physics_t * physics,
          maze_t * maze,
          solver_t * solver) {
  std::cout << "In draw" << std::endl;
  canvas.Clear();
  const double w = physics->GetWidth();
  const double h = physics->GetHeight();
  // Setup a black background.
  canvas.Rect(0, 0, w, h, "black");
  // Draw organisms & resources.
  // const auto & orgs = world->GetConstPopulation();
  // const auto & reses = world->GetConstResources();
  // const auto & disps = world->GetConstDispensers();

  //Draw maze here

  for (auto body : maze->GetBodies()) {
      std::cout << "Drawing body" << std::endl;
      canvas.Draw(body.GetShape(), "white", "black");
  }
      std::cout << "done Drawing body" << std::endl;
  canvas.Circle(solver->GetBody().GetShape(), "green");
      std::cout << "done Drawing" << std::endl;
}

void Evaluate(solver_t * solver, maze_t * maze, Physics_t * physics) {
    physics->Clear();
    AddMazeToPhysics(physics, maze);
    physics->AddBody(solver);
}

class WebInterface : public web::Animate {
private:

    web::Document doc;
    web::Canvas canvas;

    emp::Random random;

    emp::evo::World<maze_t> mazes;
    emp::evo::World<solver_t> solvers;
    Physics_t physics;

public:

    WebInterface(double width, double height) : doc("emp_base"),
                                                canvas(doc.AddCanvas(width, height, "maze")),
                                                mazes(random, "MazeWorld"),
                                                solvers(random, "SolverWorld") {

       physics.ConfigPhysics(width, height, &random, .0025);

       for (size_t i = 0; i < POP_SIZE; i++) {
           emp::AvidaGP cpu;
           cpu.PushRandom(random, GENOME_SIZE);
           solver_t solver(cpu);
           solvers.Insert(solver);
       }

       for (size_t i = 0; i < POP_SIZE; i++) {
           maze_genome_t genome;
           genome.resize(1);
           genome[0][0] = random.GetDouble();
           genome[0][1] = random.GetDouble();
           std::cout << emp::to_string(genome[0]) << std::endl;
           maze_t maze = GenomeToMaze(genome);
           mazes.Insert(maze);
       }

       doc << "<h1>Hello, World!</h1>" ;

       Start(); //start animation DoFrame() will be run repeatedly
    }

  /** Draw a frame of the animation
   *
   */
  void DoFrame () {
      Evaluate(&(solvers[0]), &(mazes[0]), &physics);
      Draw(canvas, &physics, &(mazes[0]), &(solvers[0]));
    }

  };

WebInterface interface(1000,1000);

int main() {

}
