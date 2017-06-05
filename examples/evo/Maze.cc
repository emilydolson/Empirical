#include "evo/World.h"

#include "base/vector.h"
#include "base/array.h"

#include "../../hardware/AvidaGP.h"
#include "geometry/Point2D.h"
#include "geometry/Line2D.h"
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

double MAZE_SIZE = 10;
double MAX_TIME = 1000;
constexpr size_t POP_SIZE = 5;
constexpr size_t GENOME_SIZE = 50;


class Maze {
protected:
    // Use a list because we're going to need to give Physics pointers to each
    std::list<emp::PhysicsBody2D<emp::Rect> > bodies;
    std::list<emp::PhysicsBodyOwner_Base<emp::PhysicsBody2D<emp::Rect> > > body_owners;
public:
    emp::Grid::Layout layout;
    maze_genome_t genome;
    emp::Grid::Board<void, bool, void> grid;

    int fitness = 0;

    std::list<emp::PhysicsBody2D<emp::Rect> > & GetBodies(){return bodies;}
    std::list<emp::PhysicsBodyOwner_Base<emp::PhysicsBody2D<emp::Rect>> > & GetBodyOwners(){return body_owners;}

    Maze(maze_genome_t gen) : layout(MAZE_SIZE, MAZE_SIZE), grid(layout){
        genome = gen;
        std::queue<point_pair_t> boxes;
        point_pair_t curr;
        curr[0] = emp::Point2D<int>(0,0);
        curr[1] = emp::Point2D<int>(MAZE_SIZE, MAZE_SIZE);

        for (int i = 0; i < MAZE_SIZE; i++) {
            grid.SetEdgeHValue(i, 0, true);
            grid.SetEdgeHValue(i, MAZE_SIZE, true);
            grid.SetEdgeVValue(0, i, true);
            grid.SetEdgeVValue(MAZE_SIZE, i, true);
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
                        grid.SetEdgeHValue(curr[0].GetX()+i, loc_i, true);
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
                        grid.SetEdgeVValue(loc_i, curr[0].GetY()+i, true);
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
        DrawWalls();
    }

    Maze(emp::Grid::Layout layout) :
        layout(layout),
        grid(emp::Grid::Board<void, bool, void>(layout)){;}

    Maze(const Maze& m) : layout(m.layout), genome(m.genome), grid(m.grid) {
        DrawWalls();
    }

    void DrawWalls() {
          for (int i = 0; i < grid.GetLayout().GetWidth(); i++) {
              for (int j = 0; j < grid.GetLayout().GetHeight()+1; j++) {
                  if (grid.GetEdgeHValue(i, j)){
                      bodies.emplace_back(emp::Point(i*100,(j-.1)*100), emp::Point((i+1)*100, (j+.1)*100));
                  }
              }
          }
          for (int i = 0; i < grid.GetLayout().GetWidth()+1; i++) {
              for (int j = 0; j < grid.GetLayout().GetHeight(); j++) {
                  if (grid.GetEdgeVValue(i, j)){
                      bodies.emplace_back(emp::Point((i-.1)*100,j*100), emp::Point((i+.1)*100, (j+1)*100));
                  }
              }
          }
          body_owners.resize(bodies.size());
          auto owner_it = body_owners.begin();
          for (auto body_it = bodies.begin(); body_it != bodies.end(); ++body_it) {
              emp_assert(owner_it != body_owners.end());
              owner_it->AttachBody(&(*body_it));
              body_it->SetImmobile(true);
              owner_it->SetBodyCleanup(false);
              ++owner_it;
          }

    }

};

class Solver : public emp::PhysicsBodyOwner_Base<emp::PhysicsBody2D<emp::Circle> > {
public:
    using Body_t = emp::PhysicsBody2D<emp::Circle>;
    emp::AvidaGP genome;
    using emp::PhysicsBodyOwner_Base<Body_t>::body;
    using emp::PhysicsBodyOwner_Base<Body_t>::has_body;
    emp::array<emp::Line, 6> range_finders;
    double fitness = 0;

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
  // std::cout << "In draw" << std::endl;
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

  for (auto& body : maze->GetBodies()) {
    //   std::cout << "Drawing body" << std::endl;
      canvas.Draw(body.GetShape(), "white", "black");
  }
    //   std::cout << "done Drawing body" << std::endl;
  canvas.Circle(solver->GetBody().GetShape(), "green");
  canvas.Draw(solver->range_finders[0], "red");
  canvas.Draw(solver->range_finders[1], "red");
  canvas.Draw(solver->range_finders[2], "red");
  canvas.Draw(solver->range_finders[3], "red");
  canvas.Draw(solver->range_finders[4], "red");
  canvas.Draw(solver->range_finders[5], "red");

    //   std::cout << "done Drawing" << std::endl;
}

void Evaluate(solver_t * solver, maze_t * maze, Physics_t * physics) {
    // physics->Clear();
    AddMazeToPhysics(physics, maze);
    physics->AddBody(solver);
    solver->body->SetVelocity(emp::Point(2, 2));
}

class WebInterface : public web::Animate {
private:

    web::Document doc;
    web::Canvas canvas;

    emp::Random random;

    emp::evo::World<maze_t> mazes;
    emp::evo::World<solver_t> solvers;
    Physics_t physics;

    int curr_solver = 0;
    int curr_maze = 0;
    int curr_time = 0;

public:

    WebInterface(double width, double height) : doc("emp_base"),
                                                canvas(doc.AddCanvas(width, height, "maze")),
                                                mazes(random, "MazeWorld"),
                                                solvers(random, "SolverWorld") {

       physics.ConfigPhysics(width, height, &random, .0000025);
       physics.SetDefaultCollisionResolutionFunction(
                        [this](emp::TrackedType * body1, emp::TrackedType * body2){
                            // std::cout << "COLLISION!!!" << std::endl;
                            if (physics.body_tt.IsType<emp::PhysicsBody2D<emp::Rect>*>(*body1)) {
                                if (physics.body_tt.IsType<emp::PhysicsBody2D<emp::Circle>*>(*body2)) {
                                    physics.Deflect(physics.body_tt.ToType<emp::PhysicsBody2D<emp::Circle>*>(*body2),physics.body_tt.ToType<emp::PhysicsBody2D<emp::Rect>*>(*body1)) ;
                                }
                            } else {
                                if (physics.body_tt.IsType<emp::PhysicsBody2D<emp::Rect>*>(*body2)) {
                                    physics.Deflect(physics.body_tt.ToType<emp::PhysicsBody2D<emp::Circle>*>(*body1), physics.body_tt.ToType<emp::PhysicsBody2D<emp::Rect>*>(*body2)) ;
                                }

                            }
                        });

        mazes.SetDefaultFitnessFun([this](maze_t * maze){return maze->fitness;});
        solvers.SetDefaultFitnessFun([this](solver_t * solver){return solver->fitness;});

        // Setup the mutation function.
        solvers.SetDefaultMutateFun( [](solver_t * org, emp::Random& random) {
            uint32_t num_muts = random.GetUInt(4);  // 0 to 3 mutations.
            for (uint32_t m = 0; m < num_muts; m++) {
              const uint32_t pos = random.GetUInt(GENOME_SIZE);
              org->genome.RandomizeInst(pos, random);
            }
            return (num_muts > 0);
          } );

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
           mazes.Insert(genome);
       }
       Evaluate(&(solvers[0]), &(mazes[0]), &physics);
       doc << "<h1>Hello, World!</h1>" ;

       Start(); //start animation DoFrame() will be run repeatedly
    }

  void NextGen() {
    solvers.TournamentSelect(5, POP_SIZE);
    solvers.MutatePop(1);
    mazes.TournamentSelect(5, POP_SIZE);

    mazes.Update();
    solvers.Update();

    curr_time = 0;
    curr_solver = 0;
    curr_maze = 0;
  }

  void NextEval(){
      curr_time = 0;
      curr_solver++;
      if (curr_solver >= POP_SIZE){
          curr_solver = 0;
          curr_maze++;
          if (curr_maze >= POP_SIZE) {
              NextGen();
          }
      }
      physics.Clear();
      Evaluate(&(solvers[curr_solver]), &(mazes[curr_maze]), &physics);
  }

  void EvaluateSolver(solver_t & solver) {
      solvers[0].genome.ResetHardware();

      emp::Point center = solver.body->GetShape().GetCenter();
      solver.range_finders[0] = emp::Line(center, emp::Point(center.GetX()+physics.GetWidth(), center.GetY()));
      solver.range_finders[1] = emp::Line(center, emp::Point(center.GetX()+physics.GetWidth()*0.5774, center.GetY()+physics.GetWidth()));
      solver.range_finders[2] = emp::Line(center, emp::Point(center.GetX()+physics.GetWidth()*0.5774, center.GetY()-physics.GetWidth()));
      solver.range_finders[3] = emp::Line(center, emp::Point(center.GetX()-physics.GetWidth(), center.GetY()));
      solver.range_finders[4] = emp::Line(center, emp::Point(center.GetX()-physics.GetWidth()*0.5774, center.GetY()-physics.GetWidth()));
      solver.range_finders[5] = emp::Line(center, emp::Point(center.GetX()-physics.GetWidth()*0.5774, center.GetY()+physics.GetWidth()));

      for (int i =0; i < solver.range_finders.size(); i++) {
          emp::vector<emp::Rect> intersected;
          for (auto body : physics.GetBodySet()) {
              if (physics.body_tt.IsType<emp::PhysicsBody2D<emp::Rect>*>(*body)) {
                  if (solver.range_finders[i].Intersects(physics.body_tt.ToType<emp::PhysicsBody2D<emp::Rect>*>(*body)->GetShape())) {
                      intersected.push_back(physics.body_tt.ToType<emp::PhysicsBody2D<emp::Rect>*>(*body)->GetShape());
                  }
              }
          }
          double closest = physics.GetWidth()*physics.GetWidth();
          emp::Point closest_p = center;
          for (auto rect : intersected) {
              if (center.Distance(rect.GetCenter()) < closest) {
                  closest = center.Distance(rect.GetCenter());
                  closest_p = rect.GetCenter();
              }
          }
          solver.range_finders[i].SetP2(closest_p);
          solver.genome.SetInput(i, closest);

      }

      solver.genome.Process(200);
      std::cout << solver.genome.GetOutput(0) << " " << solver.genome.GetOutput(1) << std::endl;
      solver.body->IncVelocity(emp::Point(solver.genome.GetOutput(0), solver.genome.GetOutput(1)));
      emp::Point v = solver.body->GetVelocity();
      if (v.GetX() >= 10) {
          v.SetX(9.9);
      } else if (v.GetX() <= -10) {
          v.SetX(-9.9);
      }

      if (v.GetY() >= 10) {
          v.SetY(9.9);
      } else if(v.GetY() <= -10){
          v.SetY(-9.9);
      }
      solver.body->SetVelocity(v);
      physics.Update();
      curr_time++;
      if (curr_time > MAX_TIME || center.Distance(emp::Point(physics.GetWidth(), physics.GetHeight())) < 20) {
          double score = center.Distance(emp::Point(physics.GetWidth(), physics.GetHeight()));
          solver.body->GetShape().SetCenter(20,20);
          if (!score) {
              solver.fitness++;
              mazes[curr_maze].fitness++;
          } else {
              solver.fitness += 1/score;
          }
          NextEval();
      }
  }

  /** Draw a frame of the animation
   *
   */
  void DoFrame () {
      Draw(canvas, &physics, &(mazes[curr_maze]), &(solvers[curr_solver]));
      EvaluateSolver(solvers[curr_solver]);

    }

  };

WebInterface interface(1000,1000);

int main() {

}
