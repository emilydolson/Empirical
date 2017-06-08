#include "evo/World.h"

#include "base/vector.h"
#include "base/array.h"
#include "base/Ptr.h"

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

#include "web/init.h"
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

const double MAZE_SIZE = 10;
const double MAX_TIME = 1000;
constexpr size_t POP_SIZE = 3;
constexpr size_t GENOME_SIZE = 50;
const emp::Grid::Layout layout(MAZE_SIZE, MAZE_SIZE);

class Maze {
protected:
    // Use a list because we're going to need to give Physics pointers to each
    std::list<emp::PhysicsBody2D<emp::Rect> > bodies;
    std::list<emp::PhysicsBodyOwner_Base<emp::PhysicsBody2D<emp::Rect> > > body_owners;
public:

    maze_genome_t genome;
    emp::Grid::Board<void, bool, void> grid;

    int fitness = 0;

    std::list<emp::PhysicsBody2D<emp::Rect> > & GetBodies(){return bodies;}
    std::list<emp::PhysicsBodyOwner_Base<emp::PhysicsBody2D<emp::Rect>> > & GetBodyOwners(){return body_owners;}

    Maze(maze_genome_t gen) : grid(layout){
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


    Maze(const Maze& m) : genome(m.genome), grid(m.grid) {
        DrawWalls();
    }

    ~Maze(){
        // for (auto ptr : bodies) {
        //     ptr.Delete();
        // }
    }

    void Evaluate(){
        for (auto owner : body_owners) {
            owner.Evaluate();
        }
    }

    void DrawWalls() {
          for (int i = 0; i < grid.GetLayout().GetWidth(); i++) {
              for (int j = 0; j < grid.GetLayout().GetHeight()+1; j++) {

                  if (grid.GetEdgeHValue(i, j)){
                    //   emp::Ptr<emp::PhysicsBody2D<emp::Rect>> body_ptr;
                    //   body_ptr.New(emp::Point(i*100,(j-.1)*100), emp::Point((i+1)*100, (j+.1)*100));
                      bodies.emplace_back(emp::Point(i*100,(j-.1)*100), emp::Point((i+1)*100, (j+.1)*100));

                  }
              }
          }
          for (int i = 0; i < grid.GetLayout().GetWidth()+1; i++) {
              for (int j = 0; j < grid.GetLayout().GetHeight(); j++) {
                  if (grid.GetEdgeVValue(i, j)){
                    //   emp::Ptr<emp::PhysicsBody2D<emp::Rect>> body_ptr;
                    //   body_ptr.New(emp::Point((i-.1)*100,j*100), emp::Point((i+.1)*100, (j+1)*100));
                      bodies.emplace_back(emp::Point((i-.1)*100,j*100), emp::Point((i+.1)*100, (j+1)*100));
                  }
              }
          }
          body_owners.resize(bodies.size());
          auto owner_it = body_owners.begin();
          for (auto body_it = bodies.begin(); body_it != bodies.end(); ++body_it) {
              emp_assert(owner_it != body_owners.end());
              owner_it->AttachBody(*body_it);
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
    emp::array<emp::Line, 6> range_finders;
    double fitness = 0;
    int stalled_steps = 0;
    emp::Point stall_point = emp::Point(20,20);

    // Body_t& GetBody(){return *body;}
    // emp::Ptr<Body_t> GetBodyPtr(){return body;}

    Solver(emp::AvidaGP genome) : genome(genome) {
        body.New(20, 20, 10);
        // std::cout << body.Raw() << std::endl;
        AttachBody(body);
        // SetBodyCleanup(false);
    }

    Solver(const Solver& s) : genome(s.genome) {
        body.New(20,20,10);
        // std::cout << "new copy " << body.Raw() << std::endl;
        AttachBody(body);
    }

    ~Solver(){
        // std::cout << "calling solver dest" << std::endl;
    }
};


using maze_t = Maze;
using solver_t = Solver;

using Physics_t = emp::MixedPhysics2D<emp::PhysicsBodyOwner_Base<emp::PhysicsBody2D<emp::Rect> >, solver_t>;

void AddMazeToPhysics(emp::Ptr<Physics_t> physics, emp::Ptr<maze_t> maze) {
    for (auto body : maze->GetBodyOwners()) {
        physics->AddBody(emp::Ptr<emp::PhysicsBodyOwner_Base<emp::PhysicsBody2D<emp::Rect> > >(&body));
    }
}


void PrintMaze(maze_t & maze) {
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
          emp::Ptr<Physics_t> physics,
          emp::Ptr<maze_t> maze,
          emp::Ptr<solver_t> solver) {
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

void Evaluate(emp::Ptr<solver_t> solver, emp::Ptr<maze_t> maze, emp::Ptr<Physics_t> physics) {
    // physics->Clear();
    // std::cout << "about to evalutate" << std::endl;
    AddMazeToPhysics(physics, maze);
    // std::cout << "maze added" << std::endl;
    physics->AddBody(solver);
    // std::cout << "solver added " << std::endl;
    emp_assert(!(solver->GetBodyPtr().IsNull()));
    solver->GetBodyPtr()->SetVelocity(emp::Point(0, 0));
    // std::cout << "Evaluate run" << std::endl;
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
    int generation = 0;

public:

    int GetGeneration(){ return generation;}

    WebInterface(double width, double height) : doc("emp_base"),
                                                canvas(doc.AddCanvas(width, height, "maze")),
                                                mazes(random, "MazeWorld"),
                                                solvers(random, "SolverWorld") {

       physics.ConfigPhysics(width, height, &random, .0025);
       physics.SetDefaultCollisionResolutionFunction(
                        [this](emp::Ptr<emp::TrackedType> body1, emp::Ptr<emp::TrackedType> body2){
                            // std::cout << "COLLISION!!!" << std::endl;
                            if (physics.body_tt.IsType<emp::Ptr<emp::PhysicsBody2D<emp::Rect>>>(*body1)) {
                                if (physics.body_tt.IsType<emp::Ptr<emp::PhysicsBody2D<emp::Circle>>>(*body2)) {
                                    physics.Deflect(physics.body_tt.ToType<emp::Ptr<emp::PhysicsBody2D<emp::Circle>>>(*body2),physics.body_tt.ToType<emp::Ptr<emp::PhysicsBody2D<emp::Rect>>>(*body1)) ;
                                }
                            } else {
                                if (physics.body_tt.IsType<emp::Ptr<emp::PhysicsBody2D<emp::Rect>>>(*body2)) {
                                    physics.Deflect(physics.body_tt.ToType<emp::Ptr<emp::PhysicsBody2D<emp::Circle>>>(*body1), physics.body_tt.ToType<emp::Ptr<emp::PhysicsBody2D<emp::Rect>>>(*body2)) ;
                                }

                            }
                        });

        mazes.SetDefaultFitnessFun([this](emp::Ptr<maze_t> maze){return maze->fitness;});
        solvers.SetDefaultFitnessFun([this](emp::Ptr<solver_t> solver){return solver->fitness;});

        // Setup the mutation function.
        solvers.SetDefaultMutateFun( [](emp::Ptr<solver_t> org, emp::Random& random) {
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
           solvers.Insert(cpu);
       }
    //   std::cout << "solvers made" << std::endl;
       for (size_t i = 0; i < POP_SIZE; i++) {
           maze_genome_t genome;
           genome.resize(1);
           genome[0][0] = random.GetDouble();
           genome[0][1] = random.GetDouble();
           std::cout << emp::to_string(genome[0]) << std::endl;
           mazes.Insert(genome);
       }
        //  std::cout << "mazes made" << std::endl;
       Evaluate(&(solvers[0]), &(mazes[0]), &physics);
        //  std::cout << "done with evaluate" << std::endl;
       doc << "Current solver: " << web::Live([this](){return curr_solver+1;}) << "/" << emp::to_string(POP_SIZE) <<" Current maze: "
            << web::Live([this](){return curr_maze+1;}) << "/" << emp::to_string(POP_SIZE) << " Current generation: " << web::Live([this](){return generation;}) ;

       Start(); //start animation DoFrame() will be run repeatedly
    }

  void NextGen() {
    //   std::cout << "Makign next gen" << std::endl;
    solvers.TournamentSelect(1, POP_SIZE);
    solvers.MutatePop(1);
    // mazes.TournamentSelect(5, POP_SIZE);

    mazes.Update();
    solvers.Update();

    curr_time = 0;
    curr_solver = 0;
    curr_maze = 0;
    generation++;
    //   std::cout << "done Makign next gen" << std::endl;
  }

  void NextEval(){
      // Make sure to clear physics first or we end up with stale pointers.
    //   mazes[curr_maze].Evaluate();
    //   solvers[curr_solver].Evaluate();
    //   physics.Update();
    //   std::cout << "dabout to clear physics" << std::endl;
    // std::cout << physics.GetBodySet().size() <<std::endl;
      physics.Clear();
    //   std::cout << "physics cleared" << std::endl;



    //   std::cout << "next eval" << std::endl;
      curr_time = 0;
      curr_solver++;
      if (curr_solver >= POP_SIZE){
          curr_solver = 0;
          curr_maze++;
          if (curr_maze >= POP_SIZE) {
              NextGen();
          }
      }
      emp_assert(solvers.IsOccupied(curr_solver));
      emp_assert(mazes.IsOccupied(curr_maze));
      doc.Redraw();
      Evaluate(&(solvers[curr_solver]), &(mazes[curr_maze]), &physics);
    //   std::cout << "done with next eval" << std::endl;
  }

  void EvaluateCurrSolver() {
      std::cout << "Evaluating solver" << std::endl;
      solver_t & solver = solvers[curr_solver];
      solver.genome.ResetHardware();

      emp::Point center = solver.GetBodyPtr()->GetShape().GetCenter();
      solver.range_finders[0] = emp::Line(center, emp::Point(center.GetX()+physics.GetWidth(), center.GetY()));
      solver.range_finders[1] = emp::Line(center, emp::Point(center.GetX()+physics.GetWidth()*0.5774, center.GetY()+physics.GetWidth()));
      solver.range_finders[2] = emp::Line(center, emp::Point(center.GetX()+physics.GetWidth()*0.5774, center.GetY()-physics.GetWidth()));
      solver.range_finders[3] = emp::Line(center, emp::Point(center.GetX()-physics.GetWidth(), center.GetY()));
      solver.range_finders[4] = emp::Line(center, emp::Point(center.GetX()-physics.GetWidth()*0.5774, center.GetY()-physics.GetWidth()));
      solver.range_finders[5] = emp::Line(center, emp::Point(center.GetX()-physics.GetWidth()*0.5774, center.GetY()+physics.GetWidth()));

      for (int i =0; i < solver.range_finders.size(); i++) {
          emp::vector<emp::Rect> intersected;
          for (auto body : physics.GetBodySet()) {
              if (physics.body_tt.IsType<emp::Ptr<emp::PhysicsBody2D<emp::Rect>>>(*body)) {
                  if (solver.range_finders[i].Intersects(physics.body_tt.ToType<emp::Ptr<emp::PhysicsBody2D<emp::Rect>>>(*body)->GetShape())) {
                      intersected.push_back(physics.body_tt.ToType<emp::Ptr<emp::PhysicsBody2D<emp::Rect>>>(*body)->GetShape());
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
      std::cout << "done with range finders" << std::endl;
      solver.genome.Process(200);
    //   std::cout << solver.genome.GetOutput(0) << " " << solver.genome.GetOutput(1) << std::endl;
      solver.GetBody().IncVelocity(emp::Point(solver.genome.GetOutput(0), solver.genome.GetOutput(1)));
      if (solver.GetBody().GetShape().GetCenter().IsClose(solver.stall_point, 1)) {
          solver.stalled_steps++;
          if (solver.stalled_steps > 10) {
              NextEval();
              return;
          }
      } else {
          solver.stall_point = solver.GetBody().GetShape().GetCenter();
        //   std::cout << solver.GetBodyPtr()->GetShape().GetCenter() << std::endl;
          solver.stalled_steps = 0;
      }
      std::cout << "done with running gp" << std::endl;
      if (!solver.HasBody()) {
          std::cout << "NO BODY! " << curr_solver << std::endl;
        //   std::cout << solver.GetBodyPtr().Raw() << std::endl;
      }
      solver.GetBody();
      emp::Point v = solver.GetBody().GetVelocity();
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
  std::cout << "veloctiy: " << v << std::endl;
      solver.GetBody().SetVelocity(v);

      physics.Update();
    //   std::cout << "physics updated " << v << std::endl;
      curr_time++;
      if (curr_time > MAX_TIME || center.Distance(emp::Point(physics.GetWidth(), physics.GetHeight())) < 30) {
          double score = center.Distance(emp::Point(physics.GetWidth(), physics.GetHeight()));
          solver.GetBody().GetShape().SetCenter(20,20);
          if (!score) {
              solver.fitness++;
              mazes[curr_maze].fitness++;
          } else {
              solver.fitness += 1/score;
          }
          NextEval();
      }

      #ifdef EMSCRIPTEN
      center = solver.GetBodyPtr()->GetShape().GetCenter();
      solver.range_finders[0] = emp::Line(center, emp::Point(center.GetX()+physics.GetWidth(), center.GetY()));
      solver.range_finders[1] = emp::Line(center, emp::Point(center.GetX()+physics.GetWidth()*0.5774, center.GetY()+physics.GetWidth()));
      solver.range_finders[2] = emp::Line(center, emp::Point(center.GetX()+physics.GetWidth()*0.5774, center.GetY()-physics.GetWidth()));
      solver.range_finders[3] = emp::Line(center, emp::Point(center.GetX()-physics.GetWidth(), center.GetY()));
      solver.range_finders[4] = emp::Line(center, emp::Point(center.GetX()-physics.GetWidth()*0.5774, center.GetY()-physics.GetWidth()));
      solver.range_finders[5] = emp::Line(center, emp::Point(center.GetX()-physics.GetWidth()*0.5774, center.GetY()+physics.GetWidth()));

      for (int i =0; i < solver.range_finders.size(); i++) {
          emp::vector<emp::Rect> intersected;
          for (auto body : physics.GetBodySet()) {
              if (physics.body_tt.IsType<emp::Ptr<emp::PhysicsBody2D<emp::Rect>>>(*body)) {
                  if (solver.range_finders[i].Intersects(physics.body_tt.ToType<emp::Ptr<emp::PhysicsBody2D<emp::Rect>>>(*body)->GetShape())) {
                      intersected.push_back(physics.body_tt.ToType<emp::Ptr<emp::PhysicsBody2D<emp::Rect>>>(*body)->GetShape());
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
      #endif

  }

  /** Draw a frame of the animation
   *
   */
  void DoFrame () {
    //   std::cout << "Doing frame" << std::endl;
      EvaluateCurrSolver();
      Draw(canvas, &physics, &(mazes[curr_maze]), &(solvers[curr_solver]));


    }

  };

#ifdef EMSCRIPTEN
WebInterface interface(1000,1000);
#endif

int main() {

#ifndef EMSCRIPTEN
WebInterface interface(1000,1000);
while (interface.GetGeneration() < 10){
    interface.EvaluateCurrSolver();
}

#endif

}
