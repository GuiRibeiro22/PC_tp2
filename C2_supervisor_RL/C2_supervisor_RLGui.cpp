/*
 * world_builder C++ supervisor controller (RL episodic version)
 */

#include <webots/Supervisor.hpp>
#include <webots/Emitter.hpp>

#include <iostream>
#include <string>
#include <math.h>
#include <stdlib.h>
#include <unistd.h>
#include <sys/types.h>
#include <signal.h>

#include <QFile>
#include <QXmlSimpleReader>
#include <QXmlInputSource>

#include "cblabhandler.h"

#define M_PI 3.14159265358979323846
#define PATHCUBESIZE (0.15)

// ---------------- GLOBALS ----------------

double MAX_TIME_SECONDS = 25.0;

struct cell_t {
  int x, y;
} controlCellPath[1024];

int nCellPath = 0;
int nextPathInd = 0;
int scoreControl = 0;
int lastScoreControl = 0;

webots::Node *epuck_node = nullptr;

// ---------------- UTILS ----------------

pid_t get_sibling_pid() {
  char command[256];
  snprintf(command, sizeof(command),
           "pgrep -P %d | grep -v %d", getppid(), getpid());

  FILE *fp = popen(command, "r");
  if (!fp) return 0;

  char buffer[64];
  pid_t pid = 0;
  while (fgets(buffer, sizeof(buffer), fp))
    pid = atoi(buffer);

  pclose(fp);
  return pid;
}

bool is_pid_running(int pid) {
  return (pid > 0 && kill(pid, 0) == 0);
}

// ---------------- PATH / SCORE ----------------

struct cell_t getRobotCell() {
  struct cell_t cell = {0, 0};
  if (epuck_node) {
    const double *pos = epuck_node->getPosition();
    cell.x = static_cast<int>(pos[0] / PATHCUBESIZE);
    cell.y = static_cast<int>(pos[1] / PATHCUBESIZE);
  }
  return cell;
}

void build_cell_path(cbLab *lab) {
  controlCellPath[0].x = lab->Target(0)->Center().x / PATHCUBESIZE;
  controlCellPath[0].y = lab->Target(0)->Center().y / PATHCUBESIZE;

  struct cell_t newCell = controlCellPath[0];
  newCell.x++;
  nCellPath = 1;
  int dir = 0;

  while (newCell.x != controlCellPath[0].x ||
         newCell.y != controlCellPath[0].y) {

    controlCellPath[nCellPath++] = newCell;

    int test_dirs[3] = {0, -90, 90};
    bool moved = false;

    for (int d = 0; d < 3 && !moved; d++) {
      double angle = (dir + test_dirs[d]) * M_PI / 180.0;
      int nx = round(newCell.x + cos(angle));
      int ny = round(newCell.y + sin(angle));

      if (lab->reachable(
            cbPoint(newCell.x * PATHCUBESIZE, newCell.y * PATHCUBESIZE),
            cbPoint(nx * PATHCUBESIZE, ny * PATHCUBESIZE))) {
        newCell.x = nx;
        newCell.y = ny;
        dir = (dir + test_dirs[d] + 360) % 360;
        moved = true;
      }
    }

    if (!moved) {
      std::cerr << "Lab has no closed loop.\n";
      exit(1);
    }
  }
}

void update_score() {
  struct cell_t cur = getRobotCell();
  if (cur.x == controlCellPath[nextPathInd].x &&
      cur.y == controlCellPath[nextPathInd].y) {
    nextPathInd = (nextPathInd + 1) % nCellPath;
    scoreControl++;
  }
}

// VERY SIMPLE collision proxy (replace later if needed)
bool check_collision() {
  struct cell_t c = getRobotCell();
  return (abs(c.x) > 300 || abs(c.y) > 300);
}

// ---------------- MAIN ----------------

int main(int argc, char **argv) {

  webots::Supervisor *supervisor = new webots::Supervisor();
  int timeStep = supervisor->getBasicTimeStep();

  webots::Emitter *emitter = supervisor->getEmitter("emitter");

  // MAX_TIME from env
  if (char *env = getenv("MAX_TIME"))
    MAX_TIME_SECONDS = atof(env);

  // ---------- LOAD LAB ----------
  QFile srcFile("C2-lab.xml");
  if (!srcFile.exists()) {
    std::cerr << "Lab file not found\n";
    return 1;
  }

  QXmlInputSource source(&srcFile);
  cbLabHandler labHandler;

  webots::Node *root = supervisor->getRoot();
  webots::Field *children = root->getField("children");
  labHandler.setChildrenField(children);

  QXmlSimpleReader reader;
  reader.setContentHandler(&labHandler);
  reader.setErrorHandler(&labHandler);

  if (!reader.parse(&source)) {
    std::cerr << "XML parse error\n";
    return 1;
  }

  build_cell_path(labHandler.getLab());

  // ---------- ROBOT ----------
  epuck_node = supervisor->getFromDef("EPUCK");
  webots::Field *translation = epuck_node->getField("translation");

  double startPos[3] = {
    labHandler.getLab()->Target(0)->Center().x,
    labHandler.getLab()->Target(0)->Center().y,
    0.0
  };
  translation->setSFVec3f(startPos);

  int robot_pid = get_sibling_pid();

  // ---------- EPISODE STATE ----------
  double episodeReward = 0.0;
  double episodeStartTime = supervisor->getTime();

  // ================= MAIN LOOP =================
  while (supervisor->step(timeStep) != -1) {

    double simTime = supervisor->getTime();

    if (robot_pid && !is_pid_running(robot_pid))
      continue;

    // ---- SCORE UPDATE ----
    update_score();

    int delta = scoreControl - lastScoreControl;
    if (delta > 0)
      episodeReward += delta * 1.0;   // +1 per checkpoint

    lastScoreControl = scoreControl;

    // ---- COLLISION ----
    if (check_collision()) {
      episodeReward -= 5.0;

      float rewardToSend = (float)episodeReward;
      emitter->send(&rewardToSend, sizeof(float));

      translation->setSFVec3f(startPos);
      epuck_node->resetPhysics();

      episodeReward = 0.0;
      scoreControl = 0;
      lastScoreControl = 0;
      nextPathInd = 0;
      episodeStartTime = supervisor->getTime();

      supervisor->simulationResetPhysics();
      continue;
    }

    // ---- DISPLAY ----
    std::string label = "Score: " + std::to_string(scoreControl);
    supervisor->setLabel(0, label, 0.6, 0.01, 0.1, 0xFF0000, 0.0, "Arial");

    // ---- EPISODE END (TIME) ----
    if ((simTime - episodeStartTime) >= MAX_TIME_SECONDS) {

      float rewardToSend = (float)episodeReward;
      emitter->send(&rewardToSend, sizeof(float));

      translation->setSFVec3f(startPos);
      epuck_node->resetPhysics();

      episodeReward = 0.0;
      scoreControl = 0;
      lastScoreControl = 0;
      nextPathInd = 0;
      episodeStartTime = supervisor->getTime();

      supervisor->simulationResetPhysics();
      continue;
    }
  }

  delete supervisor;
  return 0;
}

