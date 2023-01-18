
#include <stdio.h>
#include <octomap/octomap.h>
#include <octomap/math/Utils.h>

using namespace std;
using namespace octomap;

void printChanges(OcTree& tree){
  unsigned int changedOccupied = 0;
  unsigned int changedFree = 0;
  unsigned int actualOccupied = 0;
  unsigned int actualFree = 0;
  unsigned int missingChanged = 0;

  tree.expand();

  // iterate through the changed nodes
  KeyBoolMap::const_iterator it;
  for (it=tree.changedKeysBegin(); it!=tree.changedKeysEnd(); it++) {
    OcTreeNode* node = tree.search(it->first);
    if (node != NULL) {
      if (tree.isNodeOccupied(node)) {
        changedOccupied += 1;
      }
      else {
        changedFree += 1;
      }
    } else {
      missingChanged +=1;
    }
  }


  // iterate through the entire tree
  for(OcTree::tree_iterator it=tree.begin_tree(),
      end=tree.end_tree(); it!= end; ++it) {
    if (it.isLeaf()) {
      if (tree.isNodeOccupied(*it)) {
        actualOccupied += 1;
      }
      else {
        actualFree += 1;
      }
    }
  }
  
  cout << "change detection: " << changedOccupied << " occ; " << changedFree << " free; "<< missingChanged << " missing" << endl;
  cout << "actual: " << actualOccupied << " occ; " << actualFree << " free; " << endl;

  tree.prune();
}

void PrintNodeVal(OcTree &tree, point3d &pt)
{
  std::cout << "------------------\n";
  auto node = tree.updateNode(pt, 0.0f, false);
  std::cout << __FUNCTION__ << " log " << node->getLogOdds() << "\n";
}

void PrintTreeInfo(OcTree &tree)
{
  std::cout << __FUNCTION__ << "------------------\n";
  std::cout << " getClampingThresMax " << tree.getClampingThresMax() << "\n";
  std::cout << " getClampingThresMaxLog " << tree.getClampingThresMaxLog() << "\n";
  std::cout << " getClampingThresMin " << tree.getClampingThresMin() << "\n";
  std::cout << " getClampingThresMinLog " << tree.getClampingThresMinLog() << "\n";
  std::cout << " getOccupancyThres " << tree.getOccupancyThres() << "\n";
  std::cout << " getOccupancyThresLog " << tree.getOccupancyThresLog() << "\n";
  std::cout << " getProbHit " << tree.getProbHit() << "\n";
  std::cout << " getProbHitLog " << tree.getProbHitLog() << "\n";
  std::cout << " getProbMiss " << tree.getProbMiss() << "\n";
  std::cout << " getProbMissLog " << tree.getProbMissLog() << "\n";
}

void TestNodeVal(OcTree &tree, point3d &pt) {
  PrintTreeInfo(tree);

  int count = 10;
  for (int i = 0; i < count; i++) {
    tree.updateNode(pt, 1.0f, false);
    PrintNodeVal(tree, pt);
  }
  PrintNodeVal(tree, pt);
}

void TestNodeOccupy()
{
  // set your param here:
  double target_occupy_thresh = 0.9;
  double num_hit = 100.0;

  OcTree my_tree (0.05);
  point3d point_update(2.01f, 0.01f, 0.01f);
  auto node = my_tree.updateNode(point_update, 0.0f, false);

  double target_occupy_thresh_log = octomap::logodds(target_occupy_thresh);
  double target_hit_log = target_occupy_thresh_log / num_hit;
  double raw_hit_log = my_tree.getProbHitLog();
  double scale_hit_log = target_hit_log / raw_hit_log;
  double target_mis_log = scale_hit_log * my_tree.getProbMissLog();

  my_tree.setOccupancyThres(target_occupy_thresh);
  my_tree.setProbHit(octomap::probability(target_hit_log));
  my_tree.setProbMiss(octomap::probability(target_mis_log));


  // test 99 次，应该是free的
  for (int i = 0; i < 99; i++) {
    my_tree.updateNode(point_update, my_tree.getProbHitLog());
  }
  std::cout << "occupy: " << my_tree.isNodeOccupied(node) << "\n";

  // test 100次，应该就是occupy了
  for (int i = 0; i < 1; i++) {
    my_tree.updateNode(point_update, my_tree.getProbHitLog());
  }
  std::cout << "occupy: " << my_tree.isNodeOccupied(node) << "\n";
}

int main(int /*argc*/, char** /*argv*/) {
  TestNodeOccupy();
  return 1;
  //##############################################################


  OcTree tree (0.05);
  tree.enableChangeDetection(true);

  point3d origin (0.01f, 0.01f, 0.02f);
  point3d point_on_surface (4.01f,0.01f,0.01f);
  point3d point_update(2.01f, 0.01f, 0.01f);
//  tree.insertRay(origin, point_on_surface);
//  printChanges(tree);
  tree.updateNode(point_update, 2.0f);
  printChanges(tree);
  PrintNodeVal(tree, point_update);
  tree.updateNode(point_update, -2.0f);
  printChanges(tree);
  PrintNodeVal(tree, point_update);

  TestNodeVal(tree, point_update);

  cout << "generating spherical scan at " << origin << " ..." << endl;

  for (int i=-100; i<101; i++) {
    Pointcloud cloud;
    for (int j=-100; j<101; j++) {
      point3d rotated = point_on_surface;
      rotated.rotate_IP(0, DEG2RAD(i*0.5), DEG2RAD(j*0.5));
      cloud.push_back(rotated);
    }

    // insert in global coordinates:
    tree.insertPointCloud(cloud, origin, -1);
  }

  printChanges(tree);


  cout << "done." << endl;

  return 0;
}

