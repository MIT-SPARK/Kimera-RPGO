// Authors: Yun Chang

#ifndef GRAPH_UTILS_TYPES_H
#define GRAPH_UTILS_TYPES_H

#include "RobustPGO/max_clique_finder/findClique.h"
#include "RobustPGO/logger.h"

#include <map>
#include <vector>

namespace RobustPGO {
	
int findMaxClique(const Eigen::MatrixXd adjMatrix, std::vector<int>& max_clique);

int findMaxCliqueHeu(const Eigen::MatrixXd adjMatrix, std::vector<int>& max_clique);

}

#endif