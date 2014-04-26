#include <sbpl/headers.h>
#include <boost/shared_ptr.hpp>

#include <string>

class Planner
{
public:
    Planner();
    ~Planner();

    void planxythetalat(char* envCfgFilename, char* motPrimFilename);

    void writeSolution(EnvironmentNAVXYTHETALAT& env, std::vector<int> solution_stateIDs,
                   std::string filename);

    int runPlanner(int allocated_time_secs,
               std::vector<int>&solution_stateIDs);

    void initializePlanner(EnvironmentNAVXYTHETALAT& env,
                       int start_id, int goal_id,
                       double initialEpsilon,
                       bool bsearchuntilfirstsolution);

    void setEnvStartGoal(EnvironmentNAVXYTHETALAT& env,
                     double start_x, double start_y, double start_theta,
                     double goal_x, double goal_y, double goal_theta,
                     int& start_id, int& goal_id);

    void initializeEnv(EnvironmentNAVXYTHETALAT& env,
                   std::vector<sbpl_2Dpt_t>& perimeter,
                   char* envCfgFilename, char* motPrimFilename);
    void createFootprint(std::vector<sbpl_2Dpt_t>& perimeter);

private:
    boost::shared_ptr<SBPLPlanner> planner_;
};
