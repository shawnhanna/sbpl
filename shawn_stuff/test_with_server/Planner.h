#include <sbpl/headers.h>
#include <boost/shared_ptr.hpp>

#include <GeographicLib/UTMUPS.hpp>
#include <iomanip>
#include <limits>


#include <iostream>
#include <string>

class Planner
{
public:
    Planner();
    ~Planner();

    void planxythetalat(char* envCfgFilename, char* motPrimFilename,
        std::pair<double, double> start, std::pair<double,double> end);

    void writeSolution(std::string filename);

    int runPlanner(int allocated_time_secs);

    std::vector< std::pair< double, double > > getpath();
    std::vector< std::pair<double, double> > convertToGPS(std::vector< std::pair<double, double> >& utmCoords);
    std::vector< std::pair< double, double > > getgps();

    void initializePlanner(int start_id, int goal_id,
                       double initialEpsilon,
                       bool bsearchuntilfirstsolution);

    void setEnvStartGoal(double start_x, double start_y, double start_theta,
                     double goal_x, double goal_y, double goal_theta,
                     int& start_id, int& goal_id);
    bool updateStartEnd(double sx, double sy, double ex, double ey);

    void initializeEnv(std::vector<sbpl_2Dpt_t>& perimeter,
                   char* envCfgFilename, char* motPrimFilename);
    void createFootprint(std::vector<sbpl_2Dpt_t>& perimeter);


private:
    boost::shared_ptr<SBPLPlanner> planner_;
    EnvironmentNAVXYTHETALAT env;
    std::vector<int> solution_stateIDs;

    static const double startX = 589122.7;
    static const double startY = 4476725.8;
};
