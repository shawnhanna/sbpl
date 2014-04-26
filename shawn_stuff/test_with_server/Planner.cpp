#include "Planner.h"


Planner::Planner()
{
    planner_.reset();
}

Planner::~Planner()
{

}

// creating the footprint
void Planner::createFootprint(std::vector<sbpl_2Dpt_t>& perimeter){
    sbpl_2Dpt_t pt_m;
    double halfwidth = 0;
    double halflength = 0;
    pt_m.x = -halflength;
    pt_m.y = -halfwidth;
    perimeter.push_back(pt_m);
    pt_m.x = halflength;
    pt_m.y = -halfwidth;
    perimeter.push_back(pt_m);
    pt_m.x = halflength;
    pt_m.y = halfwidth;
    perimeter.push_back(pt_m);
    pt_m.x = -halflength;
    pt_m.y = halfwidth;
    perimeter.push_back(pt_m);
}

void Planner::initializeEnv(std::vector<sbpl_2Dpt_t>& perimeter,
                   char* envCfgFilename, char* motPrimFilename){
    if (!env.InitializeEnv(envCfgFilename, perimeter, motPrimFilename)) {
        printf("ERROR: InitializeEnv failed\n");
        throw SBPL_Exception();
    }
}

void Planner::setEnvStartGoal(double start_x, double start_y, double start_theta,
                     double goal_x, double goal_y, double goal_theta,
                     int& start_id, int& goal_id){

    start_id = env.SetStart(start_x, start_y, start_theta);
    goal_id = env.SetGoal(goal_x, goal_y, goal_theta);
}

void Planner::initializePlanner(int start_id, int goal_id,
                       double initialEpsilon,
                       bool bsearchuntilfirstsolution){
    // work this out later, what is bforwardsearch?
    bool bsearch = false;
    planner_.reset(new ARAPlanner(&env, bsearch));

    // set planner_ properties
    if (planner_->set_start(start_id) == 0) {
        printf("ERROR: failed to set start state\n");
        throw new SBPL_Exception();
    }
    if (planner_->set_goal(goal_id) == 0) {
        printf("ERROR: failed to set goal state\n");
        throw new SBPL_Exception();
    }
    planner_->set_initialsolution_eps(initialEpsilon);
    planner_->set_search_mode(bsearchuntilfirstsolution);
}

int Planner::runPlanner(int allocated_time_secs){
    int bRet = planner_->replan(allocated_time_secs, &solution_stateIDs);

    if (bRet)
        printf("Solution is found\n");
    else
        printf("Solution does not exist\n");
    return bRet;
}

void Planner::writeSolution(std::string filename)
{
    std::string discrete_filename;
    discrete_filename = filename + std::string(".discrete");
    FILE* fSol_discrete = fopen(discrete_filename.c_str(), "w");
    FILE* fSol = fopen(filename.c_str(), "w");
    if (fSol == NULL) {
        printf("ERROR: could not open solution file\n");
        throw SBPL_Exception();
    }

    // write the discrete solution to file
    for (size_t i = 0; i < solution_stateIDs.size(); i++) {
        int x, y, theta;
        env.GetCoordFromState(solution_stateIDs[i], x, y, theta);
        double cont_x, cont_y, cont_theta;
        cont_x = DISCXY2CONT(x, 0.1);
        cont_y = DISCXY2CONT(y, 0.1);
        cont_theta = DiscTheta2Cont(theta, 16);
        fprintf(fSol_discrete, "%d %d %d\n", x, y, theta);
    }
    fclose(fSol_discrete);

    // write the continuous solution to file
    std::vector<sbpl_xy_theta_pt_t> xythetaPath;
    env.ConvertStateIDPathintoXYThetaPath(&solution_stateIDs, &xythetaPath);
    for (unsigned int i = 0; i < xythetaPath.size(); i++) {
        fprintf(fSol, "%.3f %.3f %.3f\n", xythetaPath.at(i).x,
                                          xythetaPath.at(i).y,
                                          xythetaPath.at(i).theta);
    }
    fclose(fSol);
}

std::vector< std::pair< double, double > > Planner::getpath()
{
    std::vector< std::pair<double, double> > retVec;
    // write the continuous solution to file
    std::vector<sbpl_xy_theta_pt_t> xythetaPath;
    env.ConvertStateIDPathintoXYThetaPath(&solution_stateIDs, &xythetaPath);
    for (unsigned int i = 0; i < xythetaPath.size(); i++) {
        retVec.push_back(std::pair<double, double>(xythetaPath.at(i).x,
                                          xythetaPath.at(i).y));
    }
    return retVec;
}

void Planner::planxythetalat(char* envCfgFilename, char* motPrimFilename, std::pair<double, double> start, std::pair<double,double> end){
    // set the perimeter of the robot
    std::vector<sbpl_2Dpt_t> perimeter;
    createFootprint(perimeter);

    // initialize an environment
    initializeEnv(perimeter, envCfgFilename, motPrimFilename);

    // specify a start and goal state
    int start_id, goal_id;
    double startX = start.first, startY = start.second, endX = end.first, endY = end.second;
    setEnvStartGoal(startX, startY, 0, endX, endY, 0, start_id, goal_id);

    // initialize a planner with start and goal state
    double initialEpsilon = 2.0;
    bool bsearchuntilfirstsolution = true;
    initializePlanner(start_id, goal_id, initialEpsilon,
                      bsearchuntilfirstsolution);

    // plan
    double allocated_time_secs = 60.0; // in seconds
    runPlanner(allocated_time_secs);

    // print stats
    env.PrintTimeStat(stdout);

    // write out solutions
    std::string filename("sol.txt");
    writeSolution(filename);
}


std::vector< std::pair<double, double> > Planner::convertToGPS(std::vector< std::pair<double, double> >& utmCoords)
{
    std::cout << "Getting coordinates in GPS coords\n";
    using namespace GeographicLib;

    std::vector< std::pair<double, double> > latlonCoords;

    for (int i = 0; i < utmCoords.size(); i++)
    {
        // std::cout << "oX = "<<utmCoords[i].first<<"  oy = "<<utmCoords[i].second<<std::endl;
        double x = utmCoords[i].first+startX;
        double y = utmCoords[i].second+startY;
        // std::cout << "x = "<<x<<" y = "<<y<<"\n";

        double lat = -1, lon = -1;

        //Convert to lat, long
        int zone;
        bool northp;
        std::string zonestr = "17N";
        UTMUPS::DecodeZone(zonestr, zone, northp);
        UTMUPS::Reverse(zone, northp, x, y, lat, lon);
        // std::cout << "lat = "<<lat<<" lon = "<<lon<<"\n";

        std::pair<double, double> latlong(lat,lon);
        latlonCoords.push_back(latlong);
    }
    return latlonCoords;
}


std::vector< std::pair<double, double> > Planner::getgps()
{
    std::vector< std::pair<double, double> > path = getpath();
    // trim path
    std::vector< std::pair<double, double> > newPath;
    double lastX = -100000000;
    double lastY = -100000000;

    for (std::vector<std::pair<double, double> >::iterator i = path.begin(); i != path.end(); ++i)
    {
        double x = (*i).first;
        double y = (*i).second;

        if (fabs(lastX - x) > 2 || fabs(y - lastY) > 2)
        {
            newPath.push_back(std::pair<double, double>(x,y));
            lastX = x;
            lastY = y;
        }
    }
    return convertToGPS(newPath);
}

// Update comes in as a lat/long pair
bool Planner::updateStartEnd(double sx, double sy, double ex, double ey)
{
    using namespace GeographicLib;

    //convert to UTM

    int zone;
    bool northp;
    double newStartX, newStartY, newEndX, newEndY;
//    UTMUPS::Forward(sx, sy, zone, northp, newStartX, newStartY);
//    UTMUPS::Forward(ex, ey, zone, northp, newEndX, newEndY);

    newStartX = sx - startX;
    newStartY = sy - startY;
    newEndX = ex - startX;
    newEndY = ey - startY;

    std::cout << "starting location: "<< std::fixed << std::setprecision(std::numeric_limits< double >::digits10)
          << " " << newStartX << " " << newStartY << "\n";

    std::cout << "ending location: "<< std::fixed << std::setprecision(std::numeric_limits< double >::digits10)
          << " " << newEndX << " " << newEndY << "\n";

    int start_id, goal_id;
    setEnvStartGoal(newStartX, newStartY, 0, newEndX, newEndY, 0, start_id, goal_id);

    // set planner_ properties
    if (planner_->set_start(start_id) == 0) {
        printf("ERROR: failed to set start state\n");
        throw new SBPL_Exception();
    }
    if (planner_->set_goal(goal_id) == 0) {
        printf("ERROR: failed to set goal state\n");
        throw new SBPL_Exception();
    }

    return true;
}
