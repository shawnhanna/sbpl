// reading a text file
#include <iostream>
#include <fstream>
#include <string>
#include <stdio.h>
#include <string>
#include <iomanip>
#include <GeographicLib/UTMUPS.hpp>
#include <limits>
#include <cmath>

using namespace std;
using namespace GeographicLib;

typedef std::numeric_limits< double > dbl;

int main () {
  string line;
  ifstream infile ("../../sol.txt");
  ofstream outfile ("lat_long_output.txt");

  double startX = 591775.3;
  double startY = 4476386.2;

  double lastX = 0;
  double lastY = 0;
  /*
  {
    double lat = 40.443505, lon = -79.945562; // Starting location
    int zone;
    bool northp;
    UTMUPS::Forward(lat, lon, zone, northp, startX, startY);
    string zonestr = UTMUPS::EncodeZone(zone, northp);
    cout << "starting location: "<< fixed << setprecision(dbl::digits10)
          << " " << startX << " " << startY << "\n";
  }*/

  int count = 0;
  if (infile.is_open() && infile.is_open())
  {

    while ( getline (infile,line) )
    {
      count++;
      // cout << line << '\n';

      try {
        // Read from file
        const char *str = line.c_str();
        double x, y, theta;
        sscanf(str, "%lf %lf %lf", &x, &y, &theta);
        x = x+startX;
        y = y+startY;

        if (fabs(lastX - x) > 2 || fabs(y - lastY) > 2)
        {

          printf("%lf %lf\n",x, y);
          double lat = -1, lon = -1; // Baghdad

          //Convert to lat, long
          int zone;
          bool northp;
          string zonestr = "17N";
          UTMUPS::DecodeZone(zonestr, zone, northp);
          UTMUPS::Reverse(zone, northp, x, y, lat, lon);
          // cout << lat << " " << lon << "\n";
          outfile.precision(dbl::digits10);
          outfile << fixed << lat;
          outfile << " ";
          outfile << fixed << lon << endl;

          lastX = x;
          lastY = y;
        }
        else
        {
          // cout<< "not keeping value. too close: "<<count<<endl;
        }
      }
      catch (const exception& e) {
        outfile.close();
        infile.close();
        cerr << "Caught exception: " << e.what() << "\n";
        return 1;
      }
    }
    outfile.close();
    infile.close();
  }
  else
  {
    cout << "Unable to open file";
  }
  cout << "Count = "<<count<<endl;
  return 0;
}
