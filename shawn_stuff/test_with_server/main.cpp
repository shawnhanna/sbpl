#include <cstring>
#include <iostream>
#include <string>

#include "Planner.h"

int main(int argc, char *argv[])
{
    if (argc != 3)
    {
        std::cout << "You need at least 2 arguments: usage =\n"<<argv[0] <<" "<<"<.env config file> <motion primitives file>\n";
        exit(1);
    }
    Planner p;
    p.planxythetalat(argv[1], argv[2]);
    while (true)
    {
        //do server stuff
        sleep(1);
    }
}