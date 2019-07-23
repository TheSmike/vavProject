#include "main.h"

#include <iostream>
#include <fstream>
#include <stdlib.h>

#include <algorithm>
#include <functional>
#include <iterator>
#include <sstream>
#include <string>
#include <utility>

#include <Eigen/SVD>
#include <Eigen/LU>

using namespace std;
using namespace Eigen;


int main(int argc, char **argv) {
    cout << "Hello, world!" << std::endl;
    
    cout << "Leggo il file ... \n";
    ifstream inFile;
    inFile.open(DS_FILE);
    
    if (!inFile) {
      cerr << "Unable to open file "  << DS_FILE << endl;
      exit(1);   // call system to stop
    }

    char *x;
    
    int elementiFile;
    int numeroFotogrammi;
    int puntiUnici;
    
    inFile >> elementiFile;
    inFile >> numeroFotogrammi;
    inFile >> puntiUnici;
    
    /*while (inFile >> x) {
      stdout << x << endl;
    }*/
    
    inFile.close();
    
    cout << elementiFile << " " << numeroFotogrammi << " " << puntiUnici << endl;
    
    std::vector <Eigen::Vector3d> prev3D;
    std::vector <Eigen::Vector2d> cur_l;
    
    //<NUMERO FOTOGRAMMA> <ID PUNTO> <X LEFT> <Y LEFT>  <X RIGHT> <Y RIGHT>
    for (int i = 0; i < elementiFile){
      
    }
    
    return 0;
    
}
