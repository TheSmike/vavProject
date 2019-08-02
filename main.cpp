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
#include <vector>

#include <Eigen/SVD>
#include <Eigen/LU>

#include <map>


using namespace std;
using namespace Eigen;

static const int DISPARITY_THRESHOLD = 1;


class KeyPointMatch{
  
public: 
  Eigen::Vector2d left;
  Eigen::Vector2d right;
  
  KeyPointMatch(){
  }
  
  KeyPointMatch(float xl, float yl, float xr, float yr){
    left = Eigen::Vector2d(xl,yl);
    right = Eigen::Vector2d(xr, yr);
  }
  
};
  
std::ostream& operator<<(std::ostream &strm, const KeyPointMatch &a) {
  return strm << "left(" << a.left.x() << ", " << a.left.y() <<  "); " << "right(" << a.right.x() << ", " << a.right.y() <<  ").";
}

//functions declariation
std::map<int, Eigen::Vector3d> convertTo3DCameraPoints(std::map<int, KeyPointMatch> frame);


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
       
    
    cout << "elementiFile = " << elementiFile << "\n" << "numeroFotogrammi = " << numeroFotogrammi << "\n" << "puntiUnici = " << puntiUnici << endl;
    
    std::map<int, KeyPointMatch> frames[numeroFotogrammi];
    
    int idFrame, idPunto;
    float xl, yl, xr, yr;
    
    //leggo tutto il file e lo salvo in frames    
    cout << "Start reading file ...." << endl;
    for (int i =0; i< elementiFile; i++) {
      //esempio di riga:
      //0 0 268.093 31.8277 243.097 32.28
      //<num fotogramma> <id punto> <xleft> <yleft> <xright> <yright>
      inFile >> idFrame;
      inFile >> idPunto;
      inFile >> xl >> yl >> xr >> yr;
      frames[idFrame][idPunto] = KeyPointMatch(xl, yl, xr, yr);      
      //cout << idFrame << idPunto << xl << yl << xl << yl << endl;
    }
    inFile.close();
    cout << "end reading file!" << endl;
        
    
    
    std::map< int, Eigen::Vector3d > cur3d, prev3d;
    
    //elaborazione per ogni fotogramma
    for(int f = 0; f < numeroFotogrammi; f++){
      
      //converto i punti attuali in coordiante camera 3d tramite la disparità
      cur3d = convertTo3DCameraPoints(frames[f]);
      
      //
      
      //mi salvo come prev3d l'attuale cur3d per la prossima iterazione
      prev3d = cur3d;
      //tmp break anticipato per i test TODO eliminare alla fine degli sviluppi
      if (f == 1)
	break;
    }
    
    return 0;
    
}

std::map<int, Eigen::Vector3d> convertTo3DCameraPoints(std::map<int, KeyPointMatch> frame) {

  float u,v,d;
  int key;
  KeyPointMatch kpm; 
  Eigen::Vector3d nullv3d;
  std::map< int,  Eigen::Vector3d > m_cur;
    
  //m_cur.resize(frame.size(), nullv3d);
  //cout << "m_cur size before populate " << m_cur.size() << endl;
  int i = 0;
  std::map<int, KeyPointMatch>::iterator it;
  for (it = frame.begin(); it != frame.end(); ++it) {
    //cout << it->first << ": " << it->second << '\n';
    key = it->first;
    kpm = it->second;
    u = kpm.left.x();
    v = ( kpm.right.y() + kpm.left.y() ) / 2.f;
    d = kpm.left.x() - kpm.right.x();
    

    if (d > DISPARITY_THRESHOLD) {
	m_cur[key][0] = (u - u0) * b / d;
	m_cur[key][1] = (v - v0) * b / d; 
	m_cur[key][2] = f * b / d;
	//cout << m_cur[i].x() << ", " << m_cur[i].y() << ", " << m_cur[i].z() << endl;
	i++;
    }
  }
  //m_cur.resize(i, nullv3d);
  //cout << "m_cur size after populate " << m_cur.size() << endl;
  
  return m_cur;
}


//permette di calcolare la funzione costo di punti2d current avendo i punti3d previous
class Prev3DToCur2D { 
    std::vector<double> m_w;
    std::vector <Eigen::Vector3d> m_prev3D;
    std::vector <Eigen::Vector2d> m_cur_l;
    std::vector <Eigen::Vector2d> m_cur_r;
    

    double m_f,m_u0,m_v0,m_b;

public:
    
    Prev3DToCur2D(const std::vector<double> & w,
                  std::vector <Eigen::Vector3d> & prev3D, 
                  std::vector <Eigen::Vector2d> & cur_l,
                  std::vector <Eigen::Vector2d> & cur_r, 
                  double f,double u0,double v0,double b) 
    : m_w(w), m_prev3D(prev3D), m_cur_l(cur_l), m_cur_r(cur_r), m_f(f), m_u0(u0), m_v0(v0), m_b(b)
    {
    }
    
template <typename T>

bool operator()(const T* const *parameters, T* residuals) const {
    //param è un vettore che ha tre componenti di tipo asse-angolo
    const T*param = parameters[0];

    for(int i=0;i<m_prev3D.size();++i) {
        
        T pred[4];
        
        Predict(param, m_prev3D[i], pred, m_f, m_u0, m_v0, m_b);

        residuals[4*i+0] = T(m_w[i])*( T(m_cur_l[i].x()) - pred[0]) ;
        residuals[4*i+1] = T(m_w[i])*( T(m_cur_l[i].y()) - pred[1]) ;
        residuals[4*i+2] = T(m_w[i])*( T(m_cur_r[i].x()) - pred[2]) ;
        residuals[4*i+3] = T(m_w[i])*( T(m_cur_r[i].y()) - pred[3]) ;
        }
    return true;
}

};