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

#include <time.h>

#include <ceres/ceres.h>
#include <ceres/rotation.h>

#include "BundleAdjustmentModel.cpp"
#include "Array3dContainer.cpp"


using namespace std;
using namespace Eigen;

static const int DISPARITY_THRESHOLD = 1;
int BREAK_PT = 25;

//functions declariation
void compute3DCameraPoints(std::map<int, KeyPointMatch> & frame);
void computeRotationAndTranslation(std::map<int, KeyPointMatch> & cur, std::map<int, KeyPointMatch> & prev, Eigen::Matrix3d & R, Eigen::Vector3d & t); 
void computeBundleAdjustment(double pose_list[][6], std::vector<Array3dContainer> & pt3d_glob_list, std::map<int, KeyPointMatch> frames[], int numeroFotogrammi, int puntiUnici);
int compute3DGlobal(std::vector <Matrix3d> & Ri_list, std::vector <Vector3d> & ti_list, std::map<int, KeyPointMatch> frames[], int puntiUnici, int numeroFotogrammi, std::vector<Array3dContainer> & pt3d_glob_list);
void write3dPoint(std::vector<Array3dContainer> & pt3d_glob_list, int puntiUnici);  

template <typename T>
bool Predict(const T*param, const KeyPointMatch & prevPt, T *pred) {
    //convertiamo da vettore di eigen in oggetto T (perchè l'oggetto T a volte è un double e a volte un Jet)
    T xp[3];
    xp[0] = T(prevPt.pt3d.x());
    xp[1] = T(prevPt.pt3d.y());
    xp[2] = T(prevPt.pt3d.z());

    T xc[3];
    //ruoto il punto previous in punto current
    ceres::AngleAxisRotatePoint(param, xp, xc);
    //aggiungo la parte traslativa
    xc[0] += param[3]; 
    xc[1] += param[4];
    xc[2] += param[5];
            
    //Trasformo in 2d
    pred[0] = T(f)*(xc[0]/xc[2]) + T(u0);
    pred[1] = T(f)*(xc[1]/xc[2]) + T(v0);
    pred[2] = T(f)*((xc[0]-b)/xc[2]) + T(u0);
    pred[3] = T(f)*(xc[1]/xc[2]) + T(v0);
    
    return true;
}

//permette di calcolare la funzione costo di punti2d current avendo i punti3d previous
class Prev3DToCur2DModel { 
    std::vector <KeyPointMatch> prevList;
    std::vector <KeyPointMatch> curList;
    
public:
  
    Prev3DToCur2DModel(std::vector <KeyPointMatch> & prevList, 
                  std::vector <KeyPointMatch> & curList) 
    : prevList(prevList), curList(curList)
    {
    }
    
  template <typename T>

  bool operator()(const T* const *parameters, T* residuals) const {
      //param è un vettore che ha tre componenti di tipo asse-angolo
      const T*param = parameters[0];

      for(int i=0; i < prevList.size(); ++i) {
	  
	  T pred[4];
	  
	  Predict(param, prevList[i], pred);

	  residuals[4*i+0] = T(prevList[i].w) * ( T(curList[i].left.x()) - pred[0]) ;
	  residuals[4*i+1] = T(prevList[i].w) * ( T(curList[i].left.y()) - pred[1]) ;
	  residuals[4*i+2] = T(prevList[i].w) * ( T(curList[i].right.x()) - pred[2]) ;
	  residuals[4*i+3] = T(prevList[i].w) * ( T(curList[i].right.y()) - pred[3]) ;
	  }
      return true;
  }

};


//MAIN program

int main(int argc, char **argv) {
  cout << "------------ START ------------" << std::endl;

  std::istringstream ss(argv[1]);
  if(argc > 0){
    if (!(ss >> BREAK_PT)) {
      std::cerr << "Invalid number: " << argv[1] << '\n';
    } else if (!ss.eof()) {
      std::cerr << "Trailing characters after number: " << argv[1] << '\n';
    }
  } else {
    BREAK_PT = -1;
  }
  
  cout << "break point is " << BREAK_PT << endl;
  //

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
  int numeroFotogrammi_break;
  
  inFile >> elementiFile;
  inFile >> numeroFotogrammi;
  inFile >> puntiUnici;

  time_t t1;
  time_t t2;

  //init vo values
  vo[0] = 0.0f;
  vo[1] = 0.0f;
  vo[2] = 0.0f;
  vo[3] = 0.0f;
  vo[4] = 0.0f;   
  vo[5] = -1.0f;
          
    
    cout << "elementiFile = " << elementiFile << "\n" << "numeroFotogrammi = " << numeroFotogrammi << "\n" << "puntiUnici = " << puntiUnici << endl;
    if (BREAK_PT > -1){
     numeroFotogrammi = BREAK_PT;
     cout << "Numero di fotogrammi ridotto da parametro di input a " << numeroFotogrammi <<  " !\n";
    }
    
    std::map<int, KeyPointMatch> frames[numeroFotogrammi + 5];
    
    int idFrame, idPunto;
    float xl, yl, xr, yr;
    
    t1 = time(NULL);
    //leggo tutto il file e lo salvo in frames    
    cout << "Start reading file ...." << endl;
    
    for (int i =0; i< elementiFile; i++) {
      //esempio di riga:
      //0 0 268.093 31.8277 243.097 32.28
      //<num fotogramma> <id punto> <xleft> <yleft> <xright> <yright>
      inFile >> idFrame;
      inFile >> idPunto;
      inFile >> xl >> yl >> xr >> yr;
      frames[idFrame][idPunto] = KeyPointMatch(xl, yl, xr, yr, 1.0f);      
      //cout << idFrame << idPunto << xl << yl << xl << yl << endl;
      if (BREAK_PT != -1 && idFrame > BREAK_PT + 2)
	break;
    }
    inFile.close();
    cout << "end reading file!" << endl;
    t2 = time(NULL);
    cout << "execution time: " << (t2-t1) << " s" << endl;
    
    cout << frames[0][0] << endl;
    cout << frames[0][1] << endl;
    cout << frames[1][0] << endl;
    cout << frames[1][1] << endl;
    
    
//     std::map<int, KeyPointMatch> & cur = NULL;
//     std::map<int, KeyPointMatch> & prev = NULL;
    Eigen::Matrix3d R;
    Eigen::Vector3d t;
    Eigen::Matrix3d Ri;
    Eigen::Vector3d ti;
    /// file di serializzazione output
    std::ofstream m_out, m_out_camera;
    
    //std::vector <Matrix3d> Rc_list;
    //std::vector <Vector3d> tc_list;
    std::vector <Matrix3d> Ri_list;
    std::vector <Vector3d> ti_list;
    double pose_list[numeroFotogrammi][6];
    
    
    Ri.setIdentity();
    ti.setConstant(0);
    m_out.open("/tmp/ms.txt");
    m_out_camera.open("/tmp/ms2.txt");

    
    t1 = time(NULL);
    cout << "Start computation ..." << endl;
    //elaborazione per ogni fotogramma
    for(int f = 0; f < numeroFotogrammi; f++){
      
      if (f % 100 == 0)
	cout << "iterazione " << f << endl;
      
      std::map<int, KeyPointMatch> & cur = frames[f];
      //converto i punti attuali in coordinate camera 3d tramite la disparità
      compute3DCameraPoints(cur);
      if(f==0){
	cout << cur[0] << endl << cur[1] << endl;
	Ri_list.push_back(Ri);
	ti_list.push_back(ti);
	ceres::RotationMatrixToAngleAxis(Ri.data(), pose_list[f]);
	pose_list[f][3] = ti[0];
	pose_list[f][4] = ti[1];
	pose_list[f][5] = ti[2];
      }
      
      //l'elaborazione inizia dal secondo frame 
      if (f > 0){
	std::map<int, KeyPointMatch> & prev = frames[f-1];
	
	//quanti match tra prev e cur
	computeRotationAndTranslation(cur, prev, R, t);
	
	ti = Ri * t + ti;
	Ri = Ri * R;
	
	//Rc_list.push_back(R);
	//tc_list.push_back(t);
	Ri_list.push_back(Ri);
	ti_list.push_back(ti);
	
	
	ceres::RotationMatrixToAngleAxis(Ri.data(), pose_list[f]);
	pose_list[f][3] = ti[0];
	pose_list[f][4] = ti[1];
	pose_list[f][5] = ti[2];
	
	for(int i=0;i<3;++i) {
	  m_out << Ri(i,0) << ' ' << Ri(i,1) << ' ' << Ri(i,2) << ' ' << ti[i] << ' ';
	  m_out_camera << R(i,0) << ' ' << R(i,1) << ' ' << R(i,2) << ' ' << t[i] << ' ';
	}
	m_out << std::endl;
	m_out_camera << std::endl;
	
      }
           
      if (f == BREAK_PT)
	break;
    }
    
    t2 = time(NULL);
    cout << "end computation!" << endl;
    cout << "execution time: " << (t2-t1) << " s" << endl;
    
    m_out.close();
    m_out_camera.close();
    
    
    
    
    t1 = time(NULL);
    cout << "Start computation BA ..." << endl;
    
    //cout << "not in function " << frames << endl;
    cout << puntiUnici << endl;
    
    
    std::vector<Array3dContainer> pt3d_glob_list;
    pt3d_glob_list.resize(puntiUnici, Array3dContainer());
    
    //cout << "init " << endl;
//     pt3d_glob_list[0][0] = 1.0;
//     cout << pt3d_glob_list[0][0];

    //int compute3DGlobal(std::vector <Matrix3d> & Ri_list, std::vector <Vector3d> & ti_list, std::map<int, KeyPointMatch> frames[], int puntiUnici, int numeroFotogrammi, double output[][3]);
    int ptsNumber = compute3DGlobal(Ri_list, ti_list, frames, puntiUnici, numeroFotogrammi, pt3d_glob_list);
    write3dPoint(pt3d_glob_list, ptsNumber);
    //void computeBundleAdjustment(double pose_list[][6], double pt3d_glob_list[][3], std::map<int, KeyPointMatch> & frames[], int numeroFotogrammi, int puntiUnici);
    computeBundleAdjustment(pose_list, pt3d_glob_list, frames, numeroFotogrammi, ptsNumber); //TODO 
    
    
    
    
    t2 = time(NULL);
    cout << "end computation BA!" << endl;
    cout << "execution time: " << (t2-t1) << " s" << endl;
        
    
    cout << "------------  END  ------------" << endl;
    return 0;
    
}



void compute3DCameraPoints(std::map<int, KeyPointMatch> & frame) {

  float u,v,d;
  float x,y,z;
  int key;
  
  Eigen::Vector3d nullv3d;
  
  int i = 0;
  std::map<int, KeyPointMatch>::iterator it;
  for (it = frame.begin(); it != frame.end(); ++it) {
    //cout << it->first << ": " << it->second << '\n';
    key = it->first;
    KeyPointMatch & kpm = it->second;
    u = kpm.left.x();
    v = ( kpm.right.y() + kpm.left.y() ) / 2.f;
    d = kpm.left.x() - kpm.right.x();
    
    //cout << "#1 " << u << " " << v << " " << d << endl;
    

    if (d > DISPARITY_THRESHOLD) {
	x = (u - u0) * b / d;
	y = (v - v0) * b / d; 
	z = f * b / d;
	kpm.set3d(x,y,z);
	//cout << "3d " << kpm.pt3d.x() << ", " << kpm.pt3d.y() << ", " << kpm.pt3d.z() << endl;
	i++;
    }else {
      kpm.set3d(0, 0, -1.0);
    }
    
    //if (i > 1) break;
  }
  
  
}


void computeRotationAndTranslation(std::map<int, KeyPointMatch> & cur, std::map<int, KeyPointMatch> & prev, Eigen::Matrix3d & R, Eigen::Vector3d & t){
  
  ceres::Problem problem;
  ceres::Solver::Options options;
  ceres::Solver::Summary summary;
  //options.minimizer_progress_to_stdout=true;
  
   
  int key;
  std::vector <KeyPointMatch> prevList;
  std::vector <KeyPointMatch> curList;
  std::map<int, KeyPointMatch>::iterator it;
  int cttmp = 0;
  
  for (it = cur.begin(); it != cur.end(); ++it) {
    key = it->first;
    KeyPointMatch & kpm = it->second;
    
    if (prev.find(key) != prev.end()){
      if (kpm.pt3d.z() != -1.0 && prev[key].pt3d.z() != -1.0){
	curList.push_back(kpm);
	KeyPointMatch & kpmPrev = prev[key];
	prevList.push_back(kpmPrev);
	
// 	if (cttmp++ < 25)
// 	  cout << "key => " << key << "; cur => " << kpm << "; prev => " << kpmPrev << endl;
      }
    }
  }
  
//   for(int i = 0; i < 10; i++ ){
//     cout << " cur[" << i << "] = " << curList[i] << endl;
//     cout << "prev[" << i << "] = " << prevList[i] << endl;
//   }
  
  for(int k=0;k<2;++k) {
    
    double th = (k==0) ? 576. : 8.;
    int inliers = 0;

    for(int i = 0; i < prevList.size(); ++i) {
	KeyPointMatch & prevEl = prevList[i];
	double pred[4];
	Predict(vo, prevEl, pred);
 
	double res2 = std::pow(curList[i].left.x() - pred[0], 2) + 
		      std::pow(curList[i].left.y() - pred[1], 2) +
		      std::pow(curList[i].right.x() - pred[2], 2) +
		      std::pow(curList[i].right.y() - pred[3], 2);
	//cout << " - " <<  res2 << endl;
	prevEl.w = (res2<th) ? 1.0 : (th/res2);
	if(res2<th)
	    inliers++;
    }

    std::cout << "inliers:" << inliers << " outliers:" << prevList.size() - inliers << std::endl;
    
    Prev3DToCur2DModel *model = new Prev3DToCur2DModel(prevList, curList);
    ceres::DynamicAutoDiffCostFunction < Prev3DToCur2DModel > *cost = new ceres::DynamicAutoDiffCostFunction< Prev3DToCur2DModel >( model );

    cost->AddParameterBlock( 6 ); 
    cost->SetNumResiduals( curList.size() * 4  ); //4 osservazioni, 2 x e 2 y

    problem.AddResidualBlock(cost, 0, vo);

    ceres::Solve(options, &problem, &summary);
    
//     cout << " v0 at " << k << " iter is --> "  
//     << vo[0] << " " 
//     << vo[1] << " "
//     << vo[2] << " "
//     << vo[3] << " "
//     << vo[4] << " "
//     << vo[5] << " " 
//     << endl << endl;
  }
  
//   

  ceres::AngleAxisToRotationMatrix(vo, ceres::ColumnMajorAdapter3x3((double *)R.data())); //ColumnMajorAdapter3x3 è un adapter da Eigen a Ceres per le matrici
      
  
  t.x() = vo[3];
  t.y() = vo[4];
  t.z() = vo[5];

  
  R = R.transpose().eval();
  
  t = -R*t;  
  
  
}

int compute3DGlobal(std::vector <Matrix3d> & Ri_list, std::vector <Vector3d> & ti_list, std::map<int, KeyPointMatch> frames[], int puntiUnici, int numeroFotogrammi, std::vector<Array3dContainer> & pt3d_glob_list){
  
  int counter = -1;
  
  std::map<int, KeyPointMatch>::iterator it;
      
  //if (prev.find(key) != prev.end()){
  
  int compute = 0;
  int notCompute = 0;
  int counttmp = 0;

  for(int f = 0; f < numeroFotogrammi; f++){
    map<int, KeyPointMatch> frame = frames[f];
    std::map<int, KeyPointMatch>::iterator it;
    for (it = frame.begin(); it != frame.end(); ++it) {
      int key = it->first;
      KeyPointMatch & kpm = it->second;
      
//       if (key > counter+1){
// 	cerr << "sequenza non congruente, frame = " << f << ", counter = " << counter << ", key = " << key << ", verificare il codice.\n";
// 	exit(1);
//       }
      
      if ( pt3d_glob_list[key].isNull 
	   && !(kpm.pt3d.x() == 0.0 && kpm.pt3d.y() == 0.0 && kpm.pt3d.z() == -1.0)
	   && kpm.w < 8 ){
	
	counter++;

	Matrix3d Rt = Ri_list[f].transpose().eval();
	Vector3d t_inv = -Rt * ti_list[f];
	double angleAxis[3];
	double pt3d_glob[3];
	
	//ceres::ColumnMajorAdapter3x3((double *)R.data());
	ceres::RotationMatrixToAngleAxis(Rt.data(), angleAxis);
	
	ceres::AngleAxisRotatePoint(angleAxis, kpm.pt3d.data(), pt3d_glob);
	pt3d_glob[0] += t_inv.x();
	pt3d_glob[1] += t_inv.y();
	pt3d_glob[2] += t_inv.z();
      
	pt3d_glob_list[key] = Array3dContainer(pt3d_glob[0], pt3d_glob[1], pt3d_glob[2]);
    
	if (counttmp < 1){
	  cout << "TMP 3D GLOBAL log:\n" 
	      << "3DLoc = " << kpm.pt3d << endl
	      << "Ri    = " << Ri_list[f] << endl
	      << "Riinv = " << Rt << endl
	      << "ti    = " << ti_list[f] << endl
	      << "tiinv = " << t_inv << endl
	      << "3Dglob= " << pt3d_glob_list[key].values[0] << " " << pt3d_glob_list[key].values[1] << " " << pt3d_glob_list[key].values[2] << " " << endl;
	      counttmp++;
	}
	
	compute++;
      }
      else{
	notCompute++;
      }
    }
    if (f == BREAK_PT)
      break;
  }
  cout << "computed " << compute << " 3D global points\n" 
       << "skipped  " << notCompute << " 3D global points\n";
  
  return counter;
  
}

void computeBundleAdjustment(double pose_list[][6], std::vector<Array3dContainer> & pt3d_glob_list, std::map<int, KeyPointMatch> frames[], int numeroFotogrammi, int puntiUnici){
  //cout << "    in function " << frames << endl;
  
  ceres::Problem problem;
  ceres::Solver::Options options;
  ceres::Solver::Summary summary;
  options.minimizer_progress_to_stdout=true;
  
  
  //per ogni frame
  for(int f = 0; f < numeroFotogrammi; f++){
    map<int, KeyPointMatch> frame = frames[f];
    std::map<int, KeyPointMatch>::iterator it;
    for (it = frame.begin(); it != frame.end(); ++it) {
      int key = it->first;
      KeyPointMatch & kpm = it->second;
	
      if(pt3d_glob_list[key].isNull == false){
	
	//problem.AddResidualBlock( new AutoDiffCostFunction<F1, 1, 1, 1>(new F1), NULL, &x1, &x2);
	
	BundleAdjustmentModel *model = new BundleAdjustmentModel(kpm);
	ceres::DynamicAutoDiffCostFunction < BundleAdjustmentModel > *cost = new ceres::DynamicAutoDiffCostFunction< BundleAdjustmentModel >( model );

	cost->AddParameterBlock( 6 ); 
	cost->AddParameterBlock( 3 );
	cost->SetNumResiduals( 4 );
	problem.AddResidualBlock(cost, 0, pose_list[f], pt3d_glob_list[key].values);
      }
      
    }
  }
    
  ceres::Solve(options, &problem, &summary);

}

void write3dPoint(std::vector<Array3dContainer> & pt3d_glob_list, int puntiUnici){
  std::ofstream m_out3d;
  m_out3d.open("/tmp/output3dGlobal.txt");
  
  for (int i = 0; i<puntiUnici; i++){
    
    m_out3d << pt3d_glob_list[i].values[0] << " " << pt3d_glob_list[i].values[1] << " " << pt3d_glob_list[i].values[2] << endl;  
  }
  
  m_out3d.close();
}



