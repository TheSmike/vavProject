
class BundleAdjustmentModel { 
    KeyPointMatch kpm;
    
public:
  
  BundleAdjustmentModel(KeyPointMatch & kpm) : kpm(kpm)
  {
  }
  
  
  template <typename T>
  static bool predict(const T*pose, const T*pt3d_glob, const KeyPointMatch & kpm, T *pred) {
    //convertiamo da vettore di eigen in oggetto T (perchè l'oggetto T a volte è un double e a volte un Jet)
//     T xp[3];
//     xp[0] = T(prevPt.pt3d.x());
//     xp[1] = T(prevPt.pt3d.y());
//     xp[2] = T(prevPt.pt3d.z());

    T xc[3];
    //ruoto il punto previous in punto current
    ceres::AngleAxisRotatePoint(pose, pt3d_glob, xc);
    //aggiungo la parte traslativa
    xc[0] += pose[3]; 
    xc[1] += pose[4];
    xc[2] += pose[5];
            
    //Trasformo in 2d
    pred[0] = T(f)*(xc[0]/xc[2]) + T(u0);
    pred[1] = T(f)*(xc[1]/xc[2]) + T(v0);
    pred[2] = T(f)*((xc[0]-b)/xc[2]) + T(u0);
    pred[3] = T(f)*(xc[1]/xc[2]) + T(v0);
    
    return true;
  } 
  
  template <typename T>
  bool operator()(const T* const *parameters, T* residuals) const {
      //param è un vettore che ha tre componenti di tipo asse-angolo
      const T*pose = parameters[0];
      const T*pt3d_glob = parameters[1];
      
      T pred[4];
      
      predict(pose, pt3d_glob, kpm, pred);
      
      //Predict(param, prevList[i], pred);

      residuals[0] = ( T(kpm.left.x()) - pred[0]) ;
      residuals[1] = ( T(kpm.left.y()) - pred[1]) ;
      residuals[2] = ( T(kpm.right.x()) - pred[2]) ;
      residuals[3] = ( T(kpm.right.y()) - pred[3]) ;
      
      return true;
    
  }
  
  

};