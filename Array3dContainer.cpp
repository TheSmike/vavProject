
class Array3dContainer{
  
public: 
  double values[3]; 
  bool isNull;
 
  Array3dContainer(double a, double b, double c){
    values[0] = a;
    values[1] = b;
    values[2] = c;
    isNull = false;
  }
  Array3dContainer(){
    isNull = true;
  }

  double * getArray(){
    return values;
  }
  
};