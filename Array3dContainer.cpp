
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

std::ostream& operator<<(std::ostream &strm, const Array3dContainer &a) {
  if (a.isNull)
    return strm << "NULL";
  else
    return strm << "values = " << a.values[0] << " " << a.values[1] << " " << a.values[2] << " ";
  
}