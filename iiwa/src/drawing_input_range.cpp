#include "drawing_input_range.h"

#define PI 3.14159265

DrawingInput::DrawingInput(const std::string &path, const std::string &file_name,
                                  const char &color, const std::string &file_extension,
                                  const geometry_msgs::Pose &drawing_pose) {
  setFileName(path, file_name, color, file_extension);
  this->drawing_pose = drawing_pose;
  readDrawingFile();
  removeLongLine();
  if (this->size[0] > 0.55)
    splitByRange();
}

DrawingInput::DrawingInput(const std::string &wall_name_, const std_msgs::Float64MultiArray &ri_ranges, 
                                  const std::string &path, const std::string &file_name,
                                  const char &color, const std::string &file_extension,
                                  const geometry_msgs::Pose &drawing_pose) {
  setFileName(path, file_name, color, file_extension);
  this->wall_name = wall_name_;
  this->drawing_pose = drawing_pose;
  std::cout << "Drawing Input file setting Done\n";
  setKDTree();
  std::cout << "Set KDTree Done\n";
  readDrawingFileArb();
  std::cout << "Get Strokes Ready Done \n";
  removeLongLine();
  std::cout << "Cut long lines Done\n";
  if (this->size[0] > 0.55)
    splitByRangeArb(ri_ranges);
}


void DrawingInput::setFileName(const std::string path, const std::string file_name,
                                  const char color, const std::string file_extension) {
  this->path = path;
  this->file_name = file_name;
  this->color = color;
  this->file_extension = file_extension;
  this->file_name_full = path + file_name + color + file_extension;
}

void DrawingInput::setTargetSize (const double target_size) {
  this->target_size = target_size;
}

void DrawingInput::setDrawingSize (const double ratio) {
  this->ratio = ratio;
  double width = ratio * this->target_size;
  double height = this->target_size;
  this->size.push_back(width);
  this->size.push_back(height);
}

// read file line by line and save it in strokes
void DrawingInput::readDrawingFile() {
  std::string line;

  std::ifstream txt(ros::package::getPath("large_scale_drawing")+this->file_name_full);
  // check if text file is well opened
  if(!txt.is_open()){
    std::cout << "FILE NOT FOUND\n";
  }

  // first line indicates the size
  std::getline(txt, line); // drawing size
  std::vector<std::string> tempSplit = split(line, ' ');
  double width = stod(tempSplit[0]);
  double height = stod(tempSplit[1]);
  setDrawingSize(width/height);

  double y, z;
  Stroke stroke;
  geometry_msgs::Pose drawing_pose = this->drawing_pose;

  while(std::getline(txt, line)) {
    if(line == "End") { // stroke finished
      this->strokes.push_back(stroke);
      //stroke.clear();
      Stroke().swap(stroke);
    }
    else { // start reading strokes
      tempSplit = split(line, ' ');
      y = (-stod(tempSplit[0])+0.5) * this->ratio * this->target_size;
      z = (-stod(tempSplit[1])+0.5) * this->target_size + this->drawing_pose.position.z;
      drawing_pose.position.y = y;
      drawing_pose.position.z = z;
      stroke.push_back(drawing_pose);
    }
  }
}

// remove the long distanced lines
void DrawingInput::removeLongLine() {
  std::vector<Stroke> new_strokes;
  int num_strokes = this->strokes.size();
  for (int i = 0; i < this->strokes.size(); i++) {  // strokes
    Stroke stroke;
    geometry_msgs::Pose p1 = this->strokes[i][0];
    for (int j = 1; j < this->strokes[i].size(); j++) { // points
      geometry_msgs::Pose p2 = this->strokes[i][j];
      double dist = this->getDist(p1, p2);
      if (dist > 0.03) { // 3cm
        if (stroke.size() > 5)
          new_strokes.push_back(stroke);
        Stroke().swap(stroke);
        stroke.push_back(p2);
      }
      else {
        stroke.push_back(p2);
      }
      p1 = p2;
    }
    new_strokes.push_back(stroke);
  }
  this->strokes = new_strokes;
}

void DrawingInput::setCanvasRange () {
  double width = this->size[0];
  int num_range = int(width/this->max_range)+1;

  std::array<double, 2> range;
  range[0] = -(this->max_range*num_range/2);
  for (double r = -(this->max_range*num_range/2) + this->max_range; r <= this->max_range*num_range/2; r += this->max_range) {
    range[1] = r;
    this->ranges.push_back(range);
    range[0] = r;
  }
  range[1] = (this->max_range*num_range/2);
  if (range[0] != range[1])
    this->ranges.push_back(range);
}

void DrawingInput::splitByRange () {
  this->setCanvasRange();
  std::vector<std::vector<Stroke>> strokes_by_range (this->ranges.size());

  int range_index_prev, range_index;
  for (int i = 0; i < this->strokes.size(); i++) {
    Stroke stroke;
    range_index_prev = this->detectRange(this->strokes[i][0]);

    for (int j = 1; j < this->strokes[i].size(); j++) {
      range_index = this->detectRange(strokes[i][j]);

      if (range_index_prev != range_index) { // split stroke
        int index = std::min(range_index_prev, range_index);
        geometry_msgs::Pose contact = this->strokes[i][j];
        contact.position.y = this->ranges[index][1];
        contact.position.z = (strokes[i][j].position.z - strokes[i][j-1].position.z)*(this->ranges[index][1] - strokes[i][j-1].position.y)/(strokes[i][j].position.z - strokes[i][j-1].position.y) + strokes[i][j-1].position.z;
        stroke.push_back (contact);

        // only if stroke size is bigger than 5 points
        if (stroke.size() > 5) {
          strokes_by_range[range_index_prev].push_back(stroke);
        }
        Stroke().swap(stroke);
        stroke.push_back (contact);
        range_index_prev = range_index;
      }
      else {
        stroke.push_back(this->strokes[i][j]);
      }
    }
    strokes_by_range[range_index].push_back(stroke);
  }

  this->strokes_by_range = strokes_by_range;
  this->recenterDrawings();
}

void DrawingInput::recenterDrawings() {
  for (int i = 0; i < this->strokes_by_range.size(); i++){ //range
    double diff = (this->ranges[i][0] + this->ranges[i][1])/2;
    this->diffs.push_back(diff);
    for (int j = 0; j < this->strokes_by_range[i].size(); j++){ //strokes
       for (int k = 0; k < this->strokes_by_range[i][j].size(); k++) //pose
         this->strokes_by_range[i][j][k].position.y -= diff;
    }
  }
}

int DrawingInput::detectRange(geometry_msgs::Pose p) {
  double x = p.position.y;
  for (int i = 0; i < this->ranges.size(); i++) {
    if (ranges[i][0] < x && x <= ranges[i][1])
      return i;
  }
  return -1;
}

double DrawingInput::getDist(geometry_msgs::Pose p1, geometry_msgs::Pose p2) {
  return sqrt(pow(p1.position.x - p2.position.x, 2)+ pow(p1.position.y - p2.position.y, 2));
}

bool DrawingInput::contains(std::vector<int> vec, const int elem) {
    bool result = false;
    if( std::find(vec.begin(), vec.end(), elem) != vec.end() ){
        result = true;
    }
    return result;
}

std::vector<std::string> DrawingInput::split(const std::string input, const char delimiter){
  std::vector<std::string> dat;
  std::stringstream str(input);
  std::string temp;
  while(getline(str, temp, delimiter)){
      dat.push_back(temp);
  }
  return dat;
}

// wall related functions below
void DrawingInput::setKDTree(){
  pointVec points;
  pointVec normals;
  pointVec vns;

  point_t pt;
  point_t n;

  double min_y=1000, max_y=-1000;
  double min_z=1000, max_z=-1000;

  bool assigned = false;

  // read file
  std::ifstream infile(ros::package::getPath("large_scale_drawing")+this->wall_name);
  std::string line_;  // line read
  while (std::getline(infile, line_))
  {
    std::vector<std::string> line = split(line_, ' ');  // splitted line
    if (line[0] == "v") { // vertex
      pt = {std::stod(line[1]), std::stod(line[2]), std::stod(line[3])};
      points.push_back(pt);
      if(std::stod(line[2]) < min_y) min_y = std::stod(line[2]);
      else if (std::stod(line[2]) > max_y) max_y = std::stod(line[2]);
      if(std::stod(line[3]) < min_z) min_z = std::stod(line[3]);
      else if (std::stod(line[3]) > max_z) max_z = std::stod(line[3]);
    }
    else if (line[0] == "vn") { // vertex normal
      n = {std::stod(line[1]), std::stod(line[2]), std::stod(line[3])};
      vns.push_back(n);
    }
    if (line[0] == "f") { // face
      if (!assigned) {  // assign vector
        normals.assign(points.size(), point_t(3, 0.0));
        assigned = true;
      }
      line.erase(line.begin()); // erase f
      for (auto element: line) {
        std::vector<std::string> vvn = split(element, '/');
        int v_index = std::stoi(vvn[0])-1;
        int vn_index = std::stoi(vvn[2])-1;
        normals[v_index] = vns[vn_index];
      }
    }
  }

  // Make y = 0 as the center of the wall coordinate and z = 0 as the bottom of the wall
  double mid_y = (min_y+max_y)/2;
  for(int i = 0; i < points.size(); i++){
    points[i][1] += - mid_y;
    points[i][2] += - min_z;
  }

  this->kdtree = KDTree(points, normals);
}

void DrawingInput::readDrawingFileArb(){
  std::string line;

  std::ifstream txt(ros::package::getPath("large_scale_drawing")+this->file_name_full);
  // check if text file is well opened
  if(!txt.is_open()){
    ROS_INFO("FILE NOT FOUND");
    return;
  }

  std::cout << "File FOUND\n";

  // first line indicates the size
  std::getline(txt, line); // drawing size
  std::vector<std::string> tempSplit = split(line, ' ');
  double width = stod(tempSplit[0]);
  double height = stod(tempSplit[1]);
  setDrawingSize(width/height);

  double x, y, z;
  point_t pt, orientation;
  Stroke stroke;
  geometry_msgs::Pose drawing_pose;

  std::cout << "Start reading drawing file\n";

  while(std::getline(txt, line)) {
    if(line == "End") { // stroke finished
      this->strokes.push_back(stroke);
      //stroke.clear();
      Stroke().swap(stroke);
    }
    else { // start reading strokes
      point_t().swap(pt);

      tempSplit = split(line, ' ');
      y = (-stod(tempSplit[0])+0.5) * this->ratio * this->target_size;
      z = (-stod(tempSplit[1])+0.5) * this->target_size + this->drawing_pose.position.z;
      pt.push_back(y); pt.push_back(z);

      tie(x, orientation) = getXAndQuaternion(pt);

      drawing_pose.position.x = x;
      drawing_pose.position.y = y;
      drawing_pose.position.z = z;
      drawing_pose.orientation.x = orientation[0];
      drawing_pose.orientation.y = orientation[1];
      drawing_pose.orientation.z = orientation[2];
      drawing_pose.orientation.w = orientation[3];
      stroke.push_back(drawing_pose);
    }
  }
}

std::tuple<double, point_t> DrawingInput::getXAndQuaternion(point_t &pt){
  double x; point_t sn; 

  tie(x, sn) = getXAndSurfaceNormal(pt);

  // std::cout<< "5) Got Surface Normal\n";

  point_t zaxis{0,0,1};

  // calc cosine
  double vSize = getVectorSize(sn);
  if(vSize != 1){
    for(int i = 0; i < 3; i++){
      sn[i] = sn[i]/vSize;
    }
  }
  double cosPsi = -1*sn[2];

  // calc axis
  point_t vecA = getCrossProduct(sn, zaxis);
  vSize = getVectorSize(vecA);
  if(vSize != 1){
    for(int i = 0; i < vecA.size(); i++){
      vecA[i] = vecA[i]/vSize;
    }
  }

  // calc roation matrix
  tf::Matrix3x3 rotMat = getRotationMatrix(vecA, cosPsi);
  tf::Quaternion quat;
  rotMat.getRotation(quat);

  // std::cout<< "6) Got Rotation Matrix\n";


  point_t orientation;
  for(int i = 0; i < 4; i++){
    orientation.push_back(quat[i]);
  }

  // std::cout<< "7) Got Orientation\n";
  

  return {x, orientation};
}

std::tuple<double, point_t> DrawingInput::getXAndSurfaceNormal(point_t &pt){
  KDNodeArr quad = this->kdtree.search_quad(pt, 3);

  // std::cout<< "2) Searched Quad \n";

  pointVec coor = {quad[0]->xyz(), quad[1]->xyz(), quad[2]->xyz(), quad[3]->xyz()};
  pointVec nv = {quad[0]->n, quad[1]->n, quad[2]->n, quad[3]->n};

  // std::cout<< "3) Saved coordinate and normal vector \n";

  double dy = coor[1][1]-pt[0];
  double dz = coor[2][2]-pt[1];

  double A = (1-dy)*(1-dz);
  double B = dy*(1-dz);
  double C = (dy)*(dz);
  double D = (1-dy)*(dz);

  double fy1 = (coor[1][1]-pt[0])*coor[0][0] + (pt[0]-coor[0][1])*coor[1][0];
  fy1 = fy1/(coor[1][1]-coor[0][1]);
  double fy2 = (coor[1][1]-pt[0])*coor[3][0] + (pt[0]-coor[0][1])*coor[2][0];
  fy2 = fy2/(coor[1][1]-coor[0][1]);
  double x = (coor[3][2]-pt[1])*fy1 + (pt[1]-coor[0][2])*fy2;
  x = x/(coor[3][2]-coor[0][2]) + 0.82;

  point_t sn;
  for(int i = 0; i < 3; i++){
    sn.push_back(A*nv[0][i]+B*nv[1][i]+C*nv[2][i]+D*nv[3][i]);
  }

  // std::cout<< "4) Calced SN \n";

  return {x, sn};
}

double DrawingInput::getVectorSize(point_t &normal){
  return sqrt(normal[0]*normal[0]+normal[1]*normal[1]+normal[2]*normal[2]);
}

point_t DrawingInput::getCrossProduct(point_t &a, point_t &b){
  point_t temp;

  temp.push_back(a[1]*b[2]-a[2]*b[1]);
  temp.push_back(a[2]*b[0]-a[0]*b[2]);
  temp.push_back(a[0]*b[1]-a[1]*b[0]);

  return temp;
}

tf::Matrix3x3 DrawingInput::getRotationMatrix(point_t &axis, double c){
  double s = sqrt(1-(c*c));
  double ux = axis[0];
  double uy = axis[1];
  double uz = axis[2];

  tf::Matrix3x3 temp(c+ux*ux*(1-c),ux*uy*(1-c)-uz*s,ux*uz*(1-c)+uy*s,
                        uy*ux*(1-c)+uz*s, c+uy*uy*(1-c), uy*uz*(1-c)-ux*s,
                        uz*ux*(1-c)-uy*s, uz*uy*(1-c)+ux*s, c+uz*uz*(1-c));

  return temp;
}

void DrawingInput::setCanvasRangeArb(const std_msgs::Float64MultiArray &ri_ranges){
  // ranges: ranges calculated from Greedy Alg.
  std::cout << "Set Canvas Range\n";
  int num_range = ri_ranges.data.size();
  
  std::array<double, 2> range;
  
  range[0] = ri_ranges.data[0]; // start of drawing 

  for (int i = 1; i < num_range; i++) {
    range[1] = ri_ranges.data[i];
    this->ranges.push_back(range);
    range[0] = ri_ranges.data[i];
  }
}

void DrawingInput::splitByRangeArb(const std_msgs::Float64MultiArray &ri_ranges){
  // get range ready
  this->setCanvasRangeArb(ri_ranges);
  std::vector<std::vector<Stroke>> strokes_by_range (this->ranges.size());

  // detect which range the drawing coordinate is in
  int range_index_prev, range_index;
  std::cout << "Divide strokes to Range\n";
  
  for (int i = 0; i < this->strokes.size(); i++) {
    Stroke stroke;
    range_index_prev = this->detectRange(this->strokes[i][0]);

    for (int j = 1; j < this->strokes[i].size(); j++) {
      range_index = this->detectRange(strokes[i][j]);

      if (range_index_prev != range_index) { // split stroke
        int index = std::min(range_index_prev, range_index);
        geometry_msgs::Pose contact = this->strokes[i][j];
        contact.position.x = (strokes[i][j].position.x - strokes[i][j-1].position.x)*(this->ranges[index][1] - strokes[i][j-1].position.y)/(strokes[i][j].position.x - strokes[i][j-1].position.y) + strokes[i][j-1].position.x;
        contact.position.y = this->ranges[index][1];
        contact.position.z = (strokes[i][j].position.z - strokes[i][j-1].position.z)*(this->ranges[index][1] - strokes[i][j-1].position.y)/(strokes[i][j].position.z - strokes[i][j-1].position.y) + strokes[i][j-1].position.z;
        stroke.push_back (contact);

        // only if stroke size is bigger than 5 points
        if (stroke.size() > 5) {
          strokes_by_range[range_index_prev].push_back(stroke);
        }
        Stroke().swap(stroke);
        stroke.push_back (contact);
        range_index_prev = range_index;
      }
      else {
        stroke.push_back(this->strokes[i][j]);
      }
    }
    strokes_by_range[range_index].push_back(stroke);
  }

  this->strokes_by_range = strokes_by_range;

  std::cout << "Center strokes\n";

  // make center of splited drawing as y = 0 
  for (int i = 0; i < this->strokes_by_range.size(); i++){ //range
    double diff = (this->ranges[i][0] + this->ranges[i][1])/2;
    this->diffs.push_back(diff);
    for (int j = 0; j < this->strokes_by_range[i].size(); j++){ //strokes
       for (int k = 0; k < this->strokes_by_range[i][j].size(); k++) //pose
         this->strokes_by_range[i][j][k].position.y -= diff;
    }
  }
}


std::vector<std::vector<double>> DrawingInput::matrixMult(const std::vector<std::vector<double>> &A, const std::vector<std::vector<double>> &B){
  std::vector<std::vector<double>> ans;
  std::cout << "MULT << THIS IS WHERE THE PROBLEM IS AT THIS MOMENT\n";

  for(int i = 0; i < A.size(); i++){
    for(int j = 0; j < A[0].size(); j++){
      ans[i][j] = 0;
      for(int k=0; k < A[0].size(); k++){    
        ans[i][j]+=A[i][k]*B[k][j];    
      }  
    }
  }

  std::cout << "MULT return\n";


  return ans;
}

std::vector<std::vector<double>> DrawingInput::matrixInv(const std::vector<std::vector<double>> &m){
  std::vector<std::vector<double>> inv;
  std::cout << "INV\n";

  double A2323 = m[2][2] * m[3][3] - m[2][3] * m[3][2] ;
  double A1323 = m[2][1] * m[3][3] - m[2][3] * m[3][1] ;
  double A1223 = m[2][1] * m[3][2] - m[2][2] * m[3][1] ;
  double A0323 = m[2][0] * m[3][3] - m[2][3] * m[3][0] ;
  double A0223 = m[2][0] * m[3][2] - m[2][2] * m[3][0] ;
  double A0123 = m[2][0] * m[3][1] - m[2][1] * m[3][0] ;
  double A2313 = m[1][2] * m[3][3] - m[1][3] * m[3][2] ;
  double A1313 = m[1][1] * m[3][3] - m[1][3] * m[3][1] ;
  double A1213 = m[1][1] * m[3][2] - m[1][2] * m[3][1] ;
  double A2312 = m[1][2] * m[2][3] - m[1][3] * m[2][2] ;
  double A1312 = m[1][1] * m[2][3] - m[1][3] * m[2][1] ;
  double A1212 = m[1][1] * m[2][2] - m[1][2] * m[2][1] ;
  double A0313 = m[1][0] * m[3][3] - m[1][3] * m[3][0] ;
  double A0213 = m[1][0] * m[3][2] - m[1][2] * m[3][0] ;
  double A0312 = m[1][0] * m[2][3] - m[1][3] * m[2][0] ;
  double A0212 = m[1][0] * m[2][2] - m[1][2] * m[2][0] ;
  double A0113 = m[1][0] * m[3][1] - m[1][1] * m[3][0] ;
  double A0112 = m[1][0] * m[2][1] - m[1][1] * m[2][0] ;

  std::cout << "DET\n";

  double det = m[0][0] * ( m[1][1] * A2323 - m[1][2] * A1323 + m[1][3] * A1223 ) 
      - m[0][1] * ( m[1][0] * A2323 - m[1][2] * A0323 + m[1][3] * A0223 ) 
      + m[0][2] * ( m[1][0] * A1323 - m[1][1] * A0323 + m[1][3] * A0123 ) 
      - m[0][3] * ( m[1][0] * A1223 - m[1][1] * A0223 + m[1][2] * A0123 ) ;
  det = 1 / det;

  inv[0][0] = det *   ( m[1][1] * A2323 - m[1][2] * A1323 + m[1][3] * A1223 );
  inv[0][1] = det * - ( m[0][1] * A2323 - m[0][2] * A1323 + m[0][3] * A1223 );
  inv[0][2] = det *   ( m[0][1] * A2313 - m[0][2] * A1313 + m[0][3] * A1213 );
  inv[0][3] = det * - ( m[0][1] * A2312 - m[0][2] * A1312 + m[0][3] * A1212 );
  inv[1][0] = det * - ( m[1][0] * A2323 - m[1][2] * A0323 + m[1][3] * A0223 );
  inv[1][1] = det *   ( m[0][0] * A2323 - m[0][2] * A0323 + m[0][3] * A0223 );
  inv[1][2] = det * - ( m[0][0] * A2313 - m[0][2] * A0313 + m[0][3] * A0213 );
  inv[1][3] = det *   ( m[0][0] * A2312 - m[0][2] * A0312 + m[0][3] * A0212 );
  inv[2][0] = det *   ( m[1][0] * A1323 - m[1][1] * A0323 + m[1][3] * A0123 );
  inv[2][1] = det * - ( m[0][0] * A1323 - m[0][1] * A0323 + m[0][3] * A0123 );
  inv[2][2] = det *   ( m[0][0] * A1313 - m[0][1] * A0313 + m[0][3] * A0113 );
  inv[2][3] = det * - ( m[0][0] * A1312 - m[0][1] * A0312 + m[0][3] * A0112 );
  inv[3][0] = det * - ( m[1][0] * A1223 - m[1][1] * A0223 + m[1][2] * A0123 );
  inv[3][1] = det *   ( m[0][0] * A1223 - m[0][1] * A0223 + m[0][2] * A0123 );
  inv[3][2] = det * - ( m[0][0] * A1213 - m[0][1] * A0213 + m[0][2] * A0113 );
  inv[3][3] = det *   ( m[0][0] * A1212 - m[0][1] * A0212 + m[0][2] * A0112 );

  std::cout << "INV return\n";

  return inv;
}

void DrawingInput::relocateDrawingsArb(geometry_msgs::Pose &ridgeback, int range_index){
  // use ridgeback's position and orientation + wall spawn loaction to change world coordinate system to local coordinate system
  ROS_INFO("Calculate Transformation Matrix");
  
  double rad = ridgeback.orientation.x * PI / 180.0;
  // calculate transformation matrix
  std::cout << "T\n";
  std::vector<std::vector<double>> T; // translation matrix
  T.push_back({1,0,0,ridgeback.position.x});
  T.push_back({0,1,0,ridgeback.position.y});
  T.push_back({0,0,1,ridgeback.position.z});
  T.push_back({0,0,0,1});
  std::cout << "R\n";

  std::vector<std::vector<double>> R; // rotation matrix
  R.push_back({cos(rad),-sin(rad),0,0});
  R.push_back({sin(rad),cos(rad), 0,0});
  R.push_back({0,0, 1, 0});
  R.push_back({0,0, 0, 1});

  ROS_INFO("Calculating Inverse...");

  std::vector<std::vector<double>> trans = matrixInv(matrixMult(T, R));

  ROS_INFO("Tranforming strokes...");

  for(int j = 0; j < strokes_by_range[range_index].size(); j++){
    for (int k = 0; k < this->strokes_by_range[range_index][j].size(); k++){ //pose
      double sum = 0; // x
      sum += trans[0][0]*this->strokes_by_range[range_index][j][k].position.x;   
      sum += trans[0][1]*this->strokes_by_range[range_index][j][k].position.y;   
      sum += trans[0][2]*this->strokes_by_range[range_index][j][k].position.z;   
      sum += trans[0][3];   
      this->strokes_by_range[range_index][j][k].position.x = sum;

      sum = 0; // y
      sum += trans[1][0]*this->strokes_by_range[range_index][j][k].position.x;   
      sum += trans[1][1]*this->strokes_by_range[range_index][j][k].position.y;   
      sum += trans[1][2]*this->strokes_by_range[range_index][j][k].position.z;   
      sum += trans[1][3];   
      this->strokes_by_range[range_index][j][k].position.y = sum;

      sum = 0; // z
      sum += trans[2][0]*this->strokes_by_range[range_index][j][k].position.x;   
      sum += trans[2][1]*this->strokes_by_range[range_index][j][k].position.y;   
      sum += trans[2][2]*this->strokes_by_range[range_index][j][k].position.z;   
      sum += trans[2][3];   
      this->strokes_by_range[range_index][j][k].position.z = sum;

      sum = 0; // h
      sum += trans[3][0]*this->strokes_by_range[range_index][j][k].position.x;   
      sum += trans[3][1]*this->strokes_by_range[range_index][j][k].position.y;   
      sum += trans[3][2]*this->strokes_by_range[range_index][j][k].position.z;   
      sum += trans[3][3];   
      
      if(sum != 1){
        this->strokes_by_range[range_index][j][k].position.x /= sum;
        this->strokes_by_range[range_index][j][k].position.y /= sum;
        this->strokes_by_range[range_index][j][k].position.z /= sum;
      }
    }  
  }
}
