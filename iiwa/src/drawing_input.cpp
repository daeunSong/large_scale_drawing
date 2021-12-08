#include "drawing_input.h"

DrawingInput::DrawingInput(const std::string path, const std::string file_name,
                                  const char color, const std::string file_extension,
                                  geometry_msgs::Pose drawing_pose) {
  std::cout << "\n\n Drawing Input Constructor1 \n\n\n\n";

  setFileName(path, file_name, color, file_extension);
  this->drawing_pose = drawing_pose;

  readDrawingFile();
  removeLongLine();
  if (this->size[0] > 0.55)
    splitByRange();
}

// DrawingInput::DrawingInput(const std::string wall_name_, const std::string path, const std::string file_name,
//                                   const char color, const std::string file_extension,
//                                   const geometry_msgs::Pose drawing_pose) {
//   setFileName(path, file_name, color, file_extension);
//   this->drawing_pose = drawing_pose;
// }


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

void DrawingInput::setKDTree(){
  ROS_WARN_STREAM("SET KD TREE\n");
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

