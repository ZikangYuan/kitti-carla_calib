#include <iostream>
#include <fstream>
#include <iterator>
#include <string>
#include <vector>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <eigen3/Eigen/Dense>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <math.h>

using namespace std;

static const int READ_SIZE = 16000;
static const int WRITE_SIZE = 16000;

class Point3D {

public:
    Eigen::Vector3d raw_pt; // Raw point read from the sensor
    Eigen::Vector3d pt; // Corrected point taking into account the motion of the sensor during frame acquisition
    double alpha_timestamp; // Relative timestamp in the frame in [0.0, 1.0]
    double timestamp; // The absolute timestamp (if applicable)
    int index_frame; // The frame index

    Point3D()
    {
        alpha_timestamp = 0.0;
        timestamp = 0.0;
        index_frame = -1;
    }
};

bool time_list(Point3D &point_1, Point3D &point_2)
{
    return (point_1.alpha_timestamp < point_2.alpha_timestamp);
};

enum openMode{ fileOpenMode_OUT = 0, fileOpenMode_IN = 1 };

class File
{
public:
    File(string path, openMode flag);
    ~File();


protected:
    const openMode _mode;
    string     _path;
    fstream    _file;
};

File::File(string path, openMode flag) : _path(path), _mode(flag)
{
    switch (_mode)
    {
    case fileOpenMode_IN:
    {
        _file = fstream(_path.c_str(), ios::in | ios::binary);
        break;
    }
    case fileOpenMode_OUT:
    {
        _file = fstream(_path.c_str(), ios::out | ios::binary);
        break;
    }
    }

    if (!_file.good())
    {
        cout << "ERROR: can't open " << _path << endl;
    }
}


File::~File()
{
    _file.close();
}

enum plyFormat{ binary_little_endian = 0, binary_big_endian = 1, ascii = 2 };
enum plyTypes{ float32 = 0, float64 = 1, uchar2 = 2, int32 = 3, otherxx = -1 };


class PlyFile :
    public File
{
public:
    PlyFile(string path, openMode flag);
    ~PlyFile();
    
    void readFile(char*& points, int& pointSize, int& numPoints);
    void writeFile(char* points, int numPoints, list<string> properties, list<plyTypes> types);

    void displayInfos();
    
    static const int READ_SIZE = 16000;
    static const int WRITE_SIZE = 16000;

private:
    void readHeader();
    void writeHeader();

private:
    string    _header;
    plyFormat _format;

    int       _propertyNum;
    string*   _propertyName;
    plyTypes* _propertyType;
    int*      _propertySize;

    int   _numPoints;
    int   _pointSize;
};

PlyFile::PlyFile(string path, openMode flag) : File(path, flag), _header(""), _format(binary_little_endian), 
_propertyNum(0), _propertyType(NULL), _propertySize(NULL), _propertyName(NULL), 
_numPoints(0), _pointSize(0)
{
    if (_mode == fileOpenMode_IN)
    {
        readHeader();
    }
}


PlyFile::~PlyFile()
{
    delete[] _propertyType;
    delete[] _propertySize;
    delete[] _propertyName;
}



void PlyFile::readHeader()
{
    // GET HEADER
    // ------------------------------------------------------------------------------------------
    string tmpStr = "";
    
    do
    {
        getline(_file, tmpStr);
        _header += tmpStr + "\n";
    } while (tmpStr.find("end_header") != 0);

    
    // PARSE HEADER
    // ------------------------------------------------------------------------------------------
    stringstream streamHeader(_header);
    string strTmp = "";
    list<plyTypes> typePptTmp;
    list<int>      sizePptTmp;
    list<string>   namePptTmp;

    while (!streamHeader.eof())
    {
        streamHeader >> strTmp;
        
        if (strTmp.compare("format") == 0)
        {
            streamHeader >> strTmp;
            if (strTmp.compare("binary_little_endian") == 0)      _format = binary_little_endian;
            else if (strTmp.compare("binary_big_endian") == 0)    _format = binary_big_endian;
            else if (strTmp.compare("ascii") == 0)                _format = ascii;  
        }

        if (strTmp.compare("element") == 0)
        {
            streamHeader >> strTmp;
            if (strTmp.compare("vertex") == 0) streamHeader >> _numPoints;
        }

        if (strTmp.compare("property") == 0)
        {
            _propertyNum++;
            streamHeader >> strTmp;
            if ((strTmp.compare("float32") == 0) | (strTmp.compare("float") == 0))
            {
                typePptTmp.push_back(float32);
                sizePptTmp.push_back(4);
            }
            else if ((strTmp.compare("float64") == 0) | (strTmp.compare("double") == 0))
            {
                typePptTmp.push_back(float64);
                sizePptTmp.push_back(8);
            }
            else if ((strTmp.compare("int") == 0))
            {
                typePptTmp.push_back(int32);
                sizePptTmp.push_back(4);
            }
            else if ((strTmp.compare("uchar") == 0))
            {
                typePptTmp.push_back(uchar2);
                sizePptTmp.push_back(1);
            }
            else
            {
                typePptTmp.push_back(otherxx);
                sizePptTmp.push_back(4); // Default
            }

            streamHeader >> strTmp;
            namePptTmp.push_back(strTmp);
        }
    }
    

    // FILL PROPERTIES ARRAYS
    // ------------------------------------------------------------------------------------------
    _propertyType = new plyTypes[_propertyNum];
    _propertySize = new int[_propertyNum];
    _propertyName = new string[_propertyNum];

    for (int i(0); i < _propertyNum; i++)
    {
        _propertyType[i] = typePptTmp.front();
        _propertySize[i] = sizePptTmp.front();
        _propertyName[i] = namePptTmp.front();
        typePptTmp.pop_front();
        sizePptTmp.pop_front();
        namePptTmp.pop_front();

        _pointSize += _propertySize[i];
    }
}

void PlyFile::writeHeader()
{
    _header = "";


    _header += "ply";
    _header += "\n";


        _header += "format ";
        switch (_format)
        {
        case binary_little_endian:
        {
            _header += "binary_little_endian 1.0";
            break;
        }
        case binary_big_endian:
        {
            _header += "binary_big_endian 1.0";
            break;
        }
        case ascii:
        {
            _header += "binary_ascii 1.0";
            break;
        }
        }
        _header += "\n";


        _header += "element vertex ";
        _header += to_string(_numPoints);
        _header += "\n";


        for (int i(0); i < _propertyNum; i++)
        {
            _header += "property ";
            switch (_propertyType[i])
            {
            case float32:
            {
                _header += "float32 ";
                break;
            }
            case float64:
            {
                _header += "float64 ";
                break;
            }
            case uchar2:
            {
                _header += "uchar ";
                break;
            }
            case int32:
            {
                _header += "int ";
                break;
            }
            }
            _header += _propertyName[i];
            _header += "\n";
        }


    _header += "end_header";
    _header += "\n";

    _file << _header;
}

void PlyFile::readFile(char*& points, int& pointSize, int& numPoints)
{
    switch (_format)
    {
    case binary_little_endian:
    {
        // ----- Allocate memory ------------------------------------------------
        if (points != 0)
        {
            delete[] points;
        }

        points = new char[(unsigned long long int)_pointSize*(unsigned long long int)_numPoints];
        unsigned long long int bufferSize = (unsigned long long int)_pointSize*(unsigned long long int)_numPoints;


        // ----- Read raw data --------------------------------------------------
        unsigned long long int n = bufferSize / (unsigned long long int)READ_SIZE;
        unsigned long long int r = bufferSize % (unsigned long long int)READ_SIZE;

        for (unsigned long long int i(0); i < n; i++)
        {
            _file.read(points + i*(unsigned long long int)READ_SIZE, READ_SIZE);
        }

        _file.read(points + n*(unsigned long long int)READ_SIZE, r);

        numPoints = _numPoints;
        pointSize = _pointSize;

        break;
    }
    case binary_big_endian:
    {
        cout << "WARNING: function not implemented for binary big endian file" << endl;
        break;
    }
    case ascii:
    {
        cout << "WARNING: function not implemented for ascii file" << endl;
        break;
    }
    }   
}

void PlyFile::writeFile(char* points, int numPoints, list<string> properties, list<plyTypes> types)
{
    // ----- Set properties -------------------------------------------------
    _format = binary_little_endian;
    _numPoints = numPoints;

    if (properties.size() != types.size())
    {
        cout << "Warning : mismatch between properties and types" << endl;
        return;
    }

    _propertyNum = (int) properties.size();
    _pointSize = 0;

    _propertyType = new plyTypes[_propertyNum];
    _propertySize = new int[_propertyNum];
    _propertyName = new string[_propertyNum];


    auto propIt = properties.begin();
    auto typesIt = types.begin();
    int i = 0;

    for (int i(0); i < _propertyNum; i++)
    {
        _propertyName[i] = *propIt;
        _propertyType[i] = *typesIt;


        if (_propertyType[i] == float32)
            _propertySize[i] = 4;
        if (_propertyType[i] == float64)
            _propertySize[i] = 8;
        if (_propertyType[i] == int32)
            _propertySize[i] = 4;
        if (_propertyType[i] == uchar2)
            _propertySize[i] = 1;
        if (_propertyType[i] == otherxx)
            _propertySize[i] = 4; //default

        _pointSize += _propertySize[i];

        ++propIt;
        ++typesIt;
    }


    // ----- Write header ---------------------------------------------------
    writeHeader();


    // ----- Write points ---------------------------------------------------
    unsigned long long int bufferSize = (unsigned long long int)_pointSize*(unsigned long long int)_numPoints;
    
    unsigned long long int n = bufferSize / (unsigned long long int)WRITE_SIZE;
    unsigned long long int r = bufferSize % (unsigned long long int)WRITE_SIZE;

    for (unsigned long long int i(0); i < n; i++)
    {
        _file.write(points + i*(unsigned long long int)WRITE_SIZE, WRITE_SIZE);
    }

    _file.write(points + n*(unsigned long long int)WRITE_SIZE, r);
}

void PlyFile::displayInfos()
{
    cout << "------------------------------------------------------" << endl;
    cout << " PLY File : " << _path << endl;
    cout << "------------------------------------------------------" << endl;
    cout << "  - format     : " << _format << endl;
    cout << "  - num points : " << _numPoints << endl;
    cout << "  - properties : " << endl;
    for (int i(0); i < _propertyNum; i++)
    {
        cout << "     - " << _propertyName[i] << " :    " << _propertyType[i] << " |    " << _propertySize[i] << " bytes " << endl;
    }
    cout << "------------------------------------------------------" << endl << endl;
}

std::vector<Point3D> read_kitti_carla_pointcloud(const std::string &path, int &sizeOfPointsIn, int &numPointsIn) {
    std::vector<Point3D> frame;

    PlyFile plyFileIn(path, fileOpenMode_IN);
    char *dataIn = nullptr;
    plyFileIn.readFile(dataIn, sizeOfPointsIn, numPointsIn);
    frame.reserve(numPointsIn);

    double frame_last_timestamp = 0.0;
    double frame_first_timestamp = 1000000000.0;
    for (int i(0); i < numPointsIn; i++) {

        unsigned long long int offset =
                (unsigned long long int) i * (unsigned long long int) sizeOfPointsIn;
        Point3D new_point;
        new_point.raw_pt[0] = *((float *) (dataIn + offset));
        offset += sizeof(float);
        new_point.raw_pt[1] = *((float *) (dataIn + offset));
        offset += sizeof(float);
        new_point.raw_pt[2] = *((float *) (dataIn + offset));
        offset += sizeof(float);


        new_point.pt = new_point.raw_pt;
        double cos = *((float *) (dataIn + offset));
        offset += sizeof(float);
        new_point.timestamp = *((float *) (dataIn + offset));
        offset += sizeof(float);
        uint32_t index = *((uint32_t *) (dataIn + offset));
        offset += sizeof(uint32_t);
        uint32_t label = *((uint32_t *) (dataIn + offset));
        offset += sizeof(uint32_t);

        if (new_point.timestamp < frame_first_timestamp) {
            frame_first_timestamp = new_point.timestamp;
        }

        if (new_point.timestamp > frame_last_timestamp) {
            frame_last_timestamp = new_point.timestamp;
        }

        double r = new_point.raw_pt.norm();
        if ((r > 0.5) && (r < 100.0))
            frame.push_back(new_point);
    }

    for (int i(0); i < (int) frame.size(); i++) {
        frame[i].alpha_timestamp = min(1.0, max(0.0, 1 - (frame_last_timestamp - frame[i].timestamp) /
                                                         (frame_last_timestamp - frame_first_timestamp)));
    }
    frame.shrink_to_fit();

    delete[] dataIn;
    return frame;
}

void write_kitti_carla_pointcloud(const std::string &path, int sizeOfPointsIn, std::vector<Point3D> &frame)
{
    PlyFile plyFileOut(path, fileOpenMode_OUT);
    char *dataOut = new char[(unsigned long long int)sizeOfPointsIn * (unsigned long long int)frame.size()];

    for (int i = 0; i < frame.size(); i++) {
        unsigned long long int offset =
                (unsigned long long int) i * (unsigned long long int) sizeOfPointsIn;
        *((float *) (dataOut + offset)) = float(frame[i].pt[0]);
        offset += sizeof(float);
        *((float *) (dataOut + offset)) = float(frame[i].pt[1]);
        offset += sizeof(float);
        *((float *) (dataOut + offset)) = float(frame[i].pt[2]);
        offset += sizeof(float);

        *((float *) (dataOut + offset)) = float(1.0);
        offset += sizeof(float);
        *((float *) (dataOut + offset)) = float(frame[i].timestamp);
        offset += sizeof(float);
        *((uint32_t *) (dataOut + offset)) = uint32_t(i);
        offset += sizeof(uint32_t);
        *((uint32_t *) (dataOut + offset)) = uint32_t(0);
        offset += sizeof(uint32_t);
    }

    list<plyTypes> types;
    types.push_back(float32);
    types.push_back(float32);
    types.push_back(float32);
    types.push_back(float32);
    types.push_back(float32);
    types.push_back(int32);
    types.push_back(int32);

    list<string> properties;
    properties.push_back("x");
    properties.push_back("y");
    properties.push_back("z");
    properties.push_back("cos_angle_lidar_surface");
    properties.push_back("timestamp");
    properties.push_back("instance");
    properties.push_back("semantic");

    plyFileOut.writeFile(dataOut, frame.size(), properties, types);

    delete[] dataOut;
}

void loadPose(const std::string &path, std::vector<double> &vTime, std::vector<Eigen::Matrix4d> &vPose)
{
    std::ifstream infileGt;
    infileGt.open(path);

    Eigen::Matrix4d Ttemp = Eigen::MatrixXd::Identity(4, 4);

    while(!infileGt.eof())
    {
        std::string s;
        getline(infileGt,s);

        if(s.length() == 0)
            break;

        double timestamp;

        Eigen::Matrix3d R_temp;
        Eigen::Vector3d t_temp;

        std::stringstream ss;
        ss << s;
        ss >> R_temp(0, 0); ss >> R_temp(0, 1); ss >> R_temp(0, 2); ss >> t_temp(0, 0);
        ss >> R_temp(1, 0); ss >> R_temp(1, 1); ss >> R_temp(1, 2); ss >> t_temp(1, 0);
        ss >> R_temp(2, 0); ss >> R_temp(2, 1); ss >> R_temp(2, 2); ss >> t_temp(2, 0);
        ss >> timestamp;

        Eigen::Matrix4d Twc = Eigen::MatrixXd::Identity(4, 4);
        Twc.block<3, 3>(0, 0) = R_temp;
        Twc.block<3, 1>(0, 3) = t_temp;

        vPose.push_back(Twc);
        vTime.push_back(timestamp);
    }

    infileGt.close();

    std::cout << "load pose successfully!" << std::endl;
}

void loadTimestamp(const std::string &path, std::vector<int> &vIndex, std::vector<double> &vTime)
{
    std::ifstream infile;
    infile.open(path);

    while(!infile.eof())
    {
        std::string s;
        getline(infile, s);

        if(s.length() == 0)
            break;

        int index;
        double timestamp;

        std::stringstream ss;
        ss << s;
        ss >> index;
        ss >> timestamp;

        vIndex.push_back(index);
        vTime.push_back(timestamp);
    }

    infile.close();

    std::cout << "load pose successfully!" << std::endl;
}

int main(int argc, char** argv)
{
    if(argc != 3)
    {
        std::cout << "input error parameters!" << std::endl;
        return 1;
    }

    std::string root_path = std::string(argv[1]);
    std::string cloud_dir_path = root_path + "/generated/frames";
    std::string pose_file_path = root_path + "/generated/full_poses_lidar.txt";
    std::string time_file_path = root_path + "/generated/full_ts_camera.txt";

    std::vector<double> v_time_lidar_pose;
    std::vector<Eigen::Matrix4d> v_lidar_pose;
    loadPose(pose_file_path, v_time_lidar_pose, v_lidar_pose);

    std::vector<int> v_index_sweep;
    std::vector<double> v_time_sweep;
    loadTimestamp(time_file_path, v_index_sweep, v_time_sweep);

    std::string result_path = std::string(argv[2]);
    std::string result_sweep_path = result_path + "/correct";
    std::string result_pose_path = result_path + "/groundtruth.txt";

    int index_sweep_begin = 0;
    int index_sweep_end = 0;

    for(int i = 0; i < v_time_sweep.size(); i++)
    {
        std::stringstream lidar_data_path; 
        lidar_data_path << cloud_dir_path + "/frame_" << std::setfill('0') << std::setw(4) << v_index_sweep[i] << ".ply";
        int sizeOfPointsIn = 0, numPointsIn = 0;
        std::vector<Point3D> lidar_data = read_kitti_carla_pointcloud(lidar_data_path.str(), sizeOfPointsIn, numPointsIn);
        sort(lidar_data.begin(), lidar_data.end(), time_list);

        std::vector<double> vTime;
        std::vector<Eigen::Matrix4d> vPose;

        for(int j = index_sweep_end; j < v_time_lidar_pose.size(); j++)
        {
            if(v_time_sweep[i] - v_time_lidar_pose[j] > 1e-6)
            {
                std::cout << "error timestamp!" << std::endl;
                return 1;
            }

            vTime.push_back(v_time_lidar_pose[j]);
            vPose.push_back(v_lidar_pose[j]);

            if(fabs(v_time_lidar_pose[j] - v_time_sweep[i]) < 1e-6)
                index_sweep_begin = j;

            if(fabs(v_time_lidar_pose[j] - ((i + 1) >= v_time_sweep.size() ? v_time_lidar_pose.back() : v_time_sweep[i + 1])) < 1e-6) {
                index_sweep_end = j;
                break;
            }
        }

        double initial_time = vTime[0];
        double last_time = vTime.back();

        for(int j = 0; j < vTime.size(); j++)
            vTime[j] = (vTime[j] - initial_time) / (last_time - initial_time);

        Eigen::Matrix3d R_initial = vPose[0].block<3, 3>(0, 0);
        Eigen::Vector3d t_initial = vPose[0].block<3, 1>(0, 3);

        int m = 0;

        for(size_t j = 0; j < lidar_data.size(); ++j)
        {
            if(lidar_data[j].alpha_timestamp >= vTime[m] && lidar_data[j].alpha_timestamp <= vTime[m + 1])
            {
                Eigen::Quaterniond q_begin(vPose[m].block<3, 3>(0, 0));
                Eigen::Vector3d t_begin = vPose[m].block<3, 1>(0, 3);

                Eigen::Quaterniond q_end(vPose[m + 1].block<3, 3>(0, 0));
                Eigen::Vector3d t_end = vPose[m + 1].block<3, 1>(0, 3);

                double alpha = lidar_data[j].alpha_timestamp;
                Eigen::Quaterniond q_slerp = q_begin.slerp(alpha, q_end);
                q_slerp.normalize();
                Eigen::Vector3d t_slerp = t_begin * (1 - alpha) + t_end * alpha;

                Eigen::Vector3d pt_world = q_slerp * lidar_data[j].raw_pt + t_slerp;
                lidar_data[j].pt = R_initial.transpose() * pt_world - R_initial.transpose() * t_initial;
            }
            else if(lidar_data[j].alpha_timestamp > vTime[m + 1])
            {
                m++;
                j--;
                continue;
            }
        }

        std::stringstream save_sweep_path;
        save_sweep_path << result_sweep_path + "/frame_" << std::setfill('0') << std::setw(4) << v_index_sweep[i] << ".ply";
        write_kitti_carla_pointcloud(save_sweep_path.str(), sizeOfPointsIn, lidar_data);

        std::ofstream foutC(result_pose_path, std::ios::app);

        foutC.setf(std::ios::scientific, std::ios::floatfield);
        foutC.precision(6);

        foutC << std::fixed << R_initial(0, 0) << " " << R_initial(0, 1) << " " << R_initial(0, 2) << " " << t_initial(0, 0) << " "
                            << R_initial(1, 0) << " " << R_initial(1, 1) << " " << R_initial(1, 2) << " " << t_initial(1, 0) << " "
                            << R_initial(2, 0) << " " << R_initial(2, 1) << " " << R_initial(2, 2) << " " << t_initial(2, 0) << std::endl;

        foutC.close();
    }

    return 0;
}
