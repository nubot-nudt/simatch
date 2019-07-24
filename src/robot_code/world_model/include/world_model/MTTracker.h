#ifndef __NUBOT_VISION_MTTRACKER_H_
#define __NUBOT_VISION_MTTRACKER_H_

#include <boost/foreach.hpp>
#include <boost/smart_ptr.hpp>
#include <core.hpp>
#include <Eigen/Eigen>
#include <list>
#include <vector>
#include <bitset>
#include <map>

//#define Record_Measure
#ifdef Record_Measure
#include <boost/archive/binary_oarchive.hpp>
#include <boost/archive/binary_iarchive.hpp>
#include <boost/serialization/vector.hpp>
#include <boost/serialization/map.hpp>
#include <fstream>
#include <sstream>
using namespace boost::archive;
#endif

namespace nubot {


typedef Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> Matrix;
typedef Eigen::Matrix<double, Eigen::Dynamic, 1>			Vector;
typedef Eigen::Array <double, Eigen::Dynamic, Eigen::Dynamic> Array;
typedef Eigen::Array <double, Eigen::Dynamic, 1>			Array1d;

const double sigma_vel=pow(50.0, 2);
const double sigma_range=pow(100.0, 2);
const double sigma_bearing=pow(Angle(5,false).radian_, 2);

class DKF
{
public:
	DKF(){};
	DKF(Vector &X0,Matrix &A, Matrix &H, Matrix &Q,Matrix &R);
	~DKF(){};
	void Update(obs_info &m);
	void Predict();
	void Fuse();

public:
	Matrix A,Q,H,R;		

	Vector HRZ_sum;
	Matrix HRH_sum;
	Vector M;
	Matrix P;
};



class Filter
{
public:
	Filter(double x0,double y0);
	~Filter();
	double distance(obs_info &m);
	double variance();
	void Update(obs_info &m);
	void Predict();
	void Fuse();

    static void CalculateR(obs_info &m, Matrix &R);
    static void CalcInformationMatrix(obs_info &m);

	static bool initAQHR();
	double operator() (size_t i) 
	{return f->M(i);};

public:
    static Matrix A,Q,H,R;
    static double dt;
    DKF *f;

    bool deteced;
    int  update_cnt;

    std::bitset<7> SeenByRobots;
    int  TeammateID;
};


typedef boost::shared_ptr<Filter> Filter_ptr;


typedef std::list<Filter_ptr>  FilterList;
typedef std::vector<obs_info>  Measures;
typedef std::map<const int,DPoint>  Teammates;			// teammates
typedef std::pair<const int,DPoint> Teammate;
class MTTracker
{
public:
	MTTracker(void){};
	~MTTracker(void){};

    void MeasureAssociate(Measures &measure_datas, int robot_id=0);

	void TracksPrediction();

	void RearrangeTracks();

	void TeammateIdentify(Teammates &teammates);

	void GetAllObjects(std::vector<DPoint> &Objects);

	void GetObjects(std::vector<DPoint> &Objects, unsigned int amount=-1);

    FilterList NewTracks;
    FilterList Tracks;
};


#ifdef Record_Measure

class Measure_Record
{
public:
	friend class boost::serialization::access;
	static std::ofstream file;
	static binary_oarchive *oa;

	std::vector< std::vector<obs_info> > data;
	Teammates	robots;
    uint frame;

public:
	Measure_Record(){};
    Measure_Record(uint frame,Teammates &robots, std::vector< std::vector<obs_info> > &data)
		:robots(robots),frame(frame),data(data)
	{ save(); file.flush();};

	void save()
	{ *oa << *this;}

	template<class Archive>
	void serialize(Archive & ar, const unsigned int version)
	{
		ar & frame;
		ar & robots;
		ar & data;
	}

	static bool init()
	{
		std::ifstream tmp;	int i=0;
		while(1)
		{
			std::ostringstream name;
			name << "obstacle_fusion"<< i++ <<".dat";
            tmp.open(name.str().c_str());


			if (tmp)
				tmp.close();
			else
			{
                file.open(name.str().c_str(),std::ofstream::binary);
				break;
			}
		}
		oa=new binary_oarchive(file);	
		return true;
	}

	static void close()
	{
		delete oa;
		file.close();
	}
};

namespace boost {
	namespace serialization {


		template<class Archive>
		void serialize(Archive & ar, obs_info & m, const unsigned int version)
		{
            ar & m.world_pt.x_;
            ar & m.world_pt.y_;
            ar & m.polar_pt.angle_.radian_;
            ar & m.polar_pt.radius_;
			ar & m.base;
		}


		template<class Archive>
		void serialize(Archive & ar, DPoint & p, const unsigned int version)
		{
            ar & p.x_;
            ar & p.y_;
		}
	} // namespace serialization
} // namespace boost
#endif  // Record_Measure



template <size_t N>
class trigger
{
public:
	trigger() :cnt(0){};

	size_t cnt;
	bool operator() ()
	{
		if (++cnt<N)
			return false;
		else
		{  cnt=0; return true; }
	}
};


}

#endif //!nubot
