#include "world_model/MTTracker.h"
#include "world_model/iAuction.h"
#include <math.h>

#define _USE_MATH_DEFINES
using namespace std;
using namespace nubot;

#ifdef Record_Measure
ofstream Measure_Record::file;
binary_oarchive* Measure_Record::oa;
#endif

Matrix Filter::A(4,4);
Matrix Filter::Q(4,4);
Matrix Filter::H(2,4);
Matrix Filter::R(2,2);
double Filter::dt=1/30.0;
bool Filter::initAQHR()
{

    double dt2=dt*dt,dt3=dt2*dt;
    A << 1,	0,	dt,	0,
            0,	1,	0,	dt,
            0,	0,	1,	0,
            0,	0,	0,	1;

    Q << dt3/3,		0,	 dt2/2,      0,
            0,		dt3/3,       0,  dt2/2,
            dt2/2,	 	0,		dt,      0,
            0,		dt2/2,       0,     dt;
    Q*=sigma_vel;

    R =Matrix::Zero(2,2);
    //R.diagonal() << var_range,var_range;
    R.diagonal() << sigma_range,sigma_bearing;

    H =Matrix::Zero(2,4);
    H.diagonal() << 1,1;

    //disp(A);disp(Q);disp(H);disp(R);


#if 0
    _clearfp();
    unsigned int cw = _controlfp(0, 0);
    cw &=~(EM_OVERFLOW|EM_UNDERFLOW|EM_ZERODIVIDE|
           EM_DENORMAL|EM_INVALID);
    unsigned int cwOriginal = _controlfp(cw, MCW_EM);
#endif // _DEBUG

    return true;
}

Filter::~Filter()
{
    delete f;
}

Filter::Filter(double x0,double y0)
    :update_cnt(1),deteced(true),SeenByRobots(0),TeammateID(-1)
{

    static bool inited = initAQHR();

    Vector X0(4);
    X0<< x0,y0,0,0;

    //Array1d sigma0(2);
    //sigma0 << 100,100;

    //f=new UKF(X0,Q,R, CV, xy2polar);
    //f=new EKF(X0,f_cv,f_cv_dx,xy2polar,h_xy2polar,Q,R);
    //f=new VBAKF(X0,A,H,Q,sigma0);
    //f=new KF(X0,A,H,Q,R);

    f=new DKF(X0,A,H,Q,R);
    //f=new DEKF(X0,f_cv,f_cv_dx,xy2polar,h_xy2polar,Q,R);
}


void Filter::Update(obs_info &m)
{
    f->Update(m);

    update_cnt=min(30, update_cnt+1);
    deteced=true;
}

void Filter::Fuse()
{
    if (deteced)
        f->Fuse();
}


inline double Filter::distance(obs_info &m)
{
    double dis =  hypot(m.world_pt.x_ - f->M(0), m.world_pt.y_ - f->M(1));
    if(std::isnan(dis))
        return std::numeric_limits<double>::infinity();
    else
        return dis;
}

inline double Filter::variance()
{
    return f->P(0,0) + f->P(1,1);
}
void Filter::CalculateR(obs_info &m, Matrix &R)
{
    double co=cos(m.polar_pt.angle_.radian_),si=sin(m.polar_pt.angle_.radian_);

    Matrix Rot(2,2);
    Rot << co,-si,si,co;
    Matrix r0=Matrix::Zero(2,2);
    r0(0,0)=sigma_range;
    r0(1,1)=pow(m.polar_pt.radius_,2)*sigma_bearing;

    R= Rot*r0*Rot.transpose();
}

void Filter::CalcInformationMatrix(obs_info &m)
{
    Matrix R(2,2);
    Vector Z(2);
    Matrix &H = Filter::H;


    if (m.polar_pt.radius_==0)
        return;

    CalculateR(m,R);
    Matrix HR = H.transpose()*R.partialPivLu().inverse();

    Z<<m.world_pt.x_,m.world_pt.y_;
    Eigen::Map<Vector> HRZ(m.HRZ,4);
    Eigen::Map<Matrix> HRH(m.HRH,4,4);

    HRZ = HR*Z;
    HRH = HR*H;
}

void Filter::Predict() 
{ 
    f->Predict();


    update_cnt -= deteced? 0 : 1;
    deteced=false;


    SeenByRobots.reset();
    TeammateID = -1;
}

inline Filter_ptr getFilter(int id,FilterList list)
{
    FilterList::iterator it=list.begin();
    advance(it,id);
    return *it;
}

void MTTracker::MeasureAssociate(Measures &measure_datas, int robot_id)
{
    // 	TRACE("[M:%d] ",measure_datas.size());
    const double inf = -std::numeric_limits<double>::infinity();
    const double Gate=150;//4.6;

    if (measure_datas.empty())
        return;
    size_t num_rows=measure_datas.size();

    size_t num_cols=Tracks.size()+NewTracks.size()+measure_datas.size();

    mat assignment_mat(num_rows);
    for(size_t i=0; i<num_rows; i++)
        assignment_mat[i].resize(num_cols,-100);

    for(size_t i=0;i<measure_datas.size();i++)
    {
        obs_info &m=measure_datas[i];
        vector<double> &dis=assignment_mat[i];
        int j=0;


        for (FilterList::iterator it = Tracks.begin();it!=Tracks.end();it++,j++)
        {
            Filter &f=**it;
            dis[j] = -f.distance(m);
        }

        for (FilterList::iterator it = NewTracks.begin();it!=NewTracks.end();it++,j++)
        {
            Filter &f=**it;
            dis[j] = -f.distance(m);
        }

        for (size_t k=0;k<measure_datas.size();k++,j++)
        {
            dis[j] = -Gate;
        }
    }


    iAuction gnn(assignment_mat);
    gnn.MainAlgo();

    const size_t Ts=Tracks.size(),NTs=NewTracks.size();
    for(size_t i=0;i<measure_datas.size();i++)
    {
        size_t filterID = gnn.GetAssignedCol(i);
        obs_info &m = measure_datas[i];

        if (filterID<Ts)
        {
            Filter &f=*getFilter(filterID, Tracks);
            f.Update(m);
            f.SeenByRobots.set(robot_id);
        }

        else if(filterID<Ts+NTs)
        {
            Filter &f=*getFilter(filterID-Ts, NewTracks);
            f.Update(m);
            f.SeenByRobots.set(robot_id);
        }

        else
        {

            Filter_ptr p(new Filter(m.world_pt.x_, m.world_pt.y_));
            p->SeenByRobots.set(robot_id);
            NewTracks.push_back(p);
        }
    } // for (size_t i=0;i<measure_datas.size();i++)
}


void MTTracker::TracksPrediction()
{	

    BOOST_FOREACH(Filter_ptr &f, Tracks)
            f->Predict();

    BOOST_FOREACH(Filter_ptr &f, NewTracks)
            f->Predict();
}


void MTTracker::RearrangeTracks()
{

    Tracks.remove_if([](Filter_ptr &f){return f->update_cnt<0;});
    NewTracks.remove_if([](Filter_ptr &f){return f->update_cnt< -15;});


    //FilterList::iterator merge_start;
    auto merge_start = partition(NewTracks.begin(),NewTracks.end(),
                                 [](Filter_ptr &f){return f->update_cnt<15;});

    Tracks.splice(Tracks.end(),NewTracks,merge_start,NewTracks.end());


    BOOST_FOREACH(Filter_ptr &f, Tracks)
            f->Fuse();
    BOOST_FOREACH(Filter_ptr &f, NewTracks)
            f->Fuse();
}


void MTTracker::TeammateIdentify(Teammates &teammates)
{

    BOOST_FOREACH(Teammate &tm, teammates)
    {
        int robot_id = tm.first;
        DPoint &pos = tm.second;


        map<double,int> m;
        int idx=0;
        BOOST_FOREACH(Filter_ptr &p, Tracks)
        {
            Filter &f = *p;

            if ( f.SeenByRobots[robot_id]==false )
            {
                double distance=(pos - DPoint(f(0),f(1))).length();
                m[distance]=idx;
            }
            idx++;
        }


        BOOST_FOREACH(auto & pair, m)
        {
            int point_idx =pair.second;
            double distance=pair.first;

            if (distance<150)
            {
                Filter &f=*getFilter(point_idx, Tracks);
                f.TeammateID = robot_id;
            }
            else
                break;
        }

    }// BOOST_FOREACH(teammate_t &tm, teammates)
}


void MTTracker::GetAllObjects(vector<DPoint> &Objects)
{
    Objects.resize(Tracks.size());

    int i=0;
    BOOST_FOREACH(Filter_ptr &p, Tracks)
    {
        Filter &f=*p;
        Objects[i++]=DPoint(f(0),f(1));
    }
}

void MTTracker::GetObjects(vector<DPoint> &Objects, unsigned int N)
{
    uint size;
    if (N < Tracks.size())
    {
        size  = N;

        static trigger<10> Ten_times;
        if( Ten_times() )
            Tracks.sort([](Filter_ptr &p1,Filter_ptr &p2){return p1->variance() < p2->variance();});
    }
    else
        size = Tracks.size();

    Objects.resize(size);
    uint i=0;
    BOOST_FOREACH(Filter_ptr &p, Tracks)
    {
        if (i>=size)  break;

        Filter &f = *p;
        if ( f.TeammateID ==-1 )
            Objects[i++]=DPoint( f(0), f(1) );
    }
}

DKF::DKF(Vector &X0,Matrix &A, Matrix &H, Matrix &Q,Matrix &R)
    :A(A),H(H),Q(Q),R(R),M(X0),P(10*Q),
      HRH_sum(Matrix::Zero(4,4)),HRZ_sum(Vector::Zero(4))
{
}

void DKF::Predict()
{
    M = A * M;
    P = A * P * A.transpose() + Q;

    HRH_sum.setZero();
    HRZ_sum.setZero();
}

void DKF::Update(obs_info &m)
{
    Eigen::Map<Matrix> HRH(m.HRH,4,4);
    Eigen::Map<Vector> HRZ(m.HRZ,4);
    HRH_sum += HRH;
    HRZ_sum += HRZ;
}

void DKF::Fuse()
{
    Matrix P_inv = P.partialPivLu().inverse();

    P  = (P_inv + HRH_sum).partialPivLu().inverse();
    M = P*(P_inv*M +  HRZ_sum);
}

