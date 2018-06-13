#include "Eigen\dense"

using namespace Eigen;

typedef struct IKJoint //Inverse kinematics
{
	double InvJoint[6];
	BOOL bIKFlag;
}Ikjoint;

typedef struct PoseOfBody 
{
	double rx,ry,rz,x,y,z;
}Pose;

typedef struct OrienOfBody 
{
	double rx,ry,rz;
}Orien;

typedef struct PositionOfBody 
{
	double x,y,z;
}Pos;

typedef struct SphericalCoordinate
{
	double R,thetad,phid;
}SpheCoordi;

typedef class KINEMATICS{
public:
	
	void ik(double px,double py,double pz,double rz, double ry,double rx,struct IKJoint* InvK);
	
    void ik_adj(double des_px,double des_py,double des_pz,double des_rz, double des_ry,double des_rx,double adj_rz,double adj_ry,double adj_rx,struct IKJoint* InvK);
	void ik(struct PoseOfBody* Pose, struct IKJoint* InvK);
	void ik(double px,double py,double pz,struct IKJoint* InvK);
	void T01(double q1,Matrix4f *m01);
	void T12(double q2,Matrix4f *m12);
	void T23(double q3,Matrix4f *m23);
	void T34(double q4,Matrix4f *m34);
	void T45(double q5,Matrix4f *m45);
	void T56(double q6,Matrix4f *m56);
	void T6e(Matrix4f *m6e);
	void T06(double q1,double q2,double q3,double q4,double q5,double q6,Matrix4f *m06);
	void T0e(double q1,double q2,double q3,double q4,double q5,double q6,Matrix4f *m0e);
	void T03(double q1,double q2,double q3,Matrix4f *m03);
	void Reverse(Matrix4f *T,Matrix4f *revT);
		
	void Translation(double x,double y,double z,Matrix4f *Trans);
	void Translation(double x,double y,double z,Vector3f *Trans);
	void Rotation(double rz,double ry,double rx,Matrix4f *Rot);
	void Rotation(double rz,double ry,double rx,Matrix3f *Rot);
	void GetTransform(double q0,double qx,double qy,double qz,double tx,double ty,double tz,Matrix4f *T);
	void GetTransform(struct PoseOfBody* Pose,Matrix4f *T);
	void GetEuler(Matrix4f *T,struct PoseOfBody* pose);
	void GetEuler(Matrix3f *T,struct OrienOfBody* pose);
	void CarteToSphe(double x,double y,double z,struct SphericalCoordinate* Sphe);
	void SpheToCarte(double r,double phid,double thetad,struct PositionOfBody* pos);
	void SpheToCarte(double r,double phid,double thetad,double rx,double ry,double rz,struct PositionOfBody* pos);
	double Ori1,Ori2,Ori3,Posi1,Posi2,Posi3;	// input pose	
	double th1,th2,d3,th4,th5,th6; // output joint value

	
}Kinematics;
