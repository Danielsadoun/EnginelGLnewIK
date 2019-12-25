#include "Movable.h"

Movable::Movable()
{
	// T = Eigen::Transform<float, 3, Eigen::Affine>::Identity();
	Tout = Eigen::Transform<float, 3, Eigen::Affine>::Identity();
	Tin = Eigen::Transform<float, 3, Eigen::Affine>::Identity();
}

Movable::Movable(const Movable& mov) {
	Tout = mov.Tout;
	Tin = mov.Tin;
}

Eigen::Matrix4f Movable::MakeTranScale() {
	return (Tout.matrix() * Tin.matrix());
}


Eigen::Matrix4f Movable::MakeTrans()
{
	Eigen::Matrix4f mat = Eigen::Matrix4f::Identity();
	mat.col(3) << Tin.translation(), 1;
	return (Tout.matrix() * mat);

	//return T.matrix();
}

/*Eigen::Matrix4d Movable::MakeTranScaled() {
	return (Tout.matrix() * Tin.matrix()).cast<double>();
}*/


/*Eigen::Matrix4d Movable::MakeTransd() {
	Eigen::Matrix4d mat = Eigen::Matrix4d::Identity();
	mat.col(3) << Tin.translation(), 1;
	return (Tout.matrix() * mat).cast<double>();
}*/




void Movable::MyTranslate(Eigen::Vector3f amt)
{
	Tout.pretranslate(amt);
	//T.translate(amt);
}
//angle in radians
void Movable::MyRotate(Eigen::Vector3f rotAxis, float angle)
{
	Tout.rotate(Eigen::AngleAxisf(angle, rotAxis.normalized()));
}

void Movable::MyScale(Eigen::Vector3f amt)
{
	Tin.scale(amt);
}

void Movable::TranslateInSystem(Eigen::Matrix4f Mat, Eigen::Vector3f amt, bool preTranslate) {
	MyTranslate(Mat.block<3, 3>(0, 0).transpose() * amt);
}

void Movable::RotateInSystem(Eigen::Matrix4f Mat, Eigen::Vector3f rotAxis, double angle) {
	MyRotate(Mat.block<3, 3>(0, 0).transpose() * rotAxis, angle);
}

void Movable::SetCenterOfRotation(Eigen::Vector3f amt) {
	Tin.translate(-amt);
	Tout.translate(amt);
}

Eigen::Vector3f Movable::getCenterOfRotation() {
	return -Tin.translation();
}