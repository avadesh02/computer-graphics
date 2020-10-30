////////////////////////////////////////////////////////////////////////////////
#include <algorithm>
#include <complex>
#include <fstream>
#include <iostream>
#include <numeric>
#include <vector>
////////////////////////////////////////////////////////////////////////////////

typedef std::complex<double> Point;
typedef std::vector<Point> Polygon;

double inline det(const Point &u, const Point &v) {
	// TODO
	return 0;
}

struct Compare {
	Point p0; // Leftmost point of the poly
	bool operator ()(const Point &p1, const Point &p2) {
		return ((p1.real() - p0.real())*(p2.imag() - p0.imag()) - (p1.imag() - p0.imag())*(p2.real() - p0.real()) > 0);
	}
};

bool inline salientAngle(Point &a, Point &b, Point &c) {
	// if the z component of the cross product is less than zero than a salient angle is formed
	return ((b.real() - a.real())*(c.imag() - a.imag()) - (b.imag() - a.imag())*(c.real() - a.real()) < 0);
}

////////////////////////////////////////////////////////////////////////////////

Polygon convex_hull(std::vector<Point> &points) {
	Compare order;
	int ct = 0;
	Point swap;
	// Finding the left most point
	order.p0 = points[0];
	for (int i = 0; i < points.size() ; i ++){
		if (order.p0.imag() > points[i].imag()){
			order.p0 = points[i];
			ct = i;
		}
		if (order.p0.imag() == points[i].imag()){
			if (order.p0.real() > points[i].real()){
				order.p0 = points[i];
				ct = i;
			}
		}
	}

	// moving the bottom left most point to the start of the array
	swap = points[0];
	points[0] = points[ct];
	points[ct] = swap;
	// sorting points based on polar angle wrt p0 and x axis
	std::sort(points.begin()+1, points.end(), order);
	// computing the convex hull
	Polygon hull;
	hull.push_back(order.p0);	
	for(int i = 0; i < points.size(); i ++){
		while (salientAngle(hull[hull.size()-2], hull[hull.size()-1], points[i]) && hull.size() > 1){
			hull.pop_back();
		}
		hull.push_back(points[i]);
	}
	return hull;
}

////////////////////////////////////////////////////////////////////////////////

std::vector<Point> load_xyz(const std::string &filename) {
	std::vector<Point> points;
	std::ifstream in(filename);
	int no_points;
	double value, real, imag;
	// reading the file
	// every first element of the line is assigned to real
	// every second element of the line is assigned to imag
	// every third element is ignored
	if (in.is_open()){
		in >> no_points;
		for(int i; i < 3*no_points; i ++){
			in >> value;
			if (i % 3 == 0){
				real = value;
			} 
			if (i % 3 == 1){
				imag = value;
			}
			if (i % 3 == 2){
				points.push_back(Point(real, imag));
			}			
		}
		in.close();
	}
	else{
		std::cout << "file not found ..." << '\n';
	}
	
	return points;
}

void save_obj(const std::string &filename, Polygon &poly) {
	std::ofstream out(filename);
	if (!out.is_open()) {
		throw std::runtime_error("failed to open file " + filename);
	}
	out << std::fixed;
	for (const auto &v : poly) {
		out << "v " << v.real() << ' ' << v.imag() << " 0\n";
	}
	for (size_t i = 0; i < poly.size(); ++i) {
		out << "l " << i+1 << ' ' << 1+(i+1)%poly.size() << "\n";
	}
	out << std::endl;
}

////////////////////////////////////////////////////////////////////////////////

int main(int argc, char * argv[]) {
	if (argc <= 2) {
		std::cerr << "Usage: " << argv[0] << " points.xyz output.obj" << std::endl;
	}
	std::vector<Point> points = load_xyz(argv[1]);
	Polygon hull = convex_hull(points);
	save_obj(argv[2], hull);
	return 0;
}
