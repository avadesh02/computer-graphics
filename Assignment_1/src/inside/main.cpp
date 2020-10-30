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

// Return true iff [a,b] intersects [c,d], and store the intersection in ans
bool intersect_segment(const Point &a, const Point &b, const Point &c, const Point &d, Point &ans) {
	double a1,b1,c1, a2,b2,c2;
	// computing the coeffecient of the lines ax + by = c
	a1 = b.imag() - a.imag();
	b1 = a.real() - b.real();
	c1 = a.real()*b.imag() - b.real()*a.imag();
	
	a2 = d.imag() - c.imag();
	b2 = c.real() - d.real();
	c2 = c.real()*d.imag() - d.real()*c.imag();
	// checking if the lines are parallel or colinear
	if (a2*b1 - a1*b2 == 0){
		return false;
	}
	else{
		// computing point of intersection of the two lines
		ans.imag((a2*c1 - a1*c2)/(a2*b1 - a1*b2));
		ans.real((b2*c1 - b1*c2)/(a1*b2 - a2*b1));
		// checking if the point of intersection lies within the line segment
		if ((ans.real() - a.real())/(ans.real() - b.real()) < 0 && (ans.real() - c.real())/(ans.real() - d.real()) < 0){
			return true;
		}
		else{
			return false;
		}
	}
}

////////////////////////////////////////////////////////////////////////////////

bool is_inside(const Polygon &poly, const Point &query) {
	// 1. Compute bounding box and set coordinate of a point outside the polygon
	Point min, max;
	min = poly[0];
	max = poly[0];
	// computing the bounding box by identifying the smallest and largest x and y co-ordinate values
	// of the points in the set (The values need not belong to the same point)
	// The bounding box can then be formed with min and max.
	for (int i = 0; i < poly.size(); i ++){
		if (poly[i].real() < min.real()){
			min.real(poly[i].real());
		}
		if (poly[i].real() > max.real()){
			max.real(poly[i].real());
		}
		if (poly[i].imag() < min.imag()){
			min.imag(poly[i].imag());
		}
		if (poly[i].imag() > max.imag()){
			max.imag(poly[i].imag());
		}
	}
	// The outside point  here is computed as a value that is 10 times the value of the max point
	// This ensures that the point is outside the polygon
	Point outside(10*max.real(), 10*max.imag()), ans;
	int no_of_intersections = 0;
	// 2. Cast a ray from the query point to the 'outside' point, count number of intersections
	// for each edge in the polygon, a line segment from the point of interest to the outside point is drawn, 
	// the no of interections is then computed
	// Note : The code assumes that the edges are sequential ordered in the obj file. 
	// I was not sure how to interpret the f 1 2 3.... line as it was not explained in the github page. 
	// However, the code can be easily modified if the vertex information is given since the line segmentes can be computed
	// with the edge information. 
	for (int i = 0; i < poly.size(); i ++){
		if (i < poly.size() - 1){
			if(intersect_segment(query, outside, poly[i], poly[i+1], ans)){
				no_of_intersections ++;
			}
		}
		else{
			if(intersect_segment(query, outside, poly[i], poly[0], ans)){
				no_of_intersections ++;
			}
		}
	}
	// checking if number of intersections is even or odd.
	if(no_of_intersections % 2 == 1){
		return true;
	}
	else{
		return false;
	}
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

Polygon load_obj(const std::string &filename) {
	std::ifstream in(filename);
	Polygon poly;
	double real, imag;
	char value[10];
	int flag = 1, ct = 0;
	// Every first element of a line is either v or f
	// if v, then the first two elements are stored in the poly array
	// if the first element is f, then the loop is terminated.
	if (in.is_open()){
		while(flag){
			if (ct % 4 == 0){
				in >> value ;
				if (value[0] == 'f'){
					flag = 0;
				}
			}
			if (ct % 4 == 1){
				in >> real ;
			}
			if (ct % 4 == 2){
				in >> imag;
			}
			if (ct % 4 == 3){
				in >> value;
				poly.push_back(Point(real, imag));
			}
			ct ++;
		}
	}
	else{
		std::cout << "file not found..." << '\n';
	}
	return poly;
}

void save_xyz(const std::string &filename, const std::vector<Point> &points) {
	std::ofstream out(filename);
	if (!out.is_open()){
		throw std::runtime_error("failed to open file " + filename);
	}
	out << std::fixed;
	out << points.size() << '\n';
	for (int i = 0; i < points.size(); i ++){
		out << points[i].real() << ' ' << points[i].imag() << ' ' << 0 << '\n';
	}
	out << std::endl;
}

////////////////////////////////////////////////////////////////////////////////

int main(int argc, char * argv[]) {
	if (argc <= 3) {
		std::cerr << "Usage: " << argv[0] << " points.xyz poly.obj result.xyz" << std::endl;
	}
	std::vector<Point> points = load_xyz(argv[1]);
	Polygon poly = load_obj(argv[2]);

	std::vector<Point> result;
	for (size_t i = 0; i < points.size(); ++i) {
		if (is_inside(poly, points[i])) {
			result.push_back(points[i]);
		}
	}
	save_xyz(argv[3], result);
	return 0;
}
