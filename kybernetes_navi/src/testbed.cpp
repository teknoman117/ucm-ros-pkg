#include <iostream>
#include <cstdlib>
#include <list>
#include <math.h>
using namespace std;

#define cutoff 40

//havarsine distance  a = sin^2(delt_lat/2) + cos(lat1)cos(lat2)sin^2(delt_lon/2)
//			c = 2 * atan2(sqrt(a), sqrt(1-a)
//			d = R*c
// r -> earths radius : 6371km

//bearing   theta = atan2(sin(delt_lon)*cos(lat2),
//			cos(lat1)sin(lat2) - sin(lat1)cos(lat2)cos(delt_lon)

int main(int argc, char** argv){
	cout << argc << endl;
	list<float> x;
	list<float> y;
	for(int i = 1 ; i < argc; i+=2){
		cout << "--" << atof(argv[i]) << " " << atof(argv[i + 1]) << endl;
		x.push_back(atof(argv[i]));
		y.push_back(atof(argv[i + 1]));
		cout << i << endl;
	}
	cout << endl << endl;
	while(!x.empty()){
		cout << x.front() << " " << y.front() << endl;
		x.pop_front();
		y.pop_front();
	}

	float sonarWeight[] = {1.07, 0.5, 0.0, -0.5, -1.07};
	float sonarData[] = {50, 30, 30, 20, 10};
	
	float closest = sonarData[0];
	float farthest = sonarData[0];
	float sum = 0;

	for(int i = 1; i < 5; i++){
		if(closest > sonarData[i]){
			closest = sonarData[i];
		}
		if(farthest < sonarData[i]){
			farthest = sonarData[i];
		}
	}

	cout << "farthest : " << farthest << " closest: " << closest << endl;
	
	for(int i = 0; i < 5; i++){
		cout << i << "   " << endl;
		if(sonarData[i] < cutoff){
			float d = sonarWeight[i] / sonarData[i];
			sum += d;
			cout << "add from " << i << ": " << d << endl;
		}
	}
	
	cout << "sum: " << sum << endl;
	
	cout << "--havarsine--\n";

	float lt1 = 75, lt2 = 75.0001;
	float ln1 = 20, ln2 = 20.0001;

	lt1 = lt1 * 3.14159 / 180;
	lt2 = lt2 * 3.14159 / 180;
	ln1 = ln1 * 3.14159 / 180;
	ln2 = ln2 * 3.14159 / 180;

	float a = sin((lt1 - lt2)/2)*sin((lt1 - lt2)/2) + cos(lt1) * cos(lt2) * sin((ln1 - ln2)/2) * sin((ln1 - ln2)/2);
	float c = 2 * atan2(sqrt(a), sqrt(1-a));
	float d = 6371000 * c;
	cout << d << endl;

	cout << "--theta--\n";
	lt1 = 75; lt2 = 20.001;
	ln1 = 20; ln2 = 20.001;

	lt1 = lt1 * 3.14159 / 180;
	lt2 = lt2 * 3.14159 / 180;
	ln1 = ln1 * 3.14159 / 180;
	ln2 = ln2 * 3.14159 / 180;	

	const float theta = atan2(sin(ln2 - ln1)*cos(lt2),
			cos(lt1)*sin(lt2) - sin(lt1)*cos(lt2)*cos(ln2 - ln1));
	cout << theta << endl;
}
