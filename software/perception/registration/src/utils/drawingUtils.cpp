#include <iostream>
#include "drawingUtils.hpp"

using namespace Eigen;

void draw3dLine(bot_lcmgl_t *lcmgl, Vector3d start, Vector3d end)
{
	bot_lcmgl_begin(lcmgl, LCMGL_LINES);
	bot_lcmgl_vertex3f(lcmgl, start[0], start[1], start[2]);
	bot_lcmgl_vertex3f(lcmgl, end[0], end[1], end[2]);
	bot_lcmgl_end(lcmgl);
}

void draw3dLine(bot_lcmgl_t *lcmgl, double start_x, double start_y, double start_z, double end_x, double end_y, double end_z)
{
	bot_lcmgl_begin(lcmgl, LCMGL_LINES);
	bot_lcmgl_vertex3f(lcmgl, start_x, start_y, start_z);
	bot_lcmgl_vertex3f(lcmgl, end_x, end_y, end_z);
	bot_lcmgl_end(lcmgl);
}

void HSVtoRGB( float &r, float &g, float &b, float h, float s, float v )
{
	int i;
	float f, p, q, t;

	if( s == 0 ) {
		// achromatic (grey)
		r = g = b = v;
		return;
	}

	h /= 60;			// sector 0 to 5
	i = floor( h );
	f = h - i;			// factorial part of h
	p = v * ( 1 - s );
	q = v * ( 1 - s * f );
	t = v * ( 1 - s * ( 1 - f ) );

	switch( i ) {
		case 0:
			r = v;
			g = t;
			b = p;
			break;
		case 1:
			r = q;
			g = v;
			b = p;
			break;
		case 2:
			r = p;
			g = v;
			b = t;
			break;
		case 3:
			r = p;
			g = q;
			b = v;
			break;
		case 4:
			r = t;
			g = p;
			b = v;
			break;
		default:		// case 5:
			r = v;
			g = p;
			b = q;
			break;
	}

}

void drawPointCloud(bot_lcmgl_t *lcmgl, DP &dp_cloud)
{
	VectorXf x_values = (dp_cloud.getFeatureCopyByName("x"));
    VectorXf y_values = (dp_cloud.getFeatureCopyByName("y"));
    VectorXf z_values = (dp_cloud.getFeatureCopyByName("z"));
    vector<Eigen::Vector3f> point_cloud;
    int cloud_size = x_values.size();
    float n_supp_points = 75000.0; // max number of floats supported by lcm channel
    int step = round(cloud_size/n_supp_points);
    point_cloud.resize(n_supp_points);
    int j = 0;
    for (int point = 0; point < cloud_size; point=point+step)
    {
      if (j < point_cloud.size())
        point_cloud.at(j) << x_values(point), y_values(point), z_values(point); 
      j++;
    }
    
    drawPointCloud(lcmgl, point_cloud);
}

void drawPointCloud(bot_lcmgl_t *lcmgl, std::vector<Vector3f> point_cloud)
{
	bot_lcmgl_point_size(lcmgl, 2);
	srand (time(NULL));
	float r = (rand() % 101)/100.0;
	float g = (rand() % 101)/100.0;
	float b = (rand() % 101)/100.0;
	cout << "r: " << r << "g: " << g << "b: " << b << endl;
	bot_lcmgl_color3f(lcmgl, r, g, b);
	bot_lcmgl_begin(lcmgl, LCMGL_POINTS);
	for (Vector3f point : point_cloud)
	{
		bot_lcmgl_vertex3f(lcmgl, point(0), point(1), point(2));
	}
	bot_lcmgl_end(lcmgl);
	bot_lcmgl_switch_buffer(lcmgl);
}
/*
void drawFrames(bot_lcmgl_t *lcmgl, std::vector<Vector3f> point_cloud)
{
	bot_lcmgl_point_size(lcmgl, 2);
	srand (time(NULL));
	float r = (rand() % 101)/100.0;
	float g = (rand() % 101)/100.0;
	float b = (rand() % 101)/100.0;
	cout << "r: " << r << "g: " << g << "b: " << b << endl;
	bot_lcmgl_color3f(lcmgl, r, g, b);
	bot_lcmgl_begin(lcmgl, LCMGL_POINTS);
	for (Vector3f point : point_cloud)
	{
		bot_lcmgl_vertex3f(lcmgl, point(0), point(1), point(2));
	}
	bot_lcmgl_end(lcmgl);
	bot_lcmgl_switch_buffer(lcmgl);

  //x-axis
  lcmglBegin(LCMGL_LINES);
  lcmglColor3f(1, 0, 0);
  lcmglVertex3f(1, 0, 0);
  lcmglVertex3f(0, 0, 0);
  lcmglEnd();

  //y-axis
  lcmglBegin(LCMGL_LINES);
  lcmglColor3f(0, 1, 0);
  lcmglVertex3f(0, 1, 0);
  lcmglVertex3f(0, 0, 0);
  lcmglEnd();

  //z-axis
  lcmglBegin(LCMGL_LINES);
  lcmglColor3f(0, 0, 1);
  lcmglVertex3f(0, 0, 1);
  lcmglVertex3f(0, 0, 0);
  lcmglEnd();
}*/
