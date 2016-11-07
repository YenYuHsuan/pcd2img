#define PCL_NO_PRECOMPILE
#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/io/png_io.h>
#include <math.h>
#include <string>
#include <sstream>
using namespace std;

struct PointXYZRGBIndex
{
	PCL_ADD_POINT4D;					// preferred way of adding a XYZ+padding
	PCL_ADD_RGB;						// preferred wat of adding a RGB
	int index;							// add new variable
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW		// make sure our new allocators are aligned
} EIGEN_ALIGN16;						// enforce SSE padding for correct memory alignment

POINT_CLOUD_REGISTER_POINT_STRUCT(PointXYZRGBIndex,  // here we assume a XYZRGB + "index" (as fields)
	(float, x, x)
	(float, y, y)
	(float, z, z)
	(float, rgb, rgb)
	(uint32_t, index, index)
	)

template <typename T>
void saveDepthToPNG(const std::string &file_name, const pcl::PointCloud<T> &cloud)
{
	pcl::PointCloud<T> depth = cloud;

	double max = 0, min = 0;

	for (int i = 0; i < (int)depth.points.size(); i++) {
		if (depth.points[i].z > max) max = depth.points[i].z;
		if (depth.points[i].z < min) min = depth.points[i].z;
	}

	float scale = (max - min) / 256;

	for (int i = 0; i < (int)depth.points.size(); i++) {
		unsigned int t = depth.points[i].z / scale;

		// make sure pixel value between 0 ~ 255
		t = (t > 255) ? 255 : (t < 0) ? 0 : t;

		depth.points[i].rgba = ((t << 16) | (t << 8) | t);
	}
	pcl::io::savePNGFile(file_name, depth);
}

string PCDfilenameStr(int data_num) {
	stringstream PCDfilenamess;
	if		(data_num<10)	PCDfilenamess << "pcd000"	<< data_num ;
	else if (data_num<100)	PCDfilenamess << "pcd00"	<< data_num ;
	else if (data_num<1000)	PCDfilenamess << "pcd0"		<< data_num ;
	else {					PCDfilenamess << "pcd"		<< data_num ; }
	return PCDfilenamess.str();
}

int
main (int argc, char** argv)
{
	for (int filenum = 950; filenum < 1035; filenum++) {

		//load the BeforeTrans .pcd file 
		std::string BeforeTrans_cloudfilename = "../BeforeTrans/" + PCDfilenameStr(filenum) + ".txt";
		pcl::PointCloud<PointXYZRGBIndex>::Ptr cloud(new pcl::PointCloud<PointXYZRGBIndex>);
		//* load the original data file and check if error
		if (pcl::io::loadPCDFile<PointXYZRGBIndex>(BeforeTrans_cloudfilename, *cloud) == -1)
		{
			PCL_ERROR("Couldn't read the file  \n");
			if (filenum<1350)continue;
			else
			{
				return (-1);
				system("pause");
			}
		}

		std::cout << "Loaded "
			<< cloud->width * cloud->height
			<< " data points from " << PCDfilenameStr(filenum) << ".txt.   ";

		//load an empty cloud filled with 0
		std::string basefile = "base_pcd";
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr Newcloud(new pcl::PointCloud<pcl::PointXYZRGB>);
		pcl::io::loadPCDFile<pcl::PointXYZRGB>(basefile + ".pcd", *Newcloud);
		//* load the basefile and check if error
		if (pcl::io::loadPCDFile<pcl::PointXYZRGB>(basefile + ".pcd", *Newcloud) == -1) 
		{
			PCL_ERROR("Couldn't read file base_pcd.pcd \n");
			return (-1);
			system("pause");
		}

		
		//mapping
		for (size_t i = 0; i < cloud->points.size(); ++i) {

			//index mapping function
			int row = floor((float)cloud->points[i].index / 640) + 1;
			int column = cloud->points[i].index % 640 + 1;

			/*std::cout << "    " << cloud->points[i].x  //show data
				<< " " << cloud->points[i].y
				<< " " << cloud->points[i].z
				<< " " << cloud->points[i].rgb
				<< " " << cloud->points[i].index << std::endl
				<< " row:" << row << " col:" << column << std::endl;
			*/
			Newcloud->points[(row - 1) * 640 + (column - 1)].x = cloud->points[i].x;
			Newcloud->points[(row - 1) * 640 + (column - 1)].y = cloud->points[i].y;
			Newcloud->points[(row - 1) * 640 + (column - 1)].z = cloud->points[i].z;
			Newcloud->points[(row - 1) * 640 + (column - 1)].rgb = cloud->points[i].rgb;
		}
		cout << PCDfilenameStr(filenum) <<".txt is transformed."<< std::endl;
		//save RGB & Depth as .png file
		std::string AfterTrans_pngfilename = "../AfterTrans/RGB/" + PCDfilenameStr(filenum) + ".png";
		pcl::io::savePNGFile(AfterTrans_pngfilename, *Newcloud);

		std::string AfterTrans_depth_pngfilename = "../AfterTrans/Depth/" + PCDfilenameStr(filenum) + "_depth.png";
		saveDepthToPNG(AfterTrans_depth_pngfilename , *Newcloud);
	}
	cout << "\n\nALL DONE!!!! YAY \n\n";
  system("pause");
  return (0);
}

