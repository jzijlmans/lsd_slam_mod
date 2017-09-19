/**
* This file is part of LSD-SLAM.
*
* Copyright 2013 Jakob Engel <engelj at in dot tum dot de> (Technical University of Munich)
* For more information see <http://vision.in.tum.de/lsdslam> 
*
* LSD-SLAM is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* LSD-SLAM is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with LSD-SLAM. If not, see <http://www.gnu.org/licenses/>.
*/

#include "LiveSLAMWrapper.h"

#include <boost/thread.hpp>
#include "util/settings.h"
#include "util/globalFuncs.h"
#include "SlamSystem.h"

#include <sstream>
#include <fstream>
#include <dirent.h>
#include <algorithm>

#include "IOWrapper/ROS/ROSOutput3DWrapper.h"
#include "IOWrapper/ROS/rosReconfigure.h"

#include "util/Undistorter.h"
#include <ros/package.h>

#include "opencv2/opencv.hpp"

std::string &ltrim(std::string &s) {
        s.erase(s.begin(), std::find_if(s.begin(), s.end(), std::not1(std::ptr_fun<int, int>(std::isspace))));
        return s;
}
std::string &rtrim(std::string &s) {
        s.erase(std::find_if(s.rbegin(), s.rend(), std::not1(std::ptr_fun<int, int>(std::isspace))).base(), s.end());
        return s;
}
std::string &trim(std::string &s) {
        return ltrim(rtrim(s));
}
int getdir (std::string dir, std::vector<std::string> &files)
{
    DIR *dp;
    struct dirent *dirp;
    if((dp  = opendir(dir.c_str())) == NULL)
    {
        return -1;
    }

    while ((dirp = readdir(dp)) != NULL) {
    	std::string name = std::string(dirp->d_name);

    	if(name != "." && name != "..")
    		files.push_back(name);
    }
    closedir(dp);


    std::sort(files.begin(), files.end());

    if(dir.at( dir.length() - 1 ) != '/') dir = dir+"/";
	for(unsigned int i=0;i<files.size();i++)
	{
		if(files[i].at(0) != '/')
			files[i] = dir + files[i];
	}

    return files.size();
}

int getFile (std::string source, std::vector<std::string> &files)
{
	std::ifstream f(source.c_str());

	if(f.good() && f.is_open())
	{
		while(!f.eof())
		{
			std::string l;
			std::getline(f,l);

			l = trim(l);

			if(l == "" || l[0] == '#')
				continue;

			files.push_back(l);
		}

		f.close();

		size_t sp = source.find_last_of('/');
		std::string prefix;
		if(sp == std::string::npos)
			prefix = "";
		else
			prefix = source.substr(0,sp);

		for(unsigned int i=0;i<files.size();i++)
		{
			if(files[i].at(0) != '/')
				files[i] = prefix + "/" + files[i];
		}

		return (int)files.size();
	}
	else
	{
		f.close();
		return -1;
	}

}


using namespace lsd_slam;
int main( int argc, char** argv )
{
	ros::init(argc, argv, "LSD_SLAM");

	dynamic_reconfigure::Server<lsd_slam_core::LSDParamsConfig> srv(ros::NodeHandle("~"));
	srv.setCallback(dynConfCb);

	dynamic_reconfigure::Server<lsd_slam_core::LSDDebugParamsConfig> srvDebug(ros::NodeHandle("~Debug"));
	srvDebug.setCallback(dynConfCbDebug);

	packagePath = ros::package::getPath("lsd_slam_core")+"/";



	// get camera calibration in form of an undistorter object.
	// if no undistortion is required, the undistorter will just pass images through.
	std::string calibFile;
	Undistorter* undistorter = 0;
	if(ros::param::get("~calib", calibFile))
	{
		 undistorter = Undistorter::getUndistorterForFile(calibFile.c_str());
		 ros::param::del("~calib");
	}

	if(undistorter == 0)
	{
		printf("need camera calibration file! (set using _calib:=FILE)\n");
		exit(0);
	}




	int w = undistorter->getOutputWidth();
	int h = undistorter->getOutputHeight();

	int w_inp = undistorter->getInputWidth();
	int h_inp = undistorter->getInputHeight();

	float fx = undistorter->getK().at<double>(0, 0);
	float fy = undistorter->getK().at<double>(1, 1);
	float cx = undistorter->getK().at<double>(2, 0);
	float cy = undistorter->getK().at<double>(2, 1);
	Sophus::Matrix3f K;
	K << fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0;

	// added: get the right calibration file and right image files
	std::string rcalibFile;
	Undistorter* rundistorter = 0;
	bool stereo = false;
	std::string rsource;
	std::vector<std::string> rfiles;
	int rw ;
	int rh ;
	int rw_inp ;
	int rh_inp ;
	float rfx ;
	float rfy ;
	float rcx ;
	float rcy ;
	Sophus::Matrix3f rK;

	if(ros::param::get("~rcalib", rcalibFile) && ros::param::get("~rfiles", rsource))
	{
		stereo = true;
		rundistorter = Undistorter::getUndistorterForFile(rcalibFile.c_str());
		ros::param::del("~rcalib");

		ros::param::del("~files");


		if(getdir(rsource, rfiles) >= 0)
		{
			printf("found %d right image files in folder %s!\n", (int)rfiles.size(), rsource.c_str());
		}
		else if(getFile(rsource, rfiles) >= 0)
		{
			printf("found %d right image files in file %s!\n", (int)rfiles.size(), rsource.c_str());
		}
		else
		{
			printf("could not load right file list! wrong path / file?\n");
		}

		rw = rundistorter->getOutputWidth();
		rh = rundistorter->getOutputHeight();
		rw_inp = rundistorter->getInputWidth();
		rh_inp = rundistorter->getInputHeight();
		rfx = rundistorter->getK().at<double>(0, 0);
		rfy = rundistorter->getK().at<double>(1, 1);
		rcx = rundistorter->getK().at<double>(2, 0);
		rcy = rundistorter->getK().at<double>(2, 1);

		rK << rfx, 0.0, rcx, 0.0, rfy, rcy, 0.0, 0.0, 1.0;

	}
	else{
		printf("no right camera calibration file or right image files (set using _rcalib:=FILE and _rfiles:=DIR)\n so no stereo preformed");

	}
	// end added


	// make output wrapper. just set to zero if no output is required.
	Output3DWrapper* outputWrapper = new ROSOutput3DWrapper(w,h);


	// make slam system
	if (!stereo){
		SlamSystem* system = new SlamSystem(w, h, K, doSlam);
	}
	else{
		SlamSystem* system = new SlamSystem(w, h, K, rK, doSlam);
	}
	system->setVisualization(outputWrapper);



	// open image files: first try to open as file.
	std::string source;
	std::vector<std::string> files;
	if(!ros::param::get("~files", source))
	{
		printf("need source files! (set using _files:=FOLDER)\n");
		exit(0);
	}
	ros::param::del("~files");


	if(getdir(source, files) >= 0)
	{
		printf("found %d image files in folder %s!\n", (int)files.size(), source.c_str());
	}
	else if(getFile(source, files) >= 0)
	{
		printf("found %d image files in file %s!\n", (int)files.size(), source.c_str());
	}
	else
	{
		printf("could not load file list! wrong path / file?\n");
	}



	// get HZ
	double hz = 0;
	if(!ros::param::get("~hz", hz))
		hz = 0;
	ros::param::del("~hz");



	cv::Mat image = cv::Mat(h,w,CV_8U);
	cv::Mat rimage = cv::Mat(rh,rw,CV_8U);
	int runningIDX=0;
	float fakeTimeStamp = 0;

	ros::Rate r(hz);
//added
	if (stereo){
		for(unsigned int i=0;i<files.size();i++)
		{
			cv::Mat imageDist = cv::imread(files[i], CV_LOAD_IMAGE_GRAYSCALE);
			cv::Mat rimageDist = cv::imread(rfiles[i], CV_LOAD_IMAGE_GRAYSCALE);

			if(imageDist.rows != h_inp || imageDist.cols != w_inp)
			{
				if(imageDist.rows * imageDist.cols == 0)
					printf("failed to load image %s! skipping.\n", files[i].c_str());
				else
					printf("image %s has wrong dimensions - expecting %d x %d, found %d x %d. Skipping.\n",
							files[i].c_str(),
							w,h,imageDist.cols, imageDist.rows);
				continue;
			}

			if(rimageDist.rows != h_inp || rimageDist.cols != w_inp)
			{
				if(rimageDist.rows * rimageDist.cols == 0)
					printf("failed to load right image %s! skipping.\n", rfiles[i].c_str());
				else
					printf("right image %s has wrong dimensions - expecting %d x %d, found %d x %d. Skipping.\n",
							rfiles[i].c_str(),
							rw,rh,rimageDist.cols, rimageDist.rows);
				continue;
			}

			assert(imageDist.type() == CV_8U);
			assert(rimageDist.type() == CV_8U);

			undistorter->undistort(imageDist, image);
			assert(image.type() == CV_8U);

			rundistorter->undistort(rimageDist, rimage);
			assert(rimage.type() == CV_8U);


			if(runningIDX == 0)
					system->randomInit(image.data, fakeTimeStamp, runningIDX, rimage.data);
			else
					system->trackFrame(image.data, runningIDX ,hz == 0,fakeTimeStamp, rimage.data);


			runningIDX++;
			fakeTimeStamp+=0.03;

			if(hz != 0)
				r.sleep();

			if(fullResetRequested)
			{

				printf("FULL RESET!\n");
				delete system;

				system = new SlamSystem(w, h, K, rK, doSlam);
				system->setVisualization(outputWrapper);

				fullResetRequested = false;
				runningIDX = 0;
			}

			ros::spinOnce();

			if(!ros::ok())
				break;
		}
	}
	else{
		for(unsigned int i=0;i<files.size();i++)
		{
			cv::Mat imageDist = cv::imread(files[i], CV_LOAD_IMAGE_GRAYSCALE);

			if(imageDist.rows != h_inp || imageDist.cols != w_inp)
			{
				if(imageDist.rows * imageDist.cols == 0)
					printf("failed to load image %s! skipping.\n", files[i].c_str());
				else
					printf("image %s has wrong dimensions - expecting %d x %d, found %d x %d. Skipping.\n",
							files[i].c_str(),
							w,h,imageDist.cols, imageDist.rows);
				continue;
			}
			assert(imageDist.type() == CV_8U);

			undistorter->undistort(imageDist, image);
			assert(image.type() == CV_8U);


			if(runningIDX == 0)
				system->randomInit(image.data, fakeTimeStamp, runningIDX);
			else
				system->trackFrame(image.data, runningIDX ,hz == 0,fakeTimeStamp);


			runningIDX++;
			fakeTimeStamp+=0.03;

			if(hz != 0)
				r.sleep();

			if(fullResetRequested)
			{

				printf("FULL RESET!\n");
				delete system;

				system = new SlamSystem(w, h, K, doSlam);
				system->setVisualization(outputWrapper);

				fullResetRequested = false;
				runningIDX = 0;
			}

			ros::spinOnce();

			if(!ros::ok())
				break;
		}
	}
// end added


	system->finalize();



	delete system;
	delete undistorter;
	delete outputWrapper;
	return 0;
}
