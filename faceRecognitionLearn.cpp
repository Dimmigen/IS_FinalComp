#include <armadillo>
#include <iostream>
#include <string>
#include <sstream>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

using namespace arma;
using namespace std;

void checkImage(cv::Mat image, string name)
{
	if (image.empty())
	{
		std::cout << "failed imread(): image not found: " << name << std::endl;
		exit(EXIT_FAILURE);
		// don't let the execution continue, else imshow() will crash.
	}
}


int getSamples(mat& sample, int numbClasses, int sizeClass)
{
	cv::Mat imageMat;
	stringstream file;
	file  << "/home/simon/catkin_ws/src/Competition_3/src/data/person1/1.png";
	imageMat = cv::imread(file.str(),CV_LOAD_IMAGE_GRAYSCALE);
	checkImage(imageMat, file.str());
	
	sample.set_size(imageMat.rows*imageMat.cols*3,sizeClass*numbClasses);
	for(int i = 0; i < numbClasses; ++i)
	{
		for(int j = 0; j < sizeClass; ++j)
		{
			file.str("");
			file << "/home/simon/catkin_ws/src/Competition_3/src/data/person";
			file  << i+1 << "/" << j+1 << ".png";
			imageMat = cv::imread(file.str(),CV_LOAD_IMAGE_COLOR);//CV_LOAD_IMAGE_GRAYSCALE);
			checkImage(imageMat, file.str());	
			//cv::MatIterator_<uchar> it, end;
			cv::MatIterator_<cv::Vec3b> it, end;
			int k = 0;
			for( it = imageMat.begin<cv::Vec3b>(), end = imageMat.end<cv::Vec3b>(); it != end; ++it)
			{
				sample(k,i*(sizeClass)+j) = (*it)[0];
				++k;
				sample(k,i*(sizeClass)+j) = (*it)[1];
				++k;
				sample(k,i*(sizeClass)+j) = (*it)[2];
                ++k;
			}
		/*	double check = sum(sample.col(i*sizeClass+j));
			if(check < 1.0) cout << "Something went wrong with picture: person" << i << "pic" << j << endl;
			else cout << "Sum of col(" << i*sizeClass+j << "): " << check << endl;*/
		}
	}

	//sample.save("/home/simon/catkin_ws/src/Competition_3/src/matrices/input.mat", arma_ascii);

	return imageMat.rows*imageMat.cols;
}

void PCA(mat& samples, mat& u, vec& meanV)
{
	mat meanSample(samples);
	
	meanV.set_size(samples.n_rows);
	//Calculating mean and centering columns
	meanV = mean(samples,1);
	for(unsigned int i = 0; i < meanSample.n_cols; ++i)
	{
		meanSample.col(i) = samples.col(i) - meanV;
	}
	meanV.save("/home/simon/catkin_ws/src/Competition_3/src/matrices/PCA_mean.mat", arma_ascii);
	//Creating covariance and eigvec and eigval
	mat covariance = meanSample.t()*meanSample;
	covariance /= samples.n_cols;
	mat u1,v;
	vec l;
	svd(u1,l,v,covariance);
	u = meanSample*u1;
	for(unsigned int i = 0; i < u.n_cols; ++i)
	{
		u.col(i) /= sqrt(l.at(i)*samples.n_cols);
		
	}
	cout << l << endl;
	u.save("/home/simon/catkin_ws/src/Competition_3/src/matrices/PCA_U.mat", arma_ascii);
	//Mapping the input vector into PCA space
	samples = u.t()*meanSample;
	//cout << samples << endl;

}

void LDA(vec meanV, mat samples, mat& w, mat& classMean, int sizeClass, int numbClass)
{
	meanV = mean(samples,1);
	//cout << samples << endl;
	cout << "Doing class mean" << endl;
	for(int i = 0; i < numbClass; ++i)
	{
		classMean.col(i) = mean(samples.cols(i*sizeClass,(i+1)*sizeClass-1),1);
	}
	meanV = mean(samples,1);
	mat sb(samples.n_rows, samples.n_rows, fill::zeros);
	mat sa(samples.n_rows, samples.n_rows, fill::zeros);
	cout << "Creating sa and sb" << endl;
	for(int i = 0; i < numbClass; ++i)
	{
		vec diff = classMean.col(i) - meanV;
		sb += sizeClass*kron(diff, diff.t());
		for(int j = 0; j < sizeClass; ++j)
		{
			vec diff2 = samples.col(i*sizeClass+j) - classMean.col(i);
			sa += kron(diff2,diff2.t());
		}
	}

	cx_vec l;
	cx_mat w1;

	eig_gen(l,w1,sb.i()*sa);
	w = real(w1);
	//cout << l << endl;
	//cout << classMean << endl;
	classMean = w.t()*classMean;
	w.save("/home/simon/catkin_ws/src/Competition_3/src/matrices/LDA_w.mat", arma_ascii);
	classMean.save("/home/simon/catkin_ws/src/Competition_3/src/matrices/LDA_classMean.mat", arma_ascii);
}


int main()
{
	
	int sizeClass = 150;
	int numbClass = 12;
	int sampleSize = sizeClass*numbClass;
	
	mat samples(0,0);
	int pictureSize = getSamples(samples, numbClass, sizeClass);
	mat u(pictureSize, sampleSize);
	mat w;
	vec meanV;
	cout << "Doing PCA" << endl;
	PCA(samples, u,meanV);
	mat classMean(sampleSize, numbClass);
	cout << "Doing LDA" << endl;
	LDA(meanV, samples, w, classMean, sizeClass, numbClass);
}
