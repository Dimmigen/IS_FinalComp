#include <armadillo>
#include <iostream>
#include <string>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

using namespace arma;
using namespace std;

void PCA(mat samples, mat& u, vec& meanV)
{
	mat meanSample(samples);
	
	meanV.set_size(samples.n_rows);
	//Calculating mean and centering columns
	for(unsigned int i = 0; i < meanSample.n_rows; ++i)
	{
		double mu = mean(samples.row(i));
		meanSample.row(i) = samples.row(i) - mu;
		meanV.at(i) = mu;
	}
	cout << meanSample << endl;
	//Creating covariance and eigvec and eigval
	mat covariance = meanSample.t()*meanSample;
	covariance /= covariance.n_rows;
	//cout << covariance << endl;
	mat u1,v;
	vec l;
	svd_econ(u1,l,v,covariance);
	
	for(unsigned int i = 0; i < samples.n_cols; ++i)
	{
		u.col(i) = meanSample*u1.col(i);
		u.col(i) /= sqrt(l[i]*samples.n_cols);
	}
}

void getSamples(mat& sample)
{
	/*	cv::Mat imageMat;
    imageMat = cv::imread(file,CV_LOAD_IMAGE_GRAYSCALE);
    //Converting image to arma
	mat input( imageMat.rows,imageMat.cols);
	for(int i = 0; i < imageMat.rows; i++)
	{
		const uchar* Mi = imageMat.ptr<uchar>(i);
		for(int j = 0; j < imageMat.cols; j++) {
			input(i,j) = (double) Mi[j];
			if(input(i,j) > 255 || input(i,j) < 0)
			{
				cout << "Somehting wrong here: " << i << " " << j << endl;
				cout << input(i,j) << endl;
				cout << Mi[j] << endl;
				exit(EXIT_FAILURE);
			} 
		}
	}*/
	sample = randu<mat>(sample.n_rows,sample.n_cols);
	sample*=255;
	cout << sample << endl;
}


void LDA(vec meanV, mat samples, mat& w, mat& classMean, int sizeClass, int numbClass)
{
	for(int i = 0; i < numbClass; ++i)
	{
		classMean.col(i) = mean(samples.cols(i*sizeClass,(i+1)*sizeClass-1),1);
	}
	double p = (double) sizeClass/numbClass;
	mat sb(samples.n_rows, samples.n_rows, fill::zeros);
	mat sa(samples.n_rows, samples.n_rows, fill::zeros);
	for(int i = 0; i < numbClass; ++i)
	{
		vec diff = classMean.col(i) - meanV;
		sb += p*kron(diff, diff.t());
		for(int j = 0; j < sizeClass; ++j)
		{
			vec diff2 = samples.col(i*sizeClass+j) - classMean.col(i);
			sa += (p/sizeClass)*kron(diff2,diff2.t());
		}
	}
	cx_mat w1;
	cx_vec eigVal;
	eig_pair(eigVal,w1,sb,sa);
	w = real(w1);
	classMean = w.t()*classMean;
}


void projectPCA(mat U, vec meanV, vec& a)
{
		a = U.t()*(a-meanV);
}

void projectLDA(mat w, vec& a)
{
	a = w.t()*a;
}

int main()
{
	
	int sizeClass = 10;
	int numbClass = 2;
	int sampleSize = sizeClass*numbClass;
	int pictureSize = 10;
	
	mat samples(pictureSize, sampleSize);
	getSamples(samples);
	mat u(pictureSize, sampleSize);
	mat w;
	vec meanV;
	mat classMean(pictureSize, numbClass);
	
	PCA(samples, u,meanV);
	LDA(meanV, samples, w, classMean, sizeClass, numbClass);
}
