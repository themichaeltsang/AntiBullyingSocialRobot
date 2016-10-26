#include "hCRF.h"

#include "ros/ros.h"
#include "perceptual_filter/FeaturesArr.h"
#include <iostream>
#include <string>
#include <vector>
#include <queue>
#include <fstream>
#include <boost/thread/mutex.hpp>

#ifdef WIN32
#include <conio.h>
#endif

#include <time.h>

#ifdef _OPENMP
#include <omp.h>
#endif

using namespace std;

#define MODE_TRAIN 1
#define MODE_TEST  2
#define MODE_DEBUG 4
#define MODE_VALIDATE  8

#define TOOLBOX_CRF  1
#define TOOLBOX_HCRF  2
#define TOOLBOX_LDCRF  4
#define TOOLBOX_GHCRF  8
#define TOOLBOX_LVPERCEPTRON 32
#define TOOLBOX_SDCRF 16


int count_ = 0;

std::queue< vector<double> > featureVectorQueue_ = std::queue< vector<double> >();
std::queue< string > labelsQueue_ = std::queue< string >();
std::queue< bool > newSequenceQueue_ = std::queue< bool >();
std::queue< string > sequenceBoundaryQueue_ = std::queue< string >();
std::queue< uint64_t > sequenceNumberQueue_ = std::queue< uint64_t >();

boost::mutex queues_mutex_;


// class HCRFClassify
// {
// public:
// 	int peach_;

// 	HCRFClassify() : peach_( 0 )
// 	{
// 	}


// 	void featureCallback(const perceptual_filter::FeaturesArr &msg)
// 	{

// 		//write text file here for what? sequential features file first, 
// 		//then labels file


// 		//for ()

// 		peach_ ++;
// 		cout << "count_ "  << peach_  << endl;
// 		cout << "hello" << endl;
// 	}

// };

void featureCallback(const perceptual_filter::FeaturesArr &msg)
{

	auto const & features = msg.features;
	vector<double> featureVector;

	assert( features.size() == 1 );
	for (auto const & feature : features)
	{
		featureVector = feature.values;

	}

	auto const & labels = msg.labels;
	string classLabel = "";

	for (auto const & label : labels)
	{
		string labelName = label.labelName;
		int labelVal = label.labelValue;
		if (labelVal == 1)
		{
			classLabel = labelName;
			break;
		}
		else if (labelVal == 0)
		{
			classLabel = "nothing";
		}

	}
	bool newSequence = msg.new_sequence;

	// if (newSequence)
	// {
	// 	newSequence_ = true;
	// }

	//cout << "sequence number " << msg.sequence_number << ", boundary " << msg.sequence_boundary << endl;

	if (classLabel != "")
	{
    	boost::mutex::scoped_lock lock(queues_mutex_);
		featureVectorQueue_.push(featureVector);
		labelsQueue_.push(classLabel);
		sequenceBoundaryQueue_.push(msg.sequence_boundary);
		sequenceNumberQueue_.push(msg.sequence_number);
		//newSequenceQueue_.push(newSequence);
		//cout << "size " << featureVectorQueue_.size() << endl;

	}
}


int main(int argc, char **argv)
{
	ros::init(argc, argv, "cpp_classifiers");
	ros::NodeHandle nh;

	int rate = 100;

	ros::Rate r(rate);

	//HCRFClassify *classifier = new HCRFClassify();
	//ros::Subscriber sub_msg = nh.subscribe("/perceptual_filter/features", 1000, &HCRFClassify::featureCallback, classifier);
	ros::Subscriber sub_msg = nh.subscribe("/perceptual_filter/features", 1000, featureCallback);


	int q_size;
	int noDataTimesInARow;
	bool newSequence;
	string classLabel;
	string sequenceBoundary;
	int sequenceNumber;
	int labelValue;
	vector<double> featureVector;
	vector<vector<double>> sequenceVectors; //contains last n feature vectors
	vector<int> sequenceLabels;
	bool dataArrivedAlready = false;
	int count_ = 0;

	sequenceVectors = vector<vector<double>>();
	sequenceLabels = vector<int>();

	map<string,int> labelsMap;
    // labelsMap["punching prep"] = 0;
    // labelsMap["kicking prep"] = 1;
    // labelsMap["hitting prep"] = 2;
    // labelsMap["pointing"] = 3;
    // labelsMap["pointing and laughing"] = 4;
    // labelsMap["fist threat"] = 0; //Note
    // labelsMap["nothing"] = 5;

    // labelsMap["nothing"] = 0;
    // labelsMap["punching prep"] = 1;
    // labelsMap["kicking prep"] = 2;
    // labelsMap["hitting prep"] = 3;
    // labelsMap["pointing"] = 4;
    // labelsMap["pointing and laughing"] = 5;
    // labelsMap["fist threat"] = 1; //Note
    // labelsMap["punching"] = 6;
    // labelsMap["kicking"] = 7;
    // labelsMap["shoving"] = 8;
    // labelsMap["shoving prep"] = 9;
    // labelsMap["hitting"] = 10;

    labelsMap["pointing"] = 0;
    labelsMap["punching"] = 1;
    labelsMap["showing fist"] = 2;
    labelsMap["kicking"] = 3;
    labelsMap["hitting"] = 1;
    labelsMap["null"] = 4;


    ofstream featureFile;
	featureFile.open("dataTrain.csv");

    //ofstream labelsFile;
	//labelsFile.open("labelsTrainLDCRF.csv");

	ofstream seqLabelsFile;
	seqLabelsFile.open("seqLabelsTrain.csv");


	while (ros::ok())
	{
		{
			boost::mutex::scoped_lock lock(queues_mutex_);
			q_size = featureVectorQueue_.size();
			assert(labelsQueue_.size() == q_size);
			assert(sequenceBoundaryQueue_.size() == q_size);
			assert(sequenceNumberQueue_.size() == q_size);

			//assert(newSequenceQueue_.size() == q_size);
		}




		//cout << "q_size " << q_size << endl;

		if (q_size != 0)
		{
			//cout << "count_ " << count_ << endl;
			count_++;

			noDataTimesInARow = 0;
			string classLabel;
			{
				boost::mutex::scoped_lock lock(queues_mutex_);
				featureVector = featureVectorQueue_.front();
				classLabel = labelsQueue_.front();
				sequenceNumber = sequenceNumberQueue_.front();
				sequenceBoundary = sequenceBoundaryQueue_.front();
				// newSequence = newSequenceQueue_.front();
				featureVectorQueue_.pop();
				labelsQueue_.pop();
				sequenceNumberQueue_.pop();
				sequenceBoundaryQueue_.pop();

				// newSequenceQueue_.pop();
			}

			cout << "label " << classLabel << ", sequence number " << sequenceNumber << ", boundary " << sequenceBoundary << endl;
			//cout << "size of feature vector " << featureVector.size() << endl;
			// dataArrivedAlready = true;

			labelValue = labelsMap[classLabel];

			if (sequenceVectors.size() == 0 || sequenceLabels.size() == 0)
			{
				assert(sequenceBoundary == "start");
			}

			sequenceVectors.push_back(featureVector);
			sequenceLabels.push_back(labelValue);


			//int size = 30;
			if (sequenceBoundary == "end")
			{
				if (sequenceVectors.size() >= 5) // set a lower bound, sequences must be size 5 or more
				{
					assert(sequenceLabels.size() != 0);
					featureFile << sequenceVectors[0].size() << "," << sequenceVectors.size() << "\n";
					
					for (int i = 0; i < sequenceVectors[0].size(); i++)
					{
						for (int j = 0; j < sequenceVectors.size(); j++)
						{
							if (j < sequenceVectors.size() - 1)
							{
								featureFile << sequenceVectors[j][i] << ",";
							}
							else if (j == sequenceVectors.size() - 1)
							{
								featureFile << sequenceVectors[j][i] << "\n";
							}
							
						}
					}

					// labelsFile << "1," << sequenceLabels.size() << "\n";

					// for (int i = 0; i < sequenceLabels.size(); i++)
					// {
					// 	if (i < sequenceLabels.size() - 1)
					// 	{
					// 		labelsFile << sequenceLabels[i] << ",";
					// 	}
					// 	else if (i == sequenceLabels.size() - 1)
					// 	{
					// 		labelsFile << sequenceLabels[i] << "\n";
					// 	}
					// }

					// int max = 0;
					// int most_common = -1;
					// map<int,int> m;
					// for (auto vi = sequenceLabels.begin(); vi != sequenceLabels.end(); vi++) 
					// {

					// 	// m[*vi]++;
					// 	// if (m[*vi] > max) 
					// 	// {
					// 	// 	max = m[*vi]; 
					// 	// 	most_common = *vi;
					// 	// }
					// }
					// assert(most_common != -1);
					seqLabelsFile << sequenceLabels[0] << "\n";

				}

				sequenceVectors = vector<vector<double>>();
				sequenceLabels = vector<int>();
			}


		}





















		//cout << "q_size " << q_size << endl;

		// if (q_size != 0)
		// {
		// 	cout << "count_ " << count_ << endl;
		// 	count_++;

		// 	noDataTimesInARow = 0;
		// 	string classLabel;
		// 	{
		// 		boost::mutex::scoped_lock lock(queues_mutex_);
		// 		featureVector = featureVectorQueue_.front();
		// 		classLabel = labelsQueue_.front();
		// 		newSequence = newSequenceQueue_.front();
		// 		featureVectorQueue_.pop();
		// 		labelsQueue_.pop();
		// 		newSequenceQueue_.pop();
		// 	}
		// 	dataArrivedAlready = true;

		// 	labelValue = labelsMap[classLabel];


		// 	int size = 30;
		// 	if (newSequence)
		// 	{
		// 		if (sequenceVectors.size() != 0)
		// 		{
		// 			assert(sequenceLabels.size() != 0);
		// 			featureFile << sequenceVectors[0].size() << "," << sequenceVectors.size() << "\n";
					
		// 			for (int i = 0; i < sequenceVectors[0].size(); i++)
		// 			{
		// 				for (int j = 0; j < sequenceVectors.size(); j++)
		// 				{
		// 					if (j < sequenceVectors.size() - 1)
		// 					{
		// 						featureFile << sequenceVectors[j][i] << ",";
		// 					}
		// 					else if (j == sequenceVectors.size() - 1)
		// 					{
		// 						featureFile << sequenceVectors[j][i] << "\n";
		// 					}
							
		// 				}
		// 			}

		// 			labelsFile << "1," << sequenceLabels.size() << "\n";

		// 			for (int i = 0; i < sequenceLabels.size(); i++)
		// 			{
		// 				if (i < sequenceLabels.size() - 1)
		// 				{
		// 					labelsFile << sequenceLabels[i] << ",";
		// 				}
		// 				else if (i == sequenceLabels.size() - 1)
		// 				{
		// 					labelsFile << sequenceLabels[i] << "\n";
		// 				}
		// 			}

		// 			// int max = 0;
		// 			// int most_common = -1;
		// 			// map<int,int> m;
		// 			// for (auto vi = sequenceLabels.begin(); vi != sequenceLabels.end(); vi++) 
		// 			// {
		// 			// 	m[*vi]++;
		// 			// 	if (m[*vi] > max) 
		// 			// 	{
		// 			// 		max = m[*vi]; 
		// 			// 		most_common = *vi;
		// 			// 	}
		// 			// }
		// 			// assert(most_common != -1);
		// 			// seqLabelsFile << most_common << "\n";

		// 		}

		// 		sequenceVectors = vector<vector<double>>();
		// 		sequenceLabels = vector<int>();
		// 	}

		// 	sequenceVectors.push_back(featureVector);
		// 	sequenceLabels.push_back(labelValue);
		// }












			// labelValue = labelsMap[classLabel];

			// int size = 30;
			// if (newSequence)
			// {
			// 	sequenceVectors = vector<vector<double>>();
			// 	sequenceLabels = vector<int>();
			// }
			// if (sequenceVectors.size() == size)
			// {
			// 	sequenceVectors.push_back(featureVector);
			// 	sequenceVectors.erase(sequenceVectors.begin());
			// 	sequenceLabels.push_back(labelValue);
			// 	sequenceLabels.erase(sequenceLabels.begin());
			// }
			// else if (sequenceVectors.size() < size)
			// {
			// 	sequenceVectors.push_back(featureVector);
			// 	sequenceLabels.push_back(labelValue);
			// }


			// int random = rand() % 100;

			// if (sequenceVectors.size() == size)
			// {


			// 	int max = 0;
			// 	int most_common = -1;
			// 	map<int,int> m;
			// 	for (auto vi = sequenceLabels.begin(); vi != sequenceLabels.end(); vi++) 
			// 	{
			// 		m[*vi]++;
			// 		if (m[*vi] > max) 
			// 		{
			// 			max = m[*vi]; 
			// 			most_common = *vi;
			// 		}
			// 	}
			// 	assert(most_common != -1);

			// 	if (most_common != 0 || random < 5)
			// 	{
			// 		seqLabelsFile << most_common << "\n";
			// 		featureFile << 30 << "," << sequenceVectors.size() << "\n";
			// 		//featureFile << sequenceVectors[0].size() << "," << sequenceVectors.size() << "\n";
			// 		for (int i = 0; i < sequenceVectors[0].size(); i++)
			// 		{	
			// 			if (i >= 30) break;
			// 			for (int j = 0; j < sequenceVectors.size(); j++)
			// 			{
			// 				if (j < sequenceVectors.size() - 1)
			// 				{
			// 					featureFile << sequenceVectors[j][i] << ",";
			// 				}
			// 				else if (j == sequenceVectors.size() - 1)
			// 				{
			// 					featureFile << sequenceVectors[j][i] << "\n";
			// 				}
							
			// 			}
			// 		}				
			// 	}
				

			// }


			// labelValue = labelsMap[classLabel];

			// int size = 30;
			// if (newSequence)
			// {
			// 	sequenceVectors = vector<vector<double>>();
			// 	sequenceLabels = vector<int>();
			// }
			// if (sequenceVectors.size() == size)
			// {
			// 	sequenceVectors.push_back(featureVector);
			// 	sequenceVectors.erase(sequenceVectors.begin());
			// 	sequenceLabels.push_back(labelValue);
			// 	sequenceLabels.erase(sequenceLabels.begin());
			// }
			// else if (sequenceVectors.size() < size)
			// {
			// 	sequenceVectors.push_back(featureVector);
			// 	sequenceLabels.push_back(labelValue);
			// }

			// if (sequenceVectors.size() == size)
			// {
			// 	featureFile << sequenceVectors[0].size() << "," << sequenceVectors.size() << "\n";
			// 	for (int i = 0; i < sequenceVectors[0].size(); i++)
			// 	{
			// 		for (int j = 0; j < sequenceVectors.size(); j++)
			// 		{
			// 			if (j < sequenceVectors.size() - 1)
			// 			{
			// 				featureFile << sequenceVectors[j][i] << ",";
			// 			}
			// 			else if (j == sequenceVectors.size() - 1)
			// 			{
			// 				featureFile << sequenceVectors[j][i] << "\n";
			// 			}
						
			// 		}
			// 	}

			// 	labelsFile << "1," << sequenceLabels.size() << "\n";

			// 	for (int i = 0; i < sequenceLabels.size(); i++)
			// 	{
			// 		if (i < sequenceLabels.size() - 1)
			// 		{
			// 			labelsFile << sequenceLabels[i] << ",";
			// 		}
			// 		else if (i == sequenceLabels.size() - 1)
			// 		{
			// 			labelsFile << sequenceLabels[i] << "\n";
			// 		}
			// 	}

			// 	int max = 0;
			// 	int most_common = -1;
			// 	map<int,int> m;
			// 	for (auto vi = sequenceLabels.begin(); vi != sequenceLabels.end(); vi++) 
			// 	{
			// 		m[*vi]++;
			// 		if (m[*vi] > max) 
			// 		{
			// 			max = m[*vi]; 
			// 			most_common = *vi;
			// 		}
			// 	}
			// 	assert(most_common != -1);
			// 	seqLabelsFile << most_common << "\n";

			// }
		//}

		// if (dataArrivedAlready && q_size == 0)
		// {
		// 	noDataTimesInARow++;
		// 	//cout << "noDataTimesInARow " << noDataTimesInARow << endl; 
		// }

		// if (noDataTimesInARow > 5000)
		// {
		// 	cout << "Done!" << endl;
		// 	exit(EXIT_FAILURE);
		// }
			//create map, for labels -> number conversion
			//write to correct file format
			//end program with numInARoWreached...
			//test HCRF
			//maintain constant fps?

			//test realtime classifications


// 3,2
// 0.1,0.8
// 0.8,0.9
// 0.4,0.6
// 3,3
// 0.8,0.8,0.1
// 0.8,0.9,1.1
// 0.34,0.34,0.02
// 3,4
// 0.23,0.99,1.0,-0.1
// 0.7,0.78,0.78,0.91
// -0.7,0.48,0.2,0.3
// 3,2
// 0.2,0.2
// 0.9,0.87
// 0.1,0.227
			
// 1,3
// 2,0,2
// 1,3
// 0,1,2
// 1,4
// 1,0,1,2
// 1,2
// 2,1

		




		ros::spinOnce();
	}

	return 0;
}

