#include <GRT/GRT.h>
using namespace GRT;
using namespace std;



int main (int argc, const char * argv[])
{
	//Create a new DTW instance, using the default parameters
	DTW dtw;
    
	//Load some training data to train the classifier - the DTW uses TimeSeriesClassificationData
	TimeSeriesClassificationData trainingData;
    
    const string classifier = argv[1];
    const string filename = argv[2];
    cout << "Filename: " << filename << endl;

    //Load some training data from a file    
    if( !trainingData.load( filename ) ){
        cout << "ERROR: Failed to load training data from file\n";
        return EXIT_FAILURE;
    }

	//Use 20% of the training dataset to create a test dataset
	TimeSeriesClassificationData testData = trainingData.partition( 80 );

	//Trim the training data for any sections of non-movement at the start or end of the recordings
	
	if (classifier == "dtw")
	{
		dtw.enableTrimTrainingData(true,0.1,90);
	    
		//Train the classifier
		if( !dtw.train( trainingData ) ){
			cout << "Failed to train classifier!\n";
			return EXIT_FAILURE;
		}	
	    
		//Save the DTW model to a file
		if( !dtw.save("DTWModel.grt") ){
			cout << "Failed to save the classifier model!\n";
			return EXIT_FAILURE;
		}
	    
		//Load the DTW model from a file
		if( !dtw.load("DTWModel.grt") ){
			cout << "Failed to load the classifier model!\n";
			return EXIT_FAILURE;
		}
	    
		//Use the test dataset to test the DTW model
		double accuracy = 0;
		for(UINT i=0; i<testData.getNumSamples(); i++){
			//Get the i'th test sample - this is a timeseries
			UINT classLabel = testData[i].getClassLabel();
			MatrixDouble timeseries = testData[i].getData();
	        
			//Perform a prediction using the classifier
			if( !dtw.predict( timeseries ) ){
				cout << "Failed to perform prediction for test sampel: " << i <<"\n";
				return EXIT_FAILURE;
			}
	        
			//Get the predicted class label
			UINT predictedClassLabel = dtw.getPredictedClassLabel();
			double maximumLikelihood = dtw.getMaximumLikelihood();
			VectorDouble classLikelihoods = dtw.getClassLikelihoods();
			VectorDouble classDistances = dtw.getClassDistances();
	        
			//Update the accuracy
			if( classLabel == predictedClassLabel ) accuracy++;
	        
	        cout << "TestSample: " << i <<  "\tClassLabel: " << classLabel << "\tPredictedClassLabel: " << predictedClassLabel << "\tMaximumLikelihood: " << maximumLikelihood << endl;
		}
	    
		cout << "Test Accuracy: " << accuracy/double(testData.getNumSamples())*100.0 << "%" << endl;
	}
	else if (classifier == "chmm")
	{
	    //Create a new HMM instance
	    HMM hmm;
	    
	    //Set the HMM as a Continuous HMM
	    hmm.setHMMType( HMM_CONTINUOUS );
	    
	    //Set the downsample factor, a higher downsample factor will speed up the prediction time, but might reduce the classification accuracy
	    hmm.setDownsampleFactor( 5 );
	    
	    //Set the committee size, this sets the (top) number of models that will be used to make a prediction
	    hmm.setCommitteeSize( 10 );
	    
	    //Tell the hmm algorithm that we want it to estimate sigma from the training data
	    hmm.setAutoEstimateSigma( true );
	    
	    //Set the minimum value for sigma, you might need to adjust this based on the range of your data
	    //If you set setAutoEstimateSigma to false, then all sigma values will use the value below
	    hmm.setSigma( 20.0 );
	    
	    //Set the HMM model type to LEFTRIGHT with a delta of 1, this means the HMM can only move from the left-most state to the right-most state
	    //in steps of 1
	    hmm.setModelType( HMM_LEFTRIGHT );
	   	hmm.setDelta( 1 );

	   	hmm.setNumStates( 20 );

	    
	    //Train the HMM model
	    if( !hmm.train( trainingData ) ){
	        cout << "ERROR: Failed to train the HMM model!\n";
	        return false;
	    }
	    
	    //Save the HMM model to a file
	    if( !hmm.save( "HMMModel.grt" ) ){
	        cout << "ERROR: Failed to save the model to a file!\n";
	        return false;
	    }
	    
	    //Load the HMM model from a file
	    if( !hmm.load( "HMMModel.grt" ) ){
	        cout << "ERROR: Failed to load the model from a file!\n";
	        return false;
	    }

	    //Compute the accuracy of the HMM models using the test data
	    double numCorrect = 0;
	    double numTests = 0;
	    for(UINT i=0; i<testData.getNumSamples(); i++){
	        
	        UINT classLabel = testData[i].getClassLabel();
	        hmm.predict( testData[i].getData() );
	        
	        if( classLabel == hmm.getPredictedClassLabel() ) numCorrect++;
	        numTests++;
	        
	        VectorFloat classLikelihoods = hmm.getClassLikelihoods();
	        VectorFloat classDistances = hmm.getClassDistances();
	        
	        cout << "ClassLabel: " << classLabel;
	        cout << " PredictedClassLabel: " << hmm.getPredictedClassLabel();
	        cout << " MaxLikelihood: " << hmm.getMaximumLikelihood();
	        
	        cout << "  ClassLikelihoods: ";
	        for(UINT k=0; k<classLikelihoods.size(); k++){
	            cout << classLikelihoods[k] << "\t";
	        }
	        
	        cout << "ClassDistances: ";
	        for(UINT k=0; k<classDistances.size(); k++){
	            cout << classDistances[k] << "\t";
	        }
	        cout << endl;
	    }
	    
	    cout << "Test Accuracy: " << numCorrect/numTests*100.0 << endl;
	}

	else if (classifier == "dhmm")
	{
		//The input to the HMM must be a quantized discrete value
		//We therefore use a KMeansQuantizer to covert the N-dimensional continuous data into 1-dimensional discrete data
		const UINT NUM_SYMBOLS = 10;
		KMeansQuantizer quantizer( NUM_SYMBOLS );

		//Train the quantizer using the training data
		if( !quantizer.train( trainingData ) ){
		    cout << "ERROR: Failed to train quantizer!\n";
		    return false;
		}

		//Quantize the training data
		TimeSeriesClassificationData quantizedTrainingData( 1 );

		for(UINT i=0; i<trainingData.getNumSamples(); i++){
		    
		    UINT classLabel = trainingData[i].getClassLabel();
		    MatrixDouble quantizedSample;
		    
		    for(UINT j=0; j<trainingData[i].getLength(); j++){
		        quantizer.quantize( trainingData[i].getData().getRow(j) );
		        
		        quantizedSample.push_back( quantizer.getFeatureVector() );
		    }
		    
		    if( !quantizedTrainingData.addSample(classLabel, quantizedSample) ){
		        cout << "ERROR: Failed to quantize training data!\n";
		        return false;
		    }
		    
		}
			    //Create a new HMM instance
	    HMM hmm;
	    
	    //Set the HMM as a Discrete HMM
	    hmm.setHMMType( HMM_DISCRETE );
	    
	    //Set the number of states in each model
	    hmm.setNumStates( 10 ); //was 4
	    
	    //Set the number of symbols in each model, this must match the number of symbols in the quantizer
	    hmm.setNumSymbols( NUM_SYMBOLS );
	    
	    //Set the HMM model type to LEFTRIGHT with a delta of 1
	    hmm.setModelType( HMM_LEFTRIGHT );
	    hmm.setDelta( 1 );
	    
	    //Set the training parameters
	    hmm.setMinChange( 1.0e-5 );
	    hmm.setMaxNumEpochs( 100 );
	    hmm.setNumRandomTrainingIterations( 20 );
	    
	    //Train the HMM model
	    if( !hmm.train( quantizedTrainingData ) ){
	        cout << "ERROR: Failed to train the HMM model!\n";
	        return false;
	    }
	    
	    //Save the HMM model to a file
	    if( !hmm.save( "HMMModel.grt" ) ){
	        cout << "ERROR: Failed to save the model to a file!\n";
	        return false;
	    }
	    
	    //Load the HMM model from a file
	    if( !hmm.load( "HMMModel.grt" ) ){
	        cout << "ERROR: Failed to load the model from a file!\n";
	        return false;
	    }
	    
	    //Quantize the test data
	    TimeSeriesClassificationData quantizedTestData( 1 );
	    
	    for(UINT i=0; i<testData.getNumSamples(); i++){
	        
	        UINT classLabel = testData[i].getClassLabel();
	        MatrixDouble quantizedSample;
	        
	        for(UINT j=0; j<testData[i].getLength(); j++){
	            quantizer.quantize( testData[i].getData().getRow(j) );
	            
	            quantizedSample.push_back( quantizer.getFeatureVector() );
	        }
	        
	        if( !quantizedTestData.addSample(classLabel, quantizedSample) ){
	            cout << "ERROR: Failed to quantize training data!\n";
	            return false;
	        }
	    }
	    
	    //Compute the accuracy of the HMM models using the test data
	    double numCorrect = 0;
	    double numTests = 0;
	    for(UINT i=0; i<quantizedTestData.getNumSamples(); i++){
	        
	        UINT classLabel = quantizedTestData[i].getClassLabel();
	        hmm.predict( quantizedTestData[i].getData() );
	        
	        if( classLabel == hmm.getPredictedClassLabel() ) numCorrect++;
	        numTests++;
	        
	        VectorFloat classLikelihoods = hmm.getClassLikelihoods();
	        VectorFloat classDistances = hmm.getClassDistances();
	        
	        cout << "ClassLabel: " << classLabel;
	        cout << " PredictedClassLabel: " << hmm.getPredictedClassLabel();
	        cout << " MaxLikelihood: " << hmm.getMaximumLikelihood();
	        
	        cout << "  ClassLikelihoods: ";
	        for(UINT k=0; k<classLikelihoods.size(); k++){
	            cout << classLikelihoods[k] << "\t";
	        }
	        
	        cout << "ClassDistances: ";
	        for(UINT k=0; k<classDistances.size(); k++){
	            cout << classDistances[k] << "\t";
	        }
	        cout << endl;
	    }
		cout << "Test Accuracy: " << numCorrect/numTests*100.0 << endl;
	}
	    
	return EXIT_SUCCESS;
}



// int main (int argc, const char * argv[])
// {
//     //Parse the data filename from the argument list
//     if( argc != 2 ){
//         cout << "Error: failed to parse data filename from command line. You should run this example with one argument pointing to the data filename!\n";
//         return EXIT_FAILURE;
//     }
//     const string filename = argv[1];
//     cout << "Filename: " << filename << endl;

//     //Load some training data from a file
//     ClassificationData trainingData;
    
//     if( !trainingData.load( filename ) ){
//         cout << "ERROR: Failed to load training data from file\n";
//         return EXIT_FAILURE;
//     }
    
//     cout << "Data Loaded\n";
    
//     //Print out some stats about the training data
//     trainingData.printStats();
    
//     //Partition the training data into a training dataset and a test dataset. 80 means that 80%
//     //of the data will be used for the training data and 20% will be returned as the test dataset
//     ClassificationData testData = trainingData.partition( 80 );
    
//     //Create a new Gesture Recognition Pipeline
//     GestureRecognitionPipeline pipeline;

//     //Add a naive bayes classifier to the pipeline
//     pipeline << ANBC();
    
//     //Train the pipeline using the training data
//     if( !pipeline.train( trainingData ) ){
//         cout << "ERROR: Failed to train the pipeline!\n";
//         return EXIT_FAILURE;
//     }
    
//     //Save the pipeline to a file
// 	if( !pipeline.save( "HelloWorldPipeline.grt" ) ){
//         cout << "ERROR: Failed to save the pipeline!\n";
//         return EXIT_FAILURE;
//     }
    
// 	//Load the pipeline from a file
// 	if( !pipeline.load( "HelloWorldPipeline.grt" ) ){
//         cout << "ERROR: Failed to load the pipeline!\n";
//         return EXIT_FAILURE;
//     }
    
//     //Test the pipeline using the test data
//     if( !pipeline.test( testData ) ){
//         cout << "ERROR: Failed to test the pipeline!\n";
//         return EXIT_FAILURE;
//     }
    
//     //Print some stats about the testing
//     cout << "Test Accuracy: " << pipeline.getTestAccuracy() << endl;
    
//     //Get the class labels
//     Vector< UINT > classLabels = pipeline.getClassLabels();
    
//     cout << "Precision: ";
//     for(UINT k=0; k<pipeline.getNumClassesInModel(); k++){
//         cout << "\t" << pipeline.getTestPrecision( classLabels[k] );
//     }cout << endl;
    
//     cout << "Recall: ";
//     for(UINT k=0; k<pipeline.getNumClassesInModel(); k++){
//         cout << "\t" << pipeline.getTestRecall( classLabels[k] );
//     }cout << endl;
    
//     cout << "FMeasure: ";
//     for(UINT k=0; k<pipeline.getNumClassesInModel(); k++){
//         cout << "\t" << pipeline.getTestFMeasure( classLabels[k] );
//     }cout << endl;
    
//     MatrixFloat confusionMatrix = pipeline.getTestConfusionMatrix();
//     cout << "ConfusionMatrix: \n";
//     for(UINT i=0; i<confusionMatrix.getNumRows(); i++){
//         for(UINT j=0; j<confusionMatrix.getNumCols(); j++){
//             cout << confusionMatrix[i][j] << "\t";
//         }cout << endl;
//     }
    
//     return EXIT_SUCCESS;
// }
