/*
 * Copyright (C) 2014 michael.
 * 
 * Licensed under the Apache License, Version 2.0 (the "License"); you may not
 * use this file except in compliance with the License. You may obtain a copy of
 * the License at
 * 
 * http://www.apache.org/licenses/LICENSE-2.0
 * 
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the
 * License for the specific language governing permissions and limitations under
 * the License.
 */

package rosjava.java_classifiers.weka_classify;

import org.apache.commons.logging.Log;
import org.ros.message.MessageListener;
import org.ros.concurrent.CancellableLoop;
import org.ros.namespace.GraphName;
import org.ros.node.AbstractNodeMain;
import org.ros.node.ConnectedNode;
import org.ros.node.NodeMain;
import org.ros.node.topic.Publisher;
import org.ros.node.topic.Subscriber;

import java.text.DecimalFormat;

import java.io.IOException;
import java.io.BufferedReader;
import java.io.FileNotFoundException;
import java.io.FileReader;
import java.io.PrintWriter;

import weka.classifiers.Classifier;
import weka.classifiers.Evaluation;
import weka.classifiers.evaluation.NominalPrediction;
import weka.classifiers.rules.DecisionTable;
import weka.classifiers.rules.PART;
import weka.classifiers.trees.DecisionStump;
import weka.classifiers.trees.J48;
import weka.classifiers.bayes.BayesNet;
import weka.classifiers.bayes.HMM;
import weka.classifiers.meta.AdaBoostM1;
import weka.classifiers.meta.LogitBoost;
import weka.classifiers.trees.RandomForest;
import weka.classifiers.bayes.NaiveBayes;
import weka.classifiers.meta.RandomCommittee;
import weka.classifiers.trees.RandomTree;
import weka.classifiers.pmml.consumer.SupportVectorMachineModel;
import weka.classifiers.pmml.consumer.NeuralNetwork;


 import weka.core.FastVector;

 import weka.core.DenseInstance;
 import weka.core.Attribute;

// import weka.classifiers.bayes.BayesNet;

import weka.core.Instances;

import weka.core.SelectedTag;
import weka.core.matrix.DoubleVector;
import weka.core.matrix.Matrix;
import weka.estimators.MultivariateNormalEstimator;

import java.text.ParseException;
// import java.util.ArrayList;
// import java.util.Arrays;

import java.util.*;

import perceptual_filter.FeaturesArr;
import perceptual_filter.Feature;
import perceptual_filter.Label;


import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;

/**
 * A simple {@link Publisher} {@link Subscriber} {@link NodeMain}.
 */
public class WekaClassify extends AbstractNodeMain {

  Queue<double[]> q_;
  Queue<Boolean> qS_;
  Queue<String> qL_;
  AdaBoostM1 adamodel_;
  Instances trainingSet_;
  Instances testSample_;

  double value;
  double[] featureVector_;
  boolean newSequence_;
  Map<String,Integer> classMap_;
  BufferedReader trainfile_;
  int count_;
  private final Lock _mutex = new ReentrantLock(true);

  // simply returns an input reader for a file
  public static BufferedReader readDataFile(String filename) {
    BufferedReader inputReader = null;
 
    try {
      inputReader = new BufferedReader(new FileReader(filename));
    } catch (FileNotFoundException ex) {
      System.err.println("File not found: " + filename);
    }
 
    return inputReader;
  }
 

  private static double round(double value, int precision) {
      int scale = (int) Math.pow(10, precision);
      return (double) Math.round(value * scale) / scale;
  }

  @Override
  public GraphName getDefaultNodeName() {
    return GraphName.of("rosjava/WekaClassify");
  }





// Entry point of the program, like main method
  @Override
  public void onStart(final ConnectedNode connectedNode) {

    q_ = new LinkedList<double[]>();
    qL_ = new LinkedList<String>();
    qS_ = new LinkedList<Boolean>();
    newSequence_ = false;


    trainfile_ = readDataFile("/home/" + System.getProperty("user.name")  + "/.ros/test_leniency100_aligned_alljointsnormalized.arff");
    //BufferedReader testfile = readDataFile("/home/" + System.getProperty("user.name")  + "/.ros/test_leniency100_last2_aligned_alljointsnormalized.arff");

    try 
    {
      Instances trainingSet_ = new Instances(trainfile_);

      BufferedReader testfile = readDataFile("/home/" + System.getProperty("user.name")  + "/.ros/demo.arff");
      testSample_ = new Instances(testfile);
      testSample_.setClassIndex(testSample_.numAttributes() - 1);
      //Instances testSample_ = new Instances(testfile);

      System.out.println("loaded data");

      trainingSet_.setClassIndex(trainingSet_.numAttributes() - 1);
      //testSample_.setClassIndex(testSample_.numAttributes() - 1);


      adamodel_ = new AdaBoostM1();


      String[] adaboostOptions = adamodel_.getOptions();
      adaboostOptions[7] = "weka.classifiers.trees.J48";

      try
      {
        adamodel_.setOptions(adaboostOptions);
        adamodel_.buildClassifier(trainingSet_); // after this, start taking test instances and emit to perceptual filter
      } catch (Exception e)
      {
          System.out.println("Exception in setting options");
      }


    }
    catch (IOException e) {
      System.out.println("cannot read datafile for weka crossvalidation");
    }


// 
// Section of code that handles publishing and subsribing
//


    final Log log = connectedNode.getLog();



    // Instantiate Publisher and Subscriber
    final Publisher<std_msgs.String> publisher = connectedNode.newPublisher("weka_publish", std_msgs.String._TYPE);
    Subscriber<perceptual_filter.FeaturesArr> subscriber = 
      connectedNode.newSubscriber("/perceptual_filter/features", perceptual_filter.FeaturesArr._TYPE);
    

    // Subscriber:
    // Includes callback
    subscriber.addMessageListener(new MessageListener<perceptual_filter.FeaturesArr>() {
      @Override
      public void onNewMessage(perceptual_filter.FeaturesArr message) {
        //log.info("Features: " + message.getFeatures().toString());

        for (perceptual_filter.Feature f: message.getFeatures())
        {
          //count_++;
          featureVector_ = f.getValues().clone();
          //System.out.println("count_ " + count_);
        }

        _mutex.lock();
        q_.add(featureVector_);
        _mutex.unlock();
      }

    }/*, 1000000*/);

    // Publisher:
    // This CancellableLoop will be canceled automatically when the node shuts
    // down.
    connectedNode.executeCancellableLoop(new CancellableLoop() {
      private int sequenceNumber;
      private int noDataTimesInARow;
      private boolean dataArrivedAlready;          
      private double[] featureVector;
      private boolean newSequence;
      private ArrayList<double[]> sequenceVectors; //contains last n features vectors
      private ArrayList<ArrayList<double[]>> allSequenceVectors;
      private int count_;
      private int q_size;

      //private boolean done;

      @Override
      protected void setup() {
        sequenceNumber = 0;
        noDataTimesInARow = 0;
        dataArrivedAlready = false;
        newSequence = false;
        allSequenceVectors = new ArrayList<ArrayList<double[]>>();

      }

      @Override
      protected void loop() throws InterruptedException {

        //this is where I want to train the model...




        _mutex.lock();
        if ( q_.size() != 0)
        {
          noDataTimesInARow = 0;
          featureVector = q_.poll();

          //System.out.println("lean y " + featureVector[0]);
          // double[] featureVector2 = new double[82];
          // for (int i = 0; i<81; i++)
          // {
          //   featureVector2[i] = featureVector[i];
          // }
          // featureVector2[81] = 0;


          double classLabel;
          //System.out.println("classLabel " + inst);

          try
          {




            DenseInstance inst = new DenseInstance(1.0, featureVector);
            inst.setDataset(testSample_);

            //System.out.println(inst);
            classLabel = adamodel_.classifyInstance(inst);
            System.out.println("classLabel " + classLabel);


            std_msgs.String str = publisher.newMessage();
            str.setData( Double.toString(classLabel) );
            publisher.publish(str);
          }
          catch(Exception e)
          {
             System.out.println("excpetion at classify instance");
             System.exit(0);
          }
          


        }
        _mutex.unlock();



        //double [] fV;

                //       System.arraycopy(testSample_.instance(i).toDoubleArray(),0 , fV, 0, 82);

        //fV = new double[] {-0.115538,0.073742,0,1,0.001452,24,0,0,0,-0.707095,-0.005835,-0.707095,-0.981701,1.300011,-0.516185,-0.668884,-1.306939,0.065619,0.189134,0.302209,-1.139619,1.643171,0.857012,-0.296619,0.744138,-0.505403,1.579372,1.897539,1.033101,1.811027,-0.969934,-0.213983,1.69534,2.195517,2.440804,0.985325,-1.312545,-0.000228,1.901506,-1.51744,-0.884429,-2.45771,0.131465,-0.114324,-1.992574,-1.583262,-1.73324,-1.332902,1.123326,-1.173121,-1.27541,1.073403,0.606477,-1.57235,1.901443,0.896159,-2.046939,-1.763594,2.916221,-1.61949,-1.282632,-2.500437,2.024008,3.840735,-1.399038,0.35405,1.775203,2.700901,1.113406,1.918051,-1.18515,-1.124813,-0.234192,1.894994,-0.839116,2.801335,1.158215,-1.673753,-0.71322,-1.737168,-0.522995,10};
        

      

        Thread.sleep(1);
        //Thread.sleep(1000);
      }
    });
  }
}
