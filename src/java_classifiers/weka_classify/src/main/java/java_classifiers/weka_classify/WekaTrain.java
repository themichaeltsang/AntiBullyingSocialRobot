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
public class WekaTrain extends AbstractNodeMain {

  Queue<double[]> q_;
  Queue<Boolean> qS_;
  Queue<String> qL_;
  double value;
  double[] featureVector_;
  Instances data;
  boolean newSequence_;
  Map<String,Integer> classMap_;
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
 
  // takes a training set and builds a model to evaluate testing data
  public static Evaluation classify(Classifier model,
      Instances trainingSet, Instances testingSet) throws Exception {
    Evaluation evaluation = new Evaluation(trainingSet);
 
    model.buildClassifier(trainingSet);
    evaluation.evaluateModel(model, testingSet);
 
    return evaluation;
  }
 
  // calculates classification accuracy with predictions and reference labels stored in the same input ArrayList 
  public static double calculateAccuracy(ArrayList<NominalPrediction> predictions) {
    double correct = 0;
    double total = 0;
    for (int i = 0; i < predictions.size(); i++) {
      NominalPrediction np = (NominalPrediction) predictions.get(i);
      // if (np.predicted() != 13.0 && np.actual() != 13.0)
      // {
      //   correct++;
      // }
      // if (np.predicted() == np.actual()) {
      //   correct++;
      // }


      if (np.actual() != 13.0)
      {
        if (np.predicted() != 13.0)
        {
          correct++;
        }
        // if (np.predicted() == np.actual()) {
        //   correct++;
        // }
        total++;
      }

    }
 
    System.out.println("correct " + correct + " total " + total + " all " + predictions.size());
    return 100 * correct / total;
    //return 100 * correct / predictions.size();
  }
 
  // split input data according to parameter numberOfFolds 
  public static Instances[][] crossValidationSplit(Instances data, int numberOfFolds) {
    Instances[][] split = new Instances[2][numberOfFolds];
 
    for (int i = 0; i < numberOfFolds; i++) {
      split[0][i] = data.trainCV(numberOfFolds, i);
      split[1][i] = data.testCV(numberOfFolds, i);
    }
 
    return split;
  }

  private static double round(double value, int precision) {
      int scale = (int) Math.pow(10, precision);
      return (double) Math.round(value * scale) / scale;
  }

  @Override
  public GraphName getDefaultNodeName() {
    return GraphName.of("rosjava/WekaTrain");
  }


// Entry point of the program, like main method
  @Override
  public void onStart(final ConnectedNode connectedNode) {

    q_ = new LinkedList<double[]>();
    qL_ = new LinkedList<String>();
    qS_ = new LinkedList<Boolean>();
    newSequence_ = false;

    BufferedReader datafile = readDataFile("/home/" + System.getProperty("user.name")  + "/.ros/test.arff");

    FastVector      atts;
    FastVector      attsRel;
    FastVector      attVals;
    FastVector      attValsRel;
    Instances       dataRel;
    //double[]        vals, vals2;
    double[]        valsRel;
    //int             i;

    count_ = 0;

    // 1. set up attributes
    atts = new FastVector();
    int numberOfAttributes = 81;//19 
    for(int j = 0; j < numberOfAttributes; ++j)
    {
      atts.addElement(new Attribute("att" + j));
    }

    attValsRel = new FastVector();
    attValsRel.addElement("punching");
    attValsRel.addElement("punchingprep");
    attValsRel.addElement("kicking");
    attValsRel.addElement("kickingprep");
    attValsRel.addElement("shoving");
    attValsRel.addElement("shovingprep");
    attValsRel.addElement("hitting");
    attValsRel.addElement("hittingprep");
    attValsRel.addElement("pointing");
    attValsRel.addElement("pointingandlaughing");
    // attValsRel.addElement("tongueout");
    // attValsRel.addElement("tongueoutandhandatears");
    // attValsRel.addElement("fistthreat");
    attValsRel.addElement("nothing");

    // attValsRel.addElement("bad behavior");
    // attValsRel.addElement("normal behavior");

    classMap_ = new HashMap<String,Integer>();

    // classMap_.put("punching",0);
    // classMap_.put("punchingprep",1);
    // classMap_.put("kicking",2);
    // classMap_.put("kickingprep",3);
    // classMap_.put("shoving",4);
    // classMap_.put("shovingprep",5);
    // classMap_.put("hitting",6);
    // classMap_.put("hittingprep",7);
    // classMap_.put("pointing",8);
    // classMap_.put("pointingandlaughing",9);
    // classMap_.put("tongueout",10);
    // classMap_.put("tongueoutandhandatears",11);
    // classMap_.put("fistthreat",1); //Note
    // classMap_.put("nothing",13);

    // classMap_.put("punchingprep",0);
    // classMap_.put("kickingprep",1);
    // classMap_.put("hittingprep",2);
    // classMap_.put("pointing",3);
    // classMap_.put("pointingandlaughing",4);
    // classMap_.put("fistthreat",0); //Note
    // classMap_.put("nothing",5);
    // classMap_.put("punching",6);
    // classMap_.put("kicking",7);
    // classMap_.put("shoving",8);
    // classMap_.put("shovingprep",9);
    // classMap_.put("hitting",10);

    // all classes
    classMap_.put("punching",0);
    classMap_.put("punchingprep",1);
    classMap_.put("kicking",2);
    classMap_.put("kickingprep",3);
    classMap_.put("shoving",4);
    classMap_.put("shovingprep",5);
    classMap_.put("hitting",6);
    classMap_.put("hittingprep",7);
    classMap_.put("pointing",8);
    classMap_.put("pointingandlaughing",9);
    classMap_.put("fistthreat",1); //Note
    classMap_.put("nothing",10);

    //binary
    // classMap_.put("punchingprep",0);
    // classMap_.put("kickingprep",0);
    // classMap_.put("hittingprep",0);
    // classMap_.put("pointing",0);
    // classMap_.put("pointingandlaughing",0);
    // classMap_.put("fistthreat",0); //Note
    // classMap_.put("punching",0);
    // classMap_.put("kicking",0);
    // classMap_.put("shoving",0);
    // classMap_.put("shovingprep",0);
    // classMap_.put("hitting",0);
    // classMap_.put("nothing",1);
    
    /*attsRel = new FastVector();
    attsRel.addElement(new Attribute("Class", attValsRel));
    */
    atts.addElement(new Attribute("Class", attValsRel));
    // dataRel = new Instances("att19", attsRel, 0);
    // atts.addElement(new Attribute("att19", dataRel, 0));

    // 2. create Instances object
    data = new Instances("ArbitraryData", atts, 0);




    
    // // vals2 = new double[numberOfAttributes];
    // // for(int j = 0; j < 20; ++j)
    // // {
    // //   double random = Math.random() * 50 + 1;
    // //   vals2[j] = random;
    // // }
    // // vals2[19] = 1;

    // // for(int j = 0; j < 50; ++j)
    // //   data.add(new DenseInstance(1.0, vals));


    // // for(int j = 0; j < 50; ++j)
    // //   data.add(new DenseInstance(1.0, vals2));

    // // try {

    //   // [3 4 1 9 9 8 ] [4 ]
    //   // [3 4 1 9 9 8 ] [4 ]
    //   // [3 4 1 9 9 8 ] [4 ]
    //   // [3 4 1 9 9 8 ] [4 ]
    //   // [3 4 1 9 9 8 ] [4 ]
    //   // [3 4 1 9 9 8 ] [4 ]

     //  System.out.println("starting");

     //  Instances dataSample = data;
     //  try 
     //  {
     //    dataSample = new Instances(datafile);
     //  }
     //  catch (IOException e) {
     //    System.out.println("cannot read datafile for weka crossvalidation");
     //  }
     //  System.out.println("loaded data");


     //   //   Instances dataSample = data; //new Instances(datafile);

     //  //Instances dataSample = new Instances(datafile);

     //  dataSample.setClassIndex(dataSample.numAttributes() - 1);
   
     //  // Do 10-split cross validation
     //  Instances[][] split = crossValidationSplit(dataSample, 2);
   
     //  // Separate split into training and testing arrays
     //  Instances[] trainingSplits = split[0];
     //  Instances[] testingSplits = split[1];
   
     //  // Use a set of classifiers
     //  Classifier[] models = { 
     //      //new DecisionStump(), //one-level decision tree
     //      //new AdaBoostM1(),
     //      new BayesNet(),

     //      //new RandomCommittee(),
     //      // new RandomTree(),
     //      new NaiveBayes()
     //      //new LogitBoost(),
     //      // new RandomForest()
     //      //new NeuralNetwork(),
     //      //new DecisionTable(),//decision table majority classifier
     //      //new J48(), // a decision tree
     //      //new PART()
     //      //,//,
     //     // new HMM()
     //  };

     // // HMM hmm = new HMM();

   
     //  // Run for each model
     //  for (int j = 0; j < models.length; j++) {
   
     //    // Collect every group of predictions for current model in a FastVector
     //    ArrayList<NominalPrediction> predictions = new ArrayList<NominalPrediction>();
   
     //    // For each training-testing split pair, train and test the classifier
     //    for (int i = 0; i < trainingSplits.length; i++) {
          
     //      try {
            
     //        Evaluation validation = classify(models[j], trainingSplits[i], testingSplits[i]);
            
     //        for (int k = 0; k < validation.predictions().size() ; k++)
     //        {
     //          predictions.add((NominalPrediction) validation.predictions().get(k));
     //        }

     //        // Uncomment to see the summary for each training-testing pair.
     //        //System.out.println(models[j].toString());
     //      } catch (Exception e){
     //        System.out.println("cannot classify");
     //      }

     //    }
   
     //    // Calculate overall accuracy of current classifier on all splits
     //    double accuracy = calculateAccuracy(predictions);
   
     //    // Print current classifier's name and accuracy in a complicated,
     //    // but nice-looking way.
     //    System.out.println("Accuracy of " + models[j].getClass().getSimpleName() + ": "
     //        + String.format("%.2f%%", accuracy)
     //        + "\n---------------------------------");
     //  }



     //  System.exit(0);


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
          count_++;
          featureVector_ = f.getValues().clone();
          //System.out.println("F: " + featureVector_.length);

          System.out.println("count_ " + count_);

          // for (double val: f.getValues())
          // {
          //   //log.info("Val: " + val);
          //   value = val;
          // }
        }

        String classLabel = "";
        //postprocess labels
        for (perceptual_filter.Label fLabel : message.getLabels())
        {

          String labelName = fLabel.getLabelName();
          int labelVal = fLabel.getLabelValue();

          //System.out.println(labelName + " " + labelVal);


          if (labelName == "fist threat")
          {
            continue;
          }


          if (labelVal == 1)
          {
            classLabel = labelName;
            break;
          } 
          else if (labelVal == 0)
          {
            classLabel = "nothing";
          }
          //DO Something with these labels...
        }

        boolean newSequence = message.getNewSequence();


        // System.out.println("newSequence" + newSequence);
        // if (!newSequence)
        // {
        //   System.out.println(classLabel);
        // }
        // System.exit(0);


        if (newSequence)
        {
          newSequence_ = true;
        }

        if (classLabel != "")
        {
          //combine featureVector and label together


          double[] vals = new double[featureVector_.length + 1];

          for (int i = 0; i < featureVector_.length; i ++)
          {
            vals[i] = featureVector_[i];
          }

          int classIdx = featureVector_.length;



          // System.out.println(classLabel);
          // System.out.println(classMap.get(classLabel));

         // int classVal = classMap.get(classLabel);

          classLabel = classLabel.replaceAll("\\s+","");

          //System.out.println(classLabel);
          vals[classIdx] = classMap_.get(classLabel);

          // if (classMap.get(classLabel) == null)
          // {
          //   vals[classIdx] = 7;
          // }


          //classMap.get(classLabel);//data.attribute(classIdx).addStringValue(classLabel);
          _mutex.lock();
          q_.add(vals);

          // System.out.println("newSequence" + newSequence_);
          // System.exit(0);

          qS_.add(newSequence_);
          newSequence_ = false;

          _mutex.unlock();
          //qL.add(classLabel);
        }

        //System.exit(0);

      }


    }, 100000);

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

        std_msgs.String str = publisher.newMessage();

        //hacky way to write the arff file, see below...
       // assert q.size() == qL.size();

        //System.out.println("q size " + q_.size());
        _mutex.lock();
        q_size = q_.size();
        _mutex.unlock();

        if ( q_size != 0)
        {

          noDataTimesInARow = 0;
          String classLabel;
          //try
          //{
          _mutex.lock();
          featureVector = q_.poll();
          newSequence = qS_.poll();
        // System.out.println("polling newSequence" + newSequence);
        // System.exit(0);

          _mutex.unlock();
          if (featureVector == null || featureVector.length == 0)
          {
            System.out.println("Feature vector empty");
          }


          // int size = 30;
          // if (newSequence == true)
          // {
          //   sequenceVectors = new ArrayList<double[]>();
          // }
          // if (sequenceVectors.size() == size)
          // {
          //   sequenceVectors.add(featureVector);
          //   sequenceVectors.remove(0);
          // }
          // else if (sequenceVectors.size() < size)
          // {
          //   sequenceVectors.add(featureVector);
          // }

          // if (sequenceVectors.size() == size)
          // {
          //   allSequenceVectors.add(sequenceVectors);
          // }


          // classLabel = qL.poll();

          // double[] vals = new double[data.numAttributes()];
          // for (int i = 0; i < featureVector.length; i++)
          // {
          //   vals[i] = featureVector[i];
          // }
          // int classIdx = data.numAttributes()-1;
          // vals[classIdx] = data.attribute(classIdx).addStringValue(classLabel);


          data.add(new DenseInstance(1.0, featureVector));
          //}
          // catch(Exception e)
          // {
          //   System.out.println("Error with q.remove");
          // }
          //System.out.println("Val: " + featureVector[0]);
          dataArrivedAlready = true;

          count_ += 1;
          System.out.println("count__ " + count_);
        }
        if (dataArrivedAlready && (q_.size()==0 || featureVector == null))
        {

          //System.out.println("noDataTimesInARow" + noDataTimesInARow);

          noDataTimesInARow ++;
        }


        if (noDataTimesInARow > 15000)
        {

          // System.out.println("All sequence vectors size " + allSequenceVectors.size());

          // FastVector      atts;
          // FastVector      attsRel;
          // FastVector      attVals;
          // FastVector      attValsRel;
          // FastVector      seqIds;
          // FastVector      classIds;
          // Instances       dataRel;
          // Instances       seqData;
          // double[]        vals;//, vals2;
          // double[]        valsRel;
          // //int             i;

          // // 1. set up attributes
          // atts = new FastVector();

          // seqIds = new FastVector();
          // for(int j = 0; j < allSequenceVectors.size(); j++)
          // {
          //   seqIds.addElement("seq_" + j);
          // }
          // atts.addElement(new Attribute("seq-id", seqIds));


          // attsRel = new FastVector();
          // int numberOfAttributes = 19;
          // for(int j = 0; j < numberOfAttributes; ++j)
          // {
          //   attsRel.addElement(new Attribute("output_" + j));
          // }
          // dataRel = new Instances("sequence", attsRel, 0);
          // atts.addElement(new Attribute("sequence", dataRel, 0));

          // classIds = new FastVector();
          // //classIds.addElement("punching");
          // classIds.addElement("punchingprep");
          // //classIds.addElement("kicking");
          // classIds.addElement("kickingprep");
          // //classIds.addElement("shoving");
          // //classIds.addElement("shovingprep");
          // //classIds.addElement("hitting");
          // classIds.addElement("hittingprep");
          // classIds.addElement("pointing");
          // classIds.addElement("pointingandlaughing");
          // //classIds.addElement("tongueout");
          // //classIds.addElement("tongueoutandhandatears");
          // //classIds.addElement("fistthreat");
          // classIds.addElement("nothing");
       
          // atts.addElement(new Attribute("class", classIds));


          // // 2. create Instances object
          // seqData = new Instances("ArbitraryData", atts, 0);

          // for(int j = 0; j < allSequenceVectors.size(); j++)
          // {
          //   ArrayList<double[]> sequenceVectors = allSequenceVectors.get(j);
          //   vals = new double[seqData.numAttributes()];
          //   vals[0] = j;
          //   double[] lastSequence = sequenceVectors.get(sequenceVectors.size()-1);
          //   dataRel = new Instances(seqData.attribute(1).relation(), 0);
          //   for (int k = 0; k < sequenceVectors.size(); k++)
          //   {
          //     double[] sequence = sequenceVectors.get(k);
          //     valsRel = new double[sequence.length-1];
          //     for (int l = 0; l < sequence.length-1; l++)
          //     {
          //       valsRel[l] = sequence[l];
          //     }
          //     dataRel.add(new DenseInstance(1.0, valsRel));
          //   }
          //   vals[1] = seqData.attribute(1).addRelation(dataRel);
          //   vals[2] = lastSequence[lastSequence.length-1];
          //   seqData.add(new DenseInstance(1.0, vals));

          // }

          //write here...



          //System.out.println(allSequenceVectors);

          //System.out.println(data);


          try
          {
           // PrintWriter writer = new PrintWriter("test.arff", "UTF-8");
            PrintWriter writer = new PrintWriter("test_leniency100_aligned_alljointsnormalized.arff", "UTF-8");
            writer.println(data);
            //writer.println(seqData);

            writer.close();
            System.out.println("Finished writing arff");
          }
          catch(Exception e)
          {
            System.out.println("Exception at PrintWriter");
          }

          System.exit(0);
        } 


        //str.setData("Hello world! " + sequenceNumber + " " + value);
        //publisher.publish(str);
        sequenceNumber++;
        Thread.sleep(1);



        //Thread.sleep(1000);
      }
    });
  }
}
