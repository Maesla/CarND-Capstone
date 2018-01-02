from styx_msgs.msg import TrafficLight
import rospkg
import os
import sys
import tensorflow as tf
import numpy as np
from functools import partial

THRESHOLD = 0.50


def load_graph(frozen_graph_filename):
    # We load the protobuf file from the disk and parse it to retrieve the 
    # unserialized graph_def
    with tf.gfile.GFile(frozen_graph_filename, "rb") as f:
        graph_def = tf.GraphDef()
        graph_def.ParseFromString(f.read())

    # Then, we import the graph_def into a new Graph and returns it 
    with tf.Graph().as_default() as graph:
        # The name var will prefix every op/nodes in your graph
        # Since we load everything in a new graph, this is not needed
        tf.import_graph_def(graph_def, name="prefix")
    return graph


class TLClassifier(object):
    def __init__(self, model_path):
        self.tf_session = None
        self.predict = None
        self.clabels = [4, 2, 0, 1, 4, 4]
        self.model_path = model_path

    def get_classification(self, image_np):
        """Determines the color of the traffic light in the image
        Args:
            image_np (cv::Mat): image containing the traffic light
        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)
        """
        # Tensorflow Model Object Detection API classifier.
		# See: https://github.com/tensorflow/models/blob/master/research/object_detection/object_detection_tutorial.ipynb
        # Transfer learning with SSD Mobile Net + Coco data
		# Trained on data from: https://drive.google.com/file/d/0B-Eiyn-CUQtxdUZWMkFfQzdObUE/view
        # we thank Anthony S. for the annotations to the bag data made available through link in slack channel for this project
        # using these feature set sepcs:
        # https://github.com/tensorflow/models/blob/master/research/object_detection/g3doc/using_your_own_dataset.md
        
	ros_root = rospkg.get_ros_root()
        r = rospkg.RosPack()
        path = r.get_path('tl_detector')
        
        # set up tensorflow and traffic light classifier
        if self.tf_session is None:
            # get the traffic light classifier
            self.config = tf.ConfigProto(log_device_placement=True)
            self.config.gpu_options.per_process_gpu_memory_fraction = 0.5  # don't hog all the VRAM!
            self.config.operation_timeout_in_ms = 50000 # terminate anything that don't return in 50 seconds
            self.tf_graph = load_graph(path + self.model_path + '.pb')


            with self.tf_graph.as_default():
                self.tf_session = tf.Session(graph=self.tf_graph, config=self.config)
                # Definite input and output Tensors for self.tf_graph
                self.image_tensor = self.tf_graph.get_tensor_by_name('prefix/image_tensor:0')
                # Each score represent how level of confidence for each of the objects.
                # Score is shown on the result image, together with the class label.
                self.detection_scores = self.tf_graph.get_tensor_by_name('prefix/detection_scores:0')
                self.detection_classes = self.tf_graph.get_tensor_by_name('prefix/detection_classes:0')
                self.num_detections = self.tf_graph.get_tensor_by_name('prefix/num_detections:0')
                self.predict = True

        predict = TrafficLight.UNKNOWN
        if self.predict is not None:
            # Expand dimensions since the model expects images to have shape: [1, None, None, 3]
            image_np_expanded = np.expand_dims(image_np, axis=0)

            # Actual detection
            (scores, classes, num) = self.tf_session.run(
                [self.detection_scores, self.detection_classes, self.num_detections],
                feed_dict={self.image_tensor: image_np_expanded})

            # Visualization of the results of a detection.
            scores = np.squeeze(scores)
            classes = np.squeeze(classes).astype(np.int32)

            # calculate prediction
            c = 5
            predict = self.clabels[c]
            cc = classes[0]
            confidence = scores[0]
            if cc > 0 and cc < 4 and confidence is not None and confidence > THRESHOLD:
                c = cc
                predict = self.clabels[c]

        return predict
