import cv2
import numpy as np
import tflite_runtime.interpreter as tflite
import os
import imutils

class CrowDetection():
    def __init__(self, frame):
        # init tflite
        os.environ['USE_GPU_INFERENCE'] = "0"

        ext_delegate_path = '/lib/libvx_delegate.so'
        model_path = 'mobilenet_v1_1.0_224_quant.tflite'
        
        delegate = tflite.load_delegate(ext_delegate_path)

        self.interpreter = tflite.Interpreter(model_path=model_path, 
                                              experimental_delegates=[delegate])

        self.interpreter.allocate_tensors()

        self.CONFIDENCE_THRESHOLD = 0.3

        input_details = self.interpreter.get_input_details()

        image_dimension = np.array([frame.shape[1], frame.shape[0]])
        self.image_dimension = (image_dimension / 255).astype(int)

        # NxHxWxC, H:1, W:2
        #height = input_details[0]['shape'][1]
        self.width = input_details[0]['shape'][2]

        self.tensor_index = input_details[0]['index']

    def detect(self, image):
        image = imutils.resize(image, width=self.width)

        # add N dim
        input_data = np.expand_dims(image, axis=0)

        self.interpreter.set_tensor(self.tensor_index, input_data)
        
        # run model inference
        self.interpreter.invoke()

        # get detections result
        '''
        notice: please export with post processing (NMU) so we 
        don't need to filter the confidence boxes again
        '''

        detect_data = np.concatenate((np.squeeze(self.interpreter.tensor(252)),
                                      np.squeeze(self.interpreter.tensor(254))), 
                                      axis=1)
        '''
        classes = np.squeeze(self.interpreter.tensor(253))
        confidence_scores = np.squeeze(self.interpreter.tensor(254))
        number_of_boxes = np.squeeze(self.interpreter.tensor(255))
        '''

        # filter the confidences
        detect_data = detect_data[detect_data[:, 4] >= self.CONFIDENCE_THRESHOLD]

        number_of_boxes = detect_data.shape[1]
        
        #wh_arr = (detect_data[:][2:4] - detect_data[:][0:2])*self.image_dimension
        xy_arr = detect_data[:][0:2]*self.image_dimension
        
        return number_of_boxes, xy_arr[0], xy_arr[1]