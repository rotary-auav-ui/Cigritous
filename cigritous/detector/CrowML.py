import cv2
import numpy as np
import tflite_runtime.interpreter as tflite
import os
import imutils

class CrowML():
    def __init__(self, model_name, conf_thres, iou_thres):
        # init tflite
        os.environ['USE_GPU_INFERENCE'] = "0"

        #ext_delegate_path = '/lib/libvx_delegate.so'

        dir_path = os.path.dirname(os.path.realpath(__file__))
                
        #delegate = tflite.load_delegate(ext_delegate_path)

        self.interpreter = tflite.Interpreter(model_path=os.path.join(dir_path, model_name))#,
                                              #experimental_delegates=[delegate])

        self.interpreter.allocate_tensors()

        input_details = self.interpreter.get_input_details()
        self.output_details = self.interpreter.get_output_details()

        # NxHxWxC, H:1, W:2
        self.height = input_details[0]['shape'][1]
        self.width = input_details[0]['shape'][2]

        self.tensor_index = input_details[0]['index']

        self.iou_thres = iou_thres
        self.conf_thres = int(conf_thres*255) # quantize

    def detect(self, frame):
        # downscale image to tensor matrix
        frame = cv2.resize(frame, (self.height, self.width))

        # add N dim
        input_data = np.expand_dims(frame, axis=0)

        self.interpreter.set_tensor(self.tensor_index, input_data)
        
        # run model inference
        self.interpreter.invoke()

        # get detections result
        results = self.interpreter.get_tensor(self.output_details[0]['index'])[0]
        results = self.nms_tflite_int8(results)
        number_of_boxes = len(results)
        '''
        results matrix description:
        results[0] = top left x, not multiplied by resolution
        results[1] = top left y, not multiplied by resolution
        results[2] = bottom right x, not multiplied by resolution
        results[3] = bottom right y, not multiplied by resolution
        results[4] = class id
        results[5] = score, in integer8 scale (0-255)
        '''
        return number_of_boxes
    
    def nms_tflite_int8(self, results):
        # remove score below threshold 
        results = results[results[:,5] >= self.conf_thres]
        # sort the score
        results = results[results[:, 5].argsort()]

        # convert tflite xywh to xyxy
        xy1 = np.divide(results[:,:2] - results[:,2:4], 2)
        xy2 = np.divide(results[:,:2] + results[:,2:4], 2)

        results[:,:2] = xy1
        results[:,2:4] = xy2

        areas = results[:,2] * results[:,3]

        indices = np.arange(len(xy1))

        for i, box in enumerate(results):
            temp_indices = indices[indices != i]
            xx1 = np.maximum(box[0], results[temp_indices,0])
            yy1 = np.maximum(box[1], results[temp_indices,1])
            xx2 = np.minimum(box[2], results[temp_indices,2])
            yy2 = np.minimum(box[3], results[temp_indices,3])
            w = np.maximum(xx2 - xx1, 0)
            h = np.maximum(yy2 - yy1, 0)
            areas[areas == 0] = 1 # avoid division by zero error
            # compute the ratio of overlap
            overlap = np.divide(w * h, areas[temp_indices])

            if np.any(overlap) > self.iou_thres:
                indices = indices[indices != i]
        
        return results[indices].astype(int)
