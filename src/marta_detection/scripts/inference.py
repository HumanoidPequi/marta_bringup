import os
import cv2
import time
import rospkg
import numpy as np
import pandas as pd
import onnxruntime as ort
from yolo_utils import nms

os.environ['ORT_TENSORRT_ENGINE_CACHE_ENABLE']='1'
os.environ['ORT_TENSORRT_CACHE_PATH']='/home/marta/.cache/triton-tensorrt'
class ObjectDetection:

    def __init__(self,onnx_path): #os parametros de toda classe são passados no init/construtor
        self.onnx_path = onnx_path
        self.ort_sess = ort.InferenceSession(self.onnx_path,providers=['TensorrtExecutionProvider'])
        self.SCORE_THRESHOLD = 0.3
        self.IOU_THRESHOLD = 0.6
        self.CONF_THRESHOLD = 0.6
        self.class_list = []

        self.img_size = (512, 640)

        self.predictions = []
        self.col_names = ['score', 'cls_id','xmin','ymin','xmax','ymax']
        self.cls_names = {'ball','goalpost','robot','L-Intersection','T-Intersection','X-Intersection'}
        #self.cls_names = {'person'}
    #pega a imagem e redimensiona pro tamanho que o onnx recebe
    def format_yolov8(self, frame): #tirando o init, os demais são métodos/funções da classe
        row, col, _ = frame.shape
        _max = max(col, row)
        result = np.zeros((_max, _max, 3), np.uint8)
        result[0:row, 0:col] = frame
        return result
    #faz a predição/dá as classes/as bounding boxes
    def prediction_onnx(self,image):
        image = self.format_yolov8(image)
        #print('image',image.shape)
        input_img = np.zeros((self.img_size[0],self.img_size[1]))
        input_img = image[:self.img_size[0],:self.img_size[1]]/255.0

        input_img = input_img.transpose(2, 0, 1)
        normalized_image = input_img[np.newaxis, :, :, :].astype(np.float32)
        #normalized_image = cv2.dnn.blobFromImage(image, 1 / 255, self.img_size, swapRB=False)
        #start = time.perf_counter()
        outputs = self.ort_sess.run(None, {'images': normalized_image})
        #print(f"Inference time: {(time.perf_counter() - start)*1000:.2f} ms")
        self.predictions = outputs[0][0]
        return image
    
    #pós processamento que está sendo usado
    def unwrap_detection(self,image):

        yolo_img = self.prediction_onnx(image)    
        results = pd.DataFrame([], columns=self.col_names)

        class_ids = []
        confidences = []
        boxes = []

        rows = self.predictions.shape[0]

        image_width, image_height, _ = yolo_img.shape

        x_factor = image_width / 640
        y_factor =  image_height / 640

        for r in range(rows):
            row = self.predictions[r]
            confidence = row[4]
            if confidence >= self.CONF_THRESHOLD:

                classes_scores = row[5:]
                _, _, _, max_indx = cv2.minMaxLoc(classes_scores)
                class_id = max_indx[1]
                if (classes_scores[class_id] > self.SCORE_THRESHOLD):

                    confidences.append(confidence)

                    class_ids.append(class_id)

                    x, y, w, h = row[0].item(), row[1].item(), row[2].item(), row[3].item() 
                    # left = int((x - 0.5 * w) * x_factor)
                    # top = int((y - 0.5 * h) * y_factor)
                    # width = int(w * x_factor)
                    # height = int(h * y_factor)
                    box = [int(x), int(y),w,h]
                    boxes.append(box)

        #print(boxes)
        indexes = nms(boxes,confidences, self.IOU_THRESHOLD)
        if indexes != []:
            #print(indexes)
            boxes = np.array(boxes)
            boxes = boxes[indexes]
            confidences = np.array(confidences)
            class_ids = np.array(class_ids)
            
            conf = 100*confidences[indexes]
            class_ids = class_ids[indexes]
        else:
            boxes = []
            class_ids = []
            conf = []
        return class_ids, conf, boxes

    #pós processamento - não tá sendo usado
    def predict(self, image):
    
        yolo_img = self.prediction_onnx(image)    
        results = pd.DataFrame([], columns=self.col_names)

        if len(self.predictions)>0:
            output_data = self.predictions
            print(output_data)
            image_height,image_width,_ = yolo_img.shape
            x_factor =   image_width / self.img_size[0]
            y_factor =   image_height / self.img_size[1]

            confidences_nparray = np.amax(output_data[:,4:],axis=1)
            flages = [confidences_nparray> self.SCORE_THRESHOLD]
            pass_data = output_data[tuple(flages)]
            data_class_score = pass_data[:,4:]
            res = np.amax(data_class_score,axis=1,keepdims=True)
            class_ids_np_array =np.argmax(data_class_score,axis=1)
            data_w = pass_data
            f_confidences_nparray = np.amax(data_w[:,4:],axis=1)

            all_boxes = data_w[:,:4]
            xs = (all_boxes[:,0]-(all_boxes[:,2]*0.5))*x_factor
            ys = (all_boxes[:,1]-(all_boxes[:,3]*0.5))*x_factor
            ws = all_boxes[:,2]*x_factor
            hs = all_boxes[:, 3] * y_factor

            boxes_nparray = np.stack((xs, ys,ws,hs), axis=1).astype(np.int64)
            # Non-maximum Suppression (NMS) algorithm to remove overlapping/duplicated detections
            indexes = nms(boxes_nparray, f_confidences_nparray, self.IOU_THRESHOLD)
            #print(indexes)
            #cv2.dnn.NMSBoxes(boxes_nparray, f_confidences_nparray, self.SCORE_THRESHOLD, self.IOU_THRESHOLD)
            boxes = boxes_nparray[indexes]
            
            conf_array = np.int0(100*f_confidences_nparray[indexes,None])
            class_id = class_ids_np_array[indexes,None]
        
        
        return class_id, conf_array, boxes

if __name__=='__main__':

    package_name = 'marta_detection'
    rospack = rospkg.RosPack()
    package_path = rospack.get_path(package_name)


    #onnx_path = '/home/marta/ball_detection/best.onnx'
    #detect = ObjectDetection(onnx_path)

    #while 1:
    #for i in range(100):
        #image = cv2.imread('/home/marta/ball_detection/blur.jpeg')

        #class_ids, confidences, boxes = detect.unwrap_detection(image)
        #print(class_ids,confidences)
        #indexes = nms(np.squeeze(boxes), confidences, detect.IOU_THRESHOLD)
        #print(indexes)
