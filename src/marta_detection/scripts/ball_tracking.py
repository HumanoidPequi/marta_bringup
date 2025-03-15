#!/usr/bin/env python3
import numpy as np
import cv2 as cv
from inference import ObjectDetection
import os
import time
import rospy
import yaml
import warnings
warnings.filterwarnings("ignore")
from sensor_msgs.msg import Image
import rospkg
from cv_bridge import CvBridge
from std_msgs.msg import Int16MultiArray, Float32, Float32MultiArray

#parametros do onnx para criar o cache do tensorrt
os.environ['ORT_TENSORRT_ENGINE_CACHE_ENABLE']='1'
os.environ['ORT_TENSORRT_CACHE_PATH']='/home/marta/.cache/triton-tensorrt'

bridge = CvBridge() #Convert your ROS Image message to OpenCV2

package_name = 'marta_detection'
rospack = rospkg.RosPack()
package_path = rospack.get_path(package_name)

class Ball_tracking():

    def __init__(self) -> None:

        with open(os.path.join(package_path, 'params/params.yaml')) as f:
            cam_params = yaml.load(f, Loader=yaml.FullLoader)
        self.intrinsic = np.array(cam_params['mtx'])

        self.fx = self.intrinsic[0,0]
        self.fy = self.intrinsic[1,1]
        self.cx = 320
        self.cy = 240
        self.mask = np.zeros((480,640,3)).astype('uint8') #mask to draw the optical flow
        self.theta_z =   0 #yaw
        self.theta_y = -50 #pitch
        self.h_cam = 62 #(cm)
        self.euler = np.identity(3) #rotation matrix
        self.draw_optical = True #draw the optical flow

        rospy.init_node('ball_tracking_angles', anonymous=False)

        self.pub_ball_pos = rospy.Publisher('/ball_pose', Float32MultiArray,queue_size=1) #publish the ball position
        self.pub_freq = rospy.Publisher('frequency',Float32, queue_size = 1)
        self.pub_image = rospy.Publisher('image', Image,queue_size = 1)
        self.pub_optical_flow = rospy.Publisher('flux_opt', Float32MultiArray, queue_size = 1)
       
        self.rate = rospy.Rate(5) #taxa de publicação com a qual o código vai rodar
        self.angles = Int16MultiArray()
        self.freq = Float32()
        self.opt_flux = Float32MultiArray()
        
        self.lk_params = dict( winSize  = (15,15), #tamanho da região que ele vai usar para calcular a derivada
                        maxLevel = 7, #número de níveis da pirâmide
                        criteria = (cv.TERM_CRITERIA_EPS | cv.TERM_CRITERIA_COUNT, 10, 0.03)) #critérios de parada
        
        self.onnx_path = os.path.join(package_path, 'models/nano_480.onnx')
        self.detect = ObjectDetection(self.onnx_path)
        self.i = 0 #contador pra monitorar quantas vezes vai rodar a yolo e o optical flow
        self.yolo = True #se a yolo vai rodar ou não, se ela for False vai rodar o optical flow
        self.ball_position = Float32MultiArray()
        self.img_msg = Image() #instancia o objeto imagem do ros
        self.last_p0 = None

        self.latest_image = None

        rospy.Subscriber("/marta/front_camera/image_raw", Image, self.image_callback, queue_size=1, buff_size=2**32)

    def image_callback(self, msg):
        #image = bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        self.latest_image = bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")


    def points(self,p0): #pega o ponto central do bounding box e cria uma matriz de pontos em volta dele com um quadrado 60x60. nao sao aleatorios
        yi = int(p0[1])-30 #esse p0 é o ponto central do bounding box, detectado pela yolo, em x e y
        xi = int(p0[0])-30
        points = []
        for i in range(yi,yi + 60):
            for j in range(xi,xi+60):
                points.append([j,i])
        return np.array(points).reshape(len(points),1,2).astype('float32')
    
    def image_received(self, image): #função que vai rodar toda vez que tiver uma imagem nova. quando chegar a imagem, o callback sera executado

        if self.yolo == True:
            start = time.perf_counter() #começa a contar o tempo de execução da yolo

            class_ids, confidences, boxes = self.detect.unwrap_detection(image)

            if boxes != []:
                p0 = boxes[np.array(confidences).argmax()]
                #print(p0)
                
                xx, yy, _, _ = p0 
                self.last_p0 = p0

                self.ball_position.data = [xx,yy] #pega os dados da posicao da bola
                rospy.loginfo(f"Ball position: {self.ball_position.data}")

                self.pub_ball_pos.publish(self.ball_position) #e publica essa posicao num topico do ros
                self.i = self.i + 1
                if self.i == 2: 

                    #ultimo frame q ele fez a deteccao
                    self.old_gray = cv.cvtColor(image, cv.COLOR_BGR2GRAY) #ta armazenando o frame anterior pra usar no optical flow
                    #self.good_old = np.array([xx,yy]).reshape(1,1,2).astype('float32')
                    self.good_old = self.points(np.array([xx,yy])) #pega a regiao em volta da ultima coordenada da bola da yolo
                    self.yolo = False


            else: #quando nao tem deteccao, ele olha pra baixo e pro meio
                xx = 1000
                yy = 1000

                self.ball_position.data = [xx,yy]
                self.pub_ball_pos.publish(self.ball_position)

                # Convert the image to ROS format
                img_msg = bridge.cv2_to_imgmsg(self.latest_image, "bgr8")
                
                # Publish new image
                self.pub_image.publish(img_msg)

            end = (time.perf_counter() - start)

            self.freq.data = 1/end
            self.pub_freq.publish(self.freq)
            
        else: #se a yolo for false, ele vai rodar o optical flow
            # Create a mask image for drawing purposes
                
            start = time.perf_counter()
            frame_gray = cv.cvtColor(image, cv.COLOR_BGR2GRAY) #imagem/frame atual
            # calculate optical flow
            p1, _, _ = cv.calcOpticalFlowPyrLK(self.old_gray, frame_gray, self.good_old, None, **self.lk_params)

            self.good_new = np.mean(np.array(p1),axis=0).reshape(1,1,2).astype('float32') #media das coordenadas dos pontos da janela, pra me dar so uma posicao

            end = (time.perf_counter() - start)

            self.freq.data = 1/end
            self.pub_freq.publish(self.freq)

            if self.good_new.any(): #se tiver alguma ponto
                   
            # if (xx != cx) or (yy!= cy):
                self.ball_position.data = [self.good_new[0,0,0],self.good_new[0,0,1]]
                #rospy.loginfo(f"Ball position when blablabla: {self.ball_position.data}")
                #self.pub_ball_pos.publish(self.ball_position) #publica a posicao da bola no frame atual

                if self.draw_optical == True:
                    #se draw_optical for true, ele vai desenhar o optical flow
                    color = [[0,0,255]]
                    a,b = None, None
                    for i, (new, old) in enumerate(zip(self.good_new, self.good_old)):
                        a, b = new.ravel()
                        c, d = old.ravel()

                        _, ang = cv.cartToPolar(new, old)
                        angulo = Float32MultiArray()

                        ang_aux = ang[0] * 180 / np.pi

                        angulo.data = ang_aux

                        self.pub_optical_flow.publish(angulo) #dreca was here

                        self.mask = cv.line(self.mask, (int(a), int(b)), (int(c), int(d)), color[i], 2)
                        image = cv.circle(image, (int(a), int(b)), 5, color[i], -1)

                    # Putting the bounding box 
                    _, _, w, h = self.last_p0
                    cv.rectangle(self.latest_image, (int(a-w/2), int(b-h/2)), (int(a+w/2), int(b+h/2)), (0, 255, 0), 2)
                    cv.putText(self.latest_image, "Ball", (int(a), int(b) - 10), cv.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
                    
                    # Convert the image to ROS format
                    img_msg = bridge.cv2_to_imgmsg(self.latest_image, "bgr8")
                    
                    # Publish new image
                    self.pub_image.publish(img_msg)

            self.i = self.i +1
            #if (self.good_old[0,0,0]-self.good_new[0,0,0] >= self.tolerance) and (self.good_old[0,0,1]-self.good_new[0,0,1] >= self.tolerance):

            if self.i == 7 :
                self.yolo = True
                self.i = 0

            else: #vai fazendo o optical flow ate dar i=7
                self.old_gray = frame_gray.copy()
                self.good_old = self.points([self.good_new[0,0,0],self.good_new[0,0,1]]) #self.good_new
    
        return

if __name__=='__main__':
    bt = Ball_tracking()

    while not rospy.is_shutdown(): #enquanto o roscore estiver rodando, ele vai ficar subscrevendo/recebendo a imagem do topíco da camera e publicando a posicao da bola no topico ball_position
        if bt.latest_image is None:
            rospy.logwarn("No images received yet.")
            rospy.Rate(1.0).sleep()
            continue

        bt.image_received(bt.latest_image)
