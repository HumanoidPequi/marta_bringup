#!/usr/bin/env python3

import rospy
import cv2

from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError


#!/usr/bin/env python3

import rospy
import cv2

from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError


class ImageProcessor:

    def __init__(self):
        rospy.init_node("image_processor", anonymous=False)

        self.bridge = CvBridge()

        # Parametro do topico de entrada
        self.image_topic = rospy.get_param(
            "~image_topic",
            "/marta/front_camera/image_raw"
        )

        # Parametro do topico de saida
        self.output_topic = rospy.get_param(
            "~output_topic",
            "/marta/front_camera/image_gray"
        )

        # Assinante: recebe as imagens da camera
        self.subscriber = rospy.Subscriber(
            self.image_topic,
            Image,
            self.image_callback,
            queue_size=1
        )

        # Publicador: envia as imagens em tons de cinza
        self.publisher = rospy.Publisher(
            self.output_topic,
            Image,
            queue_size=1
        )

        rospy.loginfo("No de processamento de imagens iniciado.")
        rospy.loginfo("Topico de entrada: %s", self.image_topic)
        rospy.loginfo("Topico de saida: %s", self.output_topic)

    def image_callback(self, msg):
        try:
            # Converte a mensagem ROS para uma imagem OpenCV
            cv_image = self.bridge.imgmsg_to_cv2(
                msg,
                desired_encoding="bgr8"
            )

            # Converte a imagem colorida para tons de cinza
            gray_image = cv2.cvtColor(
                cv_image,
                cv2.COLOR_BGR2GRAY
            )

            # Converte a imagem OpenCV para uma mensagem ROS
            output_msg = self.bridge.cv2_to_imgmsg(
                gray_image,
                encoding="mono8"
            )

            # Preserva o cabecalho da imagem original
            output_msg.header = msg.header

            # Publica a imagem processada
            self.publisher.publish(output_msg)

        except CvBridgeError as error:
            rospy.logerr("Erro ao converter imagem: %s", error)


if __name__ == "__main__":
    try:
        processor = ImageProcessor()
        rospy.spin()

    except rospy.ROSInterruptException:
        pass

