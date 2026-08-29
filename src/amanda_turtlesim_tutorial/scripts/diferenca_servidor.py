#!/usr/bin/env python3
from __future__ import print_function
from amanda_turtlesim_tutorial.srv import diferenca, diferencaResponse
import rospy 

def calcular(req):
	dif = req.a - req.b
	print("%s - %s = %s" % (req.a, req.b, dif))
	return diferencaResponse(dif)

def diferenca_server():
	rospy.init_node("diferenca_servidor")
	dif = rospy.Service('diferenca', diferenca , calcular)
	rospy.spin()

if __name__ == "__main__":
    diferenca_server()
