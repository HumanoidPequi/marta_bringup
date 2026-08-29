#!/usr/bin/env python3
from __future__ import print_function
import sys
from amanda_turtlesim_tutorial.srv import diferenca
import rospy

def diferenca_cliente(x, y):
    rospy.wait_for_service('diferenca')
    try:
        cliente = rospy.ServiceProxy('diferenca', diferenca)
        res = cliente(x, y)
        return res.sum
    except rospy.ServiceException as e:
        print("Service call failed: %s" % e)

def erro():
    return "O serviço precisa de dois parâmetros"

if __name__ == "__main__":
    if len(sys.argv) == 3:
        x = int(sys.argv[1])
        y = int(sys.argv[2])
    else:
        print(erro())
        sys.exit(1)

    print("%s - %s = %s" % (x, y, diferenca_cliente(x, y)))

