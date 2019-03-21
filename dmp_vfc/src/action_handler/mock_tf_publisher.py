#!/usr/bin/env python

import requests
from math import pi
import rospy


def main():

    position = '0.2 -0.5 1.2'
    orientation = '0.0 ' + str(-pi/2.0) + ' 0.0'
    try:
        requests.put('http://localhost:5000/actions/Move',
        data={'Position': position, 'Orientation': orientation})
    except Exception as e:
        print "Cannot communicate with server"

if __name__ == "__main__":
    main()
