#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import sys
import rospy
from my_robot.srv import SemanticQuery

def main():
    rospy.init_node("semantic_nav_client", anonymous=True)
    rospy.wait_for_service("semantic_query")
    proxy = rospy.ServiceProxy("semantic_query", SemanticQuery)

    query = " ".join(sys.argv[1:]) if len(sys.argv) > 1 else "room with chair AND meeting"
    resp = proxy(query)

    if resp.success:
        print(f"BEST MATCH: {resp.place_name}")
        print(f"Coordinates: ({resp.x:.2f}, {resp.y:.2f}, yaw={resp.yaw:.2f})")
        print(f"Reason: {resp.reason}")
    else:
        print("No match:", resp.reason)

if __name__ == "__main__":
    main()

