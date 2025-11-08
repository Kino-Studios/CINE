from rclpy.qos import QosProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy 

class ROSParams:
    QOS_PROFILE = QosProfile(
        reliability = ReliabilityPolicy.RELIABLE,
        durability = DurabilityPolicy.VOLATILE,
        history = HistoryPolicy.KEEP_LAST,
        depth = 10
    )

class UserRequestedParams:
    FOCUS_OBJECT_ID = 2
    THIRDS_ANCHOR = [2 / 3.0, 1 / 3.0] # width = THIRDS_ANCHOR[0], height = THIRDS_ANCHOR[1]

    DESIRED_FOCUS_DISTANCE_M = 1.5
    FOCUS_DISTANCE_MIN_M = 1

class ImageParams:
    IMAGE_WIDTH = 1152
    IMAGE_HEIGHT = 648