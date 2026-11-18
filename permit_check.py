"""Simulated permit decisions for the Lot 10 prototype."""
import json
import math

from make_world import PARKED_CARS

DEMO_VIOLATIONS = {"7CDE482", "7EDF619"}
PERMITTED_PLATES = ({plate for plate, *_ in PARKED_CARS} - DEMO_VIOLATIONS) | {"1101", "1103"}
REVIEW_THRESHOLD = 0.75


def classify(payload):
    try:
        read = json.loads(payload)
        plate = read["plate"].strip().upper()
        confidence = float(read["confidence"])
        if not plate or not math.isfinite(confidence) or not 0 <= confidence <= 1:
            raise ValueError("invalid observation")
    except (ValueError, TypeError, KeyError, AttributeError, OverflowError):
        return {"status": "review", "reason": "invalid_observation"}

    if confidence <= REVIEW_THRESHOLD:
        status = "review"
    elif plate in PERMITTED_PLATES:
        status = "permitted"
    else:
        status = "flagged_candidate"
    return {"plate": plate, "confidence": confidence, "status": status}


def main():
    import rclpy
    from rclpy.node import Node
    from std_msgs.msg import String

    rclpy.init()
    node = Node("sim_permit_check")
    publisher = node.create_publisher(String, "/permit/result", 10)

    def receive(message):
        publisher.publish(String(data=json.dumps(classify(message.data))))

    node.create_subscription(String, "/plate/read", receive, 10)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
