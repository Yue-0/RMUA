import time
from os import path

import tf
import cv2
import torch
import rospy
import numpy as np
import pyrealsense2 as rs
from cv_bridge import CvBridge
from torchvision.ops import nms
from onnxruntime import InferenceSession as OnnxModel

import sentry.srv as sentry
from sensor_msgs.msg import Image
from roborts_msgs.msg import GimbalAngle
from roborts_msgs.srv import ShootCmd, FricWhl

__author__ = "YueLin"

RED, BLUE = tuple(range(2))
V, Q, DQ, DT, DIS = 20, 240, 12, 0.1, 4
ONNX = path.join(path.split(path.split(__file__)[0])[0], "onnx")


class D455:
    def __init__(self, width: int = 640, height: int = 480, fps: int = 30):
        self.fps = fps
        self.camera, cfg = rs.pipeline(), rs.config()
        cfg.enable_stream(rs.stream.depth, width, height, rs.format.z16, fps)
        cfg.enable_stream(rs.stream.color, width, height, rs.format.bgr8, fps)
        self.align = rs.align(rs.stream.color)
        self.camera.start(cfg)
    
    def read(self) -> list:
        frames = self.align.process(self.camera.wait_for_frames())
        return [np.asarray(frame.get_data()) for frame in (
            frames.get_color_frame(), frames.get_depth_frame()
        )]
    
    def close(self) -> None:
        self.camera.stop()


class Detector:
    def __init__(self, model: str, device: str = None, size: int = 320):
        self.size = size
        if device is None:
            device = "CUDA" if torch.cuda.is_available() else "CPU"
        self.model = OnnxModel(
            path.join(ONNX, "{}.onnx".format(model)),
            providers=["{}ExecutionProvider".format(device.upper())]
        )

    def __call__(self, image: np.ndarray, threshold: float = 0.5) -> tuple:
        size = image.shape[:2]
        image, padding = self.preprocess(image)
        result = self.inference(image[np.newaxis])
        result = self.post_process(result, threshold, size, padding)
        return tuple(map(lambda tensor: tensor.numpy(), result))

    def resize(self, image: np.ndarray, pad: int = 114) -> tuple:
        h, w, _ = image.shape
        if max(h, w) < self.size:
            size = None
            width, height = (self.size - w) >> 1, (self.size - h) >> 1
        elif h > w:
            size = (round(self.size * w / h), self.size)
            width, height = (self.size - size[0]) >> 1, 0
        else:
            size = (self.size, round(self.size * h / w))
            height, width = (self.size - size[1]) >> 1, 0
        if size is not None:
            image = cv2.resize(image, size)
        return cv2.copyMakeBorder(
            image, height, height, width, width,
            cv2.BORDER_CONSTANT, value=(pad,) * 3
        ), (height, width)

    def inference(self, inputs: np.ndarray) -> torch.Tensor:
        return torch.tensor(self.model.run(None, {"images": inputs})[0])

    def preprocess(self, image: np.ndarray) -> tuple:
        image, pad = self.resize(image)
        image = image.transpose((2, 0, 1))
        return np.float32(image) / 255, pad

    def post_process(self,
                     outputs: torch.Tensor,
                     threshold: float,
                     size: tuple,
                     padding: tuple) -> tuple:
        outputs = outputs[outputs[..., 4] > threshold]
        classes = torch.argmax(outputs[:, 5:], 1)
        outputs[:, 5:] *= outputs[:, 4:5]
        scores = torch.tensor([
            outputs[obj, classes[obj] + 5] for obj in range(outputs.shape[0])
        ])
        keep = scores >= threshold
        outputs, classes, scores = outputs[keep], classes[keep], scores[keep]
        keep = nms(self.xyxy(outputs[:, :4]), scores, 0.1)
        bboxes = outputs[keep, :4]
        bboxes[:, 0] -= padding[1]
        bboxes[:, 1] -= padding[0]
        bboxes[:, ::2] *= size[1] / (self.size - padding[1] * 2)
        bboxes[:, 1::2] *= size[0] / (self.size - padding[0] * 2)
        return bboxes.int(), classes[keep], scores[keep]

    @staticmethod
    def xyxy(x0y0wh: torch.Tensor) -> torch.Tensor:
        xyxy = torch.zeros_like(x0y0wh)
        xyxy[:, 0] = x0y0wh[:, 0] - x0y0wh[:, -2]
        xyxy[:, 1] = x0y0wh[:, 1] - x0y0wh[:, -1]
        xyxy[:, 2] = x0y0wh[:, 0] + x0y0wh[:, -2]
        xyxy[:, 3] = x0y0wh[:, 1] + x0y0wh[:, -1]
        return xyxy


class Gimbal:
    def __init__(self):
        self.dr = 5e-2        # Maximum rad of gimbal rotation
        self.pitch = 0        # The pitch angle of the gimbal
        self.enemy = 0j       # The last detected enemy position
        self.camera = D455()  # The Intel RealSence D455 camera
        self.temperature = 0  # The heat of the barrel, cannot exceed Q
        rospy.init_node("gimbal")
        self.dt = 1 / self.camera.fps
        self.tf = tf.TransformListener()
        self.img2msg = CvBridge().cv2_to_imgmsg
        self.detector = Detector("YOLOv6", "cuda")
        self.shoot = rospy.ServiceProxy("cmd_shoot", ShootCmd)
        self.wheel = rospy.ServiceProxy("cmd_fric_wheel", FricWhl)
        for service in ("cmd_shoot", "cmd_fric_wheel", "robot_id"):
            rospy.wait_for_service(service)
        self.show = rospy.Publisher(
            "frame", Image, queue_size=1
        )
        self.gimbal = rospy.Publisher(
            "cmd_gimbal_angle", GimbalAngle, queue_size=1
        )
    
    def run(self, color: int, bullet: int = 1) -> None:
        angle = GimbalAngle()
        image, depth = self.camera.read()
        enemy = self.detect(image, color)
        angle.yaw_mode = angle.pitch_mode = True
        _, self.pitch, yaw = tf.transformations.euler_from_quaternion(
            self.tf.lookupTransform(
                "base_link", "gimbal", rospy.Time()
            )[-1]
        )
        if enemy:
            x, y, w, h = enemy
            cv2.rectangle(
                image, (x - w // 2, y - h // 2),
                (x + w // 2, y + h // 2), (0, 255, 0)
            )
            d, angle.pitch_angle, angle.yaw_angle = self.angle(depth, enemy)
            self.gimbal.publish(angle)
            self.enemy = w + 0j  # imag: The number of times no enemy detected
            if d > DIS:
                self.enemy += 1j
        else:
            if self.enemy.real:
                self.enemy += 1j
                if self.enemy.imag >= self.camera.fps >> 2:
                    self.enemy *= 0
            else:
                angle.pitch_angle = angle.yaw_angle = 0
                self.gimbal.publish(angle)
        if not self.enemy:
            angle.yaw_angle = min(abs(yaw), self.dr)
            angle.pitch_angle = -np.sign(self.pitch)
            angle.pitch_angle *= min(abs(self.pitch), self.dr)
            angle.yaw_angle *= 0 if abs(yaw) < np.deg2rad(1) else -np.sign(yaw)
            self.gimbal.publish(angle)
        if self.enemy.real and self.enemy.imag < 1 and self.temperature + V < Q:
            self.temperature += V
            self.shoot(True, bullet)
        self.show.publish(self.img2msg(image, "bgr8"))

    def start(self) -> None:
        self.wheel(True)
        start = clock = 0
        color = RED if rospy.ServiceProxy(
            "robot_id", sentry.RobotID
        ).call(
            sentry.RobotIDRequest(False, True)
        ).id == BLUE else BLUE
        while not rospy.is_shutdown():
            t = time.time()
            self.run(color)
            time.sleep(max(t + self.dt - time.time(), 0))
            clock += time.time() - t
            if cool := (clock - start) // DT:
                start += DT * cool
                clock -= DT * cool
                self.temperature = max(0, self.temperature - cool * DQ)
        self.camera.close()

    def angle(self, depth: np.ndarray, coordinate: list) -> tuple:
        x, y, w, h = coordinate
        mid = depth.shape[1] >> 1
        yaw = np.tanh((mid - x) / (mid >> 2))
        yaw *= self.dr if abs(mid - x) > w >> 4 else 0
        depth = depth[
            y - h // 3:y + h // 3, x - w // 3:x + w // 3
        ] * 1e-3
        depth = np.mean(depth[depth != 0])
        a, b = -249.86 + 9.8 * depth ** 2, -2500 * depth
        c, d = 499.72 + 19.6 * depth ** 2, 2.342 * depth
        try:
            pitch = np.arctan(np.roots([a + d, b, c, -b, a - d])[-1]) * 2
        except np.linalg.LinAlgError:
            pitch = self.pitch
        pitch -= self.pitch
        pitch = np.sign(pitch) * min(abs(pitch), self.dr)
        return depth, pitch, yaw

    def detect(self, image: np.ndarray, color: int, score: float = 0.5) -> list:
        image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        bboxes, colors, scores = self.detector(image)
        keep = np.logical_and(
            scores > score, colors == color, bboxes[:, 2] <= bboxes[:, -1] * 3
        )
        bboxes, scores = bboxes[keep], scores[keep]
        if bboxes.shape[0]:
            if not self.enemy.real:
                return bboxes[np.argmax(scores)].tolist()
            return bboxes[np.argmin(np.abs(
                self.enemy.real - bboxes[:, 2]
            ))].tolist()
        return list()


Gimbal().start()
