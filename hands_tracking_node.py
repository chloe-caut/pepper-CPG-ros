import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float32MultiArray
import cv2
from cv_bridge import CvBridge
from mediapipe import solutions
from mediapipe.python.solutions import hands_connections
from mediapipe.python.solutions.drawing_utils import DrawingSpec
from mediapipe.python.solutions.hands import HandLandmark
from math import sqrt, sin
import time
import numpy as np
import matplotlib.pyplot as plt
from rclpy.qos import QoSProfile

# Initialize mediapipe hands module
mphands = solutions.hands
mpdrawing = solutions.drawing_utils

# Parameters for Hebbian learning
V0 = 0.01
q0 = 0.01
dt = 0.01
eps = 0.5

# Colors for hand tracking
_RADIUS = 4
_RED = (48, 48, 255)
_GREEN = (48, 255, 48)
_BLUE = (192, 101, 21)
_CYAN = (192, 255, 48)

# Hand landmarks
_PALM_LANDMARKS = (HandLandmark.WRIST, HandLandmark.THUMB_CMC,
                   HandLandmark.INDEX_FINGER_MCP, HandLandmark.MIDDLE_FINGER_MCP,
                   HandLandmark.RING_FINGER_MCP, HandLandmark.PINKY_MCP)
_THUMP_LANDMARKS = (HandLandmark.THUMB_MCP, HandLandmark.THUMB_IP, HandLandmark.THUMB_TIP)
_INDEX_FINGER_LANDMARKS = (HandLandmark.INDEX_FINGER_PIP, HandLandmark.INDEX_FINGER_DIP, HandLandmark.INDEX_FINGER_TIP)
_MIDDLE_FINGER_LANDMARKS = (HandLandmark.MIDDLE_FINGER_PIP, HandLandmark.MIDDLE_FINGER_DIP, HandLandmark.MIDDLE_FINGER_TIP)
_RING_FINGER_LANDMARKS = (HandLandmark.RING_FINGER_PIP, HandLandmark.RING_FINGER_DIP, HandLandmark.RING_FINGER_TIP)
_PINKY_FINGER_LANDMARKS = (HandLandmark.PINKY_PIP, HandLandmark.PINKY_DIP, HandLandmark.PINKY_TIP)
_HAND_LANDMARK_STYLE = {
    _PALM_LANDMARKS: DrawingSpec(color=_RED, thickness=-1, circle_radius=_RADIUS),
    _THUMP_LANDMARKS: DrawingSpec(color=_GREEN, thickness=-1, circle_radius=_RADIUS),
    _INDEX_FINGER_LANDMARKS: DrawingSpec(color=_GREEN, thickness=-1, circle_radius=_RADIUS),
    _MIDDLE_FINGER_LANDMARKS: DrawingSpec(color=_GREEN, thickness=-1, circle_radius=_RADIUS),
    _RING_FINGER_LANDMARKS: DrawingSpec(color=_GREEN, thickness=-1, circle_radius=_RADIUS),
    _PINKY_FINGER_LANDMARKS: DrawingSpec(color=_GREEN, thickness=-1, circle_radius=_RADIUS),
}

_HAND_CONNECTION_STYLE = {
    hands_connections.HAND_PALM_CONNECTIONS: DrawingSpec(color=_BLUE, thickness=5),
    hands_connections.HAND_THUMB_CONNECTIONS: DrawingSpec(color=_CYAN, thickness=4),
    hands_connections.HAND_INDEX_FINGER_CONNECTIONS: DrawingSpec(color=_CYAN, thickness=4),
    hands_connections.HAND_MIDDLE_FINGER_CONNECTIONS: DrawingSpec(color=_CYAN, thickness=4),
    hands_connections.HAND_RING_FINGER_CONNECTIONS: DrawingSpec(color=_CYAN, thickness=4),
    hands_connections.HAND_PINKY_FINGER_CONNECTIONS: DrawingSpec(color=_CYAN, thickness=4),
}

class NeuroneRS:
    pass

def create_NRS(nom, I_inj, w_inj, V, sigmaS, sigmaF, Af, q):
    neurone = NeuroneRS()
    neurone.nom = nom
    neurone.I_inj = I_inj
    neurone.w_inj = w_inj
    neurone.V = V
    neurone.sigmaS = sigmaS
    neurone.sigmaF = sigmaF
    neurone.Af = Af
    neurone.q = q
    neurone.toM = 0.2
    neurone.toS = 2
    return neurone

def Input(t):
    amplitude = 1
    freq = 2
    phase = 0
    I = amplitude * sin(6.28 * freq * t + phase)
    return I

def F(n):
    return n.V - n.Af * np.tanh((n.sigmaF / n.Af) * n.V)

def f_V(n, t):
    return -(F(n) + n.q - n.w_inj * n.I_inj) / n.toM

def f_Q(n, t):
    return (-n.q + n.sigmaS * n.V) / n.toS

def f_sigmaS(n, t):
    dot_sigma = 400
    y = f_V(n, t)
    racine = sqrt(np.float64(y**2 + n.V**2))
    if racine == 0:
        quotient = 0
    elif n.sigmaF < 1 + n.sigmaS:
        quotient = y / racine
        dot_sigma = 2 * eps * n.I_inj * sqrt(n.toM * n.toS) * sqrt(1 + n.sigmaS - n.sigmaF) * quotient
    else:
        dot_sigma = np.clip(dot_sigma, 300, 500)
    return dot_sigma

def update_neuron(n, t):
    n.sigmaS = n.sigmaS + dt * f_sigmaS(n, t)
    n.V = n.V + dt * f_V(n, t)
    n.q = n.q + dt * f_Q(n, t)
    t += dt
    return n.V, n.sigmaS, n.q

def plot(Vs1, Ts1, I_inj1, w_inj, liste_sigma_s):
    fig, axs = plt.subplots(3, 1, figsize=(10, 16))
    plt.suptitle(f'Etude theorique CPGs - 1 neurone - w_inj= {w_inj} - eps = {eps}', fontsize=14, x=0.09, y=1, ha='left')
   
    axs[0].plot(Ts1, I_inj1, '-y', label='I_inj - signal de forcage')
    axs[0].set_xlabel('temps')
    axs[0].set_ylabel('potentiel')
    axs[0].legend(loc='upper right')
   
    axs[1].plot(Ts1, Vs1, '-m', label='Vs - signal sortie')
    axs[1].set_ylabel('potentiel')
    axs[1].legend(loc='upper right')
   
    axs[2].plot(Ts1, liste_sigma_s, 'b-')
    axs[2].set_xlabel('temps')
    axs[2].set_ylabel('sigma S')
   
    plt.tight_layout()
    plt.show()

def get_hand_landmarks_style():
    hand_landmark_style = {}
    for k, v in _HAND_LANDMARK_STYLE.items():
        for landmark in k:
            hand_landmark_style[landmark] = v
    return hand_landmark_style

def get_hand_connections_style():
    hand_connection_style = {}
    for k, v in _HAND_CONNECTION_STYLE.items():
        for connection in k:
            hand_connection_style[connection] = v
    return hand_connection_style

def rescale_frame(frame, percent):
    width = int(frame.shape[1] * percent / 100)
    height = int(frame.shape[0] * percent / 100)
    dim = (width, height)
    return cv2.resize(frame, dim, interpolation=cv2.INTER_AREA)

class HandTrackingNode(Node):
    def __init__(self):
        super().__init__('hands_tracking_node')
        qos_profile = QoSProfile(depth=10)
        self.image_pub = self.create_publisher(Image, '/hand_tracking/image', qos_profile)
        self.neuron_pub = self.create_publisher(Float32MultiArray, '/neuron_data', qos_profile)
        self.bridge = CvBridge()
       
        self.neur1 = create_NRS(nom='RS1', I_inj=0.01, w_inj=0.05, V=0.01, sigmaS=20, sigmaF=1, Af=0.2, q=0.1)
       
        self.list_V = []
        self.list_T = []
        self.list_I_inj = []
        self.list_sigmaS = []
       
        self.hands = mphands.Hands(model_complexity=1, min_detection_confidence=0.5, min_tracking_confidence=0.5)
        self.start_time = time.time()
       
        self.cap = cv2.VideoCapture(0)
       
    def run(self):
        rate = self.create_rate(10)
        while rclpy.ok():
            ret, frame = self.cap.read()
            if not ret:
                break
           
            rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            process_frames = self.hands.process(rgb_frame)
           
            if process_frames.multi_hand_landmarks:
                for hand_landmarks in process_frames.multi_hand_landmarks:
                    mpdrawing.draw_landmarks(
                        frame,
                        hand_landmarks,
                        mphands.HAND_CONNECTIONS,
                        get_hand_landmarks_style(),
                        get_hand_connections_style()
                    )
                   
                    wrist = hand_landmarks.landmark[0]
                    x_pixel = int(wrist.x * frame.shape[1])
                    y_pixel = int(wrist.y * frame.shape[0])
                    cv2.circle(frame, (x_pixel, y_pixel), 6, (200, 0, 200), -1)
                   
                    t = time.time() - self.start_time
                    I_inj = x_pixel
                    self.neur1.I_inj = I_inj
               
                    V, sigmaS, q = update_neuron(self.neur1, t)
                   
                    self.list_V.append(V)
                    self.list_T.append(t)
                    self.list_I_inj.append(I_inj)
                    self.list_sigmaS.append(sigmaS)
                   
                    neuron_data = Float32MultiArray()
                    neuron_data.data = [V, sigmaS, q]
                    self.neuron_pub.publish(neuron_data)
                   
            resized_frame = rescale_frame(frame, percent=120)
            image_msg = self.bridge.cv2_to_imgmsg(resized_frame, 'bgr8')
            self.image_pub.publish(image_msg)
           
            rate.sleep()
       
        self.cap.release()

def main(args=None):
    rclpy.init(args=args)
    node = HandTrackingNode()
    try:
        node.run()
    except rclpy.exceptions.ROSInterruptException:
        pass
    finally:
        if rclpy.ok():
           rclpy.shutdown()

if __name__ == '__main__':
    main()
