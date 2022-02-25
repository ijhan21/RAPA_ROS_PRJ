# YOLOv5 🚀 by Ultralytics, GPL-3.0 license
"""
Run inference on images, videos, directories, streams, etc.

Usage - sources:
    $ python path/to/detect.py --weights yolov5s.pt --source 0              # webcam
                                                             img.jpg        # image
                                                             vid.mp4        # video
                                                             path/          # directory
                                                             path/*.jpg     # glob
                                                             'https://youtu.be/Zgi9g1ksQHc'  # YouTube
                                                             'rtsp://example.com/media.mp4'  # RTSP, RTMP, HTTP stream

Usage - formats:
    $ python path/to/detect.py --weights yolov5s.pt                 # PyTorch
                                         yolov5s.torchscript        # TorchScript
                                         yolov5s.onnx               # ONNX Runtime or OpenCV DNN with --dnn
                                         yolov5s.xml                # OpenVINO
                                         yolov5s.engine             # TensorRT
                                         yolov5s.mlmodel            # CoreML (MacOS-only)
                                         yolov5s_saved_model        # TensorFlow SavedModel
                                         yolov5s.pb                 # TensorFlow GraphDef
                                         yolov5s.tflite             # TensorFlow Lite
                                         yolov5s_edgetpu.tflite     # TensorFlow Edge TPU
"""

import argparse
import os
import sys
from pathlib import Path
import time
import cv2
import torch
import torch.backends.cudnn as cudnn
import numpy as np

FILE = Path(__file__).resolve()
ROOT = FILE.parents[0]  # YOLOv5 root directory
if str(ROOT) not in sys.path:
    sys.path.append(str(ROOT))  # add ROOT to PATH
ROOT = Path(os.path.relpath(ROOT, Path.cwd()))  # relative

from models.common import DetectMultiBackend
from utils.datasets import IMG_FORMATS, VID_FORMATS, LoadImages, LoadStreams
from utils.general import LOGGER, check_file, check_img_size, check_imshow, check_requirements, colorstr, increment_path, non_max_suppression, print_args, scale_coords, strip_optimizer, xyxy2xywh
from utils.plots import Annotator, colors, save_one_box
from utils.torch_utils import select_device, time_sync
import utils

import rclpy
from sensor_msgs.msg import Image, CompressedImage
from std_msgs.msg import String
from geometry_msgs.msg import Twist
import cv2
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

model = None
device = 0
@torch.no_grad()
# def run(weights=ROOT / 'yolo_turtlebot_exp_best.pt',  # model.pt path(s)
def run(weights=ROOT / 'yolov5s.pt',  # model.pt path(s)
        source=ROOT / 'data/images',  # file/dir/URL/glob, 0 for webcam
        data=ROOT / 'data/coco128.yaml',  # dataset.yaml path
        imgsz=(640, 640),  # inference size (height, width)
        conf_thres=0.25,  # confidence threshold
        iou_thres=0.45,  # NMS IOU threshold
        max_det=1000,  # maximum detections per image
        device1='',  # cuda device, i.e. 0 or 0,1,2,3 or cpu
        view_img=False,  # show results
        save_txt=False,  # save results to *.txt
        save_conf=False,  # save confidences in --save-txt labels
        save_crop=False,  # save cropped prediction boxes
        nosave=False,  # do not save images/videos
        classes=0,  # filter by class: --class 0, or --class 0 2 3
        agnostic_nms=False,  # class-agnostic NMS
        augment=False,  # augmented inference
        visualize=False,  # visualize features
        update=False,  # update all models
        project=ROOT / 'runs/detect',  # save results to project/name
        name='exp',  # save results to project/name
        exist_ok=True,  # existing project/name ok, do not increment
        line_thickness=3,  # bounding box thickness (pixels)
        hide_labels=False,  # hide labels
        hide_conf=False,  # hide confidences
        half=False,  # use FP16 half-precision inference
        dnn=False,  # use OpenCV DNN for ONNX inference
        frame=None
        ):
    source = str(source)
    save_img = not nosave and not source.endswith('.txt')  # save inference images
    # is_file = Path(source).suffix[1:] in (IMG_FORMATS + VID_FORMATS)
    # is_url = source.lower().startswith(('rtsp://', 'rtmp://', 'http://', 'https://'))
    webcam = source.isnumeric() #or source.endswith('.txt') or (is_url and not is_file)
    # if is_url and is_file:
    #     source = check_file(source)  # download

    # Directories
    save_dir = increment_path(Path(project) / name, exist_ok=exist_ok)  # increment run
    (save_dir / 'labels' if save_txt else save_dir).mkdir(parents=True, exist_ok=True)  # make dir
    global model, device
    # # Load model
    if model is None:
        device = select_device(device)
        model = DetectMultiBackend(weights, device=device, dnn=dnn, data=data)
    stride, names, pt, jit, onnx, engine = model.stride, model.names, model.pt, model.jit, model.onnx, model.engine
    # imgsz = check_img_size(imgsz, s=stride)  # check image size

    # # Half
    # half &= (pt or jit or onnx or engine) and device.type != 'cpu'  # FP16 supported on limited backends with CUDA
    # if pt or jit:
    #     model.model.half() if half else model.model.float()

    # Dataloader
    # view_img=True
    """
    if webcam:
        view_img = check_imshow()
        cudnn.benchmark = True  # set True to speed up constant image size inference
        dataset = LoadStreams(source, img_size=imgsz, stride=stride, auto=pt)
        bs = len(dataset)  # batch_size
    else:
        dataset = LoadImages(source, img_size=imgsz, stride=stride, auto=pt)
        bs = 1  # batch_size
    """
    bs =1
    vid_path, vid_writer = [None] * bs, [None] * bs

    # Run inference
    model.warmup(imgsz=(1 if pt else bs, 3, *imgsz), half=half)  # warmup
    dt, seen = [0.0, 0.0, 0.0], 0
    # for path, im, im0s, vid_cap, s in dataset:
    frame0=frame.copy()
    img = frame0[np.newaxis,::]
    img = img[..., ::-1].transpose((0, 3, 1, 2))  # BGR to RGB, BHWC to BCHW
    img = np.ascontiguousarray(img)
    dataset=[("", img, frame0, None, "")]
    for path, im, im0s, vid_cap, s in dataset:
        t1 = time_sync()
        im = torch.from_numpy(im).to(device)
        im = im.half() if half else im.float()  # uint8 to fp16/32
        im /= 255  # 0 - 255 to 0.0 - 1.0
        if len(im.shape) == 3:
            im = im[None]  # expand for batch dim
        t2 = time_sync()
        dt[0] += t2 - t1

        # Inference
        visualize = increment_path(save_dir / Path(path).stem, mkdir=True) if visualize else False
        pred = model(im, augment=augment, visualize=visualize)
        t3 = time_sync()
        dt[1] += t3 - t2

        # NMS
        pred = non_max_suppression(pred, conf_thres, iou_thres, classes, agnostic_nms, max_det=max_det)
        dt[2] += time_sync() - t3

        # Second-stage classifier (optional)
        # pred = utils.general.apply_classifier(pred, classifier_model, im, im0s)
        p=path
        im0=im0s
        ims=im
        # Process predictions
        for i, det in enumerate(pred):  # per image
            seen += 1
            # if webcam:  # batch_size >= 1
            #     p, im0, frame = path[i], im0s[i].copy(), dataset.count
            #     s += f'{i}: '
            # else:
            #     p, im0, frame = path, im0s.copy(), getattr(dataset, 'frame', 0)

            p = Path(p)  # to Path
            save_path = str(save_dir / p.name)  # im.jpg
            # txt_path = str(save_dir / 'labels' / p.stem) + ('' if dataset.mode == 'image' else f'_{frame}')  # im.txt
            s += '%gx%g ' % im.shape[2:]  # print string
            gn = torch.tensor(im0.shape)[[1, 0, 1, 0]]  # normalization gain whwh
            imc = im0.copy() if save_crop else im0  # for save_crop
            annotator = Annotator(im0, line_width=line_thickness, example=str(names))
            if len(det):
                # Rescale boxes from img_size to im0 size
                det[:, :4] = scale_coords(im.shape[2:], det[:, :4], im0.shape).round()

                # Print results
                for c in det[:, -1].unique():
                    n = (det[:, -1] == c).sum()  # detections per class
                    s += f"{n} {names[int(c)]}{'s' * (n > 1)}, "  # add to string
            
                # Write results
                for *xyxy, conf, cls in reversed(det):
                    if save_txt:  # Write to file
                        xywh = (xyxy2xywh(torch.tensor(xyxy).view(1, 4)) / gn).view(-1).tolist()  # normalized xywh
                        line = (cls, *xywh, conf) if save_conf else (cls, *xywh)  # label format
                        # with open(txt_path + '.txt', 'a') as f:
                        #     f.write(('%g ' * len(line)).rstrip() % line + '\n')

                    if save_img or save_crop or view_img:  # Add bbox to image
                        c = int(cls)  # integer class
                        label = None if hide_labels else (names[c] if hide_conf else f'{names[c]} {conf:.2f}')
                        annotator.box_label(xyxy, label, color=colors(c, True))
                        if save_crop:
                            save_one_box(xyxy, imc, file=save_dir / 'crops' / names[c] / f'{p.stem}.jpg', BGR=True)

            # Stream results
            im0 = annotator.result()
            if view_img:
                cv2.imshow(str(p), im0)
                cv2.waitKey(1)  # 1 millisecond

        # LOGGER.info(f"{str(det)}, {str(len(det))}")
        return im0, det, frame

def sort_func(x):
    return x[-1]


class BasicSubscriber(Node):
    
    def __init__(self, open_window=False):
        super().__init__('camera_subscriber_turtlebot')
        self.subscription = self.create_subscription(CompressedImage, 'camera', self.yolo_callback, qos_profile_sensor_data)
        self.publisher_ = self.create_publisher(String, '/cmd_hover', 1)
        self.publisher_cmd = self.create_publisher(Twist, '/cmd_vel', 1)
        self.bridge=CvBridge()
        self.area_threshold = 150.
        self.msg = String()
        self.open_window=open_window
        self.count=0
        self.found=0

    def yolo_callback(self, data):        
        self.twist = Twist()
        self.pre_x=0.
        self.pre_z=0.

        frame = self.bridge.compressed_imgmsg_to_cv2(data)
        # frame = cv2.resize(frame, (640, 480))
        img, det, frame = run(frame=frame)
        collect_det=list()
        if len(det):
            for d in det:
                x, y, xx , yy, conf, cls_id = d
                if cls_id==0: # 사람이면
                    collect_det.append((x, y, xx, yy, conf, cls_id, yy))
        if collect_det:
            # if self.found==0: # 찾으면 이전 동작 반대로
            #     self.twist.linear.x = self.pre_x*-1.
            #     self.twist.angular.z = self.pre_z*-1.
            #     self.publisher_cmd.publish(self.twist)
            #     LOGGER.info("Rewind")            

            self.found += 1
            collect_det.sort(key=sort_func)
            x, y, xx , yy, conf, cls_id, area = collect_det[-1]
            height,width,  channel = img.shape
            self.area_threshold=int(height*0.75)
            center=int((x+xx)/2)
            center_y=int((y+yy)/2)
            threshold = 10
            
            if center < width//2-threshold:
                LOGGER.info("Turn Left")
                self.msg.data='l'
                self.twist.linear.x = 0.1
                self.twist.angular.z = -0.5*(center - (width//2-threshold))*0.006
            elif center > width//2+threshold:
                LOGGER.info("Turn Right")
                self.msg.data='r'
                self.twist.linear.x = 0.1
                self.twist.angular.z = -0.5*(center - (width//2-threshold))*0.006
            elif self.area_threshold<area-threshold:
                LOGGER.info("Go Back")            
                self.msg.data='b'
                self.twist.linear.x = -0.1
                # self.twist.angular.z = self.twist.angular.z*-1
                # self.twist.angular.z = 0.0
            elif self.area_threshold>area+threshold:
                LOGGER.info("Go Straight")            
                self.msg.data='a'
                self.twist.linear.x = 0.2
                # self.twist.angular.z = 0.0
            else:
                LOGGER.info("Stop")            
                self.msg.data='j'
                self.twist.linear.x = 0.
                self.twist.angular.z = 0.
            self.publisher_.publish(self.msg)
            self.publisher_cmd.publish(self.twist)
            cv2.circle(img, (center, center_y), 5, (155,125,0),-1)
            LOGGER.info(f"{area}")
            LOGGER.info(f"{str(self.twist.linear.x)} {str(self.twist.angular.z)}")

        else:
            if self.found==0:
                self.twist.angular.z = 0.5
                self.publisher_cmd.publish(self.twist)
            else:
                self.twist.angular.z = 0.
                self.publisher_cmd.publish(self.twist)
            self.found-=1
            if self.found<0:
                self.found=0
        # self.count+=1
        # cv2.imwrite('imga'+str(self.count)+".png",frame)
        if self.open_window:
            cv2.imshow('img', img)
        key = cv2.waitKey(1)       
        if key == 27:
            self.msg.data='j'
            self.publisher_.publish(self.msg)
            self.twist.linear.x = 0.
            self.twist.angular.z = 0.
            self.publisher_cmd.publish(self.twist)
            time.sleep(3)
            raise KeyboardInterrupt

def main(opt=None):
    rclpy.init(args=None)
    basic_subcriber = BasicSubscriber(open_window=False)
    try:
        rclpy.spin(basic_subcriber)
    except KeyboardInterrupt:
        print("프로그램 종료")
    finally:
        basic_subcriber.destroy_node()
        rclpy.shutdown()

if __name__=="__main__":
    main()