import os,time,cv2,argparse,multiprocessing
import numpy as np
from mvnc import mvncapi as mvnc
from skimage.transform import resize
from utils.app_utils import FPS, WebcamVideoStream
from multiprocessing import Queue, Pool
from .pan_tilt_controller import PanTiltController

classes = ["aeroplane", "bicycle", "bird", "boat", "bottle", "bus", "car", "cat", "chair", "cow", "diningtable", "dog", "horse", "motorbike", "person", "pottedplant", "sheep", "sofa", "train","tvmonitor"]
dim=(448,448)
threshold = 0.2
iou_threshold = 0.5
num_class = 20
num_box = 2
grid_size = 7

def interpret_output(output, img_width, img_height):
    w_img = img_width
    h_img = img_height
    probs = np.zeros((7,7,2,20))
    class_probs = (np.reshape(output[0:980],(7,7,20)))
    #print(class_probs)
    scales = (np.reshape(output[980:1078],(7,7,2)))
    #print(scales)
    boxes = (np.reshape(output[1078:],(7,7,2,4)))
    offset = np.transpose(np.reshape(np.array([np.arange(7)]*14),(2,7,7)),(1,2,0))
    #boxes.setflags(write=1)
    boxes[:,:,:,0] += offset
    boxes[:,:,:,1] += np.transpose(offset,(1,0,2))
    boxes[:,:,:,0:2] = boxes[:,:,:,0:2] / 7.0
    boxes[:,:,:,2] = np.multiply(boxes[:,:,:,2],boxes[:,:,:,2])
    boxes[:,:,:,3] = np.multiply(boxes[:,:,:,3],boxes[:,:,:,3])

    boxes[:,:,:,0] *= w_img
    boxes[:,:,:,1] *= h_img
    boxes[:,:,:,2] *= w_img
    boxes[:,:,:,3] *= h_img

    for i in range(2):
    	for j in range(20):
    		probs[:,:,i,j] = np.multiply(class_probs[:,:,j],scales[:,:,i])
    #print (probs)
    filter_mat_probs = np.array(probs>=threshold,dtype='bool')
    filter_mat_boxes = np.nonzero(filter_mat_probs)
    boxes_filtered = boxes[filter_mat_boxes[0],filter_mat_boxes[1],filter_mat_boxes[2]]
    probs_filtered = probs[filter_mat_probs]
    classes_num_filtered = np.argmax(probs,axis=3)[filter_mat_boxes[0],filter_mat_boxes[1],filter_mat_boxes[2]]

    argsort = np.array(np.argsort(probs_filtered))[::-1]
    boxes_filtered = boxes_filtered[argsort]
    probs_filtered = probs_filtered[argsort]
    classes_num_filtered = classes_num_filtered[argsort]

    for i in range(len(boxes_filtered)):
    	if probs_filtered[i] == 0 : continue
    	for j in range(i+1,len(boxes_filtered)):
    		if iou(boxes_filtered[i],boxes_filtered[j]) > iou_threshold :
    			probs_filtered[j] = 0.0

    filter_iou = np.array(probs_filtered>0.0,dtype='bool')
    boxes_filtered = boxes_filtered[filter_iou]
    probs_filtered = probs_filtered[filter_iou]
    classes_num_filtered = classes_num_filtered[filter_iou]

    result = []
    for i in range(len(boxes_filtered)):
    	result.append([classes[classes_num_filtered[i]],boxes_filtered[i][0],boxes_filtered[i][1],boxes_filtered[i][2],boxes_filtered[i][3],probs_filtered[i]])

    return result

def iou(box1,box2):
	tb = min(box1[0]+0.5*box1[2],box2[0]+0.5*box2[2])-max(box1[0]-0.5*box1[2],box2[0]-0.5*box2[2])
	lr = min(box1[1]+0.5*box1[3],box2[1]+0.5*box2[3])-max(box1[1]-0.5*box1[3],box2[1]-0.5*box2[3])
	if tb < 0 or lr < 0 : intersection = 0
	else : intersection =  tb*lr
	return intersection / (box1[2]*box1[3] + box2[2]*box2[3] - intersection)


def get_coordinates(results):
    """
    Get x and y coordinates from YOLO classification. Returned as dictionary from classification
    to [x,y]
    """
    coords = {}
    for i in range(len(results)):
        coords[results[i][0]] = [results[i][1],results[i][2]]
    print(coords)
    return coords

if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('-src', '--source', dest='video_source', type=int,
                        default=0, help='Device index of the camera.')
    parser.add_argument('-wd', '--width', dest='width', type=int,
                        default=640, help='Width of the frames in the video stream.')
    parser.add_argument('-ht', '--height', dest='height', type=int,
                        default=480, help='Height of the frames in the video stream.')
    parser.add_argument('-num-w', '--num-workers', dest='num_workers', type=int,
                        default=2, help='Number of workers.')
    parser.add_argument('-q-size', '--queue-size', dest='queue_size', type=int,
                        default=5, help='Size of the queue.')
    args = parser.parse_args()

    # configuration NCS
    network_blob = 'graph'
    mvnc.SetGlobalOption(mvnc.GlobalOption.LOG_LEVEL, 2)
    devices = mvnc.EnumerateDevices()
    if len(devices) == 0:
    	print('No devices found')
    	quit()
    device = mvnc.Device(devices[0])
    device.OpenDevice()
    opt = device.GetDeviceOption(mvnc.DeviceOption.OPTIMISATION_LIST)
    # load blob
    with open(network_blob, mode='rb') as f:
    	blob = f.read()
    graph = device.AllocateGraph(blob)
    graph.SetGraphOption(mvnc.GraphOption.ITERATIONS, 1)
    iterations = graph.GetGraphOption(mvnc.GraphOption.ITERATIONS)
    video_capture = WebcamVideoStream(src=args.video_source,
                                      width=args.width,
                                      height=args.height).start()


    class_to_track = "tvmonitor"
    controller = PanTiltController()
    im_center = [args.width/2,args.height/2]
    while True:  # fps._numFrames < 120
        start = time.time()
        frame = video_capture.read()
        graph.LoadTensor(resize(frame/255.0,dim,1)[:,:,(2,1,0)].astype(np.float16), 'user object')
        out, userobj = graph.GetResult()
        results = interpret_output(out.astype(np.float32), frame.shape[1], frame.shape[0])
        coords = get_coordinates(results)
	offset = controller.get_offset(coords[class_to_track])
	if offset[0] < im_center[0] and controller.get_angle_pan() >= 5:
		controller.set_angle_pan(controller.get_angle_pan()+2)
	elif offset[0] < im_center[0] and controller.get_angle_pan() <= 175:
        controller.set_angle_pan(controller.get_angle_pan()-2)

    if offset[1] < im_center[1] and controller.get_angle_tilt() >= 5:
        controller.set_angle_tilt(controller.get_angle_tilt()+2)
    elif offset[1] > im_center[1] and controller.get_angle_tilt() <= 175:
        controller.set_angle_tilt(controller.get_angle_tilt()-2)

    if class_to_track != None:
        break
        end = time.time()
        print(end - start)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    video_capture.stop()
    graph.DeallocateGraph()
    device.CloseDevice()
