# ---------- Deteccion de objetos con PiCamera usando Tensorflow ----------
#
# Luis Arias Gomez, Edward Umana Williams, Guillermo Lopez Navarro
# Reconocimiento de Patrones - Sistemas embebidos de Alto Desempeno
# Tecnologico de Costa Rica
# Agosto 2019

# Este programa utiliza un clasificador con TensorFlow para deteccion de objetos.
# Se carga un modelo entrenado al cual se le brinda imagenes capturadas con la 
#    camara, lo cual retorna una etiqueta que posteriormente se utiliza en conjunto
#    con un marco que rodea el objeto, para denotar su identidad segun lo predijo
#    el modelo.

# Este codigo se basa parcialmente en el ejemplo de Google disponible en
# https://github.com/tensorflow/models/blob/master/research/object_detection/object_detection_tutorial.ipynb

# Asimismo, se utilizo la guia disponible en el link a continuacion, para la 
#     integracion de la camara RPI con la Jetson Nano.
# https://www.jetsonhacks.com/2019/04/02/jetson-nano-raspberry-pi-camera/

import os
import cv2
import numpy as np
import tensorflow as tf
import argparse
from networktables import NetworkTables

# Utilities para etiquetado en el video a mostrar y para manipulacion de
#     las etiquetas del modelo, respectivamente
from utils import visualization_utils as vis_util
from utils import label_map_util
from camera import add_camera_args, Camera

# Dimensiones del video a mostrar
IM_WIDTH = 640  # 720 #1280 # 640
IM_HEIGHT = 480  # 540 #720 # 480


# Funcion que retorna handler de la camara RPI
def gstreamer_pipeline(capture_width=IM_WIDTH, capture_height=IM_HEIGHT, display_width=IM_WIDTH,
                       display_height=IM_HEIGHT, framerate=30, flip_method=0):
    return ('nvarguscamerasrc ! '
            'video/x-raw(memory:NVMM), '
            'width=(int)%d, height=(int)%d, '
            'format=(string)NV12, framerate=(fraction)%d/1 ! '
            'nvvidconv flip-method=%d ! '
            'video/x-raw, width=(int)%d, height=(int)%d, format=(string)BGRx ! '
            'videoconvert ! '
            'video/x-raw, format=(string)BGR ! appsink' % (
            capture_width, capture_height, framerate, flip_method, display_width, display_height))


def parse_args():
    desc = (
        'Capture and display live camera video, while doing ''real-time face detection with TrtMtcnn on Jetson''Nano')
    parser = argparse.ArgumentParser(description=desc)
    parser = add_camera_args(parser)
    parser.add_argument('--minsize', type=int, default=40, help='minsize (in pixels) for detection [40]')
    args = parser.parse_args()
    return args


# Nombre del directorio que contiene el modelo a utilizar para la prediccion
# MODEL_NAME = 'ssdlite_mobilenet_v2_coco_2018_05_09'
MODEL_NAME = 'FIRSTball'
# LABELS = 'mscoco_label_map.pbtxt'
# LABELS = 'birras_labelmap_6.pbtxt'
LABELS = 'labelmap.pbtxt'

# Numero de clases que puede identificar el modelo
# NUM_CLASSES = 90
# NUM_CLASSES = 6
NUM_CLASSES = 2

NetworkTables.initialize(server='10.43.82.2')

# Directorio actual
CWD_PATH = os.getcwd()

# Ruta al archivo .pb (modelo a utilizar)
PATH_TO_CKPT = os.path.join(CWD_PATH, MODEL_NAME, 'frozen_inference_graph.pb')
# PATH_TO_CKPT = os.path.join(CWD_PATH, MODEL_NAME, 'frozen_inference_graph_LOSS_1,5.pb')

# Ruta al archivo que contiene etiquetas mapeadas a identificadores de objeto
# Este mapeo permite identificar con un nombre legible el valor predicho por 
#     la red convolutiva
PATH_TO_LABELS = os.path.join(CWD_PATH, 'labels', LABELS)

# Cargamos el mapeo de etiquetas.
# Para ello recurrimos a una libreria especializada, cargada previamente
label_map = label_map_util.load_labelmap(PATH_TO_LABELS)
categories = label_map_util.convert_label_map_to_categories(label_map, max_num_classes=NUM_CLASSES,
                                                            use_display_name=True)
category_index = label_map_util.create_category_index(categories)

# Cargamos el modelo de TensorFlow en memoria
detection_graph = tf.compat.v1.Graph()
with detection_graph.as_default():
    od_graph_def = tf.compat.v1.GraphDef()
    with tf.gfile.GFile(PATH_TO_CKPT, 'rb') as fid:
        serialized_graph = fid.read()
        od_graph_def.ParseFromString(serialized_graph)
        tf.import_graph_def(od_graph_def, name='')

    TFSess = tf.compat.v1.Session(graph=detection_graph)

# Definimos los tensores de entrada y salida para el clasificador
# El tensor de entrada es cada cuadro del video (una imagen)
image_tensor = detection_graph.get_tensor_by_name('image_tensor:0')

# Los tensores de salida corresponden a las cajas de deteccion, scores, y clases
# Las cajas corresponden a la parte de la imagen que contiene un objeto detectado
detection_boxes = detection_graph.get_tensor_by_name('detection_boxes:0')

# Cada score representa el porcentaje de asertividad de la prediccion
# El score se muestra en conjunto con la etiqueta asignada al objeto detectado
detection_scores = detection_graph.get_tensor_by_name('detection_scores:0')
detection_classes = detection_graph.get_tensor_by_name('detection_classes:0')

# Numero de objetos detectados
num_detections = detection_graph.get_tensor_by_name('num_detections:0')

# Inicializar calculo de FPS (cuadros por segundo), para mostrarlo en pantalla
frame_rate_calc = 1
freq = cv2.getTickFrequency()
font = cv2.FONT_HERSHEY_SIMPLEX

t = 0
# Nombre de la ventana del video a mostrar
WIN_NAME = 'Detection'

camnum = 0;
# Inicializamos la camara
# cap = cv2.VideoCapture(0,cv2.CAP_GSTREAMER)
# cap = cv2.VideoCapture(0)
args = parse_args()
cam = Camera(args)
cam.open_usb(0, 640, 480)
cam.open_usb(1, 640, 480)
cam.start()

# cap.set(3, 640)
# cap.set(4, 480)
# cap.set(3, 1280)
# cap.set(4, 720)

# cap1 = cv2.VideoCapture(1)
# cap1.set(3, 640)
# cap1.set(4, 480)

# if cap.isOpened():
# window_handle = cv2.namedWindow(WIN_NAME, cv2.WINDOW_NORMAL)

frameCount = 0

control = False

nextTFSessRun = cv2.getTickCount() + freq / 4
first = True
time1 = 0.0

classes = 0

sd = NetworkTables.getTable("SmartDashboard")

change = 0

while True:
    t1 = cv2.getTickCount()

    # Obtenemos un cuadro del video, y expandimos sus dimensiones a la forma
    #   [1, None, None, 3], en concordancia con lo requerido por el tensor. Una sola 
    #   columna que contiene los valores RGB de cada pixel
    # no optimizable

    camnum = sd.getNumber("cam", 0)

    # print("Object detection camera:", camnum)
    # if(change == camnum):
    # pass
    # elif(change != camnum):
    # cam.stop()
    # cam.release()
    # change = camnum
    # cam.open()
    # cam.start()
    # ret_val, frame = cap.read();

    # if(change == camnum):
    # ret_val, frame = cap.read();
    # print("cam0")
    # pass
    # elif(change != camnum):
    # ret_val, frame = cap1.read();
    # print("cam1")
    # cam.open()
    # cam.start()
    if(camnum == 1):
        cam.changecamnum(1)
        sd.putString("cam:", "Using cam 1")
    else:
        cam.changecamnum(0)
        sd.putString("cam:", "Using cam 0")

    # ret_val, frame = cap2.read();
    frame = cam.read()
    if (frame is None):
        continue
    frame.setflags(write=1)
    frame_expanded = np.expand_dims(frame, axis=0)

    # Realizamos la deteccion de objetos, proveyendo la imagen como entrada
    # print("running")
    # if(cv2.waitKey(8) != ord('s')):
    if (t1 > nextTFSessRun or first):
        # print("running")
        nextTFSessRun = t1 + freq / 4
        first = False
        (boxes, scores, classes, num) = TFSess.run(
            [detection_boxes, detection_scores, detection_classes, num_detections],
            feed_dict={image_tensor: frame_expanded})
        t = t + 1

    # DEBUG
    # print(boxes)
    # print(np.squeeze(boxes))
    # print(np.squeeze(classes))
    # print(np.squeeze(scores))

    # Dibujamos los resultados de la deteccion sobre el video mostrado
    # vis_util.visualize_boxes_and_labels_on_image_array(
    # frame,
    # np.atleast_2d(np.squeeze(boxes)),#no optimizable
    # np.atleast_1d(np.squeeze(classes).astype(np.int32)),
    # np.atleast_1d(np.squeeze(scores)),
    # category_index,
    # use_normalized_coordinates=True,
    # line_thickness=8,
    # min_score_thresh=0.50)

        width = 640
        height = 480
        ymin = int((boxes[0][0][0] * height))
        xmin = int((boxes[0][0][1] * width))
        ymax = int((boxes[0][0][2] * height))
        xmax = int((boxes[0][0][3] * width))
    
        sd.putNumber("ymax", ymax)
        sd.putNumber("ymin", ymin)
        sd.putNumber("xmax", xmax)
        sd.putNumber("xmin", xmin)
        sd.putNumber("time", t)

        midcordx = ((xmin + xmax) / 2)
        midcordy = ((ymin + ymax) / 2)
        size = (xmax - xmin)
        if (size == 0):
            distoff = 0
        else:
            distoff = midcordx - 320

    # print(ymax, xmax, xmin, ymin)
    #print(midcordx, midcordy)
        sd.putNumber("mid x", midcordx)
        sd.putNumber("mid y", midcordy)
        sd.putNumber("size", size)
        sd.putNumber("distance off", distoff)

        if (np.squeeze(scores) == 0.0):
            classes = 0
            sd.putString("class", "None")
        elif (np.squeeze(scores) != 0.0):
            classes = np.squeeze(classes)
            if (classes == 1):
                sd.putString("class", "Ball")
            elif (classes == 2):
                sd.putString("class", "Target")
    
        print(classes)
    # print(ymin,xmin,ymax,xmax)
    # F.write(ymin,xmin,ymax,xmax)
    # roi = image[ymin:ymax,xmin:xmax].copy()

    # box = np.squeeze(boxes)
    # for i in range(len(boxes)):
    # ymin = (int(box[i,0]*height))
    # xmin = (int(box[i,1]*width))
    # ymax = (int(box[i,2]*height))
    # xmax = (int(box[i,3]*width))
    # print(ymin,xmin,ymax,xmax)
    # roi = image[ymin:ymax,xmin:xmax].copy()
    # cv2.imwrite("box_{}.jpg".format(str(i)), roi)

    # Dibujamos los cuadros por segundo del video
    # cv2.putText(frame,"FPS: {0:.2f}".format(frame_rate_calc),(30,50),font,1,(255,255,0),2,cv2.LINE_AA)
    # cv2.putText(frame,"count: {0:.2f}".format(time1),(30,50),font,1,(255,255,0),2,cv2.LINE_AA)

    # Mostramos la imagen con los dibujos superpuestosqq
    # cv2.imshow(WIN_NAME, cv2.resize(image_np, (800,600))
    # cv2.imshow(WIN_NAME, frame)

    # if(cv2.waitKey(1) == ord('q')):
    # break

    # Calculo de FPS
    # t2 = cv2.getTickCount()
    # time1 = (t2-t1)/freq
    # frame_rate_calc = 1/time1

    # frameCount+=1
    # if frameCount == 3:
    #    break

    # Al presionar Q en el teclado, finalizamos la ejecucion
    # if keyboard.is_pressed('q'):
    # break
    # if(cv2.waitKey(1) == ord('q')):
    # break

# cap.release()
cam.stop()
cam.release()

cv2.destroyAllWindows()
