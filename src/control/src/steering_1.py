import matplotlib.pyplot as plt
import numpy as np
import cv2
import math
import time

path = "../source/WIN_20220925_14_36_35_Pro.mp4"
obj_b = cv2.imread('../source/corn_data/lavacorn_nb.png', cv2.IMREAD_GRAYSCALE)#wad
obj_s = cv2.imread('../source/corn_data/lavacorn_ns.png', cv2.IMREAD_GRAYSCALE)#wad
cap=cv2.VideoCapture(path) #path
obj_contours_b,_=cv2.findContours(obj_b, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)#wad
obj_contours_s,_=cv2.findContours(obj_s, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)#wad
obj_pts_b=obj_contours_b[0]#wad
obj_pts_s=obj_contours_s[0]#wad

fps = cap.get(cv2.CAP_PROP_FPS)
WIDTH  = 640
HEIGHT = 360
rate=int(WIDTH/640)
codec = cv2.VideoWriter_fourcc(*'DIVX')
out=cv2.VideoWriter('../output/output_11_corn.mp4', codec, 30.0, (int(WIDTH),int(HEIGHT)), isColor=0)

kernel_size=5

low_threshold=30
high_threshold=255

theta=np.pi/180
threshold=90

p_r_m=0
p_r_n=0
p_l_m=0
p_l_n=0

def grayscale(img):
    hsv=cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

    lower_red=(0, 150,80)
    upper_red=(125, 255, 255)

    mask_hsv=cv2.inRange(hsv, lower_red, upper_red)

    img = cv2.bitwise_and(img, img, mask=mask_hsv)
    img = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)
    return img
    
def gaussian_blur(img, kernel_size):
    return cv2.GaussianBlur(img, (kernel_size, kernel_size), 0)

def threshold(img, low_threshold, high_threshold):
    """Applies the Canny transform"""
    return cv2.threshold(img, low_threshold, high_threshold, cv2.THRESH_BINARY)

def region_of_interest(img, vertices):
    mask=np.zeros_like(img)
    
    if len(img.shape)>2:
        channel_count = img.shape[2]
        ignore_mask_color=(255,)*channel_count
    else:
        ignore_mask_color=255

    cv2.fillPoly(mask, vertices, ignore_mask_color)

    masked_image=cv2.bitwise_and(img,mask)
    return masked_image

def depart_points(img, points, r_m, r_n, l_m, l_n):

    r_list=[]
    l_list=[]
    
    for p in points:
        if point2linear_distance(r_m, r_n, p) < point2linear_distance(l_m, l_n, p):
            r_list.append(p)
        else:
            l_list.append(p)
            
    return r_list, l_list

def gradient(p1, p2):
    return (p2[1]-p1[1])/(p1[0]-p2[0])

def y_intercept(m, p):
    return (-1)*p[1]-m*p[0]

def point2linear_distance(m,n,p):
    return abs(m*p[0]-p[1]+n)/(m**2+1)**(1/2)

def linear_reg(img, left, right):
    left_x=[]
    left_y=[]
    right_x=[]
    right_y=[]

    global p_r_m
    global p_r_n
    global p_l_m
    global p_l_n
    
    for i in range(len(left)):
        left_x.append(left[i][0])
        left_y.append(left[i][1])

    for i in range(len(right)):
        right_x.append(right[i][0])
        right_y.append(right[i][1])

    left_calculated_weight=0
    right_calculated_weight=0
    if len(left)<2:
        left_calculated_weight=p_l_m
        left_calculated_bias=p_l_n
    else:
        mean_lx=np.mean(left_x)
        mean_ly=np.mean(left_y)
        left_calculated_weight=least_square(left_x, left_y, mean_lx, mean_ly)
        left_calculated_bias=mean_ly-left_calculated_weight*mean_lx
    target_l=left_calculated_weight*WIDTH+left_calculated_bias
    print(f"y = {left_calculated_weight} * X + {left_calculated_bias}")
    
    if len(right)<2:
        right_calculated_weight=p_r_m
        right_calculated_bias=p_r_n
    else:
        mean_rx=np.mean(right_x)
        mean_ry=np.mean(right_y)
        right_calculated_weight=least_square(right_x, right_y, mean_rx, mean_ry)
        right_calculated_bias=mean_ry-right_calculated_weight*mean_rx
    target_r=right_calculated_weight*WIDTH+right_calculated_bias
    print(f"y = {right_calculated_weight} * X + {right_calculated_bias}")
    img = cv2.line(img,(int(WIDTH/2),HEIGHT),(int(WIDTH/2),int(0)),(255,0,0),3)

    cross_x = (right_calculated_bias - left_calculated_bias) / (left_calculated_weight - right_calculated_weight)
    cross_y = left_calculated_weight*((right_calculated_bias - left_calculated_bias)/(left_calculated_weight - right_calculated_weight)) + left_calculated_bias

    if np.isnan(cross_x)!=True and np.isnan(cross_y)!=True:
        img = cv2.line(img,(0,int(left_calculated_bias)),(int(WIDTH),int(target_l)),(0,0,0),10)
        img = cv2.line(img,(int(0),int(right_calculated_bias)),(WIDTH,int(target_r)),(0,0,0),10)
        cv2.circle(img, (int(cross_x), int(cross_y)), 10, (0, 0, 255), -1, cv2.LINE_AA)

        if 80 < steering_theta(left_calculated_weight, right_calculated_weight) < 100:
            print('소실점 조향 서보모터 각도: ', steering_vanishing_point(cross_x))
        else:
            print("기울기 조향 서보모터 각도: ", steering_theta(left_calculated_weight, right_calculated_weight))

    p_l_m=left_calculated_weight
    p_r_m=right_calculated_weight
    p_l_n=left_calculated_bias
    p_r_n=right_calculated_bias
    #print('Done.')
    return img

def least_square(val_x, val_y, mean_x, mean_y):
    return ((val_x - mean_x) * (val_y - mean_y)).sum() / ((val_x - mean_x)**2).sum()

def steering_theta(w1, w2):
    if np.abs(w1) > np.abs(w2):  # 우회전
        if w1 * w2 < 0:  #정방향 or 약간 틀어진 방향
            w1 = -w1
            angle = np.arctan(np.abs(math.tan(w1) - math.tan(w2)) / (1 + math.tan(w1)*math.tan(w2)))
            theta = matching(angle, 0, np.pi/2, 90, 180)
        elif w1 * w2 > 0:  #극한으로 틀어진 방향
            if w1 > w2:
                theta = 90
            else:
                theta = 90
        else:
            theta = 0
    elif np.abs(w1) < np.abs(w2) :  # 좌회전
        if w1 * w2 < 0:  #정방향 or 약간 틀어진 방향
            w1 = -w1
            angle = np.arctan(np.abs(math.tan(w1) - math.tan(w2)) / (1 + math.tan(w1)*math.tan(w2)))
            theta = matching(angle, 0, np.pi/2, 90, 0)
        elif w1 * w2 > 0:  #극한으로 틀어진 방향
            if w1 > w2:
                theta = 90
            else:
                theta = 90
        else:
            theta = 0
    else:
        theta = 90

    return theta

def matching(x,input_min,input_max,output_min,output_max):
    return (x-input_min)*(output_max-output_min)/(input_max-input_min)+output_min #map()함수 정의.

def steering_vanishing_point(x):
    standard_x = int(WIDTH/2)
    diff = standard_x - x 
    if diff > 0:   #좌회전
        theta = matching(diff, 0, WIDTH/2, 90, 45)
    elif diff < 0:
        theta = matching(diff, 0, -WIDTH/2, 90, 135)

    return theta
##여기까지가 조향각 계산하는,,

## while문? 값 받아올 때마다 써서 값 받아올 때마다 함수를 이용해서 교점 구하는거랑 조향각도 계산(linear_reg) 여기서 조금씩 바꾸면 될듯?
    r_mid=[]
    l_mid=[]
    m_mid=[]

    if p_r_m==0 and p_l_m==0:
        p_r_m=0.3
        p_r_n=37
        p_l_m=-0.3
        p_l_n=238
    r_mid, l_mid = depart_points(img, m_mid, p_r_m,p_r_n,p_l_m,p_l_n)

    linear_img=linear_reg(img, l_mid, r_mid)
    dt=time.time()-t
    print("delay : ", dt)
