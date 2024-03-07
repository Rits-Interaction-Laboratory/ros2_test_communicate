# coding: utf-8
import numpy as np
import cv2

iSize = 150

def gray2color(img, max=8001, rmax=8000, rmin=0):
    rate = max // int(rmax - rmin + 1)
    img = np.where((img <= rmax) & (img >= rmin), rate * (img - rmin), np.where(img < rmin, 0,
                                                                                (img - rmax) / (max - rmax) * (
                                                                                            max - rate * (
                                                                                                rmax - rmin)) + (
                                                                                            (rmax - rmin) * rate)))

    img120 = (img / max * 255).astype("uint8")

    img120_C = cv2.cvtColor(img120, cv2.COLOR_GRAY2BGR)
    
    return img120_C

def draw_hand(obj, joint, dma=2500, dmi=750):
    connections = [[0, 1], [1, 2], [2, 3], [3, 4], [0, 5], [5, 9], [9, 13], [13, 17], [17, 0], [5, 6], [6, 7], [7, 8],
                   [9, 10], [10, 11], [11, 12], [13, 14], [14, 15], [15, 16], [17, 18], [18, 19], [19, 20]]
    colors = [[0, 0, 255], [0, 0, 255], [0, 0, 255], [0, 0, 255], [255, 102, 0], [255, 102, 0], [255, 102, 0], [255, 102, 0],
              [255, 102, 0], [0, 255, 0], [0, 255, 0], [0, 255, 0], [255, 0, 255], [255, 0, 255], [255, 0, 255],
              [0, 255, 255], [0, 255, 255], [0, 255, 255], [255, 255, 0], [255, 255, 0], [255, 255, 0]]
    resizeY, resizeX = obj.shape[0], obj.shape[1]
    if obj.dtype == np.uint8:
        img = obj
    else:
        img = gray2color(obj, rmax=dma, rmin=dmi)
    #print(data.shape)
    for j in range(joint.shape[0]):
        x = int(joint[j][0])
        y = int(joint[j][1])
        cv2.circle(img, (x, y), 1, (0, 255, 0), 1)
    for cn in range(len(connections)):
        cv2.line(img, (int(joint[connections[cn][0]][0]), int(joint[connections[cn][0]][1])), (int(joint[connections[cn][1]][0]), int(joint[connections[cn][1]][1])), colors[cn], 2)#"""
    return cv2.resize(img, (resizeX, resizeY))

def img_overlay(objs, inp):
    thre = 8000
    inp[:,:,0] = np.where((objs[:,:] >= thre), inp[:,:,0], inp[:,:,0] * 0.5)
    inp[:,:,1] = np.where((objs[:,:] >= thre), inp[:,:,1], inp[:,:,1] * 0.5)
    inp[:,:,2] = np.where((objs[:,:] >= thre), inp[:,:,2], 255)
    
    obj = cv2.resize(inp, (iSize, iSize))

    return obj

def act_overlay(obj, acts):
    frames = []
    for f in range(acts.shape[0]):
        o = obj.copy()
        imgM, imgO = 0, 0
        m = acts[f]
        im = draw_hand(o, m)

        frames.append(im)

    return frames

def show_image(objs, acts, inp):
    frames = []
    #print(acts.shape)
    #o = gray2color(objs, rmax=1500, rmin=1050)
    obj = img_overlay(objs, inp)

    for f in range(acts.shape[0]):
        o = obj.copy()
        imgM, imgO = 0, 0
        m = acts[f]
        #mm = m[f].reshape(21, 3)

        im = draw_hand(o, m)

        frames.append(im)

    return frames
