#!/usr/bin/env python
import numpy as np
import cv2
import matplotlib.pyplot as plt
import matplotlib.image as mpimg


def hitmiss(src, kernel):
    im = src / 255
    k1 = (kernel == 1).astype('uint8')
    k2 = (kernel == -1).astype('uint8')
    e1 = cv2.erode(im, k1, borderType=cv2.BORDER_CONSTANT)
    e2 = cv2.erode(1-im, k2, borderType=cv2.BORDER_CONSTANT)
    return e1 & e2



if __name__ == "__main__":
#    img = cv2.imread('bitmaps/intel.png', cv2.CV_LOAD_IMAGE_GRAYSCALE)
#    _, im_binary = cv2.threshold(img, 50, 255, cv2.THRESH_BINARY)

#    kernel = np.array([[-1,-1, 1],
#                       [-1, 1, 1],
#                       [-1,-1, 1]])
#
#    im_mask = np.zeros(im_binary.shape, np.uint8)
#
#    im_mask |= hitmiss(im_binary, kernel)
#    im_mask |= hitmiss(im_binary, np.fliplr(kernel))
#    im_mask |= hitmiss(im_binary, kernel.T)
#    im_mask |= hitmiss(im_binary, np.flipud(kernel.T))
#
#    im_dst = im_binary & ((1 - im_mask) * 255)

    img = cv2.imread('bitmaps/intel.png')
    img_bw = 255*(cv2.cvtColor(img, cv2.COLOR_BGR2GRAY) > 10).astype('uint8')
    se1 = cv2.getStructuringElement(cv2.MORPH_RECT, (5,5))
    se2 = cv2.getStructuringElement(cv2.MORPH_RECT, (2,2))
    mask = cv2.morphologyEx(img_bw, cv2.MORPH_CLOSE, se1)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, se2)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, se1)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, se2)

    mask = np.dstack([mask, mask, mask]) / 255
    out = img * mask
#    cv2.imshow('Output', out)
    cv2.waitKey(0)
    cv2.destroyAllWindows()
    im_gray = cv2.fastNlMeansDenoising(out,None,20,9,21)
    thresh = 200
    im_bw = cv2.threshold(im_gray, thresh, 255, cv2.THRESH_BINARY)[1]
    cv2.imwrite('bitmaps/interl_cl.png', im_bw)
#    plt.imshow(out)
#    cv2.imwrite('bitmaps/inteal_cl.png', im_dst)

#img = mpimg.imread('bitmaps/intel.png')
#img = cv2.imread('bitmaps/intel.png')
#    dst = cv2.fastNlMeansDenoising(img,None,15,7,21)
# plt.subplot(122),plt.imshow(dst)
# if os.path.isfile('bitmaps/intel.png'): im = array(Image.open('bitmaps/intel.png'))
# if img == None or img.size == 0:
   # print 'Image loaded is empty'
   # sys.exit(1)
# plt.subplot(121),
#    plt.imshow(dst)
#plt.show()
