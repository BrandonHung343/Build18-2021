import numpy as np
from skimage import io
from skimage.transform import rescale
import argparse

def loadIm(name):
    return io.imread(name)

def saveIm(name, im):
    io.imsave(name + '.bmp', im)

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--name', default="")
    parser.add_argument('--saveName', default="")
    args = parser.parse_args()

    im = loadIm(args.name)
    x, y, d = im.shape
    print(im.shape)
    finPix = 144
    ratio = finPix / x
    newIm = np.zeros((int(ratio*y), int(ratio*x), 3))
    if x > finPix:
        print(ratio)
        newIm[:, :, 0] = np.transpose(rescale(im[:, :, 0], ratio, anti_aliasing=False))
        newIm[:, :, 1] = np.transpose(rescale(im[:, :, 1], ratio, anti_aliasing=False))
        newIm[:, :, 2] = np.transpose(rescale(im[:, :, 2], ratio, anti_aliasing=False))


    x, y, d = newIm.shape
    saveIm(args.saveName + '_r' + str(x) + '_c' + str(y), newIm)


if __name__ == '__main__':
    main()

