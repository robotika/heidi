import cvideo

import cv2
import numpy as np

import sys
sys.path.append( ".." ) 
from pave import PaVE, isIFrame

print cvideo.init()

img = np.zeros([720,1280,3], dtype=np.uint8)

filename = sys.argv[1]
pave = PaVE()
pave.append( open( filename, "rb" ).read() )
header,payload = pave.extract()
while len(header) > 0:
  print cvideo.frame( img, isIFrame(header) and 1 or 0, payload )
  cv2.imshow('image', img)
  if cv2.waitKey(1) & 0xFF == ord('q'):
    break
  header,payload = pave.extract()

