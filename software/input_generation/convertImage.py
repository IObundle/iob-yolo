from PIL import Image
from struct import pack,unpack
import sys

def ConvertImage(filename_in,filename_out):
	# Image dimensions expected by the software
	WIDTH = 768
	HEIGHT = 576

	image = Image.open(filename_in)
	width,height = image.size

	if(width != WIDTH or height != HEIGHT):
	    image.show()
	    image = image.resize((WIDTH,HEIGHT),Image.LINEAR)
	    image.show()
	    width,height = image.size

	rgb = image.convert('RGB')

	f_out = open(filename_out, "wb")

	for y in range(HEIGHT):
	    for x in range(WIDTH):
	        r,g,b = rgb.getpixel((x,y))
	        f_out.write(pack('BBBBBBBB',r,0,g,0,b,0,0,0))

	#Close files
	f_out.close()

if __name__ == "__main__":
    if(len(sys.argv) != 3):
        print("Format: python <Script> <Image in filepath> <Image out filepath>")
    else:
        ConvertImage(sys.argv[1],sys.argv[2])