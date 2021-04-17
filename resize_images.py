from PIL import Image
import glob

dir_path = 'images'
files = [file for file in glob.glob(dir_path + '\**\*.jpg', recursive=True)]
for file in files:
    im = Image.open(file)
    imResize = im.resize((128,128), Image.ANTIALIAS)
    imResize.save(file , 'JPEG', quality=100, subsampling=0)