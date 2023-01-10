import os
 
def iterbrowse(path):
    for home, dirs, files in os.walk(path):
        for filename in files:
            yield os.path.join(home, filename)
 
 
for fullname in iterbrowse("E:\\Papertest2\\ImageStitching\\SIFT_GPU\\test"):
    print fullname
