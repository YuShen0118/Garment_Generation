import time
import os
import sys
import pickle


if __name__ == "__main__":
    if (len(sys.argv)) != 3:
        print("Please use the following command:")
        print("python extract_pose_shape.py [input path] [output path]")
        exit()
    path = sys.argv[1] + '/'
    outfolder = sys.argv[2] + '/'
    if not os.path.exists(outfolder+'train_shape'):
        os.makedirs(outfolder+'train_shape')
    if not os.path.exists(outfolder+'train_pose'):
        os.makedirs(outfolder+'train_pose')
    C = 104
    M = 10
    for c in range(C):
        for m in range(M):
            h = -1
            for htmp in range(10):
                try:
                    filename = path + 'C' + format(c, '03d') + 'M' + format(m, '02d') + 'H' + format(htmp, '02d') + '/gt.pkl'
                    myfile = open(filename, "rb") # or "a+", whatever you need
                    h = htmp
                except IOError:
                    #print "Could not open file! Please close Excel!"
                    h = h

            if h >= 0:
                filename = path + 'C' + format(c, '03d') + 'M' + format(m, '02d') + 'H' + format(h, '02d') + '/gt.pkl'
                print('reading ' + filename + ' ...')
                fin = open(filename, 'rb')
                data = pickle.load(fin)
                fin.close()
                for u in range(250):
                    idx = c*10*250 + m*250 + u

                    outpath = outfolder + '/train_shape/shape_' + format(idx, '06d') + '.txt'
                    fout = open(outpath, 'w')
                    shape = data['shape'].tolist()
                    for i in range(len(shape)):
                        fout.write(str(shape[i]) + ' ')
                    fout.close()

                    outpath = outfolder + '/train_pose/pose_' + format(idx, '06d') + '.txt'
                    fout = open(outpath, 'w')
                    pose = data['pose'][u].tolist()
                    for i in range(len(pose)):
                        fout.write(str(pose[i]) + ' ')
                    fout.close()
