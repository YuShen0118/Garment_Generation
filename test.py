import os
from collections import OrderedDict
from torch.autograd import Variable
from options.test_options import TestOptions
from data.data_loader import CreateDataLoader
from models.models import create_model
import util.util as util
from util.visualizer import Visualizer
from util import html
import torch
from random import gauss
import time

import multiprocessing

if __name__ == "__main__":
    multiprocessing.set_start_method('spawn', True)

    opt = TestOptions().parse(save=False)
    opt.nThreads = 1   # test code only supports nThreads = 1
    opt.batchSize = 1  # test code only supports batchSize = 1
    opt.serial_batches = True  # no shuffle
    opt.no_flip = True  # no flip

    data_loader = CreateDataLoader(opt)
    dataset = data_loader.load_data()
    visualizer = Visualizer(opt)
    # create website
    web_dir = os.path.join(opt.results_dir, opt.name, '%s_%s' % (opt.phase, opt.which_epoch))
    webpage = html.HTML(web_dir, 'Experiment = %s, Phase = %s, Epoch = %s' % (opt.name, opt.phase, opt.which_epoch))

    # test
    if not opt.engine and not opt.onnx:
        model = create_model(opt)
        if opt.data_type == 16:
            model.half()
        elif opt.data_type == 8:
            model.type(torch.uint8)
                
        if opt.verbose:
            print(model)
    else:
        from run_engine import run_trt_engine, run_onnx
        
    tottime = 0
    totcnt = 0
    for i, data in enumerate(dataset):
        st = int(round(time.time() * 1000))
        #if i >= opt.how_many:
        #    break
        if opt.data_type == 16:
            data['label'] = data['label'].half()
            data['inst']  = data['inst'].half()
        elif opt.data_type == 8:
            data['label'] = data['label'].uint8()
            data['inst']  = data['inst'].uint8()
        if opt.export_onnx:
            print ("Exporting to ONNX: ", opt.export_onnx)
            assert opt.export_onnx.endswith("onnx"), "Export model file should end with .onnx"
            torch.onnx.export(model, [data['label'], data['inst']],
                            opt.export_onnx, verbose=True)
            exit(0)
        minibatch = 1 
        if opt.engine:
            generated = run_trt_engine(opt.engine, minibatch, [data['label'], data['inst']])
        elif opt.onnx:
            generated = run_onnx(opt.onnx, opt.data_type, minibatch, [data['label'], data['inst']])
        else:        
            random_vector = []
            current_length = data['other_info'].size()[1]
            for i in range(256 - current_length):
                random_vector.append(gauss(0, 1))
            random_vector = torch.reshape(torch.FloatTensor(random_vector), (1, 256 - current_length))
            data['other_info'] = torch.cat((data['other_info'], random_vector), 1)

            generated = model.inference(data['label'], data['inst'], data['other_info'], data['image'])
            
        visuals = OrderedDict([('input_label', util.tensor2label(data['label'][0], opt.label_nc)),
                            ('synthesized_image', util.tensor2im(generated.data[0])), 
                            ('gt_img', util.tensor2im(data['image'][0]))])

        img_path = data['path']
        print('process image... %s' % img_path)
        visualizer.save_images(webpage, visuals, img_path)
        visualizer.save_binaryFile(webpage, generated.data[0], img_path)

        ed = int(round(time.time() * 1000))
        print ('inference time: ' + str(ed - st) + ' ms')
        if i > 1:
            tottime += ed - st
            totcnt += 1


    print ('aver inference time: ' + str(tottime / totcnt) + ' ms')
    webpage.save()
