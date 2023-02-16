#Import libraries
from pycocotools.coco import COCO
from pycocotools.cocoeval import COCOeval
import json

#Filenames
annFile = '../../instances_val2014.json'
resFile='coco_results.json'
new_annFile = 'instances_val2014_filtered.json'

#get image and category IDs from results
ID_arr = set() #avoid duplicates
catID_arr = set()
with open(resFile, 'r') as data_file:    
    data = json.load(data_file)
    for line in data:
        ID_arr.add(line['image_id'])
        catID_arr.add(line['category_id'])
ID_arr = list(ID_arr)
catID_arr = list(catID_arr)

#create new annotations File containing only the images tested so far
json_dict = {}
with open(new_annFile, 'w') as write_file:  
    with open(annFile, 'r') as data_file:
        data = json.load(data_file)
        json_dict['info'] = data['info']
        json_dict['licenses'] = data['licenses']
        json_dict['categories'] = data['categories']
        json_dict['images'] = []
        for image in data['images']:
            if(image['id'] in ID_arr):
                json_dict['images'].append(image)
        json_dict['annotations'] = []
        for annotation in data['annotations']:
            if(annotation['image_id'] in ID_arr):
                json_dict['annotations'].append(annotation)    
    json.dump(json_dict, write_file)

#initialize COCO ground truth api
cocoGt=COCO(new_annFile)

#initialize COCO detections api
cocoDt=cocoGt.loadRes(resFile)

# running evaluation
cocoEval = COCOeval(cocoGt, cocoDt, 'bbox')
#cocoEval.params.catIds = [1]
cocoEval.params.imgIds  = cocoGt.getImgIds()
cocoEval.evaluate()
cocoEval.accumulate()
cocoEval.summarize()