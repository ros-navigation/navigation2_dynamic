# detectron2_detector

This package implements a 3D object detector with [Mask RCNN](https://arxiv.org/pdf/1703.06870.pdf) ([detectron2](https://github.com/facebookresearch/detectron2) implementation) and basic pointcloud processing. 
We assume colored and ordered pointcloud aligned with pixels is available. We use Mask RCNN to produce masks for objects, then use these masks to extract points belong to the objects and estimate position and size. 

### Parameters
| parameters       | Meaning        | Default |
| ---------------- | ------------- | ------- |
| detectron_config_file | config file to load detectron model, <br> check out [model zoo](https://github.com/facebookresearch/detectron2/blob/master/MODEL_ZOO.md) | COCO-InstanceSegmentation/mask_rcnn_R_50_FPN_3x.yaml |
| detectron_score_thresh | min score to be considered as an object | 0.8 |
| pointcloud2_topic | ros topic for pointcloud data | /camera/depth/points |
| categories | list of interested [categories](https://github.com/amikelive/coco-labels/blob/master/coco-labels-2014_2017.txt) in COCO dataset, left empty `[]` to detect all | [0] (person) |
| pc_downsample_factor | factor to downsample the aligned pointcloud | 16 |
| min_mask | min number of pixels in a mask | 20 |
| nms_filter | IoU threshold for non-max suppression | 0.3 |
| outlier_filter | threshold to filter outlier points, <br> model as multivariate normal distribution | 0.5 |
