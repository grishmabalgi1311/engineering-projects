"""Official pretrained LightGlue correspondences with geometric RANSAC checks."""
from pathlib import Path
import cv2
import numpy as np
from visual_pose import wall_points


class Matcher:
    def __init__(self,cache,upscale=False):
        import torch
        from transformers import AutoImageProcessor,AutoModelForKeypointMatching
        torch.set_num_threads(1)
        self.torch=torch
        self.upscale=upscale
        name='ETH-CVG/lightglue_superpoint'
        self.processor=AutoImageProcessor.from_pretrained(name,cache_dir=Path(cache),local_files_only=True)
        self.model=AutoModelForKeypointMatching.from_pretrained(name,cache_dir=Path(cache),local_files_only=True).eval()
        self.model.config.keypoint_detector_config.max_keypoints=1024

    def points(self,reference,live):
        from PIL import Image
        images=[Image.fromarray(cv2.cvtColor(image,cv2.COLOR_BGR2RGB)) for image in (reference,live)]
        inputs=self.processor(images,return_tensors='pt',do_resize=self.upscale)
        with self.torch.inference_mode():
            outputs=self.model(**inputs)
        # Preserve subpixel coordinates: the display-oriented postprocessor
        # rounds points to integer pixels.
        first=outputs.mask[0,0]>0
        second=outputs.mask[0,1]>0
        p=outputs.keypoints[0,0][first]*self.torch.tensor([images[0].width,images[0].height])
        q=outputs.keypoints[0,1][second]*self.torch.tensor([images[1].width,images[1].height])
        matches=outputs.matches[0,0][first]
        scores=outputs.matching_scores[0,0][first]
        valid=(scores>.2)&(matches>=0)&(matches<len(q))
        return p[valid].cpu().numpy(),q[matches[valid]].cpu().numpy(),scores[valid].cpu().numpy()

    def pose(self,reference,live,camera):
        a,b,confidence=self.points(reference,live)
        self.last_points=(a,b)
        return self.pose_from_points(reference,a,b,camera)

    def pose_from_points(self,reference,a,b,camera):
        a=np.ascontiguousarray(a,dtype=np.float32)
        b=np.ascontiguousarray(b,dtype=np.float32)
        if len(a)<8:
            return None
        points,valid=wall_points(reference,a,camera)
        if valid.sum()<8:
            return None
        ok,rv,tv,inliers=cv2.solvePnPRansac(points[valid],b[valid],camera,None,
                    iterationsCount=300,reprojectionError=3,confidence=.995,flags=cv2.SOLVEPNP_EPNP)
        if not ok or inliers is None or len(inliers)<8:
            return None
        chosen=inliers.ravel()
        self.last_stats={'matches':len(a),'valid':int(valid.sum()),'inliers':len(chosen)}
        ok,rv,tv=cv2.solvePnP(points[valid][chosen],b[valid][chosen],camera,None,
                              rv,tv,True,flags=cv2.SOLVEPNP_ITERATIVE)
        if not ok:
            return None
        matrix,_=cv2.Rodrigues(rv)
        if abs(tv[1,0])>.5 or abs(matrix[1,0])>.15 or abs(matrix[1,2])>.15:
            return None
        return (float(tv[0,0]),float(tv[2,0]),float(np.arctan2(matrix[0,2],matrix[2,2])),len(chosen))
