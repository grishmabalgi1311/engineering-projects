"""Compare textured wall pixels visible in both views using image-derived depths."""
import cv2
import numpy as np
from visual_pose import wall_points


def visible_error(reference, live, pose, camera):
    h,w=reference.shape[:2]
    yy,xx=np.mgrid[4:h:4,4:w:4]
    pixels=np.c_[xx.ravel(),yy.ravel()].astype(np.float32)
    points,valid=wall_points(reference,pixels,camera)
    texture=abs(cv2.Laplacian(cv2.cvtColor(reference,cv2.COLOR_BGR2GRAY).astype(np.float32),cv2.CV_32F))
    valid &= texture[pixels[:,1].astype(int),pixels[:,0].astype(int)]>8
    x,z,yaw,_=pose;c,s=np.cos(yaw),np.sin(yaw)
    R=np.array([[c,0,s],[0,1,0],[-s,0,c]])
    transformed=points@R.T+[x,0,z]
    q=transformed@camera.T
    with np.errstate(invalid='ignore',divide='ignore'):
        uv=q[:,:2]/q[:,2:3]
    inside=valid&(transformed[:,2]>0)&np.isfinite(uv).all(axis=1)
    inside &= (uv[:,0]>=2)&(uv[:,0]<w-2)&(uv[:,1]>=2)&(uv[:,1]<h-2)
    ids=np.flatnonzero(inside)
    if len(ids)<80:return None
    live_points,visible=wall_points(live,uv[ids],camera)
    visible &= abs(live_points[:,2]-transformed[ids,2])<.1*np.maximum(1.,transformed[ids,2])
    ids=ids[visible]
    if len(ids)<80:return None
    first=cv2.GaussianBlur(reference,(5,5),0).astype(np.float32)
    second=cv2.GaussianBlur(live,(5,5),0).astype(np.float32)
    warped=cv2.remap(second,uv[ids,0].astype(np.float32)[None],uv[ids,1].astype(np.float32)[None],cv2.INTER_LINEAR)[0]
    p=pixels[ids].astype(int)
    errors=np.mean(abs(first[p[:,1],p[:,0]]-warped),axis=1)/255
    bands=[float(np.mean(errors[p[:,0]*4//w==i])) for i in range(4) if (p[:,0]*4//w==i).sum()>=20]
    return dict(mean=float(errors.mean()),worst=max(bands,default=1.),bands=len(bands),
                pixels=len(ids),coverage=float(len(ids)/max(1,valid.sum())))


def supported(reference, live, pose, camera):
    x,z,yaw,n=pose;c,s=np.cos(yaw),np.sin(yaw)
    inverse=(-c*x+s*z,-s*x-c*z,-yaw,n)
    for a,b,p in ((reference,live,pose),(live,reference,inverse)):
        e=visible_error(a,b,p,camera)
        if e is None or e['mean']>.05 or e['worst']>.10 or e['bands']<2 or e['coverage']<.2:
            return False
    return True
