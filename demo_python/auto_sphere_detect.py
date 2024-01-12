from itk_tool_box import SimpleITKTool
import SimpleITK as sitk
import numpy as np
import cv2
import math
import pyransac3d as pyrsc
from sklearn.cluster import DBSCAN
import copy
import matplotlib.pyplot as plt

class AutoSphereDetect():
    def __init__(self):
        self.itk_tool = SimpleITKTool()
        self.error = 0
        self.msg = "detect success"
    
    
    def window_transform(self,img, windowWidth, windowCenter, normal=False):
        """
        return: trucated image according to window center and window width
        and normalized to [0,1]
        """
        ct_array = sitk.GetArrayFromImage(img)   
        minWindow = float(windowCenter) - 0.5*float(windowWidth)
        new_array = (ct_array - minWindow) / float(windowWidth)
        new_array[new_array < 0] = 0
        new_array[new_array > 1] = 1
        if not normal:
            new_array = (new_array * 255).astype('float32')
        new_image=sitk.GetImageFromArray(new_array)
        new_image.CopyInformation(img)
        return new_image
    
    def cal_ROIVolume(self,spacing,voxel_num):
        unitVol = np.prod(spacing)
        roiVol = unitVol * voxel_num
        return roiVol

    def seg_img(self,flag,img,shift_img):
        spacing = img.GetSpacing()
        ct_array = sitk.GetArrayFromImage(img)
        
        thre_mask = self.itk_tool.threshlod_seg(shift_img,256,254)
        connect_filter = sitk.ConnectedComponentImageFilter()
        connect_filter.SetFullyConnected(False)
        connect_re_mask = connect_filter.Execute(thre_mask)
        num_connected_label = connect_filter.GetObjectCount()
        np_connect_array = sitk.GetArrayFromImage(connect_re_mask)
        
        lss_filter = sitk.LabelShapeStatisticsImageFilter()
        lss_filter.Execute(connect_re_mask)
        
        #according volume to filter big component
        small_islands_mask = np.zeros_like(np_connect_array) 
        for label in range(1, num_connected_label + 1):
            voxel_num = lss_filter.GetNumberOfPixels(label)
            
            if flag == 0 and -50 < self.cal_ROIVolume(spacing,voxel_num)-268 < 80:
                small_islands_mask[np_connect_array == label] = 100
                
            if flag == 1 and -100 < self.cal_ROIVolume(spacing,voxel_num)-905 < 100:
                small_islands_mask[np_connect_array == label] = 100
                print(voxel_num)
                
        
        seed_list =[]
        if flag == 0:
            max_raidus = round(4/spacing[0])
        elif flag == 1:
            max_raidus = round(6/spacing[0])
            
        for j in range(small_islands_mask.shape[0]):
            slice = np.array( small_islands_mask[j,:,:],dtype=np.uint8)
            if slice.max() > 0:
                circles = cv2.HoughCircles(slice, cv2.HOUGH_GRADIENT, 1, 20, param1 = 10,
                    param2 = 10, minRadius = max_raidus-1, maxRadius = max_raidus)
                if circles is not None:
                    circles = np.uint16(np.around(circles))
                    for label in circles[0, :]:
                        seed_list.append([label[0],label[1],j])
        
        seg_mask = self.itk_tool.region_grow(shift_img, np.asarray(seed_list,dtype='int').tolist(),256,254)
         
        sphere_mask = connect_filter.Execute(seg_mask)
        sphere_array = sitk.GetArrayFromImage(sphere_mask)
        
        lss_filter.Execute(sphere_mask)
        num_connected_label = connect_filter.GetObjectCount()
        hu_and_volume = []
        for label in range(1,num_connected_label + 1):
            voxel_num = lss_filter.GetNumberOfPixels(label)
            mean = np.max(ct_array[sphere_array == label])
            hu_and_volume.append([self.cal_ROIVolume(spacing,voxel_num),mean])
            
        #classification
        sphere_pred = DBSCAN(eps = 100, min_samples = 1).fit_predict(np.asarray(hu_and_volume)) 
        if num_connected_label != 4:
            if num_connected_label > 4:
                sphere_label,sphere_copy = self.get_class_index(sphere_array,sphere_pred)
                if sphere_label != -1:
                    num_connected_label = 4
                    sphere_mask = sitk.GetImageFromArray(sphere_copy)
                    sphere_mask.CopyInformation(img)
                else:
                    self.msg = "sphere class missing correlation"
                    self.error = 2  # sphere class missing correlation
            else:
                self.msg = "sphere detect failed, detected num is lower than 4"
                self.error = 1 #sphere detect failed
        else:
            sphere_class = sphere_pred.tolist()
            if sphere_class.count(0) != 4:
                self.msg = "sphere class missing correlation"
                self.error = 2 # sphere class missing correlation
      
        return sphere_mask, num_connected_label
    
    def get_class_index(self,sphere_array,pre_array):
        pred = pre_array.tolist()
        sphere_label = -1
        for i in range(len(pred)):
            if pred.count(i) == 4:
                sphere_label = i
                break
        sphere_copy = copy.deepcopy(sphere_array)
        for i in range(len(pred)):
            if pred[i] != sphere_label:
                sphere_copy[sphere_array==(i + 1)] = 0
        return sphere_label, sphere_copy
            
            
    def sphere_fit(self,point_set):
        sphere_rsc = pyrsc.Sphere()
        if point_set.shape[0] > 10:
           center,radius,inlier = sphere_rsc.fit(point_set,thresh = 1, maxIteration = 100)
        else:
            self.error = 3 # point set not available
            self.msg = "sphere point cloud not meet the requirements"
            center = 0
            radius = 0
        return center,radius
    
    
    def get_physicalPoints(self,img,contour_arr,label):
        index = np.argwhere(contour_arr==label)
        sphere_points = []
        for e in index:
            sphere_points.append(img.TransformContinuousIndexToPhysicalPoint(e.tolist()[::-1]))
  
        return sphere_points
    
    def get_center_set(self, mask, num):
        contour = sitk.LabelContour(mask) 
        contour_arr = sitk.GetArrayFromImage(contour)
        center_list = []
        radius_list = []
        for label in range(1,num + 1):
            point_set = np.asarray(self.get_physicalPoints(mask,contour_arr,label))
            center, radius = self.sphere_fit(point_set)
            center_list.append(center)
            radius_list.append(radius)
                
        return center_list, radius_list
        
    def get_distance(self,vec1, vec2):
        #assert vec1.shape == vec2.shape, r'The input parameter shape must be the same' 
        return math.sqrt((vec1[0]-vec2[0])**2 + (vec1[1]-vec2[1])**2 + (vec1[2]-vec2[2])**2 )
    
    def get_continuous_index(self,img,center_list):
        index_list = []
        for i in range(len(center_list)):
            index = img.TransformPhysicalPointToContinuousIndex(center_list[i])
            index_list.append(index)
        return index_list
    
    def reorder_sphere_list(self,flag,center_list):
        if flag == 0 and len(center_list) == 4:
            center = np.mean(np.asarray(center_list),axis=0)
            point_right =[]
            point_left =[]
            for p in center_list:
                if p[0] > center[0]:
                    point_left.append(p)
                else:
                    point_right.append(p)
            point_left.sort(key=lambda x:x[1]) 
            point_right.sort(key=lambda x:x[1])
            if abs(self.get_distance(point_left[0],point_left[1])-26.57) < 1.0:
                reordered_list = [point_right[0],point_left[0],point_left[1],point_right[1]] 
            else:
                #reflect
                reordered_list = [point_left[0],point_right[0],point_right[1],point_left[1]]   
            return reordered_list
        elif flag ==1 and len(center_list) == 4:
            center = np.mean(np.asarray(center_list),axis=0)
            point_right =[]
            point_left =[]
            for p in center_list:
                if p[0] > center[0]:
                    point_left.append(p)
                else:
                    point_right.append(p)
            point_left.sort(key=lambda x:x[1]) 
            point_right.sort(key=lambda x:x[1])
            if abs(point_left[0][2]-point_left[1][2])> 5.0:
                reordered_list = [point_right[1],point_left[1],point_left[0],point_right[0]] 
            else:
                #reflect
                reordered_list = [point_left[1],point_right[1],point_right[0],point_left[0]] 
            
            return center_list
           
    def run(self,flag,file_path):
        img = sitk.ReadImage(file_path)
        img_array = sitk.GetArrayFromImage(img)
        for idx in range(0,img.GetSize()[2]):
            plt.imshow(img_array[idx,:,:],cmap=plt.cm.gray)
        shift_img = self.window_transform(img,1000,300) #shift img to bone window -200-800
        mask,num = self.seg_img(flag,img,shift_img) 
        center_physi_list,radius_list = self.get_center_set(mask,num)
        center_physi_list = self.reorder_sphere_list(flag,center_physi_list)
        center_index_list = self.get_continuous_index(img,center_physi_list)
        center_array = np.hstack([center_index_list,np.asarray(center_physi_list),np.asarray(radius_list).reshape(len(radius_list),1)])
        return center_array,self.error

if __name__ == '__main__': 
  
    #file_path = 'D:/DataSet/sphere_test/203_1.5_2.nii.gz'  
    file_path = "F:/0_project/demo_hub/demo_python/test2.mha"
    #file_path = "D:\\DataSet\\test.nii.gz"
    auto_detect = AutoSphereDetect()
    center_array,error = auto_detect.run(1,file_path)
    
    
