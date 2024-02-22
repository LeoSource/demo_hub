# -*- coding: utf-8 -*-
# @File   		: dcm2mha.py
# @Description	: convert DICOM series files to .mha file
# @Date   		: 2024/02/02 14:34:40
# @Author	    : zxliao, zhixiangleo@163.com

import os
import pydicom
import SimpleITK as sitk
import numpy as np
import matplotlib.pyplot as plt

# https://cloud.tencent.com/developer/article/1652244
dcmfile_list = []
# dcm_path = "C:/Default_D/leo/MSpace/prca/dicom/20231229/002/1.2.840.113619.2.428.3.695552.238.1703812878.766"
dcm_path = "F:/0_project/prca/dicom/20240122/1.2.840.113619.2.428.3.695552.36.1705889324.84"

def read_dcm_series(file_path):
    reader = sitk.ImageSeriesReader()
    dcm_names = reader.GetGDCMSeriesFileNames(file_path)
    reader.SetFileNames(dcm_names)
    return reader.Execute()

def dcm2mha_with_pydicom():
    for dir_name,subdir_list,file_list in os.walk(dcm_path):
        for file_name in file_list:
            if ".dcm" in file_name.lower():
                # print(file_name)
                dcmfile_list.append(os.path.join(dir_name,file_name))

    ref_ds = pydicom.read_file(dcmfile_list[0])
    pixel_dims = (int(ref_ds.Rows), int(ref_ds.Columns), len(dcmfile_list))
    pixel_spacing = (float(ref_ds.PixelSpacing[0]), float(ref_ds.PixelSpacing[1]), float(ref_ds.SliceThickness))
    origin = ref_ds.ImagePositionPatient
    dcm_array = np.zeros(pixel_dims,dtype=ref_ds.pixel_array.dtype)

    for dcm_name in dcmfile_list:
        dcm = pydicom.read_file(dcm_name)
        dcm_array[:,:,dcmfile_list.index(dcm_name)] = dcm.pixel_array
        plt.figure
        plt.imshow(dcm.pixel_array,cmap='gray')
        plt.show()
    dcm_array = np.transpose(dcm_array,(2,0,1))
    sitk_image = sitk.GetImageFromArray(dcm_array,isVector=False)
    sitk_image.SetSpacing(pixel_spacing)
    sitk_image.SetOrigin(origin)
    sitk.WriteImage(sitk_image,"test1.mha")

def dcm2mha_with_sitk():
    # read dicom series image
    reader = sitk.ImageSeriesReader()
    dcm_names = reader.GetGDCMSeriesFileNames(dcm_path)
    reader.SetFileNames(dcm_names)
    image = reader.Execute()

    # show dicom series image message
    size = image.GetSize()
    print("Image size:",size)
    spacing = image.GetSpacing() # x,y,z
    print("Image spacing:",spacing)
    direction = image.GetDirection() #x,y,z
    print("Image direction:",direction)
    origin = image.GetOrigin() # x,y,z
    print("Image origin:",origin)

    # write dicom image
    sitk.WriteImage(image,"20240122_4.mha")

    # sitk image to numpy data
    image_array = sitk.GetArrayFromImage(image) # z,y,x
    print("np_array size:",image_array.shape)

    # image2 = sitk.GetImageFromArray(image_array)
    # image2.SetSpacing(spacing)
    # image2.SetDirection(direction)
    # image2.SetOrigin(origin)
    # sitk.WriteImage(image2,"test2.mha")

def resample_with_spacing(img,dest_spacing):
    src_size = img.GetSize()
    src_spacing = img.GetSpacing()
    dest_size = np.array(src_size)*np.array(src_spacing)/np.array(dest_spacing)
    dest_size = np.round(dest_size).astype(np.int32).tolist()

    return sitk.Resample(img,dest_size,sitk.Transform(),sitk.sitkLinear,img.GetOrigin(),
                         dest_spacing,img.GetDirection(),0,img.GetPixelIDValue())

def resample_with_size(img,dest_size):
    src_size = img.GetSize()
    src_spacing = img.GetSpacing()
    dest_spacing = np.array(src_size)*np.array(src_spacing)/np.array(dest_size)

    return sitk.Resample(img,dest_size,sitk.Transform(),sitk.sitkLinear,img.GetOrigin(),
                         dest_spacing,img.GetDirection(),0,img.GetPixelIDValue())

if __name__ == '__main__':
    dcm2mha_with_sitk()
    # dcm2mha_with_pydicom()
    # img = read_dcm_series(dcm_path)
    # img_resampled = resample_with_spacing(img,[0.5,0.5,0.5])
    img_resampled = resample_with_size(img,[1024,1024,500])
    sitk.WriteImage(img_resampled,"resampled.mha")
