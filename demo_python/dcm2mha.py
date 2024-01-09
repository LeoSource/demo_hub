# !/bin/python3

import os
import pydicom
import SimpleITK as sitk

# https://cloud.tencent.com/developer/article/1652244
dcmfile_list = []
dcm_path = "C:/Default_D/leo/MSpace/prca/dicom/20231229/002/1.2.840.113619.2.428.3.695552.238.1703812878.766"
for dir_name,subdir_list,file_list in os.walk(dcm_path):
    for file_name in file_list:
        if ".dcm" in file_name.lower():
            # print(file_name)
            dcmfile_list.append(os.path.join(dir_name,file_name))

reader = sitk.ImageSeriesReader()
dcm_names = reader.GetGDCMSeriesFileNames(dcm_path)
image = reader.Execute()
