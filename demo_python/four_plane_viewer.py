# -*- coding: utf-8 -*-
# @File   		: four_pane_viewer.py
# @Description	: do something
# @Date   		: 2024/04/02 11:01:17
# @Author	    : zxliao, zhixiangleo@163.com

import vtk

colors = vtk.vtkNamedColors()
dcm_path = "F:/0_project/prca/dicom/20240225/2024.02.25-144314-STD-1.3.12.2.1107.5.99.3/20240225/1.3.12.2.1107.5.1.7.120479.30000024022512255527200003523"


class DICOMViewer():
    def __init__(self,dcm_path:str):
        reader = vtk.vtkDICOMImageReader()
        reader.SetDirectoryName(dcm_path)
        reader.Update()
        flip = vtk.vtkImageFlip()
        flip.SetFilteredAxes(2)
        flip.SetInputConnection(reader.GetOutputPort())
        flip.Update()
        vtk_img_data = flip.GetOutput()
        vtk_img_dims = vtk_img_data.GetDimensions()

        riw = vtk.vtkResliceImageViewer()
        rw = vtk.vtkRenderWindow()
        iren = vtk.vtkRenderWindowInteractor()
        riw.SetInputData(vtk_img_data)
        riw.SetRenderWindow(rw)
        riw.SetupInteractor(iren)
        # riw.SetSize(512,512)
        # riw.SetColorLevel(0)
        # riw.SetColorWindow(500)
        riw.GetRenderer().SetBackground(colors.GetColor3d('gray'))
        riw.SetSliceOrientation(2)
        riw.SetResliceModeToAxisAligned()

        # ipw = vtk.vtkImagePlaneWidget()
        # ipw.SetInteractor(iren)
        # ipw.RestrictPlaneToVolumeOn()
        # ipw.GetPlaneProperty().SetColor(colors.GetColor3d('blue'))
        # # ipw.SetTexturePlaneProperty(vtk.vtkPro)
        # ipw.SetResliceInterpolateToLinear()
        # ipw.SetInputData(vtk_img_data)
        # ipw.SetPlaneOrientation(2)
        # # ipw.SetSliceIndex(round(vtk_img_dims[2]/2))
        # ipw.SetSliceIndex(0)
        # ipw.DisplayTextOn()
        # ipw.SetDefaultRenderer(riw.GetRenderer())
        # # ipw.SetWindowLevel(5000,200,1)
        # ipw.On()
        # ipw.InteractionOff()

        rc = vtk.vtkResliceCursor()
        center = vtk_img_data.GetCenter()
        rc.SetCenter(center[0],center[1],center[2])
        rc.SetThickMode(0)
        rc.SetThickness(10,10,10)
        rc.SetHole(0)
        rc.SetImage(vtk_img_data)
        rcw = vtk.vtkResliceCursorWidget()
        rcp = vtk.vtkResliceCursorLineRepresentation()
        rcw.SetInteractor(iren)
        rcw.SetRepresentation(rcp)
        rcp.GetResliceCursorActor().GetCursorAlgorithm().SetResliceCursor(rc)
        rcp.GetResliceCursorActor().GetCursorAlgorithm().SetReslicePlaneNormal(2)
        # reslice = rcp.GetReslice()
        # reslice.SetInputData(vtk_img_data)
        # reslice.AutoCropOutputOn()
        # reslice.Update()
        rcw.SetDefaultRenderer(riw.GetRenderer())
        rcw.SetEnabled(1)
        rcp.SetWindowLevel(500,0,0)
        

        riw.Render()
        riw.GetInteractor().Start()



if __name__=='__main__':
    viewer = DICOMViewer(dcm_path)
