# -*- coding: utf-8 -*-
# @File   		: dicom_viewer.py
# @Description	: do something
# @Date   		: 2024/03/20 14:44:16
# @Author	    : zxliao, zhixiangleo@163.com

import vtk
import SimpleITK as sitk

dcm_path = "F:/0_project/prca/dicom/20240225/2024.02.25-144314-STD-1.3.12.2.1107.5.99.3/20240225/1.3.12.2.1107.5.1.7.120479.30000024022512255527200003523"

def sitk_read_dcm_series(file_path:str):
    reader = sitk.ImageSeriesReader()
    dcm_names = reader.GetGDCMSeriesFileNames(file_path)
    reader.SetFileNames(dcm_names)
    return reader.Execute()

colors = vtk.vtkNamedColors()

class CustomInteractorStyle(vtk.vtkInteractorStyleImage):
    def __init__(self, img_viewer):
        self.AddObserver(vtk.vtkCommand.MouseWheelForwardEvent, self.on_scroll_forward)
        self.AddObserver(vtk.vtkCommand.MouseWheelBackwardEvent, self.on_scroll_backward)
        self.AddObserver(vtk.vtkCommand.LeftButtonPressEvent, self.on_left_button_press)
        self.AddObserver(vtk.vtkCommand.LeftButtonReleaseEvent, self.on_left_button_release)
        self.AddObserver(vtk.vtkCommand.MouseMoveEvent, self.on_mouse_move)
        # self.AddObserver("RightButtonPressEvent", self.onRightButtonDown)
        # self.AddObserver("RightButtonReleaseEvent", self.onRightButtonUp)
        # self.AddObserver("MiddleButtonPressEvent", self.onMiddleButtonDown)
        # self.AddObserver("MiddleButtonReleaseEvent", self.onMiddleButtonUp)

        self.img_viewer = img_viewer
        self.sitk_img = sitk_read_dcm_series(dcm_path)

        self.slice_max = img_viewer.GetSliceMax()
        self.slice_min = img_viewer.GetSliceMin()
        self.slice = round(0.5*(self.slice_min+self.slice_max))
        self.img_viewer.SetSlice(self.slice)
        self.wl = img_viewer.GetColorLevel()
        self.wd = img_viewer.GetColorWindow()

        self.isLeftButtonDown = False
        self.isRightButtonDown = False

    def on_mouse_move(self, obj, event):
        print('mouse move')
        super().OnMouseMove()

    def on_scroll_forward(self, obj, event):
        print('scroll forward')
        # super().OnMouseWheelForward()
        if self.slice<self.slice_max:
            self.slice += 1
            self.img_viewer.SetSlice(self.slice)
            self.img_viewer.Render()

    def on_scroll_backward(self, obj, event):
        # super().OnMouseWheelBackward()
        if self.slice>self.slice_min:
            self.slice -= 1
            self.img_viewer.SetSlice(self.slice)
            self.img_viewer.Render()

    def on_left_button_press(self, obj, event):
        print('left button pressed')
        self.isLeftButtonDown = True
        self.startPosition = self.GetInteractor().GetEventPosition()
        super().OnLeftButtonDown()

    def on_left_button_release(self, obj, event):
        self.isLeftButtonDown = False
        super().OnLeftButtonUp()

    # def onRightButtonDown(self, obj, event):
    #     self.isRightButtonDown = True
    #     self.startPosition = self.GetInteractor().GetEventPosition()
    #     super().OnRightButtonDown()

    # def onRightButtonUp(self, obj, event):
    #     self.isRightButtonDown = False
    #     self.isRightButtonDown = False
    #     super().OnRightButtonUp()

    # def onMiddleButtonDown(self, obj, event):
    #     self.isMiddleButtonDown = True
    #     self.startPosition = self.GetInteractor().GetEventPosition()
    #     super().OnMiddleButtonDown()

    # def onMiddleButtonUp(self, obj, event):
    #     self.isMiddleButtonDown = False
    #     super().OnMiddleButtonUp()

class DICOMViewer():
    def __init__(self,dcm_path:str):
        reader = vtk.vtkDICOMImageReader()
        reader.SetDirectoryName(dcm_path)
        reader.Update()
        # vtk_img_data = reader.GetOutput()
        flip = vtk.vtkImageFlip()
        flip.SetFilteredAxes(2)
        flip.SetInputConnection(reader.GetOutputPort())
        flip.Update()

        # rens = [vtk.vtkRenderer() for _ in range(4)]
        ren_windows = [vtk.vtkRenderWindow() for _ in range(4)]
        # irens = [vtk.vtkRenderWindowInteractor() for _ in range(4)]
        for idx,window in enumerate(ren_windows):
            # window.AddRenderer(rens[idx])
            window.SetSize(512,512)
            window.SetPosition(512*(idx%2),512*(idx//2))

        # for idx,orientation in enumerate(["Axial","Coronal","Sagittal"]):#横断面,冠状面,矢状面
        #     img_viewer = vtk.vtkImageViewer2()
        #     img_viewer.SetInputConnection(reader.GetOutputPort())

        iren = vtk.vtkRenderWindowInteractor()

        self.view_axial = vtk.vtkImageViewer2()
        self.view_axial.SetInputConnection(flip.GetOutputPort())
        self.view_axial.GetRenderer().SetBackground(colors.GetColor3d('gray'))
        self.view_axial.SetRenderWindow(ren_windows[0])
        self.view_axial.GetRenderWindow().SetWindowName("Axial")
        style_axial = CustomInteractorStyle(self.view_axial)
        # style_axial = vtk.vtkInteractorStyleImage()
        style_axial.SetDefaultRenderer(self.view_axial.GetRenderer())
        self.view_axial.SetupInteractor(iren)

        iren.SetInteractorStyle(style_axial)

        # self.view_coronal = vtk.vtkImageViewer2()
        # self.view_coronal.SetInputConnection(reader.GetOutputPort())
        # self.view_coronal.SetRenderWindow(ren_windows[1])

        # self.view_sagittal = vtk.vtkImageViewer2()
        # self.view_sagittal.SetInputConnection(reader.GetOutputPort())
        # self.view_sagittal.SetRenderWindow(ren_windows[2])


        self.view_axial.GetRenderer().ResetCamera()
        self.view_axial.Render()

        iren.Start()

if __name__=='__main__':
    dcm_viewer = DICOMViewer(dcm_path)
