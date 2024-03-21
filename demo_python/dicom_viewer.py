# -*- coding: utf-8 -*-
# @File   		: dicom_viewer.py
# @Description	: do something
# @Date   		: 2024/03/20 14:44:16
# @Author	    : zxliao, zhixiangleo@163.com

import vtk

dcm_path = "F:/0_project/prca/dicom/20240225/2024.02.25-144314-STD-1.3.12.2.1107.5.99.3/20240225/1.3.12.2.1107.5.1.7.120479.30000024022512255527200003523"

class CustomInteractorStyle(vtk.vtkInteractorStyleImage):
    def __init__(self, img_viewer):
        self.AddObserver(vtk.vtkCommand.MouseMoveEvent, self.on_mouse_move)
        self.AddObserver(vtk.vtkCommand.MouseWheelForwardEvent, self.on_scroll_forward)
        self.AddObserver(vtk.vtkCommand.MouseWheelBackwardEvent, self.on_scroll_backward)
        # self.AddObserver(vtk.vtkCommand.LeftButtonPressEvent, self.on_left_button_press)
        # self.AddObserver(vtk.vtkCommand.LeftButtonReleaseEvent, self.on_left_button_release)
        # self.AddObserver("RightButtonPressEvent", self.onRightButtonDown)
        # self.AddObserver("RightButtonReleaseEvent", self.onRightButtonUp)
        # self.AddObserver("MiddleButtonPressEvent", self.onMiddleButtonDown)
        # self.AddObserver("MiddleButtonReleaseEvent", self.onMiddleButtonUp)

        self.img_viewer = img_viewer

        self.lastPickedActor = None
        self.lastPickedPosition = [0, 0, 0]

        self.windowWidth = 400
        self.windowLevel = 40

        self.isLeftButtonDown = False
        self.isRightButtonDown = False
        self.isMiddleButtonDown = False
        self.startPosition = [0, 0]

    def on_mouse_move(self, obj, event):
        interactor = self.GetInteractor()
        x, y = interactor.GetEventPosition()

        picker = vtk.vtkCellPicker()
        picker.SetTolerance(0.001)
        picker.Pick(x, y, 0, self.GetDefaultRenderer())
        pos = picker.GetPickPosition()
        print(pos)
        # print(interactor.GetEventPosition())

        if self.isLeftButtonDown:
            # Adjust window level based on mouse movement
            dx = x - self.startPosition[0]
            dy = y - self.startPosition[1]
            self.windowLevel += dy
            self.windowWidth += dx
            print(f"WL: {self.windowLevel}, WD: {self.windowWidth}")
        elif self.isMiddleButtonDown:
            # Move the camera to simulate image panning
            camera = self.GetDefaultRenderer().GetActiveCamera()
            camera.Azimuth(self.startPosition[0] - x)
            camera.Elevation(y - self.startPosition[1])
            camera.OrthogonalizeViewUp()
            self.startPosition = [x, y]
        super().OnMouseMove()
        # self.GetInteractor().Render()

    def on_scroll_forward(self, obj, event):
        print("Scrolling forward")
        # Increase slice number or zoom in

    def on_scroll_backward(self, obj, event):
        print("Scrolling backward")
        # Decrease slice number or zoom out

    def on_left_button_press(self, obj, event):
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
    def __init__(self,dcm_path):
        reader = vtk.vtkDICOMImageReader()
        reader.SetDirectoryName(dcm_path)
        reader.Update()

        ren_windows = [vtk.vtkRenderWindow() for _ in range(4)]
        rens = [vtk.vtkRenderer() for _ in range(4)]
        # irens = [vtk.vtkRenderWindowInteractor() for _ in range(4)]
        for idx,window in enumerate(ren_windows):
            window.AddRenderer(rens[idx])
            window.SetSize(512,512)
            window.SetPosition(512*(idx%2),512*(idx//2))

        # for idx,orientation in enumerate(["Axial","Coronal","Sagittal"]):#横断面,冠状面,矢状面
        #     img_viewer = vtk.vtkImageViewer2()
        #     img_viewer.SetInputConnection(reader.GetOutputPort())

        iren = vtk.vtkRenderWindowInteractor()

        self.view_axial = vtk.vtkImageViewer2()
        self.view_axial.SetInputConnection(reader.GetOutputPort())
        self.view_axial.SetRenderWindow(ren_windows[0])
        self.view_axial.GetRenderWindow().SetWindowName("Axial")
        self.view_axial.SetupInteractor(iren)
        style_axial = CustomInteractorStyle(self.view_axial)
        style_axial.SetDefaultRenderer(self.view_axial.GetRenderer())
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
