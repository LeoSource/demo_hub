# -*- coding: utf-8 -*-
# @File   		: vtk_dcm_slice.py
# @Description	: do something
# @Date   		: 2024/03/07 09:56:35
# @Author	    : zxliao, zhixiangleo@163.com

import vtk
import tkinter as tk
from vtkmodules.tk.vtkTkRenderWindowInteractor import vtkTkRenderWindowInteractor

dcm_path = "F:/0_project/prca/dicom/20240225/2024.02.25-144314-STD-1.3.12.2.1107.5.99.3/20240225/1.3.12.2.1107.5.1.7.120479.30000024022512255527200003523"

# helper class to format slice status message
class StatusMessage:
    def format(slice:int,slice_max:int):
        return f'{slice+1}/{slice_max+1}'

class MyInteractorStyleImage(vtk.vtkInteractorStyleImage):
    def __init__(self) -> None:
        super().__init__()

        self.AddObserver(vtk.vtkCommand.MouseWheelForwardEvent,self.mouse_wheel_forward_event)
        self.AddObserver(vtk.vtkCommand.MouseWheelBackwardEvent,self.mouse_wheel_backward_event)

        self.img_viewer = None
        self.status_mapper = None
        self.slice = 0
        self.slice_min = 0
        self.slice_max = 0

    def set_image_viewer(self,img_viewer):
        self.img_viewer = img_viewer
        self.slice_min = img_viewer.GetSliceMin()
        self.slice_max = img_viewer.GetSliceMax()
        # self.slice = round(0.5*(self.slice_min+self.slice_max))
        self.slice = 0
        print(f'Slice: Min = {self.slice_min}, Max = {self.slice_max}')

    def set_status_mapper(self,status_mapper):
        self.status_mapper = status_mapper
        msg = StatusMessage.format(self.slice,self.slice_max)
        self.status_mapper.SetInput(msg)

    def move_slice_forward(self):
        if self.slice<self.slice_max:
            self.slice += 1
            print(f'MoveSliceForward::Slice = {self.slice}')
            self.img_viewer.SetSlice(self.slice)
            msg = StatusMessage.format(self.slice,self.slice_max)
            self.status_mapper.SetInput(msg)
            self.img_viewer.Render()

    def move_slice_backward(self):
        if self.slice>self.slice_min:
            self.slice -= 1
            print(f'MoveSliceBackward::Slice = {self.slice}')
            self.img_viewer.SetSlice(self.slice)
            msg = StatusMessage.format(self.slice,self.slice_max)
            self.status_mapper.SetInput(msg)
            self.img_viewer.Render()

    def mouse_wheel_forward_event(self,obj,event):
        self.move_slice_forward()

    def mouse_wheel_backward_event(self,obj,event):
        self.move_slice_backward()

def read_dicom_slice():
    colors = vtk.vtkNamedColors()

    dcm_reader = vtk.vtkDICOMImageReader()
    dcm_reader.SetDirectoryName(dcm_path)
    img_width = dcm_reader.GetWidth()
    img_height = dcm_reader.GetHeight()
    dcm_reader.Update()
    # print(dcm_reader.GetPixelSpacing())
    # print(dcm_reader.GetImagePositionPatient())
    img_data = dcm_reader.GetOutput()#vtk.vtkImageData
    # print(img_data.GetPointData(10,20,30))

    img_viewer = vtk.vtkImageViewer2()
    img_viewer.SetInputConnection(dcm_reader.GetOutputPort())
    iren = vtk.vtkRenderWindowInteractor()
    img_viewer.SetupInteractor(iren)
    # img_viewer.SetSlice(100)

    slice_text_prop = vtk.vtkTextProperty()
    slice_text_prop.SetFontFamilyToCourier()
    slice_text_prop.SetFontSize(20)
    slice_text_prop.SetVerticalJustificationToBottom()
    slice_text_prop.SetJustificationToLeft()

    slice_text_mapper = vtk.vtkTextMapper()
    slice_text_mapper.SetTextProperty(slice_text_prop)

    slice_text_actor = vtk.vtkActor2D()
    slice_text_actor.SetMapper(slice_text_mapper)
    slice_text_actor.SetPosition(15,10)

    usage_text_prop = vtk.vtkTextProperty()
    usage_text_prop.SetFontFamilyToCourier()
    usage_text_prop.SetFontSize(14)
    usage_text_prop.SetVerticalJustificationToTop()
    usage_text_prop.SetJustificationToLeft()

    usage_text_mapper = vtk.vtkTextMapper()
    usage_text_mapper.SetInput(
        'Slice with mouse wheel\n - Zoom with pressed right\n '
        ' mouse button while dragging'
    )
    usage_text_mapper.SetTextProperty(usage_text_prop)

    usage_text_actor = vtk.vtkActor2D()
    usage_text_actor.SetMapper(usage_text_mapper)
    usage_text_actor.GetPositionCoordinate().SetCoordinateSystemToNormalizedDisplay()
    usage_text_actor.GetPositionCoordinate().SetValue(0.05,0.95)

    istyle = MyInteractorStyleImage()
    istyle.set_image_viewer(img_viewer)
    istyle.set_status_mapper(slice_text_mapper)
    iren.SetInteractorStyle(istyle)
    iren.Render()

    # print(img_viewer.GetColorLevel())
    img_viewer.GetRenderer().AddActor2D(slice_text_actor)
    img_viewer.GetRenderer().AddActor2D(usage_text_actor)
    img_viewer.GetRenderer().ResetCamera()
    img_viewer.GetRenderer().SetBackground(colors.GetColor3d('slavegray'))
    img_viewer.GetRenderWindow().SetSize(img_width,img_height)
    img_viewer.GetRenderWindow().SetWindowName('ReadDICOMSeries')
    img_viewer.Render()

    iren.Start()

def vtk_in_tkinter():
    root_window = tk.Tk()
    root_window.title('vtk in tkinter')
    root_window.geometry('1000x1000')
    vtk_frame = tk.Frame(root_window)
    vtk_frame.pack(fill=tk.BOTH,expand=1)

    colors = vtk.vtkNamedColors()

    sphere = vtk.vtkSphereSource()
    sphere.SetCenter(0,0,0)
    sphere.SetRadius(5)
    sphere.SetThetaResolution(64)
    sphere.SetPhiResolution(64)
    sphere_mapper = vtk.vtkPolyDataMapper()
    sphere_mapper.SetInputConnection(sphere.GetOutputPort())
    sphere_actor = vtk.vtkActor()
    sphere_actor.SetMapper(sphere_mapper)

    renderer = vtk.vtkRenderer()
    renderer.AddActor(sphere_actor)
    ren_win = vtk.vtkRenderWindow()
    ren_win.AddRenderer(renderer)
    interactor = vtkTkRenderWindowInteractor(root_window,rw=ren_win)
    interactor.Initialize()
    interactor.Start()
    ren_win.Render()

    root_window.mainloop()

if __name__=='__main__':
    read_dicom_slice()


