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
class SliceMessage:
    def format(slice:int,slice_max:int):
        return f'{slice+1}/{slice_max+1}'
    
class WindowMessage:
    def format(wd:int,wl:int):
        return f'WD:{round(wd)}\nWL:{round(wl)}'

class MyInteractorStyleImage(vtk.vtkInteractorStyleImage):
    def __init__(self) -> None:
        super().__init__()

        self.AddObserver(vtk.vtkCommand.MouseWheelForwardEvent,self.mouse_wheel_forward_event)
        self.AddObserver(vtk.vtkCommand.MouseWheelBackwardEvent,self.mouse_wheel_backward_event)
        self.AddObserver(vtk.vtkCommand.LeftButtonPressEvent,self.on_left_button_press)
        self.AddObserver(vtk.vtkCommand.LeftButtonReleaseEvent,self.on_left_button_release)
        self.AddObserver(vtk.vtkCommand.MouseMoveEvent,self.on_mouse_move)
        # self.AddObserver(vtk.vtkCommand.WindowLevelEvent,self.mouse_move_event)

        self.img_viewer = None
        self.slice_mapper = None
        self.slice_slider_widget = None
        self.window_mapper = None
        self.slice = 0
        self.slice_min = 0
        self.slice_max = 0
        self.wl = 0
        self.wd = 0
        self.is_left_button_pressed = False
        self.last_pos = None

    def mouse_wheel_forward_event(self,obj,event):
        self.move_slice_forward()

    def mouse_wheel_backward_event(self,obj,event):
        self.move_slice_backward()

    def on_left_button_press(self, obj, event):
        self.is_left_button_pressed = True
        self.last_pos = self.GetInteractor().GetEventPosition()
        super().OnLeftButtonDown()  # Call parent method to ensure proper behavior

    def on_mouse_move(self, obj, event):
        if self.is_left_button_pressed:
            new_pos = self.GetInteractor().GetEventPosition()
            dx = new_pos[0] - self.last_pos[0]
            dy = new_pos[1] - self.last_pos[1]

            # Adjust window level based on dx, dy
            # This is a simplified example; you may want to adjust the scale of adjustment
            self.wl += dy
            self.wd += dx

            # Update the image viewer and text display
            self.img_viewer.SetColorLevel(round(self.wl))
            self.img_viewer.SetColorWindow(round(self.wd))
            msg = WindowMessage.format(self.wd, self.wl)
            self.window_mapper.SetInput(msg)
            self.img_viewer.Render()
            print(f'{self.img_viewer.GetColorLevel()},{self.img_viewer.GetColorWindow()}')

            self.last_pos = new_pos
        else:
            super().OnMouseMove()  # Call parent method to ensure proper behavior

    def on_left_button_release(self, obj, event):
        self.is_left_button_pressed = False
        super().OnLeftButtonUp()  # Call parent method to ensure proper behavior

    def OnMouseMove(self) -> None:
        pass
        # super().OnMouseMove()
        # print(self.GetInteractor().GetEventPosition())
        # print(f'wl:{self.img_viewer.GetColorLevel()},wd:{self.img_viewer.GetColorWindow()}')
        # return super().OnMouseMove()

    # def WindowLevel(self) -> None:
        # print('123')
        # print(f'wl:{self.img_viewer.GetColorLevel()},wd:{self.img_viewer.GetColorWindow()}')
        # return super().WindowLevel()



    def set_image_viewer(self,img_viewer):
        img_viewer.SetSliceOrientationToXY()
        self.img_viewer = img_viewer
        self.slice_min = img_viewer.GetSliceMin()
        self.slice_max = img_viewer.GetSliceMax()
        self.wl = img_viewer.GetColorLevel()
        self.wd = img_viewer.GetColorWindow()
        # self.slice = round(0.5*(self.slice_min+self.slice_max))
        self.slice = 0
        # self.img_viewer.GetRenderer().SetBackground([1,0,1])
        print(f'Slice: Min = {self.slice_min}, Max = {self.slice_max}')
        print(f'wd:{self.wd},wl:{self.wl}')

    def set_slice_mapper(self,slice_mapper):
        self.slice_mapper = slice_mapper
        msg = SliceMessage.format(self.slice,self.slice_max)
        self.slice_mapper.SetInput(msg)

    def set_window_mapper(self,window_mapper):
        self.window_mapper = window_mapper
        msg = WindowMessage.format(self.wd,self.wl)
        self.window_mapper.SetInput(msg)

    def move_slice_forward(self):
        if self.slice<self.slice_max:
            self.slice += 1
            print(f'MoveSliceForward::Slice = {self.slice}')
            # print(f'window level:{self.wl},window width:{self.wd}')
            self.updata_slice()

    def move_slice_backward(self):
        if self.slice>self.slice_min:
            self.slice -= 1
            print(f'MoveSliceBackward::Slice = {self.slice}')
            self.updata_slice()

    def updata_slice(self):
        self.img_viewer.SetSlice(self.slice)
        msg = SliceMessage.format(self.slice,self.slice_max)
        self.slice_mapper.SetInput(msg)
        self.slice_slider_widget.GetRepresentation().SetValue(self.slice)
        self.wd = self.img_viewer.GetColorWindow()
        self.wl = self.img_viewer.GetColorLevel()
        # msg = WindowMessage.format(self.wd,self.wl)
        # self.window_mapper.SetInput(msg)
        self.img_viewer.Render()

    def create_slice_slider(self,iren):
        slider_rep = vtk.vtkSliderRepresentation2D()
        slider_rep.SetMinimumValue(self.slice_min)
        slider_rep.SetMaximumValue(self.slice_max)
        slider_rep.SetValue(self.slice)
        slider_rep.GetSliderProperty().SetColor(1,0,0)
        slider_rep.GetSelectedProperty().SetColor(0,0,1)
        slider_rep.GetTubeProperty().SetColor(1,1,0)
        slider_rep.GetCapProperty().SetColor(0,1,1)
        slider_rep.GetPoint1Coordinate().SetCoordinateSystemToNormalizedDisplay()
        slider_rep.GetPoint1Coordinate().SetValue(0.95,0.3)
        slider_rep.GetPoint2Coordinate().SetCoordinateSystemToNormalizedDisplay()
        slider_rep.GetPoint2Coordinate().SetValue(0.95,0.7)
        slider_rep.SetSliderLength(0.02)
        slider_rep.SetSliderWidth(0.02)
        slider_rep.SetTubeWidth(0.005)
        slider_rep.SetEndCapWidth(0.03)
        slider_rep.ShowSliderLabelOff()
        # slider_rep.ShowSliderLabelOn()
        # slider_rep.SetLabelFormat("%d")

        self.slice_slider_widget = vtk.vtkSliderWidget()
        self.slice_slider_widget.SetRepresentation(slider_rep)
        self.slice_slider_widget.SetAnimationModeToAnimate()
        self.slice_slider_widget.SetInteractor(iren)
        self.slice_slider_widget.EnabledOn()

        self.slice_slider_widget.AddObserver(vtk.vtkCommand.InteractionEvent,self.slice_slider_callback)

    def slice_slider_callback(self,obj,event):
        slider_rep = obj.GetRepresentation()
        self.slice = round(slider_rep.GetValue())
        self.updata_slice()

    def create_slice_text(self):
        slice_text_prop = vtk.vtkTextProperty()
        slice_text_prop.SetFontFamilyToCourier()
        slice_text_prop.SetFontSize(20)
        slice_text_prop.SetVerticalJustificationToBottom()
        slice_text_prop.SetJustificationToLeft()

        slice_text_mapper = vtk.vtkTextMapper()
        slice_text_mapper.SetTextProperty(slice_text_prop)
        self.set_slice_mapper(slice_text_mapper)

        slice_text_actor = vtk.vtkActor2D()
        slice_text_actor.SetMapper(slice_text_mapper)
        slice_text_actor.SetPosition(15,10)

        self.img_viewer.GetRenderer().AddActor2D(slice_text_actor)

    def create_window_text(self):
        window_text_prop = vtk.vtkTextProperty()
        window_text_prop.SetFontFamilyToCourier()
        window_text_prop.SetFontSize(20)
        window_text_prop.SetVerticalJustificationToCentered()
        window_text_prop.SetJustificationToLeft()

        window_text_mapper = vtk.vtkTextMapper()
        window_text_mapper.SetTextProperty(window_text_prop)
        self.set_window_mapper(window_text_mapper)

        window_text_actor = vtk.vtkActor2D()
        window_text_actor.SetMapper(window_text_mapper)
        window_text_actor.SetPosition(270,30)

        self.img_viewer.SetColorLevel(46)
        self.img_viewer.SetColorWindow(240)
        self.img_viewer.GetRenderer().AddActor2D(window_text_actor)


def read_dicom_slice():
    colors = vtk.vtkNamedColors()

    dcm_reader = vtk.vtkDICOMImageReader()
    dcm_reader.SetDirectoryName(dcm_path)
    dcm_reader.Update()
    img_width = dcm_reader.GetWidth()
    img_height = dcm_reader.GetHeight()
    print(dcm_reader.GetImageOrientationPatient())
    # print(dcm_reader.GetPixelSpacing())
    # print(dcm_reader.GetImagePositionPatient())
    img_data = dcm_reader.GetOutput()#vtk.vtkImageData
    # print(img_data.GetPointData(10,20,30))
    flip = vtk.vtkImageFlip()
    flip.SetFilteredAxes(2)
    flip.SetInputData(img_data)
    flip.Update()

    img_viewer = vtk.vtkImageViewer2()
    # img_viewer.SetInputConnection(dcm_reader.GetOutputPort())
    img_viewer.SetInputConnection(flip.GetOutputPort())
    iren = vtk.vtkRenderWindowInteractor()
    img_viewer.SetupInteractor(iren)
    # img_viewer.SetSlice(100)

    # print(img_viewer.GetColorLevel())
    # img_viewer.GetRenderer().AddActor2D(usage_text_actor)
    dicom_renderer = vtk.vtkRenderer()
    dicom_renderer.SetBackground(colors.GetColor3d('red'))
    # img_viewer.GetRenderer().AddActor2D(usage_annotation)
    # img_viewer.GetRenderer().ResetCamera()
    # img_viewer.GetRenderer().SetBackground(colors.GetColor3d('black'))
    # img_viewer.GetRenderWindow().SetNumberOfLayers(2)
    # dicom_renderer.SetLayer(0)
    # img_viewer.GetRenderWindow().AddRenderer(dicom_renderer)
    img_viewer.GetRenderWindow().SetSize(img_width,img_height)
    img_viewer.GetRenderWindow().SetWindowName('ReadDICOMSeries')
    # img_viewer.SetRenderer(dicom_renderer)
    img_viewer.SetColorLevel(112)
    img_viewer.SetColorWindow(443)

    usage_text_prop = vtk.vtkTextProperty()
    usage_text_prop.SetFontFamilyToCourier()
    usage_text_prop.SetFontSize(14)
    usage_text_prop.SetVerticalJustificationToTop()
    usage_text_prop.SetJustificationToLeft()

    # usage_text_mapper = vtk.vtkTextMapper()
    # usage_text_mapper.SetInput(
    #     'Slice with mouse wheel\n - Zoom with pressed right\n '
    #     ' mouse button while dragging'
    # )
    # usage_text_mapper.SetTextProperty(usage_text_prop)

    # usage_text_actor = vtk.vtkActor2D()
    # usage_text_actor.SetMapper(usage_text_mapper)
    # usage_text_actor.GetPositionCoordinate().SetCoordinateSystemToNormalizedDisplay()
    # usage_text_actor.GetPositionCoordinate().SetValue(0.05,0.95)

    usage_annotation = vtk.vtkCornerAnnotation()
    usage_annotation.SetTextProperty(usage_text_prop)
    usage_annotation.SetText(2,"hello world!")
    print(usage_annotation.GetWindowLevel())

    mt_interactor = MyInteractorStyleImage()
    mt_interactor.SetDefaultRenderer(img_viewer.GetRenderer())
    mt_interactor.set_image_viewer(img_viewer)
    mt_interactor.create_slice_text()
    mt_interactor.create_window_text()
    mt_interactor.create_slice_slider(iren)
    iren.SetInteractorStyle(mt_interactor)
    # iren.Render()

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


