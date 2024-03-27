# -*- coding: utf-8 -*-
# @File   		: dicom_viewer.py
# @Description	: do something
# @Date   		: 2024/03/20 14:44:16
# @Author	    : zxliao, zhixiangleo@163.com

import vtk
import SimpleITK as sitk

dcm_path = "D:/Leo/0project/prca/dicom/20231229/002/1.2.840.113619.2.428.3.695552.238.1703812878.766"

def sitk_read_dcm_series(file_path:str)->sitk.Image:
    reader = sitk.ImageSeriesReader()
    dcm_names = reader.GetGDCMSeriesFileNames(file_path)
    reader.SetFileNames(dcm_names)
    return reader.Execute()

colors = vtk.vtkNamedColors()

class CustomInteractorStyle(vtk.vtkInteractorStyleImage):
    def __init__(self, img_viewer:vtk.vtkImageViewer2,iren:vtk.vtkRenderWindowInteractor):
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

        self.slice_max = int(img_viewer.GetSliceMax())
        self.slice_min = int(img_viewer.GetSliceMin())
        self.slice = int(0.5*(self.slice_min+self.slice_max))
        self.img_viewer.SetSlice(self.slice)
        self.wl = int(img_viewer.GetColorLevel())
        self.wd = int(img_viewer.GetColorWindow())
        self.is_left_button_pressed = False
        self.last_pos = None
        self.pos_ct = [0,0,0]
        self.pos_img = [0,0,0]
        self.ct_value = 0

        self.create_left_down_corner_annotation()
        self.create_right_down_corner_annotation()
        self.create_slice_slider(iren)

    def on_mouse_move(self, obj, event):
        if self.is_left_button_pressed:
            new_pos = self.GetInteractor().GetEventPosition()
            dx = new_pos[0] - self.last_pos[0]
            dy = new_pos[1] - self.last_pos[1]
            # Adjust window level based on dx, dy
            # This is a simplified example; you may want to adjust the scale of adjustment
            scale = 1
            self.wl += round(scale*dy)
            self.wd += round(scale*dx)
            # Update the image viewer and text display
            self.img_viewer.SetColorLevel(self.wl)
            self.img_viewer.SetColorWindow(self.wd)
            self.calc_cursor_position(new_pos)
            self.updata_left_down_corner_annotation()
            self.update_right_down_corner_annotation()
            self.last_pos = new_pos
            self.img_viewer.Render()
        else:
            super().OnMouseMove()

    def on_scroll_forward(self, obj, event):
        # super().OnMouseWheelForward()
        if self.slice<self.slice_max:
            self.slice += 1
            self.img_viewer.SetSlice(self.slice)
            self.slice_slider_widget.GetRepresentation().SetValue(self.slice)
            self.updata_left_down_corner_annotation()
            self.img_viewer.Render()

    def on_scroll_backward(self, obj, event):
        # super().OnMouseWheelBackward()
        if self.slice>self.slice_min:
            self.slice -= 1
            self.img_viewer.SetSlice(self.slice)
            self.slice_slider_widget.GetRepresentation().SetValue(self.slice)
            self.updata_left_down_corner_annotation()
            self.img_viewer.Render()

    def on_left_button_press(self, obj, event):
        self.is_left_button_pressed = True
        self.last_pos = self.GetInteractor().GetEventPosition()
        self.calc_cursor_position(self.last_pos)
        super().OnLeftButtonDown()

    def on_left_button_release(self, obj, event):
        self.is_left_button_pressed = False
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
        
    def calc_cursor_position(self,pick_pos):
        picker = vtk.vtkCellPicker()
        picker.SetTolerance(0.001)
        picker.Pick(pick_pos[0], pick_pos[1], 0, self.img_viewer.GetRenderer())
        if picker.GetCellId()!=-1:
            pos = picker.GetPickPosition()
            vtk_idx = [0,0,0]
            dataset = picker.GetDataSet()
            structuredPoints = vtk.vtkStructuredPoints()
            structuredPoints.CopyStructure(dataset)
            structuredPoints.ComputeStructuredCoordinates(pos, vtk_idx,[0,0,0])
            itk_idx = vtk_idx
            itk_idx[1] = 512-1-vtk_idx[1]
            self.pos_img = itk_idx
            self.ct_value = self.sitk_img.GetPixel(int(itk_idx[0]),int(itk_idx[1]),int(itk_idx[2]))
            self.pos_ct = self.sitk_img.TransformContinuousIndexToPhysicalPoint(itk_idx)
            self.update_right_down_corner_annotation()
        
    def create_left_down_corner_annotation(self):
        text_prop = vtk.vtkTextProperty()
        text_prop.SetFontFamilyToCourier()
        text_prop.SetFontSize(14)
        text_prop.SetVerticalJustificationToCentered()
        text_prop.SetJustificationToLeft()

        self.left_down_corner_annotation = vtk.vtkCornerAnnotation()
        self.left_down_corner_annotation.SetTextProperty(text_prop)
        self.left_down_corner_annotation.SetText(0,'W:{} L:{}\nSlice:{}/{}'.format(
            self.wd,self.wl,self.slice+1,self.slice_max+1))
        self.img_viewer.GetRenderer().AddActor2D(self.left_down_corner_annotation)

    def updata_left_down_corner_annotation(self):
        self.left_down_corner_annotation.SetText(0,'W:{} L:{}\nSlice:{}/{}'.format(
            self.wd,self.wl,self.slice+1,self.slice_max+1))

    def create_right_down_corner_annotation(self):
        text_prop = vtk.vtkTextProperty()
        text_prop.SetFontFamilyToCourier()
        text_prop.SetFontSize(10)
        text_prop.SetVerticalJustificationToCentered()
        text_prop.SetJustificationToRight()

        self.right_down_corner_annotation = vtk.vtkCornerAnnotation()
        self.right_down_corner_annotation.SetTextProperty(text_prop)
        self.right_down_corner_annotation.SetText(1,'World units:--\nX:-- Y:-- Z:--\nValue:--')
        self.img_viewer.GetRenderer().AddActor2D(self.right_down_corner_annotation)

    def update_right_down_corner_annotation(self):
         self.right_down_corner_annotation.SetText(1,'World units:{:.1f},{:.1f},{:.1f}\nX:{} Y:{} Z:{}\nValue:{}'.format(
            self.pos_ct[0],self.pos_ct[1],self.pos_ct[2],self.pos_img[0]+1,self.pos_img[1]+1,self.pos_img[2]+1,self.ct_value))

    def create_slice_slider(self,iren:vtk.vtkRenderWindowInteractor):
        slider_rep = vtk.vtkSliderRepresentation2D()
        slider_rep.SetMinimumValue(self.slice_min)
        slider_rep.SetMaximumValue(self.slice_max)
        slider_rep.SetValue(self.slice)
        slider_rep.GetSliderProperty().SetColor(colors.GetColor3d('slate_grey_dark'))
        slider_rep.GetSelectedProperty().SetColor(colors.GetColor3d('light_grey'))
        slider_rep.GetTubeProperty().SetColor(colors.GetColor3d('lamp_black'))
        slider_rep.GetCapProperty().SetColor(colors.GetColor3d('light_grey'))
        slider_rep.GetPoint1Coordinate().SetCoordinateSystemToNormalizedDisplay()
        slider_rep.GetPoint1Coordinate().SetValue(0.95,0.3)
        slider_rep.GetPoint2Coordinate().SetCoordinateSystemToNormalizedDisplay()
        slider_rep.GetPoint2Coordinate().SetValue(0.95,0.7)
        slider_rep.SetSliderLength(0.02)
        slider_rep.SetSliderWidth(0.02)
        slider_rep.SetTubeWidth(0.005)
        slider_rep.SetEndCapWidth(0.02)
        slider_rep.ShowSliderLabelOff()

        self.slice_slider_widget = vtk.vtkSliderWidget()
        self.slice_slider_widget.SetRepresentation(slider_rep)
        self.slice_slider_widget.SetAnimationModeToAnimate()
        self.slice_slider_widget.SetInteractor(iren)
        self.slice_slider_widget.AddObserver(vtk.vtkCommand.InteractionEvent,self.slice_slider_callback)
        self.slice_slider_widget.EnabledOn()

    def slice_slider_callback(self,obj,event):
        slider_rep = obj.GetRepresentation()
        self.slice = round(slider_rep.GetValue())
        self.img_viewer.SetSlice(self.slice)
        self.updata_left_down_corner_annotation()
        self.img_viewer.Render()

    def create_cross_line(self):
        crosshair_lines = vtk.vtkCellArray()
        crosshair_points = vtk.vtkPoints()
        crosshair_points.SetNumberOfPoints(4)        


class DICOMViewer():
    def __init__(self,dcm_path:str):
        reader = vtk.vtkDICOMImageReader()
        reader.SetDirectoryName(dcm_path)
        reader.Update()
        pixel_spacing = reader.GetPixelSpacing()
        vtk_img_data = reader.GetOutput()
        # vtk_img_data.SetSpacing(2,2,1)
        flip = vtk.vtkImageFlip()
        flip.SetFilteredAxes(2)
        flip.SetInputData(vtk_img_data)
        flip.Update()

        # rens = [vtk.vtkRenderer() for _ in range(4)]
        # ren_windows = [vtk.vtkRenderWindow() for _ in range(4)]
        # irens = [vtk.vtkRenderWindowInteractor() for _ in range(4)]
        # for idx,window in enumerate(ren_windows):
            # window.AddRenderer(rens[idx])
            # window.SetSize(512,512)
            # window.SetPosition(512*(idx%2),512*(idx//2))

        # for idx,orientation in enumerate(["Axial","Coronal","Sagittal"]):#横断面,冠状面,矢状面
        #     img_viewer = vtk.vtkImageViewer2()
        #     img_viewer.SetInputConnection(reader.GetOutputPort())

        self.view_axial = vtk.vtkImageViewer2()
        self.view_axial.SetInputConnection(flip.GetOutputPort())
        self.view_axial.GetRenderer().SetBackground(colors.GetColor3d('gray'))
        self.view_axial.GetRenderWindow().SetWindowName("Axial")
        self.view_axial.SetSize(512,512)
        self.view_axial.SetPosition(0,0)
        iren = vtk.vtkRenderWindowInteractor()
        self.view_axial.SetupInteractor(iren)

        style_axial = CustomInteractorStyle(self.view_axial,iren)
        # style_axial = vtk.vtkInteractorStyleImage()
        # style_axial.SetDefaultRenderer(self.view_axial.GetRenderer())
        iren.SetInteractorStyle(style_axial)

        # self.view_coronal = vtk.vtkImageViewer2()
        # self.view_coronal.SetInputConnection(reader.GetOutputPort())
        # self.view_coronal.SetRenderWindow(ren_windows[1])

        # self.view_sagittal = vtk.vtkImageViewer2()
        # self.view_sagittal.SetInputConnection(reader.GetOutputPort())
        # self.view_sagittal.SetRenderWindow(ren_windows[2])

        self.view_axial.Render()
        iren.Start()

if __name__=='__main__':
    dcm_viewer = DICOMViewer(dcm_path)
