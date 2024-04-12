# -*- coding: utf-8 -*-
# @File   		: dicom_viewer.py
# @Description	: do something
# @Date   		: 2024/03/20 14:44:16
# @Author	    : zxliao, zhixiangleo@163.com

import vtk
import SimpleITK as sitk
import asyncio
import threading
import time
import sys

dcm_path = "F:/0_project/prca/dicom/20240225/2024.02.25-144314-STD-1.3.12.2.1107.5.99.3/20240225/1.3.12.2.1107.5.1.7.120479.30000024022512255527200003523"

def sitk_read_dcm_series(file_path:str)->sitk.Image:
    reader = sitk.ImageSeriesReader()
    dcm_names = reader.GetGDCMSeriesFileNames(file_path)
    reader.SetFileNames(dcm_names)
    return reader.Execute()

colors = vtk.vtkNamedColors()

class CustomInteractorStyle(vtk.vtkInteractorStyleImage):
    def __init__(self, img_viewer:vtk.vtkImageViewer2,
                 iren:vtk.vtkRenderWindowInteractor,
                 sitk_img:sitk.Image,
                 vtk_img:vtk.vtkImageData):
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
        self.sitk_img = sitk_img
        self.img_size = self.sitk_img.GetSize()

        self.slice_max = int(img_viewer.GetSliceMax())
        self.slice_min = int(img_viewer.GetSliceMin())
        self.slice = int(0.5*(self.slice_min+self.slice_max))
        self.img_viewer.SetSlice(self.slice)
        self.wl = int(img_viewer.GetColorLevel())
        self.wd = int(img_viewer.GetColorWindow())
        self.is_left_button_pressed = False
        self.last_view_pos = [0,0,0]
        self.view_pos = [self.img_size[0]/2, self.img_size[1]/2,self.slice]
        self.pos_ct = [0,0,0]
        self.pos_img = [0,0,0]
        self.ct_value = 0
        self.cross_len = 100
        self.cross_line = False

        self.create_left_down_corner_annotation()
        self.create_right_down_corner_annotation()
        self.create_slice_slider(iren)
        # self.create_cross_line()
        self.create_cross_line2(iren,vtk_img)

    def on_mouse_move(self, obj, event):
        if self.is_left_button_pressed:
            new_pos = self.GetInteractor().GetEventPosition()
            dx = new_pos[0] - self.last_view_pos[0]
            dy = new_pos[1] - self.last_view_pos[1]
            # Adjust window level based on dx, dy
            # This is a simplified example; you may want to adjust the scale of adjustment
            scale = 1
            self.wl += round(scale*dy)
            self.wd += round(scale*dx)
            # Update the image viewer and text display
            self.img_viewer.SetColorLevel(self.wl)
            self.img_viewer.SetColorWindow(self.wd)
            self.calc_cursor_position(new_pos)
            self.view_pos[0] = new_pos[0]
            self.view_pos[1] = new_pos[1]
            self.update_cross_line()
            self.update_left_down_corner_annotation()
            self.update_right_down_corner_annotation()
            self.last_view_pos = new_pos
            self.img_viewer.Render()
        else:
            super().OnMouseMove()

    def on_scroll_forward(self, obj, event):
        # super().OnMouseWheelForward()
        if self.slice<self.slice_max:
            self.slice += 1
            self.img_viewer.SetSlice(self.slice)
            self.slice_slider_widget.GetRepresentation().SetValue(self.slice)
            self.view_pos[2] = self.slice
            self.update_left_down_corner_annotation()
            self.update_cross_line()
            self.img_viewer.Render()

    def on_scroll_backward(self, obj, event):
        # super().OnMouseWheelBackward()
        if self.slice>self.slice_min:
            self.slice -= 1
            self.img_viewer.SetSlice(self.slice)
            self.slice_slider_widget.GetRepresentation().SetValue(self.slice)
            self.view_pos[2] = self.slice
            self.update_left_down_corner_annotation()
            self.update_cross_line()
            self.img_viewer.Render()

    def on_left_button_press(self, obj, event):
        self.is_left_button_pressed = True
        self.last_view_pos = self.GetInteractor().GetEventPosition()
        self.view_pos[0] = self.last_view_pos[0]
        self.view_pos[1] = self.last_view_pos[1]
        self.calc_cursor_position(self.last_view_pos)
        self.update_cross_line()
        super().OnLeftButtonDown()
        self.img_viewer.Render()

    def on_left_button_release(self, obj, event):
        self.is_left_button_pressed = False
        super().OnLeftButtonUp()
        
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

    def update_left_down_corner_annotation(self):
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
        self.view_pos[2] = self.slice
        self.update_left_down_corner_annotation()
        self.update_cross_line()
        self.img_viewer.Render()

    def create_cross_line(self):
        cross_lines = vtk.vtkCellArray()
        self.cross_points = vtk.vtkPoints()
        self.cross_points.SetNumberOfPoints(4)
        cross_length = 100
        self.cross_points.InsertPoint(0, self.img_size[0]/2, self.img_size[1]/2 - self.cross_len, self.slice*3.5)
        self.cross_points.InsertPoint(1, self.img_size[0]/2, self.img_size[1]/2 + self.cross_len, self.slice*3.5)
        self.cross_points.InsertPoint(2, self.img_size[0]/2 - self.cross_len, self.img_size[1]/2, self.slice*3.5)
        self.cross_points.InsertPoint(3, self.img_size[0]/2 + self.cross_len, self.img_size[1]/2, self.slice*3.5)
        
        cross_lines.InsertNextCell(2)
        cross_lines.InsertCellPoint(0)
        cross_lines.InsertCellPoint(1)

        cross_lines.InsertNextCell(2)
        cross_lines.InsertCellPoint(2)
        cross_lines.InsertCellPoint(3)

        cross_polydata = vtk.vtkPolyData()
        cross_polydata.SetPoints(self.cross_points)
        cross_polydata.SetLines(cross_lines)
        cross_mapper = vtk.vtkPolyDataMapper()
        cross_mapper.SetInputData(cross_polydata)
        crosshair_actor = vtk.vtkActor()
        crosshair_actor.SetMapper(cross_mapper)
        crosshair_actor.GetProperty().SetLineStipplePattern(0xF0F0)  # 设置虚线样式
        crosshair_actor.GetProperty().SetLineStippleRepeatFactor(2)
        crosshair_actor.GetProperty().SetColor(colors.GetColor3d('blue'))
        self.img_viewer.GetRenderer().AddActor(crosshair_actor)

    def update_cross_line(self):
        if self.cross_line:
            self.cross_points.InsertPoint(0, self.view_pos[0], self.view_pos[1] - self.cross_len, self.view_pos[2]+self.img_size[2])
            self.cross_points.InsertPoint(1, self.view_pos[0], self.view_pos[1] + self.cross_len, self.view_pos[2]+self.img_size[2])
            self.cross_points.InsertPoint(2, self.view_pos[0] - self.cross_len, self.view_pos[1], self.view_pos[2]+self.img_size[2])
            self.cross_points.InsertPoint(3, self.view_pos[0] + self.cross_len, self.view_pos[1], self.view_pos[2]+self.img_size[2])
            self.cross_points.Modified()

    def create_cross_line2(self,iren,vtk_img):
        reslice_cursor = vtk.vtkResliceCursor()
        reslice_cursor.SetCenter(256,256,1)
        reslice_cursor.SetThickMode(1)
        reslice_cursor.SetThickness(10,10,10)
        reslice_cursor.SetHole(0)
        reslice_cursor.SetImage(vtk_img)

        cursorWidget = vtk.vtkResliceCursorWidget()
        cursorWidget.SetInteractor(iren)
        rep = vtk.vtkResliceCursorThickLineRepresentation()
        rep.SetWindowLevel(500,0,0)
        rep.GetResliceCursorActor().GetCursorAlgorithm().SetResliceCursor(reslice_cursor)
        cursorWidget.SetRepresentation(rep)
        cursorWidget.EnabledOn()


class DICOMViewer():
    def __init__(self,dcm_path:str):
        self.sitk_img = sitk_read_dcm_series(dcm_path)
        reader = vtk.vtkDICOMImageReader()
        reader.SetDirectoryName(dcm_path)
        reader.Update()
        pixel_spacing = reader.GetPixelSpacing()
        vtk_img_data = reader.GetOutput()
        reslice = vtk.vtkImageReslice()
        reslice.SetInputData(vtk_img_data)
        reslice.SetResliceAxesDirectionCosines(1,0,0,0,1,0,0,0,-1)
        # vtk_img_data.SetSpacing(2,2,1)
        flip = vtk.vtkImageFlip()
        # flip.SetFilteredAxes(0)
        flip.SetFilteredAxes(2)
        # flip.SetFilteredAxes(1)
        flip.SetInputData(vtk_img_data)
        flip.Update()
        self.vtk_img_data = flip.GetOutput()
        # self.view_axial = vtk.vtkResliceImageViewer()
        self.view_axial = vtk.vtkImageViewer2()
        self.view_coronal = vtk.vtkImageViewer2()
        self.view_sagittal = vtk.vtkImageViewer2()
        self.view_axial.SetSliceOrientationToXY()
        self.view_coronal.SetSliceOrientationToXZ()
        self.view_sagittal.SetSliceOrientationToYZ()
        self.view_axial.SetInputData(flip.GetOutput())
        self.view_coronal.SetInputConnection(reslice.GetOutputPort())
        self.view_sagittal.SetInputConnection(reslice.GetOutputPort())


        self.num_windows = 3

    async def axial(self):
        self.view_axial.GetRenderer().SetBackground(colors.GetColor3d('gray'))
        self.view_axial.GetRenderWindow().SetWindowName("Axial")
        self.view_axial.SetSize(512,512)
        self.view_axial.SetPosition(0,0)
        iren = vtk.vtkRenderWindowInteractor()
        self.view_axial.SetupInteractor(iren)
        style_axial = CustomInteractorStyle(self.view_axial,iren,self.sitk_img,self.vtk_img_data)
        # style_axial = vtk.vtkInteractorStyleImage()
        # style_axial.SetDefaultRenderer(self.view_axial.GetRenderer())
        iren.SetInteractorStyle(style_axial)
        iren.AddObserver(vtk.vtkCommand.ExitEvent,self.close_window)
        self.view_axial.Render()
        await asyncio.sleep(0.1)
        iren.Start()

    async def sagittal(self):
        self.view_sagittal.GetRenderer().SetBackground(colors.GetColor3d('gray'))
        self.view_sagittal.GetRenderWindow().SetWindowName("Sagittal")
        self.view_sagittal.SetSize(512,512)
        self.view_sagittal.SetPosition(512,0)
        iren = vtk.vtkRenderWindowInteractor()
        self.view_sagittal.SetupInteractor(iren)
        style_sagittal = CustomInteractorStyle(self.view_sagittal,iren,self.sitk_img,self.vtk_img_data)
        iren.SetInteractorStyle(style_sagittal)
        iren.AddObserver(vtk.vtkCommand.ExitEvent,self.close_window)
        self.view_sagittal.Render()
        await asyncio.sleep(0.1)
        iren.Start()

    async def coronal(self):
        self.view_coronal.GetRenderer().SetBackground(colors.GetColor3d('gray'))
        self.view_coronal.GetRenderWindow().SetWindowName("Coronal")
        self.view_coronal.SetSize(512,512)
        self.view_coronal.SetPosition(0,512)
        iren = vtk.vtkRenderWindowInteractor()
        self.view_coronal.SetupInteractor(iren)
        style_coronal = CustomInteractorStyle(self.view_coronal,iren,self.sitk_img,self.vtk_img_data)
        iren.SetInteractorStyle(style_coronal)
        iren.AddObserver(vtk.vtkCommand.ExitEvent,self.close_window)
        self.view_coronal.Render()
        await asyncio.sleep(0.1)
        iren.Start()

    def close_window(self, obj, event):
        self.num_windows -= 1
        if self.num_windows == 0:
            sys.exit()

async def main():
    viewer = DICOMViewer(dcm_path)
    task1 = asyncio.create_task(viewer.axial())
    task2 = asyncio.create_task(viewer.coronal())
    task3 = asyncio.create_task(viewer.sagittal())
    await asyncio.gather(task1,task2,task3)

if __name__=='__main__':
    asyncio.run(main())
