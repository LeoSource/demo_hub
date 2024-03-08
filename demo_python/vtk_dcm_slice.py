# -*- coding: utf-8 -*-
# @File   		: vtk_dcm_slice.py
# @Description	: do something
# @Date   		: 2024/03/07 09:56:35
# @Author	    : zxliao, zhixiangleo@163.com

import vtk

dcm_path = "F:/0_project/prca/dicom/20240225/2024.02.25-144314-STD-1.3.12.2.1107.5.99.3/20240225/1.3.12.2.1107.5.1.7.120479.30000024022512255527200003523"

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
        self.slice = round(0.5*(self.slice_min+self.slice_max))
        print(f'Slice: Min = {self.slice_min}, Max = {self.slice_max}')

    def move_slice_forward(self):
        if self.slice<self.slice_max:
            self.slice += 1
            print(f'MoveSliceForward::Slice = {self.slice}')
            self.img_viewer.SetSlice(self.slice)
            self.img_viewer.Render()

    def move_slice_backward(self):
        if self.slice>self.slice_min:
            self.slice -= 1
            print(f'MoveSliceBackward::Slice = {self.slice}')
            self.img_viewer.SetSlice(self.slice)
            self.img_viewer.Render()

    def mouse_wheel_forward_event(self,obj,event):
        self.move_slice_forward()

    def mouse_wheel_backward_event(self,obj,event):
        self.move_slice_backward()

if __name__=='__main__':
    colors = vtk.vtkNamedColors()

    dcm_reader = vtk.vtkDICOMImageReader()
    dcm_reader.SetDirectoryName(dcm_path)
    dcm_reader.Update()
    # print(dcm_reader.GetPixelSpacing())
    # print(dcm_reader.GetImagePositionPatient())
    img_data = dcm_reader.GetOutput()#vtk.vtkImageData

    img_viewer = vtk.vtkImageViewer2()
    img_viewer.SetInputConnection(dcm_reader.GetOutputPort())
    iren = vtk.vtkRenderWindowInteractor()
    img_viewer.SetupInteractor(iren)
    # img_viewer.SetSlice(100)

    istyle = MyInteractorStyleImage()
    istyle.set_image_viewer(img_viewer)
    iren.SetInteractorStyle(istyle)
    iren.Render()

    print(img_viewer.GetColorLevel())
    img_viewer.GetRenderer().ResetCamera()
    img_viewer.GetRenderer().SetBackground(colors.GetColor3d('slavegray'))
    img_viewer.GetRenderWindow().SetSize(800,800)
    img_viewer.GetRenderWindow().SetWindowName('ReadDICOMSeries')
    img_viewer.Render()

    iren.Start()


