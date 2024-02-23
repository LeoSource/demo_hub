# -*- coding: utf-8 -*-
# @File   		: vtk_dcm.py
# @Description	: do something
# @Date   		: 2024/02/22 15:59:56
# @Author	    : zxliao, zhixiangleo@163.com

import vtk

dcm_path = "F:/0_project/prca/dicom/20240122/1.2.840.113619.2.428.3.695552.36.1705889324.84"
stl_file = "./puncture_robot_model/needle.stl"

def load_stl_to_actor(filename):
    reader = vtk.vtkSTLReader()
    reader.SetFileName(filename)
    mapper = vtk.vtkPolyDataMapper()
    mapper.SetInputConnection(reader.GetOutputPort())
    actor = vtk.vtkActor()
    actor.SetMapper(mapper)
    return actor

def show_dcm_3d():
    colors = vtk.vtkNamedColors()

    reader = vtk.vtkDICOMImageReader()
    reader.SetDirectoryName(dcm_path)

    # contourfilter = vtk.vtkContourFilter()
    contourfilter = vtk.vtkMarchingCubes()
    contourfilter.SetInputConnection(reader.GetOutputPort())
    # contourfilter.ComputeNormalsOn()
    contourfilter.SetValue(0, 200)
 
    normal = vtk.vtkPolyDataNormals()
    normal.SetInputConnection(contourfilter.GetOutputPort())
    normal.SetFeatureAngle(60)
 
    conMapper = vtk.vtkPolyDataMapper()
    conMapper.SetInputConnection(normal.GetOutputPort())
    conMapper.ScalarVisibilityOff()
 
    conActor = vtk.vtkActor()
    conActor.SetMapper(conMapper)
 
    boxFilter = vtk.vtkOutlineFilter()
    boxFilter.SetInputConnection(reader.GetOutputPort())
    boxMapper = vtk.vtkPolyDataMapper()
    boxMapper.SetInputConnection(boxFilter.GetOutputPort())
    boxActor = vtk.vtkActor()
    boxActor.SetMapper(boxMapper)
    boxActor.GetProperty().SetColor(colors.GetColor3d('white'))

    axes_actor = vtk.vtkAxesActor()
    axes_actor.SetTotalLength(40,40,40)
    axes_actor.AxisLabelsOff()

    needle_actor = load_stl_to_actor(stl_file)
    needle_actor.GetProperty().SetColor(colors.GetColor3d('wheat'))
 
    camera = vtk.vtkCamera()
    camera.SetViewUp(0, 0, -1)
    camera.SetPosition(0, 1, 0)
    camera.SetFocalPoint(0, 0, 0)
    camera.ComputeViewPlaneNormal()
    camera.Dolly(1.5)
 
    render = vtk.vtkRenderer()
    render.AddActor(conActor)
    render.AddActor(boxActor)
    render.AddActor(axes_actor)
    render.AddActor(needle_actor)
    render.SetActiveCamera(camera)
    render.ResetCamera()

    ren_win = vtk.vtkRenderWindow()
    ren_win.AddRenderer(render)
    ren_win.SetSize(800,800)
 
    iren = vtk.vtkRenderWindowInteractor()
    iren.SetRenderWindow(ren_win)
    style = vtk.vtkInteractorStyleTrackballCamera()
    iren.SetInteractorStyle(style)
    iren.Initialize()
    iren.Start()

if __name__=='__main__':
    show_dcm_3d()

